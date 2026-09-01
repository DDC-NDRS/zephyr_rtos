/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Infineon Technologies AG,
 * SPDX-FileCopyrightText: or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * PWM driver for Infineon MCUs using the TCPWM block.
 */

#define DT_DRV_COMPAT infineon_tcpwm_pwm

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>

#include <infineon_kconfig.h>
#include <zephyr/drivers/timer/ifx_tcpwm.h>
#include <zephyr/dt-bindings/pwm/pwm_ifx_tcpwm.h>
#include <zephyr/drivers/clock_control/clock_control_ifx_cat1.h>

#include <cy_tcpwm_pwm.h>
#include <cy_tcpwm_counter.h>
#include <cy_trigmux.h>
#include <cy_gpio.h>
#include <cy_sysclk.h>

#include <zephyr/logging/log.h>

#if defined(CONFIG_PWM_EVENT) || defined(CONFIG_PWM_CAPTURE)
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

/* Infineon TCPWM PWM channels are single-channel */
#define IFX_TCPWM_PWM_CH 0

/* IF_ENABLED()/COND_CODE_1() need a symbol that literally expands to 0 or 1 - neither
 * CONFIG_PWM_EVENT nor CONFIG_PWM_CAPTURE alone captures "this instance needs an ISR",
 * since either one independently justifies wiring the interrupt.
 */
#define IFX_TCPWM_PWM_IRQ_ENABLED 1
#else
#define IFX_TCPWM_PWM_IRQ_ENABLED 0
#endif /* CONFIG_PWM_EVENT || CONFIG_PWM_CAPTURE */

#ifdef CONFIG_PWM_EVENT
#include <zephyr/drivers/pwm/pwm_utils.h>
#endif /* CONFIG_PWM_EVENT */

LOG_MODULE_REGISTER(pwm_ifx_tcpwm, CONFIG_PWM_LOG_LEVEL);

struct ifx_tcpwm_pwm_config {
    TCPWM_GRP_CNT_Type* reg_base;
    const struct pinctrl_dev_config* pcfg;
    bool resolution_32_bits;
    uint32_t tcpwm_index;
    uint32_t index;
    uint32_t clk_dst;

    #ifdef CONFIG_PWM_CAPTURE
    bool capture_input;
    uint32_t trigmux_in;
    uint32_t trigmux_out;
    #endif /* CONFIG_PWM_CAPTURE */
};

#ifdef CONFIG_PWM_CAPTURE
/* Free-running-counter edge timestamps for one capture channel - see
 * ifx_tcpwm_capture_init()'s comment for the capture scheme.
 */
struct ifx_tcpwm_pwm_capture_data {
    pwm_capture_callback_handler_t callback;
    void* user_data;
    bool continuous;
    bool period_capture;
    bool pulse_capture;
    bool have_prev_rising;
    bool have_pulse;

    uint32_t prev_rising;
    uint32_t pulse_cycles;
};
#endif /* CONFIG_PWM_CAPTURE */

struct ifx_tcpwm_pwm_data {
    struct ifx_cat1_clock clock;

    #ifdef CONFIG_PM_DEVICE
    /* Last values programmed via set_cycles, replayed on resume to
     * restore the PWM after DS-RAM (all peripheral state lost).
     */
    uint32_t last_period_cycles;
    uint32_t last_pulse_cycles;
    pwm_flags_t last_flags;

    /* Whether the counter was running when suspend was entered, so
     * resume only restarts a PWM that was previously active.
     */
    bool was_running;
    #endif /* CONFIG_PM_DEVICE */

    #if defined(CONFIG_PWM_EVENT) || defined(CONFIG_PWM_CAPTURE)
    struct k_spinlock lock;
    #endif

    #ifdef CONFIG_PWM_EVENT
    sys_slist_t event_callbacks;
    #endif /* CONFIG_PWM_EVENT */

    #ifdef CONFIG_PWM_CAPTURE
    struct ifx_tcpwm_pwm_capture_data capture;
    #endif /* CONFIG_PWM_CAPTURE */
};

static int ifx_tcpwm_output_init(const struct device* dev) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    cy_en_tcpwm_status_t status;
    int ret;

    cy_stc_tcpwm_pwm_config_t const pwm_config = {
        .pwmMode           = CY_TCPWM_PWM_MODE_PWM,
        .clockPrescaler    = CY_TCPWM_PWM_PRESCALER_DIVBY_1,
        .pwmAlignment      = CY_TCPWM_PWM_LEFT_ALIGN,
        .runMode           = CY_TCPWM_PWM_CONTINUOUS,
        .countInputMode    = CY_TCPWM_INPUT_LEVEL,
        .countInput        = CY_TCPWM_INPUT_1,
        .enableCompareSwap = true,
        .enablePeriodSwap  = true,
        .line_out_sel      = CY_TCPWM_OUTPUT_PWM_SIGNAL,
        .linecompl_out_sel = CY_TCPWM_OUTPUT_INVERTED_PWM_SIGNAL
    };

    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return (ret);
    }

    /* Configure the TCPWM to be a PWM */
    status = IFX_TCPWM_PWM_Init(config->reg_base, &pwm_config);
    if (status != CY_TCPWM_SUCCESS) {
        return (-ENOTSUP);
    }

    return (0);
}

#ifdef CONFIG_PWM_CAPTURE
static int ifx_tcpwm_capture_init(const struct device* dev) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    cy_en_tcpwm_status_t status;
    cy_en_trigmux_status_t trigmux_status;
    int ret;

    /*
     * Free-running up-counter with independent rising-edge (CC0) and falling-edge
     * (CC1) capture on the same pinctrl-selected input pin. Period is derived from
     * consecutive CC0 timestamps, pulse width from a CC1 timestamp relative to the
     * preceding CC0 - same edge-timestamp-delta technique as NXP's eMIOS ICU driver
     * (pwm_nxp_s32_emios.c), adapted to TCPWM's two independent capture registers
     * instead of a shared edge FIFO.
     *
     * CY_TCPWM_INPUT_TRIG_0 is this instance's own dedicated pinctrl-routed capture
     * trigger - verified against this exact board's ModusToolbox-generated config
     * (modules/hal/infineon/zephyr-ifx-cycfg/pse84/bcu_mv25/cycfg_peripherals.c,
     * tcpwm_0_group_0_cnt_5/6_config: both resolve TCPWM0_GRP0_CNTx_CAPTURE0_VALUE to
     * CY_TCPWM_INPUT_TRIG_0).
     */
    cy_stc_tcpwm_counter_config_t const capture_config = {
        .period            = config->resolution_32_bits ? UINT32_MAX : UINT16_MAX,
        .clockPrescaler    = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
        .runMode           = CY_TCPWM_COUNTER_CONTINUOUS,
        .countDirection    = CY_TCPWM_COUNTER_COUNT_UP,
        .compareOrCapture  = CY_TCPWM_COUNTER_MODE_CAPTURE,
        .interruptSources  = CY_TCPWM_INT_NONE,
        .captureInputMode  = CY_TCPWM_INPUT_RISINGEDGE,
        .captureInput      = CY_TCPWM_INPUT_TRIG_0,
        .reloadInputMode   = CY_TCPWM_INPUT_LEVEL,
        .reloadInput       = CY_TCPWM_INPUT_0,
        .startInputMode    = CY_TCPWM_INPUT_LEVEL,
        .startInput        = CY_TCPWM_INPUT_0,
        .stopInputMode     = CY_TCPWM_INPUT_LEVEL,
        .stopInput         = CY_TCPWM_INPUT_0,
        .countInputMode    = CY_TCPWM_INPUT_LEVEL,
        .countInput        = CY_TCPWM_INPUT_1,
        .capture1InputMode = CY_TCPWM_INPUT_FALLINGEDGE,
        .capture1Input     = CY_TCPWM_INPUT_TRIG_0,
    };

    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return (ret);
    }

    /* Route the pinctrl-selected pin's HSIOM trigger-mux output into this instance's
     * CY_TCPWM_INPUT_TRIG_0 capture input - the pin function alone (HSIOM) doesn't wire
     * the signal all the way to the TCPWM; the trigger mux itself needs a separate,
     * explicit connection. trigmux-in/trigmux-out come straight from devicetree (per-SoC
     * generated trigger-mux table values), not hardcoded here, so this stays board-
     * agnostic.
     */
    trigmux_status = Cy_TrigMux_Connect(config->trigmux_in, config->trigmux_out, false,
                                        TRIGGER_TYPE_LEVEL);
    if (trigmux_status != CY_TRIGMUX_SUCCESS) {
        return (-EIO);
    }

    TCPWM_GRP_CNT_Type* tcpwm = config->reg_base;
    status = IFX_TCPWM_Counter_Init(tcpwm, &capture_config);
    if (status != CY_TCPWM_SUCCESS) {
        return (-ENOTSUP);
    }

    IFX_TCPWM_Counter_Enable(tcpwm);
    IFX_TCPWM_TriggerStart_Single(tcpwm);

    return (0);
}
#endif /* CONFIG_PWM_CAPTURE */

static int ifx_tcpwm_pwm_init(const struct device* dev) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* const data = dev->data;
    cy_en_tcpwm_status_t status;

    /* Connect this TCPWM to the peripheral clock */
    status = ifx_cat1_utils_peri_pclk_assign_divider(config->clk_dst, &data->clock);
    if (status != CY_RSLT_SUCCESS) {
        return (-EIO);
    }

    #ifdef CONFIG_PWM_CAPTURE
    if (config->capture_input) {
        return ifx_tcpwm_capture_init(dev);
    }
    #endif /* CONFIG_PWM_CAPTURE */

    return ifx_tcpwm_output_init(dev);
}

static int ifx_tcpwm_pwm_set_cycles(const struct device* dev, uint32_t channel,
                                    uint32_t period_cycles, uint32_t pulse_cycles,
                                    pwm_flags_t flags) {
    ARG_UNUSED(channel);

    const struct ifx_tcpwm_pwm_config* config = dev->config;
    #ifdef CONFIG_PM_DEVICE
    struct ifx_tcpwm_pwm_data* const data = dev->data;
    #endif /* CONFIG_PM_DEVICE */
    uint32_t pwm_status;
    uint32_t ctrl_temp;

    #ifdef CONFIG_PWM_CAPTURE
    if (config->capture_input) {
        return (-ENOTSUP);
    }
    #endif /* CONFIG_PWM_CAPTURE */

    if (!config->resolution_32_bits &&
        ((period_cycles > UINT16_MAX) || (pulse_cycles > UINT16_MAX))) {
        /* 16-bit resolution */
        if (period_cycles > UINT16_MAX) {
            LOG_ERR("Period cycles more than 16-bits (%u)", period_cycles);
        }

        if (pulse_cycles > UINT16_MAX) {
            LOG_ERR("Pulse cycles more than 16-bits (%u)", pulse_cycles);
        }

        return (-EINVAL);
    }

    TCPWM_GRP_CNT_Type* tcpwm = config->reg_base;
    if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED) {
        tcpwm->CTRL |= TCPWM_GRP_CNT_V2_CTRL_QUAD_ENCODING_MODE_Msk;
    }
    else {
        tcpwm->CTRL &= ~TCPWM_GRP_CNT_V2_CTRL_QUAD_ENCODING_MODE_Msk;
    }

    ctrl_temp = tcpwm->CTRL & ~TCPWM_GRP_CNT_V2_CTRL_PWM_DISABLE_MODE_Msk;

    tcpwm->CTRL = ctrl_temp | _VAL2FLD(TCPWM_GRP_CNT_V2_CTRL_PWM_DISABLE_MODE,
                                       (flags & PWM_IFX_TCPWM_OUTPUT_MASK) >>
                                       PWM_IFX_TCPWM_OUTPUT_POS);

    /* If the PWM is not yet running, write the period and compare directly pwm won't start
     * correctly.
     */
    pwm_status = IFX_TCPWM_PWM_GetStatus(tcpwm);
    if ((pwm_status & TCPWM_GRP_CNT_V2_STATUS_RUNNING_Msk) == 0) {
        if ((period_cycles != 0) && (pulse_cycles != 0)) {
            IFX_TCPWM_PWM_SetPeriod0(tcpwm, period_cycles - 1);
            IFX_TCPWM_PWM_SetCompare0Val(tcpwm, pulse_cycles);
        }
    }

    /* Special case, if period_cycles is 0, set the period and compare to zero.  If we were to
     * disable the PWM, the output would be set to High-Z, whereas this will set the output to
     * the zero duty cycle state instead.
     */
    if (period_cycles == 0) {
        IFX_TCPWM_PWM_SetPeriod1(tcpwm, 0);
        IFX_TCPWM_PWM_SetCompare0BufVal(tcpwm, 0);
        IFX_TCPWM_TriggerCaptureOrSwap_Single(tcpwm);
    }
    else {
        /* Update period and compare values using buffer registers so the new values take
         * effect on the next TC event.  This prevents glitches in PWM output depending on
         * where in the PWM cycle the update occurs.
         */
        IFX_TCPWM_PWM_SetPeriod1(tcpwm, period_cycles - 1);
        IFX_TCPWM_PWM_SetCompare0BufVal(tcpwm, pulse_cycles);

        /* Trigger the swap by writing to the SW trigger command register.
         */
        IFX_TCPWM_TriggerCaptureOrSwap_Single(tcpwm);
    }

    /* Enable the TCPWM for PWM mode of operation */
    IFX_TCPWM_PWM_Enable(tcpwm);

    /* Start the TCPWM block */
    IFX_TCPWM_TriggerStart_Single(tcpwm);

    #ifdef CONFIG_PM_DEVICE
    /* Cache the request so it can be replayed on resume after DS-RAM. */
    data->last_period_cycles = period_cycles;
    data->last_pulse_cycles  = pulse_cycles;
    data->last_flags         = flags;
    data->was_running        = true;
    #endif /* CONFIG_PM_DEVICE */

    return (0);
}

static int ifx_tcpwm_pwm_get_cycles_per_sec(const struct device* dev, uint32_t channel,
                                            uint64_t* cycles) {
    ARG_UNUSED(channel);

    struct ifx_tcpwm_pwm_data* const data = dev->data;
    const struct ifx_tcpwm_pwm_config* config = dev->config;

    *cycles = ifx_cat1_utils_peri_pclk_get_frequency(config->clk_dst, &data->clock);

    return (0);
}

#ifdef CONFIG_PWM_CAPTURE
static ALWAYS_INLINE uint32_t ifx_tcpwm_capture_delta(uint32_t first, uint32_t second,
                                                      uint32_t period_reg) {
    if (first <= second) {
        return (second - first);
    }

    /* Free-running counter wrapped between the two timestamps. */
    return ((period_reg - first) + second + 1U);
}

/* Processes one ISR entry's worth of capture events. CC1 (falling) is handled before CC0
 * (rising) so that, in the normal one-edge-per-ISR-call case, a pulse captured during this
 * period is available in time for the CC0 branch below to close out the callback for that
 * same period - this ordering is only meaningful when both bits are pending together
 * (delayed ISR entry), which the fixed rising-then-falling-then-rising cycle shape makes
 * rare in practice.
 */
static void ifx_tcpwm_pwm_capture_isr(const struct device* dev, uint32_t pending) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* data = dev->data;
    struct ifx_tcpwm_pwm_capture_data* cap = &data->capture;
    uint32_t period_reg = config->resolution_32_bits ? UINT32_MAX : UINT16_MAX;
    TCPWM_GRP_CNT_Type* tcpwm = config->reg_base;

    if ((pending & CY_TCPWM_INT_ON_CC1) != 0) {
        uint32_t falling = IFX_TCPWM_Counter_GetCapture1Val(tcpwm);

        if (cap->have_prev_rising) {
            cap->pulse_cycles = ifx_tcpwm_capture_delta(cap->prev_rising, falling,
                                                        period_reg);
            cap->have_pulse = true;
        }
    }

    if ((pending & CY_TCPWM_INT_ON_CC0) != 0) {
        uint32_t rising = IFX_TCPWM_Counter_GetCapture0Val(tcpwm);

        if (cap->have_prev_rising && (cap->callback != NULL) &&
            (!cap->pulse_capture || cap->have_pulse)) {
            uint32_t period = cap->period_capture ?
                              ifx_tcpwm_capture_delta(cap->prev_rising, rising, period_reg) :
                              0U;
            uint32_t pulse = cap->pulse_capture ? cap->pulse_cycles : 0U;

            cap->callback(dev, IFX_TCPWM_PWM_CH, period, pulse, 0, cap->user_data);
            cap->have_pulse = false;

            if (!cap->continuous) {
                IFX_TCPWM_SetInterruptMask(tcpwm, CY_TCPWM_INT_NONE);
            }
        }

        cap->prev_rising = rising;
        cap->have_prev_rising = true;
    }
}
#endif /* CONFIG_PWM_CAPTURE */

#if defined(CONFIG_PWM_EVENT) || defined(CONFIG_PWM_CAPTURE)
static void ifx_tcpwm_pwm_isr(const struct device* dev) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    uint32_t pending;

    pending = IFX_TCPWM_GetInterruptStatusMasked(config->reg_base);
    IFX_TCPWM_ClearInterrupt(config->reg_base, pending);

    #ifdef CONFIG_PWM_CAPTURE
    if (config->capture_input) {
        ifx_tcpwm_pwm_capture_isr(dev, pending);
        return;
    }
    #endif /* CONFIG_PWM_CAPTURE */

    #ifdef CONFIG_PWM_EVENT
    struct ifx_tcpwm_pwm_data* data = dev->data;

    if ((pending & CY_TCPWM_INT_ON_TC) != 0) {
        pwm_fire_event_callbacks(&data->event_callbacks, dev, IFX_TCPWM_PWM_CH,
                                 PWM_EVENT_TYPE_PERIOD);
    }

    if ((pending & CY_TCPWM_INT_ON_CC0) != 0) {
        pwm_fire_event_callbacks(&data->event_callbacks, dev, IFX_TCPWM_PWM_CH,
                                 PWM_EVENT_TYPE_COMPARE_CAPTURE);
    }
    #endif /* CONFIG_PWM_EVENT */
}

#ifdef CONFIG_PWM_EVENT
static void ifx_tcpwm_pwm_update_interrupts(const struct device* dev) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* data = dev->data;
    struct pwm_event_callback* cb;
    struct pwm_event_callback* tmp;
    uint32_t mask = CY_TCPWM_INT_NONE;

    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->event_callbacks, cb, tmp, node) {
        if ((cb->event_mask & PWM_EVENT_TYPE_PERIOD) != 0) {
            mask |= CY_TCPWM_INT_ON_TC;
        }

        if ((cb->event_mask & PWM_EVENT_TYPE_COMPARE_CAPTURE) != 0) {
            mask |= CY_TCPWM_INT_ON_CC0;
        }
    }

    IFX_TCPWM_ClearInterrupt(config->reg_base, mask);
    IFX_TCPWM_SetInterruptMask(config->reg_base, mask);
}

static int ifx_tcpwm_pwm_manage_event_callback(const struct device* dev,
                                               struct pwm_event_callback* callback, bool set) {
    struct ifx_tcpwm_pwm_data* data = dev->data;
    int ret;

    ret = pwm_manage_event_callback(&data->event_callbacks, callback, set);
    if (ret < 0) {
        return (ret);
    }

    K_SPINLOCK(&data->lock) {
        ifx_tcpwm_pwm_update_interrupts(dev);
    }

    return (0);
}
#endif /* CONFIG_PWM_EVENT */
#endif /* CONFIG_PWM_EVENT || CONFIG_PWM_CAPTURE */

#ifdef CONFIG_PWM_CAPTURE
static int ifx_tcpwm_pwm_capture_configure(const struct device* dev, uint32_t channel,
                                           pwm_flags_t flags, pwm_capture_callback_handler_t cb,
                                           void* user_data) {
    ARG_UNUSED(channel);

    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* data = dev->data;

    if (!config->capture_input) {
        return (-ENOTSUP);
    }

    if (!flags) {
        LOG_ERR("Invalid PWM capture flag");
        return (-EINVAL);
    }

    K_SPINLOCK(&data->lock) {
        data->capture.callback   = cb;
        data->capture.user_data  = user_data;
        data->capture.continuous =
            ((flags & PWM_CAPTURE_MODE_MASK) == PWM_CAPTURE_MODE_CONTINUOUS);
        data->capture.period_capture   = ((flags & PWM_CAPTURE_TYPE_PERIOD) != 0);
        data->capture.pulse_capture    = ((flags & PWM_CAPTURE_TYPE_PULSE ) != 0);
        data->capture.have_prev_rising = false;
        data->capture.have_pulse       = false;
    }

    return (0);
}

static int ifx_tcpwm_pwm_capture_enable(const struct device* dev, uint32_t channel) {
    ARG_UNUSED(channel);

    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* data = dev->data;
    uint32_t mask;

    if (!config->capture_input) {
        return (-ENOTSUP);
    }

    if (data->capture.callback == NULL) {
        LOG_ERR("Callback is not configured");
        return (-EINVAL);
    }

    mask = CY_TCPWM_INT_ON_CC0;
    if (data->capture.pulse_capture) {
        mask |= CY_TCPWM_INT_ON_CC1;
    }

    K_SPINLOCK(&data->lock) {
        data->capture.have_prev_rising = false;
        data->capture.have_pulse       = false;

        IFX_TCPWM_ClearInterrupt(config->reg_base,
                                 CY_TCPWM_INT_ON_CC0 | CY_TCPWM_INT_ON_CC1);
        IFX_TCPWM_SetInterruptMask(config->reg_base, mask);
    }

    return (0);
}

static int ifx_tcpwm_pwm_capture_disable(const struct device* dev, uint32_t channel) {
    ARG_UNUSED(channel);

    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* data = dev->data;

    if (!config->capture_input) {
        return (-ENOTSUP);
    }

    K_SPINLOCK(&data->lock) {
        IFX_TCPWM_SetInterruptMask(config->reg_base, CY_TCPWM_INT_NONE);
    }

    return (0);
}
#endif /* CONFIG_PWM_CAPTURE */

#ifdef CONFIG_PM_DEVICE
static int ifx_tcpwm_pwm_pm_action(const struct device* dev, enum pm_device_action action) {
    const struct ifx_tcpwm_pwm_config* config = dev->config;
    struct ifx_tcpwm_pwm_data* const data = dev->data;
    int ret;

    switch (action) {
        case PM_DEVICE_ACTION_SUSPEND :
            /* Clock gate the block; clock tree left untouched. */
            IFX_TCPWM_PWM_Disable(config->reg_base);
            break;

        case PM_DEVICE_ACTION_RESUME :
            IFX_TCPWM_PWM_Enable(config->reg_base);
            /* Restart the PWM if it was running before suspend. */
            if (data->was_running) {
                ret = ifx_tcpwm_pwm_set_cycles(dev, 0, data->last_period_cycles,
                                               data->last_pulse_cycles, data->last_flags);
                if (ret < 0) {
                    return (ret);
                }
            }
            break;

        #if defined(CONFIG_PM_S2RAM) || defined(CONFIG_PM_DEVICE_POWER_DOMAIN)
        case PM_DEVICE_ACTION_TURN_ON :
            /* Power was removed so re-initialize the peripheral */
            ret = ifx_tcpwm_pwm_init(dev);
            if (ret < 0) {
                return (ret);
            }
            break;
        #endif /* CONFIG_PM_S2RAM || CONFIG_PM_DEVICE_POWER_DOMAIN */

        default :
            return (-ENOTSUP);
    }

    return (0);
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(pwm, ifx_tcpwm_pwm_api) = {
    .set_cycles         = ifx_tcpwm_pwm_set_cycles,
    .get_cycles_per_sec = ifx_tcpwm_pwm_get_cycles_per_sec,

    #ifdef CONFIG_PWM_EVENT
    .manage_event_callback = ifx_tcpwm_pwm_manage_event_callback,
    #endif /* CONFIG_PWM_EVENT */

    #ifdef CONFIG_PWM_CAPTURE
    .configure_capture = ifx_tcpwm_pwm_capture_configure,
    .enable_capture    = ifx_tcpwm_pwm_capture_enable,
    .disable_capture   = ifx_tcpwm_pwm_capture_disable,
    #endif /* CONFIG_PWM_CAPTURE */
};

/*
 * Initialize the peripheral clock divider from devicetree.
 *
 * PSoC Edge SoCs require an extra peri_group index argument to
 * IFX_CAT1_PERIPHERAL_GROUP_ADJUST, so the two variants are selected
 * at compile time based on CONFIG_SOC_FAMILY_INFINEON_EDGE.
 */
#if defined(CONFIG_SOC_FAMILY_INFINEON_EDGE)
#define PWM_PERI_CLOCK_INIT(n)                                  \
    .clock = {                                                  \
        .block = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(              \
                    DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 0), \
                    DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
                    DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)), \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel), \
    }
#else
#define PWM_PERI_CLOCK_INIT(n)                                  \
    .clock = {                                                  \
        .block = IFX_CAT1_PERIPHERAL_GROUP_ADJUST(              \
                    DT_PROP_BY_IDX(DT_INST_PHANDLE(n, clocks), peri_group, 1), \
                    DT_INST_PROP_BY_PHANDLE(n, clocks, div_type)), \
        .channel = DT_INST_PROP_BY_PHANDLE(n, clocks, channel), \
    }
#endif

/*
 * Per-instance wrapper init for PWM event support.
 *
 * IRQ_CONNECT requires compile-time constant arguments derived from the
 * devicetree instance index (n).  This macro generates a wrapper that
 * calls the main init function first, configures the interrupt.  This avoids
 * duplicating the bulk ofg the init logic for every pwm instance.
 */
#define INFINEON_TCPWM_PWM_IRQ_INIT(n)                          \
    static int ifx_tcpwm_pwm_init_##n(const struct device* dev) { \
        int ret;                                                \
                                                                \
        ret = ifx_tcpwm_pwm_init(dev);                          \
        if (ret < 0) {                                          \
            return (ret);                                       \
        }                                                       \
                                                                \
        IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority), \
                    ifx_tcpwm_pwm_isr, DEVICE_DT_INST_GET(n), 0); \
        irq_enable(DT_IRQN(DT_INST_PARENT(n)));                 \
                                                                \
        return (0);                                             \
    }

/*
 * Optional per-instance capture_input config field, only emitted (and only relevant)
 * when CONFIG_PWM_CAPTURE is enabled - keeps the "capture-input" DT property lookup out
 * of the config struct initializer entirely on builds without PWM capture support.
 */
#ifdef CONFIG_PWM_CAPTURE
#define PWM_CAPTURE_INPUT_INIT(n)                               \
    .capture_input = DT_INST_PROP(n, capture_input),            \
    .trigmux_in    = DT_INST_PROP_OR(n, trigmux_in, 0),         \
    .trigmux_out   = DT_INST_PROP_OR(n, trigmux_out, 0),
#else
#define PWM_CAPTURE_INPUT_INIT(n)
#endif /* CONFIG_PWM_CAPTURE */

/* clang-format off */
/*
 * Per-instance device instantiation macro.
 *
 * Defines pinctrl state, mutable driver data, immutable config, and
 * registers the device.  When CONFIG_PWM_EVENT is enabled, the IRQ
 * wrapper init function is also emitted via INFINEON_TCPWM_PWM_IRQ_INIT.
 *
 * In the DEVICE_DT_INST_DEFINE below, the COND_CODE_1 macro is used to
 * use the wrapper init function when CONFIG_PWM_EVENT is enabled and use
 * the main init function otherwise.
 */
#define INFINEON_TCPWM_PWM_INIT(n)                              \
    IF_ENABLED(IFX_TCPWM_PWM_IRQ_ENABLED, (INFINEON_TCPWM_PWM_IRQ_INIT(n))) \
    PINCTRL_DT_INST_DEFINE(n);                                  \
                                                                \
    PM_DEVICE_DT_INST_DEFINE(n, ifx_tcpwm_pwm_pm_action);       \
                                                                \
    static struct ifx_tcpwm_pwm_data ifx_tcpwm_pwm##n##_data = {PWM_PERI_CLOCK_INIT(n)}; \
                                                                \
    static struct ifx_tcpwm_pwm_config DT_CONST pwm_tcpwm_config_##n = {    \
        .reg_base    = (TCPWM_GRP_CNT_Type*)DT_REG_ADDR(DT_INST_PARENT(n)), \
        .tcpwm_index = (DT_REG_ADDR(DT_INST_PARENT(n)) -                    \
                        DT_REG_ADDR(DT_PARENT(DT_INST_PARENT(n)))) /        \
                        DT_REG_SIZE(DT_INST_PARENT(n)),                     \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                          \
        .resolution_32_bits =                                               \
            (DT_PROP(DT_INST_PARENT(n), resolution) == 32) ? true : false,  \
        .index = (DT_REG_ADDR(DT_INST_PARENT(n)) -                          \
                  DT_REG_ADDR(DT_PARENT(DT_INST_PARENT(n)))) /              \
                  DT_REG_SIZE(DT_INST_PARENT(n)),                           \
        .clk_dst = DT_PROP(DT_INST_PARENT(n), clk_dst),                     \
        PWM_CAPTURE_INPUT_INIT(n)                                           \
    };                                                                      \
                                                                            \
    DEVICE_DT_INST_DEFINE(                                                  \
        n, COND_CODE_1(IFX_TCPWM_PWM_IRQ_ENABLED,                           \
                       (ifx_tcpwm_pwm_init_##n), (ifx_tcpwm_pwm_init)),     \
        PM_DEVICE_DT_INST_GET(n), &ifx_tcpwm_pwm##n##_data, &pwm_tcpwm_config_##n, \
        POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &ifx_tcpwm_pwm_api);
/* clang-format on */

DT_INST_FOREACH_STATUS_OKAY(INFINEON_TCPWM_PWM_INIT)
