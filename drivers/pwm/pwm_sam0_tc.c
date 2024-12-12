/*
 * Copyright (c) 2020 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PWM driver using the SAM0 Timer/Counter for Control Applications (TCC) in Normal PWM (NPWM) mode.
 * Supports the SAMD21 and SAMD5x series.
 */

#define DT_DRV_COMPAT atmel_sam0_tc_pwm

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

/* Static configuration */
struct pwm_sam0_tc_config {
    Tc* regs;
    const struct pinctrl_dev_config* pcfg;
	uint8_t channels;
	uint8_t counter_size;
	uint16_t prescaler;
	uint32_t freq;

#ifdef MCLK
    volatile uint32_t* mclk;
	uint32_t mclk_mask;
	uint16_t gclk_id;
#else
	uint32_t pm_apbcmask;
	uint16_t gclk_clkctrl_id;
#endif
};

/* Wait for the peripheral to finish all commands */
static void wait_synchronization(Tc* regs) {
    while (regs->COUNT8.SYNCBUSY.reg != 0) {
        /* pass */
    }
}

static int pwm_sam0_tc_get_cycles_per_sec(const struct device* dev, uint32_t channel, uint64_t* cycles) {
    const struct pwm_sam0_tc_config* const cfg = dev->config;

    if (channel >= cfg->channels) {
        return (-EINVAL);
    }

    *cycles = cfg->freq;

    return (0);
}

static int pwm_sam0_tc_set_cycles(const struct device* dev, uint32_t channel,
                                  uint32_t period_cycles, uint32_t pulse_cycles,
                                  pwm_flags_t flags) {
    const struct pwm_sam0_tc_config* const cfg = dev->config;
    Tc* regs = cfg->regs;
	uint32_t top = 1 << cfg->counter_size;
	uint32_t invert_mask = 1 << channel;
	bool invert = ((flags & PWM_POLARITY_INVERTED) != 0);
	bool inverted = ((regs->COUNT8.DRVCTRL.vec.INVEN & invert_mask) != 0);

	if (channel >= cfg->channels) {
		return -EINVAL;
	}

	if (period_cycles >= top || pulse_cycles >= top) {
		return -EINVAL;
	}

	/*
	 * Update the buffered width and period.  These will be automatically
	 * loaded on the next cycle.
	 */
	regs->COUNT8.CCBUF[channel].reg = TCC_CCBUF_CCBUF(pulse_cycles);
	regs->COUNT8.PERBUF.reg         = TCC_PERBUF_PERBUF(period_cycles);

	if (invert != inverted) {
		regs->COUNT8.CTRLA.bit.ENABLE = 0;
		wait_synchronization(regs);

		regs->COUNT8.DRVCTRL.vec.INVEN ^= invert_mask;
		regs->COUNT8.CTRLA.bit.ENABLE   = 1;
		wait_synchronization(regs);
	}

	return 0;
}

static int pwm_sam0_tc_init(const struct device* dev) {
    const struct pwm_sam0_tc_config* const cfg = dev->config;
    Tc* regs = cfg->regs;
    int retval;

	/* Enable the clocks */
#ifdef MCLK
    GCLK->PCHCTRL[cfg->gclk_id].reg = (GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN);
    *cfg->mclk |= cfg->mclk_mask;
#else
	GCLK->CLKCTRL.reg = cfg->gclk_clkctrl_id | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	PM->APBCMASK.reg |= cfg->pm_apbcmask;
#endif

	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	regs->COUNT8.CTRLA.bit.SWRST = 1;
	wait_synchronization(regs);

	regs->COUNT8.CTRLA.reg = (cfg->prescaler | TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT8_Val));
	regs->COUNT8.WAVE.reg  = TCC_WAVE_WAVEGEN_NPWM;
	regs->COUNT8.PER.reg   = TCC_PER_PER(1);

	regs->COUNT8.CTRLA.bit.ENABLE = 1;
	wait_synchronization(regs);

	return 0;
}

static const struct pwm_driver_api pwm_sam0_tc_driver_api = {
	.set_cycles = pwm_sam0_tc_set_cycles,
	.get_cycles_per_sec = pwm_sam0_tc_get_cycles_per_sec,
};

#ifdef MCLK
#define PWM_SAM0_TC_INIT_CLOCKS(inst)					       \
	.mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),	       \
	.mclk_mask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),	       \
	.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch)
#else
#define PWM_SAM0_TC_INIT_CLOCKS(inst)					       \
	.pm_apbcmask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, pm, bit)),	       \
	.gclk_clkctrl_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, clkctrl_id)
#endif

#define PWM_SAM0_TC_INIT(inst)						       \
	PINCTRL_DT_INST_DEFINE(inst);					       \
	static const struct pwm_sam0_tc_config pwm_sam0_tc_config_##inst = {	       \
		.regs = (Tc *)DT_INST_REG_ADDR(inst),			       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		       \
		.channels = DT_INST_PROP(inst, channels),		       \
		.counter_size = DT_INST_PROP(inst, counter_size),	       \
		.prescaler = UTIL_CAT(TCC_CTRLA_PRESCALER_DIV, DT_INST_PROP(inst, prescaler)),	       \
		.freq = SOC_ATMEL_SAM0_GCLK1_FREQ_HZ / DT_INST_PROP(inst, prescaler),       \
		PWM_SAM0_TC_INIT_CLOCKS(inst),				       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(inst, &pwm_sam0_tc_init, NULL,		       \
			    NULL, &pwm_sam0_tc_config_##inst,		       \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,   \
			    &pwm_sam0_tc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SAM0_TC_INIT)