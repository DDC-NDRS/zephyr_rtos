/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 STMicroelectronics
 */

#define DT_DRV_COMPAT microchip_lan8742

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microchip_lan8742, CONFIG_PHY_LOG_LEVEL);

#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/util_macro.h>

#include "phy_mii.h"

#define ANY_INT_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)

struct phy_lan8742_dev_config {
    uint8_t phy_addr;
    bool no_reset;
    enum phy_link_speed default_speeds;
    uint16_t led1_func_sel;
    uint16_t led2_func_sel;
    uint8_t mdix_mode;
    bool edpd;
    bool apply_mdix_edpd;
    const struct device* const mdio;
    struct gpio_dt_spec gpio_reset;

    #if ANY_INT_GPIO
    struct gpio_dt_spec gpio_int;           /* zeroed if not in DT */
    #endif /* ANY_INT_GPIO */
    uint32_t reset_assert_us;
    uint32_t reset_deassert_us;
};

struct phy_lan8742_dev_data {
    struct device const* dev;
    phy_callback_t cb;
    void* cb_data;
    struct phy_link_state state;
    struct k_mutex lock;
    struct k_work_delayable monitor_work;
    bool autoneg_in_progress;
    k_timepoint_t autoneg_timeout;

    #if ANY_INT_GPIO
    struct gpio_callback int_cb;
    #endif /* ANY_INT_GPIO */
};

#define MII_INVALID_PHY_ID  UINT32_MAX

/* How often to poll auto-negotiation status while waiting for it to complete */
#define MII_AUTONEG_POLL_INTERVAL_MS        100
/* Retry delay when the monitor finds the PHY busy */
#define PHY_BUSY_RETRY_MS                   10
#define LAN8742_MMDACR_MMD_DEVAD            3

/* Wait up to 0.6 s for the BMCR soft-reset bit to self-clear (IEEE 802.3, 22.2.4.1.1) */
#define LAN8742_RESET_POLL_MS               50
#define LAN8742_RESET_RETRIES               12
#define LAN8742_MMDAADR_WUCSR_INDEX         32784

/* Indirect MMD access registers and WUCSR LED function select fields */
#define LAN8742_REG_MMDACR          13
#define LAN8742_REG_MMDAADR         14
#define LAN8742_MMDACR_FUNC_DATA    BIT(14)
#define LAN8742_WUCSR_LED_MASK      GENMASK(14, 11)

/* Mode Control/Status register */
#define LAN8742_REG_MCSR            17
#define LAN8742_MCSR_EDPWRDOWN      BIT(13)
#define LAN8742_MCSR_ALTINT         BIT(6)

/* Special Modes register */
#define LAN8742_REG_SMR             18
#define LAN8742_SMR_MODE_MASK       GENMASK(7, 5)
#define LAN8742_SMR_MODE_100FD_NOAN 3U      /* MODE[2:0] straps 000..011 disable autoneg */
#define LAN8742_SMR_MODE_PWRDOWN    6U

/* Special Control/Status Indications register */
#define LAN8742_REG_SCSIR           27
#define LAN8742_SCSIR_AMDIXCTRL     BIT(15) /* 1 = auto-MDIX disabled */
#define LAN8742_SCSIR_CH_SELECT     BIT(13) /* 0 = MDI, 1 = MDIX (manual only) */

/* Interrupt Source Flag (read clears) and Interrupt Mask registers */
#define LAN8742_REG_ISR             29
#define LAN8742_REG_IMR             30
#define LAN8742_INT_ANEG_COMPLETE   BIT(6)
#define LAN8742_INT_LINK_DOWN       BIT(4)

/* PHY Special Control/Status register */
#define LAN8742_REG_PSCSR           31
#define LAN8742_PSCSR_AUTODONE      BIT(12)
#define LAN8742_PSCSR_HCDSPEED_MASK GENMASK(4, 2)
#define LAN8742_HCDSPEED_10HD       0x01U   /* 10BASE-T half-duplex */
#define LAN8742_HCDSPEED_10FD       0x05U   /* 10BASE-T full-duplex */
#define LAN8742_HCDSPEED_100HD      0x02U   /* 100BASE-T half-duplex */
#define LAN8742_HCDSPEED_100FD      0x06U   /* 100BASE-T full-duplex */

/* Values of the mdix-mode devicetree property (enum index) */
#define LAN8742_MDIX_AUTO           0
#define LAN8742_MDIX_MDI            1
#define LAN8742_MDIX_MDIX           2

static void phy_lan8742_invoke_link_cb(struct device const* dev);

/* MDIO access helpers, the only place that dereferences the MDIO bus and PHY address */
static int phy_lan8742_rd(struct device const* dev, uint8_t reg, uint16_t* val) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;

    return (mdio_read(cfg->mdio, cfg->phy_addr, reg, val));
}

static int phy_lan8742_wr(struct device const* dev, uint8_t reg, uint16_t val) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;

    return (mdio_write(cfg->mdio, cfg->phy_addr, reg, val));
}

static int phy_lan8742_rd45(struct device const* dev, uint8_t devad, uint16_t reg,
                            uint16_t* val) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;

    return (mdio_read_c45(cfg->mdio, cfg->phy_addr, devad, reg, val));
}

static int phy_lan8742_wr45(struct device const* dev, uint8_t devad, uint16_t reg,
                            uint16_t val) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;

    return (mdio_write_c45(cfg->mdio, cfg->phy_addr, devad, reg, val));
}

static int phy_lan8742_hw_reset(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    int ret;

    ret = gpio_pin_configure_dt(&cfg->gpio_reset, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        LOG_ERR("failed to initialize GPIO for reset");
    }
    else {
        k_busy_wait(cfg->reset_assert_us);

        ret = gpio_pin_set_dt(&cfg->gpio_reset, 0);
        if (ret != 0) {
            LOG_ERR("failed to de-assert reset GPIO");
        }
        else {
            (void) k_usleep(cfg->reset_deassert_us);
        }
    }

    return (ret);
}

static int phy_lan8742_soft_reset(struct device const* dev) {
    uint16_t value;
    int retries;
    int ret;

    ret = phy_lan8742_wr(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        /* Wait up to 0.6s for the reset sequence to finish. According to
         * IEEE 802.3, Section 2, Subsection 22.2.4.1.1 a PHY reset may take
         * up to 0.5 s.
         */
        ret = -ETIMEDOUT;

        for (retries = 0; (retries < LAN8742_RESET_RETRIES) && (ret == -ETIMEDOUT); retries++) {
            (void) k_sleep(K_MSEC(LAN8742_RESET_POLL_MS));

            if (phy_lan8742_rd(dev, MII_BMCR, &value) < 0) {
                ret = -EIO;
            }
            else if ((value & MII_BMCR_RESET) == 0U) {
                ret = 0;
            }
            else {
                /* Reset still in progress, poll again */
            }
        }
    }

    return (ret);
}

static int phy_lan8742_reset(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    bool need_soft_reset = true;
    int ret = 0;

    /* A hardware reset replaces the soft reset */
    if (cfg->gpio_reset.port != NULL) {
        if (gpio_is_ready_dt(&cfg->gpio_reset) == true) {
            ret = phy_lan8742_hw_reset(dev);
            need_soft_reset = false;
        }
        else {
            LOG_WRN("PHY (%d) reset GPIO not ready, using soft reset", cfg->phy_addr);
        }
    }

    if (ret == 0) {
        /* Let the PHY settle after reset before its first MDIO access */
        (void) k_sleep(K_MSEC(25));
    }

    if ((ret == 0) && (need_soft_reset == true)) {
        ret = phy_lan8742_soft_reset(dev);
    }

    return (ret);
}

static int phy_lan8742_get_id(struct device const* dev, uint32_t* phy_id) {
    uint16_t value;
    int ret;

    ret = phy_lan8742_rd(dev, MII_PHYID1R, &value);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        *phy_id = ((uint32_t)value << 16);

        ret = phy_lan8742_rd(dev, MII_PHYID2R, &value);
        if (ret < 0) {
            ret = -EIO;
        }
        else {
            *phy_id |= value;
            ret = 0;
        }
    }

    return (ret);
}

static int phy_lan8742_check_id(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    uint32_t phy_id = 0U;
    int ret = 0;

    /* A failed ID read is not fatal, the PHY might still work */
    if (phy_lan8742_get_id(dev, &phy_id) == 0) {
        if (phy_id == MII_INVALID_PHY_ID) {
            LOG_ERR("No PHY found at address %d", cfg->phy_addr);
            ret = -EINVAL;
        }
        else {
            LOG_INF("PHY (%d) ID 0x%X", cfg->phy_addr, phy_id);
        }
    }

    return (ret);
}

/* Read the speed/duplex resolved by the PHY (register 31, HCDSPEED) */
static int phy_lan8742_get_hcd_speed(struct device const* dev, enum phy_link_speed* speed) {
    uint16_t pscsr;
    int ret;

    ret = phy_lan8742_rd(dev, LAN8742_REG_PSCSR, &pscsr);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((pscsr & LAN8742_PSCSR_AUTODONE) == 0U) {
        ret = -EAGAIN;
    }
    else {
        ret = 0;

        switch (FIELD_GET(LAN8742_PSCSR_HCDSPEED_MASK, pscsr)) {
            case LAN8742_HCDSPEED_100FD :
                *speed = LINK_FULL_100BASE;
                break;

            case LAN8742_HCDSPEED_100HD :
                *speed = LINK_HALF_100BASE;
                break;

            case LAN8742_HCDSPEED_10FD :
                *speed = LINK_FULL_10BASE;
                break;

            case LAN8742_HCDSPEED_10HD :
                *speed = LINK_HALF_10BASE;
                break;

            default :
                ret = -EINVAL;
                break;
        }
    }

    return (ret);
}

/* Resolve the link speed, preferring the PHY result (it also covers parallel detection) */
static int phy_lan8742_resolve_speed(struct device const* dev, enum phy_link_speed* speed) {
    uint16_t anar_reg = 0U;
    uint16_t anlpar_reg = 0U;
    uint16_t common;
    int ret;

    ret = phy_lan8742_get_hcd_speed(dev, speed);
    if (ret != 0) {
        /* Read PHY default advertising parameters and the peer device capability */
        ret = phy_lan8742_rd(dev, MII_ANAR, &anar_reg);
        if (ret == 0) {
            ret = phy_lan8742_rd(dev, MII_ANLPAR, &anlpar_reg);
        }

        if (ret != 0) {
            ret = -EIO;
        }
        else {
            common = (anar_reg & anlpar_reg);

            if ((common & MII_ADVERTISE_100_FULL) != 0U) {
                *speed = LINK_FULL_100BASE;
            }
            else if ((common & MII_ADVERTISE_100_HALF) != 0U) {
                *speed = LINK_HALF_100BASE;
            }
            else if ((common & MII_ADVERTISE_10_FULL) != 0U) {
                *speed = LINK_FULL_10BASE;
            }
            else {
                *speed = LINK_HALF_10BASE;
            }
        }
    }

    return (ret);
}

static int phy_lan8742_update_link_state(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    struct phy_link_state* const state = &ctx->state;
    enum phy_link_speed new_speed;
    uint16_t bmcr_reg;
    uint16_t bmsr_reg;
    int ret;

    ret = phy_lan8742_rd(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U) {
        /* If link is down, we can stop here. */
        state->speed = 0;

        if (state->is_up == true) {
            state->is_up = false;
            LOG_INF("PHY (%d) is down", cfg->phy_addr);
            ret = 0;
        }
        else {
            ret = -EAGAIN;
        }
    }
    else {
        ret = phy_lan8742_rd(dev, MII_BMCR, &bmcr_reg);

        if (ret < 0) {
            ret = -EIO;
        }
        else if ((bmcr_reg & MII_BMCR_AUTONEG_ENABLE) == 0U) {
            /* If auto-negotiation is not enabled, we only need to check the link speed */
            new_speed = phy_mii_get_link_speed_bmcr_reg(dev, bmcr_reg);

            if ((state->speed != new_speed) || (state->is_up == false)) {
                state->is_up = true;
                state->speed = new_speed;

                LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                        PHY_LINK_IS_SPEED_100M(state->speed)  ? "100"  : "10",
                        PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");

                ret = 0;
            }
            else {
                ret = -EAGAIN;
            }
        }
        else if (state->is_up == true) {
            /* Auto-negotiation enabled and the link was already up last time we checked:
             * the link state has not changed.
             */
            ret = -EAGAIN;
        }
        else {
            /* The link was down, start the auto-negotiation sequence. */
            state->is_up = true;

            LOG_DBG("PHY (%d) Starting MII PHY auto-negotiate sequence", cfg->phy_addr);

            ctx->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));

            ret = -EINPROGRESS;
        }
    }

    return (ret);
}

/* Auto-negotiation did not complete: drop the half-started link so that the next poll retries
 * instead of waiting for a link-down event that never comes. Returns 0 when a link that was
 * usable is now down (state changed), @p err otherwise.
 */
static int phy_lan8742_autoneg_failed(struct device const* dev, int err) {
    struct phy_lan8742_dev_data* const ctx = dev->data;
    struct phy_link_state* const state = &ctx->state;
    bool was_usable = ((state->is_up == true) && (state->speed != 0));

    state->is_up = false;
    state->speed = 0;

    return ((was_usable == true) && (err != -EIO)) ? 0 : err;
}

static int phy_lan8742_autoneg_poll(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    struct phy_link_state* const state = &ctx->state;
    enum phy_link_speed speed;
    uint16_t bmsr_reg;
    int ret;

    /* On some PHY chips, the BMSR bits are latched, so the first read may show incorrect
     * status. A second read clears the latched bits and gives the correct status.
     */
    ret = phy_lan8742_rd(dev, MII_BMSR, &bmsr_reg);
    if (ret == 0) {
        ret = phy_lan8742_rd(dev, MII_BMSR, &bmsr_reg);
    }

    if (ret != 0) {
        ret = -EIO;
    }
    else if ((bmsr_reg & MII_BMSR_AUTONEG_COMPLETE) == 0U) {
        if (sys_timepoint_expired(ctx->autoneg_timeout) == true) {
            LOG_DBG("PHY (%d) auto-negotiate timeout", cfg->phy_addr);
            ret = -ETIMEDOUT;
        }
        else {
            ret = -EINPROGRESS;
        }
    }
    else {
        LOG_DBG("PHY (%d) auto-negotiate sequence completed", cfg->phy_addr);

        ret = phy_lan8742_resolve_speed(dev, &speed);
        if (ret == 0) {
            state->speed = speed;
            state->is_up = ((bmsr_reg & MII_BMSR_LINK_STATUS) != 0U);

            LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                    PHY_LINK_IS_SPEED_100M(state->speed)  ? "100"  : "10",
                    PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
        }
    }

    return (ret);
}

static int phy_lan8742_check_autoneg_completion(struct device const* dev) {
    int ret;

    ret = phy_lan8742_autoneg_poll(dev);
    if ((ret != 0) && (ret != -EINPROGRESS)) {
        ret = phy_lan8742_autoneg_failed(dev, ret);
    }

    return (ret);
}

static void phy_lan8742_monitor_work(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct phy_lan8742_dev_data* const ctx =
        CONTAINER_OF(dwork, struct phy_lan8742_dev_data, monitor_work);
    struct device const* dev = ctx->dev;
    k_timeout_t delay;
    int rc;

    if (k_mutex_lock(&ctx->lock, K_NO_WAIT) != 0) {
        /* Busy (cfg_link or get_link running): retry soon so that a pending interrupt,
         * which would not produce another edge, is not left to the slow poll.
         */
        delay = K_MSEC(PHY_BUSY_RETRY_MS);
    }
    else {
        #if ANY_INT_GPIO
        struct phy_lan8742_dev_config const* const cfg = dev->config;
        uint16_t isr;

        if (cfg->gpio_int.port != NULL) {
            /* Reading the Interrupt Source Flag register de-asserts nINT */
            (void) phy_lan8742_rd(dev, LAN8742_REG_ISR, &isr);
        }
        #endif /* ANY_INT_GPIO */

        if (ctx->autoneg_in_progress) {
            rc = phy_lan8742_check_autoneg_completion(dev);
        }
        else {
            /* If autonegotiation is not in progress, just update the link state */
            rc = phy_lan8742_update_link_state(dev);
        }

        ctx->autoneg_in_progress = (rc == -EINPROGRESS);

        k_mutex_unlock(&ctx->lock);

        /* If link state has changed and a callback is set, invoke callback */
        if (rc == 0) {
            phy_lan8742_invoke_link_cb(dev);
        }

        delay = (ctx->autoneg_in_progress == true)
                ? K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS)
                : K_MSEC(CONFIG_PHY_MONITOR_PERIOD);
    }

    (void) k_work_reschedule(&ctx->monitor_work, delay);
}

static int phy_lan8742_read(struct device const* dev, uint16_t reg_addr, uint32_t* data) {
    struct phy_lan8742_dev_data* const ctx = dev->data;
    uint16_t val;
    int ret;

    (void) k_mutex_lock(&ctx->lock, K_FOREVER);

    ret = phy_lan8742_rd(dev, (uint8_t)reg_addr, &val);
    if (ret == 0) {
        *data = val;
    }

    k_mutex_unlock(&ctx->lock);

    return (ret);
}

static int phy_lan8742_write(struct device const* dev, uint16_t reg_addr, uint32_t data) {
    struct phy_lan8742_dev_data* const ctx = dev->data;
    int ret;

    (void) k_mutex_lock(&ctx->lock, K_FOREVER);

    ret = phy_lan8742_wr(dev, (uint8_t)reg_addr, (uint16_t)data);

    k_mutex_unlock(&ctx->lock);

    return (ret);
}

static int phy_lan8742_cfg_link(struct device const* dev, enum phy_link_speed adv_speeds,
                                enum phy_cfg_link_flag flags) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    int ret;

    if (cfg->mdio == NULL) {
        ret = -ENODEV;
    }
    else {
        (void) k_mutex_lock(&ctx->lock, K_FOREVER);

        if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
            /* If auto-negotiation is disabled, only one speed can be selected.
             * If gigabit is not supported, this speed must not be 1000M.
             */
            ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
            if (ret >= 0) {
                ctx->autoneg_in_progress = false;
                (void) k_work_reschedule(&ctx->monitor_work, K_NO_WAIT);
            }
        }
        else {
            ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, false);
            if (ret >= 0) {
                LOG_DBG("PHY (%d) Starting MII PHY auto-negotiate sequence", cfg->phy_addr);
                ctx->autoneg_in_progress = true;
                ctx->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
                (void) k_work_reschedule(&ctx->monitor_work,
                                         K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
            }
        }

        if (ret == -EALREADY) {
            LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
        }

        k_mutex_unlock(&ctx->lock);
    }

    return (ret);
}

static int phy_lan8742_get_link_state(struct device const* dev, struct phy_link_state* state) {
    struct phy_lan8742_dev_data* const ctx = dev->data;

    (void) k_mutex_lock(&ctx->lock, K_FOREVER);

    (void) memcpy(state, &ctx->state, sizeof(struct phy_link_state));

    if (state->speed == 0) {
        /* If speed is 0, then link is also down, happens when autonegotiation is in
         * progress
         */
        state->is_up = false;
    }

    k_mutex_unlock(&ctx->lock);

    return (0);
}

static void phy_lan8742_invoke_link_cb(struct device const* dev) {
    struct phy_lan8742_dev_data* const ctx = dev->data;
    struct phy_link_state state;

    if (ctx->cb != NULL) {
        (void) phy_lan8742_get_link_state(dev, &state);

        ctx->cb(dev, &state, ctx->cb_data);
    }
}

static int phy_lan8742_link_cb_set(struct device const* dev, phy_callback_t cb, void* user_data) {
    struct phy_lan8742_dev_data* const ctx = dev->data;

    ctx->cb = cb;
    ctx->cb_data = user_data;

    /**
     * Immediately invoke the callback to notify the caller of the
     * current link status.
     */
    phy_lan8742_invoke_link_cb(dev);

    return (0);
}

static DEVICE_API(ethphy, phy_lan8742_driver_api) = {
    .get_link    = phy_lan8742_get_link_state,
    .link_cb_set = phy_lan8742_link_cb_set,
    .cfg_link    = phy_lan8742_cfg_link,
    .read        = phy_lan8742_read,
    .write       = phy_lan8742_write,
};

static int phy_lan8742_update_bits(struct device const* dev, uint8_t reg,
                                   uint16_t mask, uint16_t val) {
    uint16_t reg_val;
    int ret;

    ret = phy_lan8742_rd(dev, reg, &reg_val);
    if (ret == 0) {
        reg_val = (reg_val & ~mask) | (val & mask);

        ret = phy_lan8742_wr(dev, reg, reg_val);
    }

    return (ret);
}

/* Indirect MMD register update through the MMD Access Control and Address/Data
 * registers (13 and 14).
 */
static int phy_lan8742_mmd_update_bits_indirect(struct device const* dev, uint8_t devad,
                                                uint16_t addr, uint16_t mask, uint16_t val) {
    uint16_t reg_val;
    int ret;

    ret = phy_lan8742_wr(dev, LAN8742_REG_MMDACR, devad);
    if (ret == 0) {
        ret = phy_lan8742_wr(dev, LAN8742_REG_MMDAADR, addr);
    }

    if (ret == 0) {
        ret = phy_lan8742_wr(dev, LAN8742_REG_MMDACR,
                             LAN8742_MMDACR_FUNC_DATA | devad);
    }

    if (ret == 0) {
        ret = phy_lan8742_rd(dev, LAN8742_REG_MMDAADR, &reg_val);
    }

    if (ret == 0) {
        reg_val = (reg_val & ~mask) | (val & mask);
        ret = phy_lan8742_wr(dev, LAN8742_REG_MMDAADR, reg_val);
    }

    return (ret);
}

/* MMD register update: Clause 45 access if the MDIO driver has it, else indirect access */
static int phy_lan8742_mmd_update_bits(struct device const* dev, uint8_t devad, uint16_t addr,
                                       uint16_t mask, uint16_t val) {
    uint16_t reg_val;
    int ret;

    ret = phy_lan8742_rd45(dev, devad, addr, &reg_val);
    if (ret == 0) {
        reg_val = (reg_val & ~mask) | (val & mask);

        ret = phy_lan8742_wr45(dev, devad, addr, reg_val);
    }

    /* Either access may be missing on the controller: fall back to indirect access */
    if (ret == -ENOSYS) {
        ret = phy_lan8742_mmd_update_bits_indirect(dev, devad, addr, mask, val);
    }
    else {
        /* Clause 45 access succeeded, or failed with another error: report the result */
    }

    return (ret);
}

/* Check the MODE[2:0] straps, which set the register defaults after reset */
static int phy_lan8742_check_straps(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    uint16_t smr;
    uint32_t mode;
    int ret;

    ret = phy_lan8742_rd(dev, LAN8742_REG_SMR, &smr);
    if (ret == 0) {
        mode = FIELD_GET(LAN8742_SMR_MODE_MASK, smr);

        if (mode == LAN8742_SMR_MODE_PWRDOWN) {
            LOG_ERR("PHY (%d) MODE straps select power-down mode", cfg->phy_addr);
            ret = -ENODEV;
        }
        else if (mode <= LAN8742_SMR_MODE_100FD_NOAN) {
            LOG_WRN("PHY (%d) MODE straps (%u) disable auto-negotiation, the driver "
                    "re-enables it", cfg->phy_addr, mode);
        }
        else {
            /* Auto-negotiation enabled by the straps */
        }
    }

    return (ret);
}

static int phy_lan8742_cfg_mdix_edpd(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    uint16_t val = 0U;
    int ret;

    /* Only when the devicetree sets mdix-mode or energy-detect-power-down: otherwise the
     * reset or bootloader configuration is kept, as in the baseline driver.
     */
    if (cfg->apply_mdix_edpd == false) {
        return (0);
    }

    if (cfg->mdix_mode != LAN8742_MDIX_AUTO) {
        val = LAN8742_SCSIR_AMDIXCTRL;
        if (cfg->mdix_mode == LAN8742_MDIX_MDIX) {
            val |= LAN8742_SCSIR_CH_SELECT;
        }
    }

    ret = phy_lan8742_update_bits(dev, LAN8742_REG_SCSIR,
                                  LAN8742_SCSIR_AMDIXCTRL | LAN8742_SCSIR_CH_SELECT, val);
    if (ret == 0) {
        ret = phy_lan8742_update_bits(dev, LAN8742_REG_MCSR, LAN8742_MCSR_EDPWRDOWN,
                                      cfg->edpd ? LAN8742_MCSR_EDPWRDOWN : 0U);
    }

    return (ret);
}

/* Only touch the LED function fields, WUCSR also holds Wake-on-LAN and interface bits */
static int phy_lan8742_cfg_led(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    uint16_t wucsr;
    int ret;

    wucsr = (uint16_t)((cfg->led1_func_sel << 13) | (cfg->led2_func_sel << 11));

    /* The indirect MMD access is a multi-step sequence, keep other users off the bus */
    (void) k_mutex_lock(&ctx->lock, K_FOREVER);

    ret = phy_lan8742_mmd_update_bits(dev, LAN8742_MMDACR_MMD_DEVAD,
                                      LAN8742_MMDAADR_WUCSR_INDEX,
                                      LAN8742_WUCSR_LED_MASK, wucsr);

    k_mutex_unlock(&ctx->lock);

    return (ret);
}

#if ANY_INT_GPIO
static void phy_lan8742_int_isr(const struct device* port, struct gpio_callback* cb,
                                uint32_t pins) {
    struct phy_lan8742_dev_data* const ctx = CONTAINER_OF(cb, struct phy_lan8742_dev_data,
                                                          int_cb);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    /* Re-evaluate the link now instead of waiting for the next poll */
    (void) k_work_reschedule(&ctx->monitor_work, K_NO_WAIT);
}

static int phy_lan8742_cfg_int(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    uint16_t isr;
    int ret;

    if (cfg->gpio_int.port == NULL) {
        ret = 0;
    }
    else if (gpio_is_ready_dt(&cfg->gpio_int) == false) {
        LOG_ERR("PHY (%d) interrupt GPIO not ready", cfg->phy_addr);
        ret = -ENODEV;
    }
    else {
        ret = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);

        if (ret == 0) {
            gpio_init_callback(&ctx->int_cb, phy_lan8742_int_isr, BIT(cfg->gpio_int.pin));

            ret = gpio_add_callback_dt(&cfg->gpio_int, &ctx->int_cb);
        }

        if (ret == 0) {
            /* Primary interrupt system (ALTINT = 0): auto-negotiation complete and link
             * down. There is no link-up event in forced mode, so the periodic poll stays
             * active.
             */
            ret = phy_lan8742_update_bits(dev, LAN8742_REG_MCSR, LAN8742_MCSR_ALTINT, 0U);
        }

        if (ret == 0) {
            ret = phy_lan8742_wr(dev, LAN8742_REG_IMR,
                                 LAN8742_INT_ANEG_COMPLETE | LAN8742_INT_LINK_DOWN);
        }

        if (ret == 0) {
            /* Clear stale flags, then enable the pin interrupt (nINT is active low) */
            (void) phy_lan8742_rd(dev, LAN8742_REG_ISR, &isr);

            ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
        }
    }

    return (ret);
}
#endif /* ANY_INT_GPIO */

static int phy_lan8742_init(struct device const* dev) {
    struct phy_lan8742_dev_config const* const cfg = dev->config;
    struct phy_lan8742_dev_data* const ctx = dev->data;
    int link_ret;
    int ret = 0;

    ctx->state.is_up = false;
    k_work_init_delayable(&ctx->monitor_work, phy_lan8742_monitor_work);

    if (device_is_ready(cfg->mdio) == false) {
        ret = -ENODEV;
    }

    if ((ret == 0) && (cfg->no_reset == false)) {
        ret = phy_lan8742_reset(dev);
        if (ret < 0) {
            LOG_ERR("Failed to reset PHY (%d): %d", cfg->phy_addr, ret);
        }
    }

    if (ret == 0) {
        ret = phy_lan8742_check_id(dev);
    }

    if (ret == 0) {
        ret = phy_lan8742_check_straps(dev);
    }

    if (ret == 0) {
        ret = phy_lan8742_cfg_led(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure LED functions (%d)", ret);
        }
    }

    if (ret == 0) {
        ret = phy_lan8742_cfg_mdix_edpd(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure auto-MDIX/EDPD (%d)", ret);
        }
    }

    #if ANY_INT_GPIO
    if (ret == 0) {
        ret = phy_lan8742_cfg_int(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt (%d)", ret);
        }
    }
    #endif /* ANY_INT_GPIO */

    if (ret == 0) {
        /* Advertise default speeds */
        link_ret = phy_lan8742_cfg_link(dev, cfg->default_speeds, 0);
        if (link_ret == -EALREADY) {
            ctx->autoneg_in_progress = true;
            ctx->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
        }
        else if (link_ret < 0) {
            LOG_ERR("PHY (%d) failed to configure link (%d)", cfg->phy_addr, link_ret);
            ret = link_ret;
        }
        else {
            /* Auto-negotiation restarted by phy_lan8742_cfg_link() */
        }
    }

    if (ret == 0) {
        /* Schedule the monitor work, if not already scheduled by phy_lan8742_cfg_link(). */
        (void) k_work_schedule(&ctx->monitor_work, K_NO_WAIT);
    }

    return (ret);
}

#if ANY_INT_GPIO
#define LAN8742_INT_GPIO(n)                                     \
    .gpio_int = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define LAN8742_INT_GPIO(n)
#endif /* ANY_INT_GPIO */

#define PHY_LAN8742_CONFIG(n)                                   \
    BUILD_ASSERT((PHY_INST_GENERATE_DEFAULT_SPEEDS(n) != 0),    \
                 "At least one valid speed must be configured for this driver"); \
                                                                \
    static const struct phy_lan8742_dev_config phy_lan8742_dev_config_##n = { \
        .phy_addr       = DT_INST_REG_ADDR(n),                  \
        .no_reset       = DT_INST_PROP(n, no_reset),            \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),  \
        .led1_func_sel  = DT_ENUM_IDX(DT_DRV_INST(n), led1_function_select), \
        .led2_func_sel  = DT_ENUM_IDX(DT_DRV_INST(n), led2_function_select), \
        .mdix_mode      = DT_INST_ENUM_IDX(n, mdix_mode),       \
        .edpd           = DT_INST_PROP(n, energy_detect_power_down), \
        .apply_mdix_edpd = (DT_INST_NODE_HAS_PROP(n, mdix_mode) || \
                            DT_INST_NODE_HAS_PROP(n, energy_detect_power_down)), \
        LAN8742_INT_GPIO(n)                                     \
        .mdio           = DEVICE_DT_GET(DT_INST_PARENT(n)),     \
        .gpio_reset     = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}), \
        .reset_assert_us   = DT_INST_PROP_OR(n, reset_assert_us, 1000), \
        .reset_deassert_us = DT_INST_PROP_OR(n, reset_deassert_us, 0)   \
    }

#define PHY_LAN8742_DATA(n)                                     \
    static struct phy_lan8742_dev_data phy_lan8742_dev_data_##n = { \
        .dev = DEVICE_DT_INST_GET(n),                           \
        .lock = Z_MUTEX_INITIALIZER(phy_lan8742_dev_data_##n.lock), \
    }

#define PHY_LAN8742_DEVICE(n)                                   \
    PHY_LAN8742_CONFIG(n);                                      \
    PHY_LAN8742_DATA(n);                                        \
    DEVICE_DT_INST_DEFINE(n, phy_lan8742_init, NULL, &phy_lan8742_dev_data_##n, \
                          &phy_lan8742_dev_config_##n, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, \
                          &phy_lan8742_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PHY_LAN8742_DEVICE)
