/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_lan8710ai

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_lan8710ai, CONFIG_PHY_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/gpio.h>

#include "phy_mii.h"

#define LAN8710_PHY_ID1      0x0007U
#define LAN8710_PHY_ID2      0xC0F0U
#define LAN8710_PHY_ID2_MASK 0xFFF0U

#define MII_INVALID_PHY_ID   UINT32_MAX

/* Mode Control/Status register */
#define LAN8710_REG_MCSR            17
#define LAN8710_MCSR_EDPWRDOWN      BIT(13)
#define LAN8710_MCSR_ALTINT         BIT(6)

/* Special Modes register */
#define LAN8710_REG_SMR             18
#define LAN8710_SMR_MODE_MASK       GENMASK(7, 5)
#define LAN8710_SMR_MODE_100FD_NOAN 3U      /* MODE[2:0] straps 000..011 disable autoneg */
#define LAN8710_SMR_MODE_PWRDOWN    6U

/* Special Control/Status Indications register */
#define LAN8710_REG_SCSIR           27
#define LAN8710_SCSIR_AMDIXCTRL     BIT(15) /* 1 = auto-MDIX disabled */
#define LAN8710_SCSIR_CH_SELECT     BIT(13) /* 0 = MDI, 1 = MDIX (manual only) */

/* Interrupt Source Flag (read clears) and Interrupt Mask registers */
#define LAN8710_REG_ISR             29
#define LAN8710_REG_IMR             30
#define LAN8710_INT_ANEG_COMPLETE   BIT(6)
#define LAN8710_INT_LINK_DOWN       BIT(4)

/* PHY Special Control/Status register */
#define LAN8710_REG_PSCSR           31
#define LAN8710_PSCSR_AUTODONE      BIT(12)
#define LAN8710_PSCSR_HCDSPEED_MASK GENMASK(4, 2)
#define LAN8710_HCDSPEED_10HD       0x01U   /* 10BASE-T half-duplex */
#define LAN8710_HCDSPEED_10FD       0x05U   /* 10BASE-T full-duplex */
#define LAN8710_HCDSPEED_100HD      0x02U   /* 100BASE-T half-duplex */
#define LAN8710_HCDSPEED_100FD      0x06U   /* 100BASE-T full-duplex */

/* Values of the mdix-mode devicetree property (enum index) */
#define LAN8710_MDIX_AUTO           0
#define LAN8710_MDIX_MDI            1
#define LAN8710_MDIX_MDIX           2

/* Wait up to 0.6 s for the BMCR soft-reset bit to self-clear (IEEE 802.3, 22.2.4.1.1) */
#define LAN8710_RESET_POLL_MS       10
#define LAN8710_RESET_RETRIES       60

/* How often to poll auto-negotiation status while waiting for it to complete */
#define MII_AUTONEG_POLL_INTERVAL_MS 100
/* Retry delay when the monitor finds the PHY busy */
#define PHY_BUSY_RETRY_MS            10

#define ANY_RESET_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define ANY_INT_GPIO   DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)

struct phy_lan8710ai_config {
    const struct device* mdio_dev;
    uint8_t phy_addr;
    bool no_reset;
    enum phy_link_speed default_speeds;
    uint8_t mdix_mode;
    bool edpd;
    bool apply_mdix_edpd;

    #if ANY_INT_GPIO
    struct gpio_dt_spec int_gpio;           /* zeroed if not in DT */
    #endif /* ANY_INT_GPIO */

    #if ANY_RESET_GPIO
    struct gpio_dt_spec reset_gpio;         /* zeroed if not in DT */
    uint32_t reset_assert_us;
    uint32_t reset_deassert_us;
    #endif /* ANY_RESET_GPIO */
};

struct phy_lan8710ai_data {
    struct device const* dev;               /* self-pointer for work handler */
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

static void phy_lan8710ai_invoke_link_cb(struct device const* dev);
static int  phy_lan8710ai_check_autoneg_completion(struct device const* dev);

/* -------------------------------------------------------------------
 * Low-level MDIO read/write helpers
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_read(struct device const* dev, uint16_t reg_addr, uint32_t* data) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    uint16_t val;
    int ret;

    ret = mdio_read(cfg->mdio_dev, cfg->phy_addr, (uint8_t)reg_addr, &val);
    if (ret == 0) {
        *data = val;
    }

    return (ret);
}

static int phy_lan8710ai_write(struct device const* dev, uint16_t reg_addr, uint32_t data) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    int ret;

    ret = mdio_write(cfg->mdio_dev, cfg->phy_addr, (uint8_t)reg_addr, (uint16_t)data);

    return (ret);
}

/* -------------------------------------------------------------------
 * PHY reset: hard reset through GPIO if wired, else BMCR soft-reset
 * ------------------------------------------------------------------- */
#if ANY_RESET_GPIO
static int phy_lan8710ai_hw_reset(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    int ret;

    ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure RST pin (%d)", ret);
    }
    else {
        k_busy_wait(cfg->reset_assert_us);

        ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to de-assert RST pin (%d)", ret);
        }
        else {
            (void)k_usleep(cfg->reset_deassert_us);
        }
    }

    return (ret);
}
#endif /* ANY_RESET_GPIO */

static int phy_lan8710ai_soft_reset(struct device const* dev) {
    uint32_t bmcr;
    int retries;
    int ret;

    ret = phy_lan8710ai_write(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        ret = -ETIMEDOUT;

        for (retries = 0; (retries < LAN8710_RESET_RETRIES) && (ret == -ETIMEDOUT); retries++) {
            k_msleep(LAN8710_RESET_POLL_MS);

            if (phy_lan8710ai_read(dev, MII_BMCR, &bmcr) < 0) {
                ret = -EIO;
            }
            else if ((bmcr & MII_BMCR_RESET) == 0U) {
                ret = 0;
            }
            else {
                /* Reset still in progress, poll again */
            }
        }
    }

    return (ret);
}

static int phy_lan8710ai_reset(struct device const* dev) {
    int ret = 0;
    bool need_soft_reset = true;

    #if ANY_RESET_GPIO
    const struct phy_lan8710ai_config* cfg = dev->config;

    /* A hardware reset replaces the soft reset */
    if (cfg->reset_gpio.port != NULL) {
        if (gpio_is_ready_dt(&cfg->reset_gpio) == true) {
            ret = phy_lan8710ai_hw_reset(dev);
            need_soft_reset = false;
        }
        else {
            LOG_WRN("PHY (%d) reset GPIO not ready, using soft reset", cfg->phy_addr);
        }
    }
    #endif /* ANY_RESET_GPIO */

    if ((ret == 0) && (need_soft_reset == true)) {
        ret = phy_lan8710ai_soft_reset(dev);
    }

    return (ret);
}

static int phy_lan8710ai_get_id(struct device const* dev, uint32_t* id1, uint32_t* id2) {
    int ret;

    ret = phy_lan8710ai_read(dev, MII_PHYID1R, id1);
    if (ret == 0) {
        ret = phy_lan8710ai_read(dev, MII_PHYID2R, id2);
    }

    if (ret != 0) {
        ret = -EIO;
    }

    return (ret);
}

static int phy_lan8710ai_verify_id(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    uint32_t id1 = 0U;
    uint32_t id2 = 0U;
    int ret;

    ret = phy_lan8710ai_get_id(dev, &id1, &id2);
    if (ret < 0) {
        LOG_ERR("Failed to read PHY (%d) ID: %d", cfg->phy_addr, ret);
    }
    else if (((id1 << 16) | id2) == MII_INVALID_PHY_ID) {
        LOG_ERR("No PHY found at address %d", cfg->phy_addr);
        ret = -EINVAL;
    }
    else if ((id1 != LAN8710_PHY_ID1) || ((id2 & LAN8710_PHY_ID2_MASK) != LAN8710_PHY_ID2)) {
        LOG_ERR("PHY (%d) unexpected ID 0x%04x:0x%04x", cfg->phy_addr, id1, id2);
        ret = -ENODEV;
    }
    else {
        LOG_INF("PHY (%d) ID 0x%04x:0x%04x", cfg->phy_addr, id1, id2);
    }

    return (ret);
}

/* -------------------------------------------------------------------
 * Read the speed/duplex resolved by the PHY (register 31, HCDSPEED).
 * Returns 0 on success, negative if autonegotiation is not done or the
 * encoding is not a valid 10/100 value.
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_get_hcd_speed(struct device const* dev, enum phy_link_speed* speed) {
    uint32_t pscsr;
    int ret;

    ret = phy_lan8710ai_read(dev, LAN8710_REG_PSCSR, &pscsr);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((pscsr & LAN8710_PSCSR_AUTODONE) == 0U) {
        ret = -EAGAIN;
    }
    else {
        ret = 0;

        switch (FIELD_GET(LAN8710_PSCSR_HCDSPEED_MASK, pscsr)) {
            case LAN8710_HCDSPEED_100FD :
                *speed = LINK_FULL_100BASE;
                break;

            case LAN8710_HCDSPEED_100HD :
                *speed = LINK_HALF_100BASE;
                break;

            case LAN8710_HCDSPEED_10FD :
                *speed = LINK_FULL_10BASE;
                break;

            case LAN8710_HCDSPEED_10HD :
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
static int phy_lan8710ai_resolve_speed(struct device const* dev, enum phy_link_speed* speed) {
    uint32_t anar_reg = 0U;
    uint32_t anlpar_reg = 0U;
    uint32_t common;
    int ret;

    ret = phy_lan8710ai_get_hcd_speed(dev, speed);
    if (ret != 0) {
        /* Read PHY advertising parameters and the link partner abilities */
        ret = phy_lan8710ai_read(dev, MII_ANAR, &anar_reg);
        if (ret == 0) {
            ret = phy_lan8710ai_read(dev, MII_ANLPAR, &anlpar_reg);
        }

        if (ret != 0) {
            ret = -EIO;
        }
        else {
            common = anar_reg & anlpar_reg;

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

/* -------------------------------------------------------------------
 * Link state handling. Called from monitor_work with the lock held.
 * Returns 0 when the state changed, -EAGAIN when nothing changed and
 * -EINPROGRESS while auto-negotiation is still running.
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_update_link_state(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    enum phy_link_speed new_speed;
    uint32_t bmcr_reg;
    uint32_t bmsr_reg;
    int ret;

    ret = phy_lan8710ai_read(dev, MII_BMSR, &bmsr_reg);
    if (ret == 0) {
        /* Link status is latched low: the second read gives the current state */
        ret = phy_lan8710ai_read(dev, MII_BMSR, &bmsr_reg);
    }

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
        ret = phy_lan8710ai_read(dev, MII_BMCR, &bmcr_reg);

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

            data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));

            ret = phy_lan8710ai_check_autoneg_completion(dev);
        }
    }

    return (ret);
}

/* Auto-negotiation did not complete: drop the half-started link so that the next poll retries
 * instead of waiting for a link-down event that never comes. Returns 0 when a link that was
 * usable is now down (state changed), @p err otherwise.
 */
static int phy_lan8710ai_autoneg_failed(struct device const* dev, int err) {
    struct phy_lan8710ai_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    bool was_usable = ((state->is_up == true) && (state->speed != 0));

    state->is_up = false;
    state->speed = 0;

    return ((was_usable == true) && (err != -EIO)) ? 0 : err;
}

static int phy_lan8710ai_autoneg_poll(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    enum phy_link_speed speed;
    uint32_t bmsr_reg;
    int ret;

    ret = phy_lan8710ai_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((bmsr_reg & MII_BMSR_AUTONEG_COMPLETE) == 0U) {
        if (sys_timepoint_expired(data->autoneg_timeout)) {
            LOG_DBG("PHY (%d) auto-negotiate timeout", cfg->phy_addr);
            ret = -ETIMEDOUT;
        }
        else {
            ret = -EINPROGRESS;
        }
    }
    else {
        /* Link status bit is latched low, so read it again to get current status */
        if (unlikely((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U)) {
            ret = phy_lan8710ai_read(dev, MII_BMSR, &bmsr_reg);

            if (ret < 0) {
                ret = -EIO;
            }
            else if ((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U) {
                ret = -EAGAIN;
            }
            else {
                ret = 0;
            }
        }

        if (ret == 0) {
            LOG_DBG("PHY (%d) auto-negotiate sequence completed", cfg->phy_addr);

            ret = phy_lan8710ai_resolve_speed(dev, &speed);
            if (ret == 0) {
                state->speed = speed;
                state->is_up = true;

                LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                        PHY_LINK_IS_SPEED_100M(state->speed)  ? "100"  : "10",
                        PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
            }
        }
    }

    return (ret);
}

static int phy_lan8710ai_check_autoneg_completion(struct device const* dev) {
    int ret;

    ret = phy_lan8710ai_autoneg_poll(dev);
    if ((ret != 0) && (ret != -EINPROGRESS)) {
        ret = phy_lan8710ai_autoneg_failed(dev, ret);
    }

    return (ret);
}

/* -------------------------------------------------------------------
 * Delayed work: periodic link monitor
 * ------------------------------------------------------------------- */
static void phy_lan8710ai_monitor_work_fn(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct phy_lan8710ai_data* data =
        CONTAINER_OF(dwork, struct phy_lan8710ai_data, monitor_work);
    struct device const* dev = data->dev;
    k_timeout_t delay;
    int rc;

    if (k_mutex_lock(&data->lock, K_NO_WAIT) != 0) {
        /* Busy (cfg_link or get_link running): retry soon so that a pending interrupt,
         * which would not produce another edge, is not left to the slow poll.
         */
        delay = K_MSEC(PHY_BUSY_RETRY_MS);
    }
    else {
        #if ANY_INT_GPIO
        const struct phy_lan8710ai_config* cfg = dev->config;
        uint32_t isr;

        if (cfg->int_gpio.port != NULL) {
            /* Reading the Interrupt Source Flag register de-asserts nINT */
            (void)phy_lan8710ai_read(dev, LAN8710_REG_ISR, &isr);
        }
        #endif /* ANY_INT_GPIO */

        if (data->autoneg_in_progress) {
            rc = phy_lan8710ai_check_autoneg_completion(dev);
        }
        else {
            /* If autonegotiation is not in progress, just update the link state */
            rc = phy_lan8710ai_update_link_state(dev);
        }

        data->autoneg_in_progress = (rc == -EINPROGRESS);

        delay = data->autoneg_in_progress ? K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS)
                                          : K_MSEC(CONFIG_PHY_MONITOR_PERIOD);

        (void)k_mutex_unlock(&data->lock);

        /* If link state has changed and a callback is set, invoke callback. This is done
         * without the lock held so that the callback may block or call back into the PHY.
         */
        if (rc == 0) {
            phy_lan8710ai_invoke_link_cb(dev);
        }
    }

    (void)k_work_reschedule(&data->monitor_work, delay);
}

/* -------------------------------------------------------------------
 * PHY API implementations
 * ------------------------------------------------------------------- */
static int phy_lan8710ai_cfg_link(struct device const* dev,
                                  enum phy_link_speed adv_speeds,
                                  enum phy_cfg_link_flag flags) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    int ret;

    (void)k_mutex_lock(&data->lock, K_FOREVER);

    if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
        ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
        if (ret >= 0) {
            data->autoneg_in_progress = false;
            (void)k_work_reschedule(&data->monitor_work, K_NO_WAIT);
        }
    }
    else {
        ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, false);
        if (ret >= 0) {
            LOG_DBG("PHY (%d) Starting MII PHY auto-negotiate sequence", cfg->phy_addr);
            data->autoneg_in_progress = true;
            data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
            (void)k_work_reschedule(&data->monitor_work, K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
        }
    }

    if (ret == -EALREADY) {
        LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
    }

    (void)k_mutex_unlock(&data->lock);

    return (ret);
}

static int phy_lan8710ai_get_link(struct device const* dev,
                                  struct phy_link_state* state) {
    struct phy_lan8710ai_data* data = dev->data;

    (void)k_mutex_lock(&data->lock, K_FOREVER);

    *state = data->state;

    if (state->speed == 0) {
        /* Speed 0 means the link is not (yet) usable, e.g. autonegotiation in progress */
        state->is_up = false;
    }

    (void)k_mutex_unlock(&data->lock);

    return (0);
}

static void phy_lan8710ai_invoke_link_cb(struct device const* dev) {
    struct phy_lan8710ai_data* data = dev->data;
    struct phy_link_state state;

    if (data->cb != NULL) {
        (void)phy_lan8710ai_get_link(dev, &state);

        data->cb(dev, &state, data->cb_data);
    }
}

static int phy_lan8710ai_link_cb_set(struct device const* dev,
                                     phy_callback_t cb, void* user_data) {
    struct phy_lan8710ai_data* data = dev->data;

    data->cb = cb;
    data->cb_data = user_data; /* caller's opaque pointer (e.g. MAC dev) */

    /* Immediately notify the caller of the current link status. */
    phy_lan8710ai_invoke_link_cb(dev);

    return (0);
}

static int phy_lan8710ai_update_bits(struct device const* dev, uint16_t reg,
                                     uint32_t mask, uint32_t val) {
    uint32_t reg_val;
    int ret;

    ret = phy_lan8710ai_read(dev, reg, &reg_val);
    if (ret == 0) {
        reg_val = (reg_val & ~mask) | (val & mask);

        ret = phy_lan8710ai_write(dev, reg, reg_val);
    }

    return (ret);
}

/* Check the MODE[2:0] straps, which set the register defaults after reset */
static int phy_lan8710ai_check_straps(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    uint32_t smr;
    uint32_t mode;
    int ret;

    ret = phy_lan8710ai_read(dev, LAN8710_REG_SMR, &smr);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        mode = FIELD_GET(LAN8710_SMR_MODE_MASK, smr);

        if (mode == LAN8710_SMR_MODE_PWRDOWN) {
            LOG_ERR("PHY (%d) MODE straps select power-down mode", cfg->phy_addr);
            ret = -ENODEV;
        }
        else if (mode <= LAN8710_SMR_MODE_100FD_NOAN) {
            LOG_WRN("PHY (%d) MODE straps (%u) disable auto-negotiation, the driver "
                    "re-enables it", cfg->phy_addr, mode);
        }
        else {
            /* Auto-negotiation enabled by the straps */
        }
    }

    return (ret);
}

/* One read-modify-write of a register, used to describe register setup as a table */
struct phy_lan8710ai_bits_op {
    uint16_t reg;
    uint32_t mask;
    uint32_t val;
};

static int phy_lan8710ai_update_bits_seq(struct device const* dev,
                                         const struct phy_lan8710ai_bits_op* ops,
                                         size_t count) {
    size_t i;
    int ret = 0;

    for (i = 0U; (i < count) && (ret == 0); i++) {
        ret = phy_lan8710ai_update_bits(dev, ops[i].reg, ops[i].mask, ops[i].val);
    }

    return (ret);
}

/* SCSIR value for the configured MDI/MDIX mode */
static uint32_t phy_lan8710ai_mdix_val(const struct phy_lan8710ai_config* cfg) {
    uint32_t val = 0U;

    if (cfg->mdix_mode != LAN8710_MDIX_AUTO) {
        val = LAN8710_SCSIR_AMDIXCTRL;
        if (cfg->mdix_mode == LAN8710_MDIX_MDIX) {
            val |= LAN8710_SCSIR_CH_SELECT;
        }
    }

    return (val);
}

static int phy_lan8710ai_cfg_mdix_edpd(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    const struct phy_lan8710ai_bits_op ops[] = {
        {LAN8710_REG_SCSIR, LAN8710_SCSIR_AMDIXCTRL | LAN8710_SCSIR_CH_SELECT,
         phy_lan8710ai_mdix_val(cfg)},
        {LAN8710_REG_MCSR, LAN8710_MCSR_EDPWRDOWN, cfg->edpd ? LAN8710_MCSR_EDPWRDOWN : 0U},
    };
    int ret = 0;

    /* Only when the devicetree sets mdix-mode or energy-detect-power-down: otherwise the
     * reset or bootloader configuration is kept, as in the baseline driver.
     */
    if (cfg->apply_mdix_edpd == true) {
        ret = phy_lan8710ai_update_bits_seq(dev, ops, ARRAY_SIZE(ops));
    }

    if (ret < 0) {
        LOG_ERR("Failed to configure auto-MDIX/EDPD (%d)", ret);
    }

    return (ret);
}

#if ANY_INT_GPIO
static void phy_lan8710ai_int_isr(const struct device* port, struct gpio_callback* cb,
                                  uint32_t pins) {
    struct phy_lan8710ai_data* data = CONTAINER_OF(cb, struct phy_lan8710ai_data, int_cb);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    /* Re-evaluate the link now instead of waiting for the next poll */
    (void)k_work_reschedule(&data->monitor_work, K_NO_WAIT);
}

/* Primary interrupt system (ALTINT = 0): auto-negotiation complete and link down.
 * There is no link-up event in forced mode, so the periodic poll stays active.
 */
static const struct phy_lan8710ai_bits_op phy_lan8710ai_int_ops[] = {
    {LAN8710_REG_MCSR, LAN8710_MCSR_ALTINT, 0U},
    {LAN8710_REG_IMR, LAN8710_INT_ANEG_COMPLETE | LAN8710_INT_LINK_DOWN,
     LAN8710_INT_ANEG_COMPLETE | LAN8710_INT_LINK_DOWN},
};

static int phy_lan8710ai_cfg_int(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    uint32_t isr;
    int ret;

    if (cfg->int_gpio.port == NULL) {
        ret = 0;
    }
    else if (gpio_is_ready_dt(&cfg->int_gpio) == false) {
        LOG_ERR("PHY (%d) interrupt GPIO not ready", cfg->phy_addr);
        ret = -ENODEV;
    }
    else {
        ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);

        if (ret == 0) {
            gpio_init_callback(&data->int_cb, phy_lan8710ai_int_isr, BIT(cfg->int_gpio.pin));

            ret = gpio_add_callback_dt(&cfg->int_gpio, &data->int_cb);
        }

        if (ret == 0) {
            ret = phy_lan8710ai_update_bits_seq(dev, phy_lan8710ai_int_ops,
                                                ARRAY_SIZE(phy_lan8710ai_int_ops));
        }

        if (ret == 0) {
            /* Clear stale flags, then enable the pin interrupt (nINT is active low) */
            (void)phy_lan8710ai_read(dev, LAN8710_REG_ISR, &isr);

            ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        }

        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt (%d)", ret);
        }
    }

    return (ret);
}
#endif /* ANY_INT_GPIO */

static int phy_lan8710ai_init_reset(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    int ret = 0;

    if (cfg->no_reset == false) {
        ret = phy_lan8710ai_reset(dev);
        if (ret < 0) {
            LOG_ERR("Failed to reset PHY (%d): %d", cfg->phy_addr, ret);
        }
    }

    return (ret);
}

static int phy_lan8710ai_init_link(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    int ret;

    /* Advertise default speeds */
    ret = phy_lan8710ai_cfg_link(dev, cfg->default_speeds, 0);
    if (ret == -EALREADY) {
        /* Link already configured, e.g. by a bootloader: just wait for it to complete */
        (void) k_mutex_lock(&data->lock, K_FOREVER);
        data->autoneg_in_progress = true;
        data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
        (void) k_mutex_unlock(&data->lock);
        ret = 0;
    }
    else if (ret < 0) {
        LOG_ERR("Failed to configure link (%d)", ret);
    }
    else {
        /* Auto-negotiation restarted by phy_lan8710ai_cfg_link() */
    }

    if (ret == 0) {
        /* Schedule the monitor work, if not already scheduled by phy_lan8710ai_cfg_link(). */
        (void)k_work_schedule(&data->monitor_work, K_NO_WAIT);
    }

    return (ret);
}

/* Initialization steps, run in order until one fails */
static int (*const phy_lan8710ai_init_steps[])(struct device const*) = {
    phy_lan8710ai_init_reset,
    phy_lan8710ai_verify_id,
    phy_lan8710ai_check_straps,
    phy_lan8710ai_cfg_mdix_edpd,
    #if ANY_INT_GPIO
    phy_lan8710ai_cfg_int,
    #endif /* ANY_INT_GPIO */
    phy_lan8710ai_init_link,
};

static int phy_lan8710ai_init(struct device const* dev) {
    const struct phy_lan8710ai_config* cfg = dev->config;
    struct phy_lan8710ai_data* data = dev->data;
    size_t i;
    int ret = 0;

    data->state.is_up = false;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->monitor_work, phy_lan8710ai_monitor_work_fn);
    data->dev = dev;

    if (device_is_ready(cfg->mdio_dev) == false) {
        LOG_ERR_DEVICE_NOT_READY(cfg->mdio_dev);
        ret = -ENODEV;
    }

    for (i = 0U; (i < ARRAY_SIZE(phy_lan8710ai_init_steps)) && (ret == 0); i++) {
        ret = phy_lan8710ai_init_steps[i](dev);
    }

    return (ret);
}

/* -------------------------------------------------------------------
 * Driver API and instantiation
 * ------------------------------------------------------------------- */
static DEVICE_API(ethphy, phy_lan8710ai_driver_api) = {
    .cfg_link    = phy_lan8710ai_cfg_link,
    .get_link    = phy_lan8710ai_get_link,
    .link_cb_set = phy_lan8710ai_link_cb_set,
    .read        = phy_lan8710ai_read,
    .write       = phy_lan8710ai_write
};

#if ANY_INT_GPIO
#define LAN8710AI_INT_GPIO(n)                                   \
    .int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define LAN8710AI_INT_GPIO(n)
#endif /* ANY_INT_GPIO */

#if ANY_RESET_GPIO
/* Defaults of 10 ms keep the timing of the previous hard-coded sequence */
#define LAN8710AI_RESET_GPIO(n)                                 \
    .reset_gpio        = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}), \
    .reset_assert_us   = DT_INST_PROP_OR(n, reset_assert_us, 10000), \
    .reset_deassert_us = DT_INST_PROP_OR(n, reset_deassert_us, 10000),
#else
#define LAN8710AI_RESET_GPIO(n)
#endif /* ANY_RESET_GPIO */

#define LAN8710AI_INIT(n)                                       \
    BUILD_ASSERT((PHY_INST_GENERATE_DEFAULT_SPEEDS(n) != 0),    \
                 "At least one valid speed must be configured for this driver"); \
                                                                \
    static struct phy_lan8710ai_data phy_lan8710ai_data_##n;    \
    static const struct phy_lan8710ai_config phy_lan8710ai_cfg_##n = { \
        .phy_addr       = DT_INST_REG_ADDR(n),                  \
        .no_reset       = DT_INST_PROP(n, no_reset),            \
        .mdio_dev       = DEVICE_DT_GET(DT_INST_BUS(n)),        \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),  \
        .mdix_mode      = DT_INST_ENUM_IDX(n, mdix_mode),       \
        .edpd           = DT_INST_PROP(n, energy_detect_power_down), \
        .apply_mdix_edpd = (DT_INST_NODE_HAS_PROP(n, mdix_mode) || \
                            DT_INST_NODE_HAS_PROP(n, energy_detect_power_down)), \
        LAN8710AI_INT_GPIO(n)                                   \
        LAN8710AI_RESET_GPIO(n)                                 \
    };                                                          \
    DEVICE_DT_INST_DEFINE(n, phy_lan8710ai_init, NULL,          \
                          &phy_lan8710ai_data_##n,              \
                          &phy_lan8710ai_cfg_##n, POST_KERNEL,  \
                          CONFIG_PHY_INIT_PRIORITY,             \
                          &phy_lan8710ai_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LAN8710AI_INIT)
