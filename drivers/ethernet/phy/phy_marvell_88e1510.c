/*
 * Copyright (c) 2024 Marvell / Zephyr Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT marvell_88e1510

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(phy_marvell_88e1510, CONFIG_PHY_LOG_LEVEL);

#include "phy_mii.h"

/* PHY OUI + model, ignoring the 4-bit revision field */
#define MARVELL_88E1510_PHY_ID      0x01410DD0U
#define MARVELL_88E1510_PHY_ID_MASK 0xFFFFFFF0U

/* Chip specific registers */
#define MARVELL_PAGE_SELECT_REG     22
#define MARVELL_LED_FUNC_CTRL_REG   16      /* Table 112: LED[2:0] Function Control Register */
#define MARVELL_LED_POLARITY_REG    17      /* Table 113: LED[2:0] Polarity Control Register */

#define MARVELL_LED_POLARITY_LED01_MASK     GENMASK(3, 0)
#define MARVELL_LED_POLARITY_LED01_ON_HIGH  0x5U
#define MARVELL_LED_FUNC_MASK       (BIT(12) | GENMASK(7, 0))
#define MARVELL_LED_TIMER_CTRL_REG  18      /* Table 114: LED Timer Control Register */
#define MARVELL_LED_TCR_INT_ACTIVE_LOW BIT(11)
#define MARVELL_LED_TCR_INTN_ENABLE BIT(7)

/* Table 78: Copper Specific Status Register 1 */
#define MARVELL_PHY_STATUS_REG      17
#define MARVELL_PHY_STATUS_SPEED_MASK GENMASK(15, 14)
#define MARVELL_PHY_STATUS_SPEED_10   0U
#define MARVELL_PHY_STATUS_SPEED_100  1U
#define MARVELL_PHY_STATUS_SPEED_1000 2U
#define MARVELL_PHY_STATUS_FULL_DUPLEX BIT(13)
#define MARVELL_PHY_STATUS_RESOLVED BIT(11)
#define MARVELL_PHY_STATUS_LINK     BIT(10)

/* Table 79: Copper Specific Interrupt Enable Register */
/* Table 80: Copper Interrupt Status Register */
#define MARVELL_INT_ENABLE_REG      18
#define MARVELL_INT_STATUS_REG      19      /* read clears */
#define MARVELL_INT_SPEED_CHANGED   BIT(14)
#define MARVELL_INT_DUPLEX_CHANGED  BIT(13)
#define MARVELL_INT_AUTONEG_DONE    BIT(11)
#define MARVELL_INT_LINK_CHANGED    BIT(10)

#define MII_INVALID_PHY_ID          UINT32_MAX
#define MII_AUTONEG_POLL_INTERVAL_MS 100
/* Retry delay when the monitor finds the PHY busy */
#define PHY_BUSY_RETRY_MS            10

/* Wait up to 0.6 s for the BMCR soft-reset bit to self-clear (IEEE 802.3, 22.2.4.1.1) */
#define MARVELL_RESET_POLL_MS       50
#define MARVELL_RESET_RETRIES       12U

/* 1000BASE-T Status register offset relative to 1KTCR for capability comparison */
#define MII_1KSTSR_OFFSET 2

#define ANY_RESET_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define ANY_INT_GPIO   DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)

struct phy_88e1510_config {
    const struct device* mdio;
    uint8_t phy_addr;
    bool no_reset;
    enum phy_link_speed default_speeds;
    uint8_t led0_func_sel;
    uint8_t led1_func_sel;

    #if ANY_INT_GPIO
    struct gpio_dt_spec gpio_int;           /* zeroed if not in DT */
    #endif

    #if ANY_RESET_GPIO
    struct gpio_dt_spec reset_gpio;
    uint32_t reset_assert_us;
    uint32_t reset_deassert_us;
    #endif
};

struct phy_88e1510_data {
    struct device const* dev;
    phy_callback_t cb;
    void* cb_data;
    struct phy_link_state state;
    struct k_mutex lock;
    struct k_work_delayable monitor_work;
    bool gigabit_supported;
    bool autoneg_in_progress;
    k_timepoint_t autoneg_timeout;

    #if ANY_INT_GPIO
    struct gpio_callback int_cb;
    #endif
};

static void phy_88e1510_invoke_link_cb(struct device const* dev);
static int  phy_88e1510_check_autoneg_completion(struct device const* dev);
static int  phy_88e1510_select_page(struct device const* dev, uint8_t page_no);
static int  phy_88e1510_cfg_led(struct device const* dev, uint8_t led1_func, uint8_t led0_func);
#if ANY_INT_GPIO
static int  phy_88e1510_cfg_int(struct device const* dev);
#endif

static int phy_88e1510_read_raw(struct device const* dev, uint16_t reg_addr, uint32_t* data) {
    const struct phy_88e1510_config* cfg = dev->config;
    uint16_t reg_val;
    int ret;

    ret = mdio_read(cfg->mdio, cfg->phy_addr, (uint8_t)reg_addr, &reg_val);
    if (ret == 0) {
        *data = reg_val;
    }

    return (ret);
}

static int phy_88e1510_write_raw(struct device const* dev, uint16_t reg_addr, uint32_t data) {
    const struct phy_88e1510_config* cfg = dev->config;
    int ret;

    ret = mdio_write(cfg->mdio, cfg->phy_addr, (uint8_t)reg_addr, (uint16_t)data);

    return (ret);
}

/* Register access as seen by the PHY API users: serialized against the paged sequences below.
 * The lock is recursive, so callers that already hold it can use these too.
 */
static int phy_88e1510_read(struct device const* dev, uint16_t reg_addr, uint32_t* data) {
    struct phy_88e1510_data* priv = dev->data;
    int ret;

    (void) k_mutex_lock(&priv->lock, K_FOREVER);
    ret = phy_88e1510_read_raw(dev, reg_addr, data);
    (void) k_mutex_unlock(&priv->lock);

    return (ret);
}

static int phy_88e1510_write(struct device const* dev, uint16_t reg_addr, uint32_t data) {
    struct phy_88e1510_data* priv = dev->data;
    int ret;

    (void) k_mutex_lock(&priv->lock, K_FOREVER);
    ret = phy_88e1510_write_raw(dev, reg_addr, data);
    (void) k_mutex_unlock(&priv->lock);

    return (ret);
}

#if ANY_RESET_GPIO
static int phy_88e1510_hw_reset(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
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
            (void) k_usleep(cfg->reset_deassert_us);
        }
    }

    return (ret);
}
#endif /* ANY_RESET_GPIO */

/* Soft reset via BMCR bit 15 */
static int phy_88e1510_soft_reset(struct device const* dev) {
    uint32_t value;
    uint32_t retries;
    int ret;

    ret = phy_88e1510_write(dev, MII_BMCR, MII_BMCR_RESET);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        /* Poll until reset self-clears, up to ~600 ms (IEEE 802.3 §22.2.4.1.1) */
        ret = -ETIMEDOUT;

        for (retries = 0U; (retries < MARVELL_RESET_RETRIES) && (ret == -ETIMEDOUT); retries++) {
            k_msleep(MARVELL_RESET_POLL_MS);

            if (phy_88e1510_read(dev, MII_BMCR, &value) < 0) {
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

static int phy_88e1510_reset(struct device const* dev) {
    int ret = 0;
    bool need_soft_reset = true;

    #if ANY_RESET_GPIO
    const struct phy_88e1510_config* cfg = dev->config;

    /* A hardware reset replaces the soft reset */
    if (cfg->reset_gpio.port != NULL) {
        if (gpio_is_ready_dt(&cfg->reset_gpio) == true) {
            ret = phy_88e1510_hw_reset(dev);
            need_soft_reset = false;
        }
        else {
            LOG_WRN("PHY (%d) reset GPIO not ready, using soft reset", cfg->phy_addr);
        }
    }
    #endif /* ANY_RESET_GPIO */

    if ((ret == 0) && (need_soft_reset == true)) {
        ret = phy_88e1510_soft_reset(dev);
    }

    return (ret);
}

static int phy_88e1510_get_id(struct device const* dev, uint32_t* phy_id) {
    uint32_t value;
    int ret;

    ret = phy_88e1510_read(dev, MII_PHYID1R, &value);
    if (ret < 0) {
        ret = -EIO;
    }
    else {
        *phy_id = value << 16;

        ret = phy_88e1510_read(dev, MII_PHYID2R, &value);
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

static int phy_88e1510_verify_id(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    uint32_t phy_id = 0U;
    int ret;

    ret = phy_88e1510_get_id(dev, &phy_id);
    if (ret < 0) {
        LOG_ERR("PHY (%d) ID read failed: %d", cfg->phy_addr, ret);
    }
    else if (phy_id == MII_INVALID_PHY_ID) {
        LOG_ERR("No PHY found at address %d", cfg->phy_addr);
        ret = -EINVAL;
    }
    else if ((phy_id & MARVELL_88E1510_PHY_ID_MASK) != MARVELL_88E1510_PHY_ID) {
        LOG_ERR("PHY (%d) unexpected ID 0x%08X (expected 0x%08X)",
                cfg->phy_addr, phy_id, MARVELL_88E1510_PHY_ID);
        ret = -ENODEV;
    }
    else {
        LOG_INF("PHY (%d) Marvell 88E1510 ID 0x%08X", cfg->phy_addr, phy_id);
    }

    return (ret);
}

static int phy_88e1510_read_gigabit_flag(struct device const* dev, bool* supported) {
    uint32_t bmsr_reg;
    uint32_t estat_reg;
    int ret;

    *supported = false;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((bmsr_reg & MII_BMSR_EXTEND_STATUS) != 0U) {
        ret = phy_88e1510_read(dev, MII_ESTAT, &estat_reg);

        if (ret < 0) {
            ret = -EIO;
        }
        else if ((estat_reg & (MII_ESTAT_1000BASE_T_HALF | MII_ESTAT_1000BASE_T_FULL)) != 0U) {
            *supported = true;
        }
        else {
            /* No 1000BASE-T capability reported */
        }
    }
    else {
        /* No extended status register, 10/100 only */
    }

    return (ret);
}

/* Read the resolved speed and duplex from the PHY specific status register */
static int phy_88e1510_get_resolved_speed(struct device const* dev, enum phy_link_speed* speed) {
    uint32_t status;
    bool full;
    int ret;

    ret = phy_88e1510_read(dev, MARVELL_PHY_STATUS_REG, &status);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((status & (MARVELL_PHY_STATUS_RESOLVED | MARVELL_PHY_STATUS_LINK)) !=
             (MARVELL_PHY_STATUS_RESOLVED | MARVELL_PHY_STATUS_LINK)) {
        ret = -EAGAIN;
    }
    else {
        full = ((status & MARVELL_PHY_STATUS_FULL_DUPLEX) != 0U);
        ret = 0;

        switch (FIELD_GET(MARVELL_PHY_STATUS_SPEED_MASK, status)) {
            case MARVELL_PHY_STATUS_SPEED_1000 :
                *speed = full ? LINK_FULL_1000BASE : LINK_HALF_1000BASE;
                break;

            case MARVELL_PHY_STATUS_SPEED_100 :
                *speed = full ? LINK_FULL_100BASE : LINK_HALF_100BASE;
                break;

            case MARVELL_PHY_STATUS_SPEED_10 :
                *speed = full ? LINK_FULL_10BASE : LINK_HALF_10BASE;
                break;

            default :
                ret = -EINVAL;
                break;
        }
    }

    return (ret);
}

/* Resolve the speed from the advertised and link partner abilities */
static int phy_88e1510_resolve_speed_from_abilities(struct device const* dev,
                                                    enum phy_link_speed* speed) {
    struct phy_88e1510_data* data = dev->data;
    uint32_t anar_reg = 0U;
    uint32_t anlpar_reg = 0U;
    uint32_t c1kt_reg = 0U;
    uint32_t s1kt_reg = 0U;
    uint32_t common;
    int ret;

    ret = phy_88e1510_read(dev, MII_ANAR, &anar_reg);
    if (ret == 0) {
        ret = phy_88e1510_read(dev, MII_ANLPAR, &anlpar_reg);
    }

    if ((ret == 0) && data->gigabit_supported) {
        ret = phy_88e1510_read(dev, MII_1KTCR, &c1kt_reg);

        if (ret == 0) {
            ret = phy_88e1510_read(dev, MII_1KSTSR, &s1kt_reg);
        }

        s1kt_reg = (s1kt_reg >> MII_1KSTSR_OFFSET);
    }

    if (ret != 0) {
        ret = -EIO;
    }
    else {
        common = anar_reg & anlpar_reg;

        if ((c1kt_reg & s1kt_reg & MII_ADVERTISE_1000_FULL) != 0U) {
            *speed = LINK_FULL_1000BASE;
        }
        else if ((c1kt_reg & s1kt_reg & MII_ADVERTISE_1000_HALF) != 0U) {
            *speed = LINK_HALF_1000BASE;
        }
        else if ((common & MII_ADVERTISE_100_FULL) != 0U) {
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

    return (ret);
}

/* Prefer the speed resolved by the PHY, fall back to the register based resolution */
static int phy_88e1510_resolve_speed(struct device const* dev, enum phy_link_speed* speed) {
    int ret;

    ret = phy_88e1510_get_resolved_speed(dev, speed);
    if (ret != 0) {
        ret = phy_88e1510_resolve_speed_from_abilities(dev, speed);
    }

    return (ret);
}

static int phy_88e1510_update_link_state(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    enum phy_link_speed new_speed;
    uint32_t bmcr_reg;
    uint32_t bmsr_reg;
    int ret;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
    if (ret < 0) {
        ret = -EIO;
    }
    else if ((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U) {
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
        ret = phy_88e1510_read(dev, MII_BMCR, &bmcr_reg);

        if (ret < 0) {
            ret = -EIO;
        }
        else if ((bmcr_reg & MII_BMCR_AUTONEG_ENABLE) == 0U) {
            new_speed = phy_mii_get_link_speed_bmcr_reg(dev, bmcr_reg);

            if ((state->speed != new_speed) || (state->is_up == false)) {
                state->is_up = true;
                state->speed = new_speed;

                LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                        PHY_LINK_IS_SPEED_1000M(state->speed) ? "1000" :
                        (PHY_LINK_IS_SPEED_100M(state->speed) ?  "100" : "10"),
                        PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");

                ret = 0;
            }
            else {
                ret = -EAGAIN;
            }
        }
        else if (state->is_up == true) {
            ret = -EAGAIN;
        }
        else {
            state->is_up = true;

            LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);

            data->autoneg_timeout = sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));

            ret = phy_88e1510_check_autoneg_completion(dev);
        }
    }

    return (ret);
}

/* Auto-negotiation did not complete: drop the half-started link so that the next poll retries
 * instead of waiting for a link-down event that never comes. Returns 0 when a link that was
 * usable is now down (state changed), err otherwise.
 */
static int phy_88e1510_autoneg_failed(struct device const* dev, int err) {
    struct phy_88e1510_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    bool was_usable = ((state->is_up == true) && (state->speed != 0));

    state->is_up = false;
    state->speed = 0;

    return ((was_usable == true) && (err != -EIO)) ? 0 : err;
}

static int phy_88e1510_autoneg_poll(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    struct phy_link_state* const state = &data->state;
    enum phy_link_speed speed;
    uint32_t bmsr_reg;
    int ret;

    ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);
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
        /* BMSR link-status bit is latched-low; re-read to get current value */
        if (unlikely((bmsr_reg & MII_BMSR_LINK_STATUS) == 0U)) {
            ret = phy_88e1510_read(dev, MII_BMSR, &bmsr_reg);

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
            LOG_DBG("PHY (%d) auto-negotiate completed", cfg->phy_addr);

            ret = phy_88e1510_resolve_speed(dev, &speed);
            if (ret == 0) {
                state->speed = speed;
                state->is_up = true;

                LOG_INF("PHY (%d) Link speed %s Mb, %s duplex", cfg->phy_addr,
                        PHY_LINK_IS_SPEED_1000M(state->speed) ? "1000" :
                        (PHY_LINK_IS_SPEED_100M(state->speed) ?  "100" : "10"),
                        PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
            }
        }
    }

    return (ret);
}

static int phy_88e1510_check_autoneg_completion(struct device const* dev) {
    int ret;

    ret = phy_88e1510_autoneg_poll(dev);
    if ((ret != 0) && (ret != -EINPROGRESS)) {
        ret = phy_88e1510_autoneg_failed(dev, ret);
    }

    return (ret);
}

static void phy_88e1510_monitor_work_handler(struct k_work* work) {
    struct k_work_delayable* dwork = k_work_delayable_from_work(work);
    struct phy_88e1510_data* data =
        CONTAINER_OF(dwork, struct phy_88e1510_data, monitor_work);
    struct device const* dev = data->dev;
    k_timeout_t delay;
    int ret;

    if (k_mutex_lock(&data->lock, K_NO_WAIT) != 0) {
        /* Busy (cfg_link or get_link running): retry soon so that a pending interrupt,
         * which would not produce another edge, is not left to the slow poll.
         */
        delay = K_MSEC(PHY_BUSY_RETRY_MS);
    }
    else {
        #if ANY_INT_GPIO
        const struct phy_88e1510_config* cfg = dev->config;
        uint32_t isr;

        /* Reading the status register releases INTn */
        if (cfg->gpio_int.port != NULL) {
            (void)phy_88e1510_read(dev, MARVELL_INT_STATUS_REG, &isr);
        }
        #endif /* ANY_INT_GPIO */

        if (data->autoneg_in_progress) {
            ret = phy_88e1510_check_autoneg_completion(dev);
        }
        else {
            ret = phy_88e1510_update_link_state(dev);
        }

        data->autoneg_in_progress = (ret == -EINPROGRESS);

        k_mutex_unlock(&data->lock);

        if (ret == 0) {
            phy_88e1510_invoke_link_cb(dev);
        }

        delay = data->autoneg_in_progress ? K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS)
                                          : K_MSEC(CONFIG_PHY_MONITOR_PERIOD);
    }

    (void) k_work_reschedule(&data->monitor_work, delay);
}

static int phy_88e1510_cfg_link(struct device const* dev, enum phy_link_speed adv_speeds,
                                enum phy_cfg_link_flag flags) {
    const struct phy_88e1510_config* cfg  = dev->config;
    struct phy_88e1510_data* data = dev->data;
    int ret;

    (void) k_mutex_lock(&data->lock, K_FOREVER);

    if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) != 0U) {
        if (!data->gigabit_supported && PHY_LINK_IS_SPEED_1000M(adv_speeds)) {
            LOG_ERR("PHY (%d) Gigabit not supported", cfg->phy_addr);
            ret = -ENOTSUP;
        }
        else {
            ret = phy_mii_set_bmcr_reg_autoneg_disabled(dev, adv_speeds);
            if (ret >= 0) {
                data->autoneg_in_progress = false;
                (void) k_work_reschedule(&data->monitor_work, K_NO_WAIT);
            }
        }
    }
    else {
        ret = phy_mii_cfg_link_autoneg(dev, adv_speeds, data->gigabit_supported);
        if (ret >= 0) {
            LOG_DBG("PHY (%d) Starting auto-negotiate sequence", cfg->phy_addr);
            data->autoneg_in_progress = true;
            data->autoneg_timeout =
                sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
            (void) k_work_reschedule(&data->monitor_work,
                                     K_MSEC(MII_AUTONEG_POLL_INTERVAL_MS));
        }
    }

    if (ret == -EALREADY) {
        LOG_DBG("PHY (%d) Link already configured", cfg->phy_addr);
    }

    k_mutex_unlock(&data->lock);

    return (ret);
}

static int phy_88e1510_get_link_state(struct device const* dev, struct phy_link_state* state) {
    struct phy_88e1510_data* data = dev->data;

    (void) k_mutex_lock(&data->lock, K_FOREVER);

    (void) memcpy(state, &data->state, sizeof(struct phy_link_state));

    /* Speed == 0 means autoneg is in progress; report link as down */
    if (state->speed == 0) {
        state->is_up = false;
    }

    k_mutex_unlock(&data->lock);

    return (0);
}

static void phy_88e1510_invoke_link_cb(struct device const* dev) {
    struct phy_88e1510_data* data = dev->data;
    struct phy_link_state state;

    if (data->cb != NULL) {
        (void) phy_88e1510_get_link_state(dev, &state);

        data->cb(dev, &state, data->cb_data);
    }
}

static int phy_88e1510_link_cb_set(struct device const* dev, phy_callback_t cb, void* user_data) {
    struct phy_88e1510_data* data = dev->data;

    data->cb = cb;
    data->cb_data = user_data;

    phy_88e1510_invoke_link_cb(dev);

    return (0);
}

static int phy_88e1510_init_reset(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    int ret = 0;

    if (cfg->no_reset == false) {
        ret = phy_88e1510_reset(dev);
        if (ret < 0) {
            LOG_ERR("PHY (%d) reset failed: %d", cfg->phy_addr, ret);
        }
    }

    return (ret);
}

static int phy_88e1510_init_caps(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    int ret;

    ret = phy_88e1510_read_gigabit_flag(dev, &data->gigabit_supported);
    if (ret < 0) {
        LOG_ERR("PHY (%d) capability read failed: %d", cfg->phy_addr, ret);
    }

    return (ret);
}

static int phy_88e1510_init_led(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;

    /* A failed LED configuration is not fatal, it is logged by the callee */
    (void)phy_88e1510_cfg_led(dev, cfg->led1_func_sel, cfg->led0_func_sel);

    return (0);
}

static int phy_88e1510_init_link(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    int link_ret;

    /* Advertise default speeds. A failure is logged here and returned from init. */
    link_ret = phy_88e1510_cfg_link(dev, cfg->default_speeds, 0);
    if (link_ret == -EALREADY) {
        data->autoneg_in_progress = true;
        data->autoneg_timeout =
            sys_timepoint_calc(K_MSEC(CONFIG_PHY_AUTONEG_TIMEOUT_MS));
        link_ret = 0;
    }
    else if (link_ret < 0) {
        LOG_ERR("PHY (%d) failed to configure link (%d)", cfg->phy_addr, link_ret);
    }
    else {
        /* Auto-negotiation restarted by phy_88e1510_cfg_link() */
    }

    if (link_ret >= 0) {
        (void)k_work_schedule(&data->monitor_work, K_NO_WAIT);
        link_ret = 0;
    }

    return (link_ret);
}

/* Initialization steps, run in order until one fails */
static int (*const phy_88e1510_init_steps[])(struct device const*) = {
    phy_88e1510_init_reset,
    phy_88e1510_verify_id,
    phy_88e1510_init_caps,
    phy_88e1510_init_led,
    #if ANY_INT_GPIO
    phy_88e1510_cfg_int,
    #endif /* ANY_INT_GPIO */
    phy_88e1510_init_link,
};

static int phy_88e1510_init(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    size_t i;
    int ret = 0;

    data->state.is_up = false;
    k_work_init_delayable(&data->monitor_work, phy_88e1510_monitor_work_handler);

    if (device_is_ready(cfg->mdio) == false) {
        LOG_ERR("MDIO device not ready");
        ret = -ENODEV;
    }

    for (i = 0U; (i < ARRAY_SIZE(phy_88e1510_init_steps)) && (ret == 0); i++) {
        ret = phy_88e1510_init_steps[i](dev);
    }

    return (ret);
}

static int phy_88e1510_select_page(struct device const* dev, uint8_t page_no) {
    int ret;

    ret = phy_88e1510_write(dev, MARVELL_PAGE_SELECT_REG, page_no);

    return (ret);
}

static int phy_88e1510_update_bits(struct device const* dev, uint16_t reg, uint32_t mask,
                                   uint32_t val) {
    uint32_t reg_val;
    int ret;

    ret = phy_88e1510_read(dev, reg, &reg_val);
    if (ret == 0) {
        ret = phy_88e1510_write(dev, reg, (reg_val & ~mask) | (val & mask));
    }

    return (ret);
}

static int phy_88e1510_cfg_led(struct device const* dev, uint8_t led1_func, uint8_t led0_func) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    int ret;

    (void) k_mutex_lock(&data->lock, K_FOREVER);

    ret = phy_88e1510_select_page(dev, 3);
    if (ret == 0) {
        /* @see Table 113: LED[2:0] Polarity Control Register
         * LED[1] and LED[0] polarity 01 = On - drive the pin high, Off - drive it low
         * (LED[1] and LED[0] are assumed connected to the anode of the LEDs). LED[2] and the
         * other bits keep their current values.
         */
        ret = phy_88e1510_update_bits(dev, MARVELL_LED_POLARITY_REG,
                                      MARVELL_LED_POLARITY_LED01_MASK,
                                      MARVELL_LED_POLARITY_LED01_ON_HIGH);

        /* @see Table 112: LED[2:0] Function Control Register. LED[2] is left alone, it may
         * be used as INTn.
         */
        if (ret == 0) {
            ret = phy_88e1510_update_bits(dev, MARVELL_LED_FUNC_CTRL_REG,
                                          MARVELL_LED_FUNC_MASK,
                                          BIT(12) | ((uint32_t)led1_func << 4) | led0_func);
        }

        (void)phy_88e1510_select_page(dev, 0);
    }

    (void) k_mutex_unlock(&data->lock);

    if (ret < 0) {
        LOG_ERR("PHY (%d) LED config failed: %d", cfg->phy_addr, ret);
    }

    return (ret);
}

#if ANY_INT_GPIO
static void phy_88e1510_int_isr(const struct device* port, struct gpio_callback* cb,
                                uint32_t pins) {
    struct phy_88e1510_data* data = CONTAINER_OF(cb, struct phy_88e1510_data, int_cb);

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    /* Re-evaluate the link now instead of waiting for the next poll */
    (void)k_work_reschedule(&data->monitor_work, K_NO_WAIT);
}

/* Route INTn to the LED[2]/INTn pin, active low */
static int phy_88e1510_cfg_int_pin(struct device const* dev) {
    struct phy_88e1510_data* data = dev->data;
    uint32_t val;
    int ret;

    (void) k_mutex_lock(&data->lock, K_FOREVER);

    ret = phy_88e1510_select_page(dev, 3);
    if (ret == 0) {
        ret = phy_88e1510_read(dev, MARVELL_LED_TIMER_CTRL_REG, &val);
        if (ret == 0) {
            val |= MARVELL_LED_TCR_INTN_ENABLE | MARVELL_LED_TCR_INT_ACTIVE_LOW;
            ret = phy_88e1510_write(dev, MARVELL_LED_TIMER_CTRL_REG, val);
        }

        (void)phy_88e1510_select_page(dev, 0);
    }

    (void) k_mutex_unlock(&data->lock);

    return (ret);
}

static int phy_88e1510_cfg_int(struct device const* dev) {
    const struct phy_88e1510_config* cfg = dev->config;
    struct phy_88e1510_data* data = dev->data;
    uint32_t val;
    int ret;

    if (cfg->gpio_int.port == NULL) {
        ret = 0;
    }
    else if (gpio_is_ready_dt(&cfg->gpio_int) == false) {
        ret = -ENODEV;
    }
    else {
        ret = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);

        if (ret == 0) {
            gpio_init_callback(&data->int_cb, phy_88e1510_int_isr, BIT(cfg->gpio_int.pin));

            ret = gpio_add_callback_dt(&cfg->gpio_int, &data->int_cb);
        }

        if (ret == 0) {
            ret = phy_88e1510_cfg_int_pin(dev);
        }

        if (ret == 0) {
            ret = phy_88e1510_write(dev, MARVELL_INT_ENABLE_REG,
                                    MARVELL_INT_AUTONEG_DONE | MARVELL_INT_LINK_CHANGED |
                                    MARVELL_INT_SPEED_CHANGED | MARVELL_INT_DUPLEX_CHANGED);
        }

        if (ret == 0) {
            /* Clear stale flags, then enable the pin interrupt (INTn is active low) */
            (void)phy_88e1510_read(dev, MARVELL_INT_STATUS_REG, &val);

            ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
        }

        if (ret < 0) {
            LOG_ERR("PHY (%d) interrupt config failed: %d", cfg->phy_addr, ret);
        }
    }

    return (ret);
}
#endif /* ANY_INT_GPIO */

static DEVICE_API(ethphy, phy_88e1510_driver_api) = {
    .get_link    = phy_88e1510_get_link_state,
    .link_cb_set = phy_88e1510_link_cb_set,
    .cfg_link    = phy_88e1510_cfg_link,
    .read        = phy_88e1510_read,
    .write       = phy_88e1510_write,
};

#if ANY_RESET_GPIO
#define RESET_GPIO_FIELDS(n) \
    .reset_gpio        = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}), \
    .reset_assert_us   = DT_INST_PROP_OR(n, reset_assert_us, 10000),       \
    .reset_deassert_us = DT_INST_PROP_OR(n, reset_deassert_us, 50000),
#else
#define RESET_GPIO_FIELDS(n)
#endif /* ANY_RESET_GPIO */

#if ANY_INT_GPIO
#define INT_GPIO_FIELD(n) \
    .gpio_int = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INT_GPIO_FIELD(n)
#endif /* ANY_INT_GPIO */

#define PHY_88E1510_CONFIG(n)                                  \
    BUILD_ASSERT((PHY_INST_GENERATE_DEFAULT_SPEEDS(n) != 0),    \
                 "At least one speed must be configured");      \
    static struct phy_88e1510_config DT_CONST phy_88e1510_config_##n = { \
        .mdio     = DEVICE_DT_GET(DT_INST_BUS(n)),              \
        .phy_addr = DT_INST_REG_ADDR(n),                        \
        .no_reset = DT_INST_PROP(n, no_reset),                  \
        .default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(n),  \
        .led0_func_sel  = DT_ENUM_IDX(DT_DRV_INST(n), led0_function_select), \
        .led1_func_sel  = DT_ENUM_IDX(DT_DRV_INST(n), led1_function_select), \
        INT_GPIO_FIELD(n)                                       \
        RESET_GPIO_FIELDS(n)                                    \
    }

#define PHY_88E1510_DATA(n)                                     \
    static struct phy_88e1510_data phy_88e1510_data_##n = {     \
        .dev = DEVICE_DT_INST_GET(n),                           \
        .cb  = NULL,                                            \
        .lock = Z_MUTEX_INITIALIZER(phy_88e1510_data_##n.lock), \
    }

#define PHY_88E1510_DEVICE(n)                                   \
    PHY_88E1510_CONFIG(n);                                      \
    PHY_88E1510_DATA(n);                                        \
    DEVICE_DT_INST_DEFINE(n, phy_88e1510_init, NULL,            \
                          &phy_88e1510_data_##n, &phy_88e1510_config_##n, \
                          POST_KERNEL, CONFIG_PHY_INIT_PRIORITY, \
                          &phy_88e1510_driver_api)

DT_INST_FOREACH_STATUS_OKAY(PHY_88E1510_DEVICE)
