/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "airoc_whd_hal_common.h"
#include "airoc_wifi.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                 Function
 ******************************************************/

#define AIROC_WIFI_PIN_SETTLE_MS (100)
#define AIROC_WIFI_BT_PULSE_MS   (100)

/* Drive an optional power-up pin of the module to its active level, if the board declares it */
static int airoc_wifi_assert_pin(const struct gpio_dt_spec *spec, const char *name)
{
	int ret;

	if (spec->port == NULL) {
		return 0;
	}

	if (!device_is_ready(spec->port)) {
		LOG_ERR("Error: failed to configure %s %s pin %d", name, spec->port->name,
			spec->pin);
		return -EIO;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Error %d: failed to drive %s %s pin %d", ret, name, spec->port->name,
			spec->pin);
		return ret;
	}

	LOG_INF("%s driven active", name);
	return 0;
}

int airoc_wifi_power_on(const struct device *dev)
{
#if DT_INST_NODE_HAS_PROP(0, wifi_reg_on_gpios)
	int ret;
	const struct airoc_wifi_config *config = dev->config;

	/* Check WIFI REG_ON gpio instance */
	if (!device_is_ready(config->wifi_reg_on_gpio.port)) {
		LOG_ERR("Error: failed to configure wifi_reg_on %s pin %d",
			config->wifi_reg_on_gpio.port->name, config->wifi_reg_on_gpio.pin);
		return -EIO;
	}

#if defined(AIROC_WIFI_SDIO_DAT2_HOLD)
	/*
	 * Some modules (Ezurio Sona IF513) latch SDIO_DATA2 when WL_REG_ON first rises and need
	 * it high, while the board may have no pull-up on it. Drive it high as a GPIO across that
	 * edge; the SDHC gets the pin back after the power-up delay below.
	 */
	if (!device_is_ready(config->wifi_sdio_dat2_gpio.port)) {
		LOG_ERR("Error: failed to configure sdio_dat2 %s pin %d",
			config->wifi_sdio_dat2_gpio.port->name, config->wifi_sdio_dat2_gpio.pin);
		return -EIO;
	}

	ret = gpio_pin_configure_dt(&config->wifi_sdio_dat2_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Error %d: failed to hold sdio_dat2 %s pin %d high", ret,
			config->wifi_sdio_dat2_gpio.port->name, config->wifi_sdio_dat2_gpio.pin);
		return ret;
	}
#endif /* AIROC_WIFI_SDIO_DAT2_HOLD */

	/*
	 * Infineon's reference BSP has the wake pins low from boot, before WL_REG_ON rises. Do the
	 * same here if the board declares the pin.
	 */
	ret = airoc_wifi_assert_pin(&config->wifi_dev_wake_gpio, "wl_dev_wake");
	if (ret) {
		return ret;
	}

	if (config->wifi_dev_wake_gpio.port != NULL) {
		/* The reference waits the same after it drives the wake pin low, before WL_REG_ON */
		k_msleep(AIROC_WIFI_PIN_SETTLE_MS);
	}

	/*
	 * Allow CBUCK regulator to discharge, then raise WL_REG_ON. It is never driven low here:
	 * the chip is shared with Bluetooth, which would fail, and Infineon's reference BSP skips
	 * the low phase for the same reason. The module holds the pin low until it is raised.
	 */
	k_msleep(WLAN_CBUCK_DISCHARGE_MS);

	/* WIFI power on */
	ret = gpio_pin_configure_dt(&config->wifi_reg_on_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Error %d: failed to raise wifi_reg_on %s pin %d", ret,
			config->wifi_reg_on_gpio.port->name, config->wifi_reg_on_gpio.pin);
		return ret;
	}
	k_msleep(WLAN_POWER_UP_DELAY_MS);

	/*
	 * The chip only answers on SDIO once BT_REG_ON rises while WL_REG_ON is already up. This is
	 * the pin-level part of the reference's Bluetooth open (its autobaud step), which runs
	 * before the Wi-Fi host driver touches the bus. The Bluetooth firmware download is not
	 * needed. The module holds the pin low until it is raised here.
	 */
	ret = airoc_wifi_assert_pin(&config->wifi_bt_reg_on_gpio, "bt_reg_on");
	if (ret) {
		return ret;
	}

	if (config->wifi_bt_reg_on_gpio.port != NULL) {
		k_msleep(AIROC_WIFI_BT_PULSE_MS);
	}

#if defined(AIROC_WIFI_SDIO_DAT2_HOLD)
	/* WL_REG_ON has risen and the module had time to latch DAT2: give the pin back to the SDHC */
	ret = pinctrl_apply_state(config->sdio_pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Error %d: failed to give sdio_dat2 back to the SDHC", ret);
		return ret;
	}
	LOG_INF("WL_REG_ON raised with SDIO DAT2 held high");
#endif /* AIROC_WIFI_SDIO_DAT2_HOLD */
#endif /* DT_INST_NODE_HAS_PROP(0, reg_on_gpios) */

	return 0;
}

/*
 * Implement WHD memory wrappers
 */

inline void *whd_mem_malloc(size_t size)
{
	return k_malloc(size);
}

inline void *whd_mem_calloc(size_t nitems, size_t size)
{
	return k_calloc(nitems, size);
}

inline void whd_mem_free(void *ptr)
{
	k_free(ptr);
}

inline void whd_mem_memcpy(void *dest, const void *src, size_t len)
{
	memcpy(dest, src, len);
}

inline void whd_mem_memset(void *buf, int val, size_t len)
{
	memset(buf, val, len);
}

#ifdef __cplusplus
} /* extern "C" */
#endif
