/*
 * Copyright (c) 2026 NDR Solution (Thailand) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Flash-API shim wrapping a FRAM device, so flash-only consumers
 * (Settings ZMS/NVS, FLASH_MAP, the flash shell) can use a FRAM chip as
 * their backing store. FRAM needs no erase-before-write, so ".erase()" is
 * emulated by filling the requested range with the device's erase value.
 */

#define DT_DRV_COMPAT infineon_fram_flash

#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/fram.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_infineon_fram_shim, CONFIG_FLASH_LOG_LEVEL);

/* Chunk size for the write-loop used to emulate ".erase()" via fill-writes. */
#define FRAM_SHIM_ERASE_CHUNK   128U

struct flash_infineon_fram_shim_config {
    struct device const* fram_dev;
    struct flash_parameters params;
    struct flash_pages_layout layout;
};

static int flash_infineon_fram_shim_read(struct device const* dev, off_t offset,
                                         void* data, size_t len) {
    struct flash_infineon_fram_shim_config const* config = dev->config;
    int ret;

    ret = fram_read(config->fram_dev, offset, data, len);

    return (ret);
}

static int flash_infineon_fram_shim_write(struct device const* dev, off_t offset,
                                          void const* data, size_t len) {
    struct flash_infineon_fram_shim_config const* config = dev->config;
    int ret;

    ret = fram_write(config->fram_dev, offset, data, len);

    return (ret);
}

static int flash_infineon_fram_shim_erase(struct device const* dev, off_t offset, size_t size) {
    struct flash_infineon_fram_shim_config const* config = dev->config;
    uint8_t erase_buf[FRAM_SHIM_ERASE_CHUNK];
    size_t chunk;
    int ret;

    if (((offset % config->layout.pages_size) != 0U) ||
        ((size % config->layout.pages_size) != 0U)) {
        LOG_WRN("erase offset/size not sector-aligned");
        return (-EINVAL);
    }

    memset(erase_buf, config->params.erase_value, sizeof(erase_buf));

    while (size > 0U) {
        chunk = MIN(size, sizeof(erase_buf));

        ret = fram_write(config->fram_dev, offset, erase_buf, chunk);
        if (ret < 0) {
            LOG_ERR("failed to fill-erase FRAM (err %d)", ret);
            return (ret);
        }

        offset += chunk;
        size   -= chunk;
    }

    return (0);
}

static struct flash_parameters const* flash_infineon_fram_shim_get_parameters(
    struct device const* dev) {
    struct flash_infineon_fram_shim_config const* config = dev->config;

    return (&config->params);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_infineon_fram_shim_page_layout(struct device const* dev,
                                                 struct flash_pages_layout const** layout,
                                                 size_t* layout_size) {
    struct flash_infineon_fram_shim_config const* config = dev->config;

    *layout      = &config->layout;
    *layout_size = 1U;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_infineon_fram_shim_init(struct device const* dev) {
    struct flash_infineon_fram_shim_config const* config = dev->config;

    if (!device_is_ready(config->fram_dev)) {
        LOG_ERR("backing FRAM device not ready");
        return (-ENODEV);
    }

    return (0);
}

static DEVICE_API(flash, flash_infineon_fram_shim_api) = {
    .read  = flash_infineon_fram_shim_read,
    .write = flash_infineon_fram_shim_write,
    .erase = flash_infineon_fram_shim_erase,
    .get_parameters = flash_infineon_fram_shim_get_parameters,

    #if defined(CONFIG_FLASH_PAGE_LAYOUT)
    .page_layout = flash_infineon_fram_shim_page_layout,
    #endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

#define FLASH_INFINEON_FRAM_SHIM_INIT(inst)                             \
    static const struct flash_infineon_fram_shim_config                 \
        flash_infineon_fram_shim_config_##inst = {                      \
        .fram_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, fram)),         \
        .params = {                                                     \
            .write_block_size = DT_INST_PROP(inst, write_block_size),   \
            .caps        = {.no_explicit_erase = false},                \
            .erase_value = 0xFFU,                                       \
        },                                                              \
                                                                        \
        .layout = {                                                     \
            .pages_size  = DT_INST_PROP(inst, erase_block_size),        \
            .pages_count = (DT_PROP(DT_INST_PHANDLE(inst, fram), size) / \
                            DT_INST_PROP(inst, erase_block_size)),      \
        },                                                              \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst, flash_infineon_fram_shim_init, NULL, NULL,      \
                          &flash_infineon_fram_shim_config_##inst, POST_KERNEL, \
                          CONFIG_FLASH_INFINEON_FRAM_SHIM_INIT_PRIORITY,        \
                          &flash_infineon_fram_shim_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_INFINEON_FRAM_SHIM_INIT)
