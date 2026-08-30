/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>

#include <cmsis_core.h>
#include <OsIf.h>

#ifdef CONFIG_XIP
/*
 * Emit IVT only for standalone XIP or MCUboot itself
 * But not for the cases where Zephyr Image is loaded by MCUboot
 */
#if !defined(CONFIG_BOOTLOADER_MCUBOOT) || defined(CONFIG_MCUBOOT)
/* Image Vector Table structure definition for S32K3XX
 * @see 32.5 Image vector table
 */
struct ivt {
    uint32_t header;                        /* Image vector table marker */
    uint32_t boot_configure;                /* Boot configuration word (BCW) */
    const uint32_t  reserved_1;
    uint32_t const* cm7_0_start_address;
    const uint32_t  reserved_2;
    uint32_t const* cm7_1_start_address;
    const uint32_t  reserved_3;
    uint32_t const* cm7_2_start_address;
    const uint32_t  reserved_4;
    uint32_t const* lc_configure;
    uint32_t const* cm7_3_start_address;
    uint32_t const* hse_fw_header_start_address;
    const uint32_t  reserved_5[2];
    uint32_t const* xrdc_mdac_configure;    /* XRDC input MDAC configuration Data */
    uint32_t const* reserved_6;
    uint32_t const* recovery_app_start_address; /* Start Address of Application Core for Recovery Application */
    uint32_t const* length_recovery_app;    /* Length of Application */
    const uint8_t   reserved_7[156];
    const uint8_t   random_iv[12];          /* Random IV for GMAC calculation of IVT */
    const uint8_t   gmac[16];               /* GMAC of the IVT. Reserved for Unsecure BAF */
};

#define IVT_MAGIC_MARKER        0x5AA55AA5

#define BCW_CM7_0_ENABLE        BIT(0)
#define BCW_CM7_1_ENABLE        BIT(1)
#define BCW_CM7_2_ENABLE        BIT(2)
#define BCW_BOOT_SEQ_SECURE     BIT(3)
#define BCW_APP_SWT0_INIT       BIT(5)
#define BCW_CM7_3_ENABLE        BIT(8)

extern char _vector_start[];

/*
 * Attribute 'used' forces the compiler to emit ivt_header
 * even if nothing references it.
 * IVT for SoC S32K344, the minimal boot configuration is:
 * - Watchdog (SWT0) is disabled (default value).
 * - Non-Secure Boot is used (default value).
 * - Ungate clock for Cortex-M7_0 after boot.
 * - Application start address of Cortex-M7_0 is application's vector table.
 */
const struct ivt ivt_header __attribute__((section(".ivt_header"), used)) = {
    .header              = IVT_MAGIC_MARKER,
    .boot_configure      = BCW_CM7_0_ENABLE,
    .cm7_0_start_address = (void const*)_vector_start,
    .cm7_1_start_address = NULL,
    .cm7_2_start_address = NULL,
    .lc_configure        = NULL,
    .cm7_3_start_address = NULL,
    .hse_fw_header_start_address = NULL,
};
#endif
#endif /* CONFIG_XIP */

void soc_early_init_hook(void) {
    sys_cache_instr_enable();
    sys_cache_data_enable();

    OsIf_Init(NULL);
}
