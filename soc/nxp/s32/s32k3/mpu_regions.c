/*
 * Copyright 2023, 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/arch/arm/mpu/arm_mpu_mem_cfg.h>

#if !defined(CONFIG_XIP)
extern char _rom_attr[];
#endif

#define REGION_PERIPHERAL_BASE_ADDRESS 0x40000000
#define REGION_PERIPHERAL_SIZE         REGION_512M
#define REGION_PPB_BASE_ADDRESS        0xE0000000
#define REGION_PPB_SIZE                REGION_1M

static struct arm_mpu_region mpu_regions[] = {

	/* ERR011573: use first region to prevent speculative access in entire memory space */
	MPU_REGION_ENTRY("UNMAPPED", 0, {REGION_4G | MPU_RASR_XN_Msk | P_NA_U_NA_Msk}),

	/* Keep before CODE region so it can be overlapped by SRAM CODE in non-XIP systems */
	MPU_REGION_ENTRY("SRAM", DT_CHOSEN_SRAM_ADDR, REGION_RAM_ATTR(REGION_SRAM_SIZE)),

#ifdef CONFIG_XIP
	/*
	 * CONFIG_FLASH_SIZE (8144K on S32K358) buckets to REGION_8M, but
	 * CONFIG_FLASH_BASE_ADDRESS (0x00400000) is only 4M-aligned, not the
	 * 8M alignment an 8M RASR region requires. The MPU hardware ignores
	 * the low bits of the base for the region's size, so a single 8M
	 * region here silently protects 0x000000-0x800000 instead of the
	 * intended 0x400000-0xC00000, leaving the top half of flash
	 * unmapped (HardFault on read/exec there). Use two naturally
	 * 4M-aligned regions instead to cover the full physical window.
	 */
	MPU_REGION_ENTRY("FLASH_LO", CONFIG_FLASH_BASE_ADDRESS, REGION_FLASH_ATTR(REGION_4M)),
	MPU_REGION_ENTRY("FLASH_HI", CONFIG_FLASH_BASE_ADDRESS + 0x400000, REGION_FLASH_ATTR(REGION_4M)),
#else
	/* Run from SRAM */
	MPU_REGION_ENTRY("CODE", DT_CHOSEN_SRAM_ADDR, {(uint32_t)_rom_attr}),
#endif

	MPU_REGION_ENTRY("PERIPHERALS", REGION_PERIPHERAL_BASE_ADDRESS,
			 REGION_IO_ATTR(REGION_PERIPHERAL_SIZE)),

	MPU_REGION_ENTRY("PPB", REGION_PPB_BASE_ADDRESS, REGION_PPB_ATTR(REGION_PPB_SIZE)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
