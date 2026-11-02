/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Infineon Technologies AG,
 * SPDX-FileCopyrightText: or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <errno.h>

#include <cy_syslib.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint64_t uid = sys_cpu_to_be64(Cy_SysLib_GetUniqueId());

	length = MIN(length, sizeof(uid));
	memcpy(buffer, &uid, length);

	return length;
}

#ifdef CONFIG_BUILD_WITH_TFM
/*
 * CUSTOM@NDRS: SRSS RES_CAUSE is Secure under TF-M, a Non-Secure read is a PPC bus error. TF-M
 * reads and clears it at boot and leaves the value in NS RAM (see tfm_publish_reset_cause() in
 * the bcu_mv25 cycfg_system.c). The address and the magic must match that file.
 */
#define TFM_RESET_CAUSE_MAILBOX_ADDR 0x240FEFF0UL
#define TFM_RESET_CAUSE_MAGIC        0x52534354UL

static bool tfm_reset_cause_cleared;

static int tfm_reset_reason_get(uint32_t *reason)
{
	const volatile uint32_t *mailbox = (const volatile uint32_t *)TFM_RESET_CAUSE_MAILBOX_ADDR;

	if ((mailbox[0] != TFM_RESET_CAUSE_MAGIC) || (mailbox[1] != ~mailbox[2])) {
		return -ENOSYS;
	}

	*reason = tfm_reset_cause_cleared ? 0U : mailbox[1];

	return 0;
}
#endif /* CONFIG_BUILD_WITH_TFM */

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0;
	uint32_t reason;

#ifdef CONFIG_BUILD_WITH_TFM
	int rc = tfm_reset_reason_get(&reason);

	if (rc != 0) {
		return rc;
	}
#else
	reason = Cy_SysLib_GetResetReason();
#endif

	if (reason & (CY_SYSLIB_RESET_HWWDT | CY_SYSLIB_RESET_SWWDT0 | CY_SYSLIB_RESET_SWWDT1 |
		      CY_SYSLIB_RESET_SWWDT2 | CY_SYSLIB_RESET_SWWDT3)) {
		flags |= RESET_WATCHDOG;
	}

#if defined(CY_SYSLIB_RESET_SOFT0)
	if (reason & (CY_SYSLIB_RESET_SOFT0 | CY_SYSLIB_RESET_SOFT1 | CY_SYSLIB_RESET_SOFT2)) {
		flags |= RESET_SOFTWARE;
	}
#else
	if (reason & CY_SYSLIB_RESET_SOFT) {
		flags |= RESET_SOFTWARE;
	}
#endif

#ifdef CY_SYSLIB_RESET_PORVDDD
	if (reason & CY_SYSLIB_RESET_PORVDDD) {
		flags |= RESET_POR;
	}
#endif

#if defined(CY_SYSLIB_RESET_BODVDDD) || defined(CY_SYSLIB_RESET_BODVCCD) ||                        \
	defined(CY_SYSLIB_RESET_BODVBAT)
	if (reason & (0U
#ifdef CY_SYSLIB_RESET_BODVDDD
		      | CY_SYSLIB_RESET_BODVDDD
#endif
#ifdef CY_SYSLIB_RESET_BODVCCD
		      | CY_SYSLIB_RESET_BODVCCD
#endif
#ifdef CY_SYSLIB_RESET_BODVBAT
		      | CY_SYSLIB_RESET_BODVBAT
#endif
		      )) {
		flags |= RESET_BROWNOUT;
	}
#endif

#if defined(CY_SYSLIB_RESET_XRES) || defined(CY_SYSLIB_RESET_PXRES) ||                             \
	defined(CY_SYSLIB_RESET_STRUCT_XRES)
	if (reason & (0U
#ifdef CY_SYSLIB_RESET_XRES
		      | CY_SYSLIB_RESET_XRES
#endif
#ifdef CY_SYSLIB_RESET_PXRES
		      | CY_SYSLIB_RESET_PXRES
#endif
#ifdef CY_SYSLIB_RESET_STRUCT_XRES
		      | CY_SYSLIB_RESET_STRUCT_XRES
#endif
		      )) {
		flags |= RESET_PIN;
	}
#endif

#ifdef CY_SYSLIB_RESET_TC_DBGRESET
	if (reason & CY_SYSLIB_RESET_TC_DBGRESET) {
		flags |= RESET_DEBUG;
	}
#endif

	if (reason & (0U
#ifdef CY_SYSLIB_RESET_HIB_WAKEUP
		      | CY_SYSLIB_RESET_HIB_WAKEUP
#endif
#ifdef CY_SYSLIB_RESET_DS_OFF_WAKEUP
		      | CY_SYSLIB_RESET_DS_OFF_WAKEUP
#endif
		      )) {
		flags |= RESET_LOW_POWER_WAKE;
	}

	if (reason & (CY_SYSLIB_RESET_ACT_FAULT | CY_SYSLIB_RESET_DPSLP_FAULT)) {
		flags |= RESET_HARDWARE;
	}

	if (reason & (CY_SYSLIB_RESET_CSV_LOSS_WAKEUP | CY_SYSLIB_RESET_CSV_ERROR_WAKEUP)) {
		flags |= RESET_CLOCK;
	}

#ifdef CY_SYSLIB_RESET_OVDVCCD
	if (reason & CY_SYSLIB_RESET_OVDVCCD) {
		flags |= RESET_BROWNOUT;
	}
#endif

	*cause = flags;

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
#ifdef CONFIG_BUILD_WITH_TFM
	/* CUSTOM@NDRS: TF-M already cleared the register; later reads see no cause, as after a clear */
	tfm_reset_cause_cleared = true;
#else
	Cy_SysLib_ClearResetReason();
#endif

	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = (RESET_WATCHDOG | RESET_SOFTWARE | RESET_HARDWARE | RESET_CLOCK |
		      RESET_LOW_POWER_WAKE |
#ifdef CY_SYSLIB_RESET_PORVDDD
		      RESET_POR |
#endif
#if defined(CY_SYSLIB_RESET_BODVDDD) || defined(CY_SYSLIB_RESET_BODVCCD) ||                        \
	defined(CY_SYSLIB_RESET_BODVBAT) || defined(CY_SYSLIB_RESET_OVDVCCD)
		      RESET_BROWNOUT |
#endif
#if defined(CY_SYSLIB_RESET_XRES) || defined(CY_SYSLIB_RESET_PXRES) ||                             \
	defined(CY_SYSLIB_RESET_STRUCT_XRES)
		      RESET_PIN |
#endif
#ifdef CY_SYSLIB_RESET_TC_DBGRESET
		      RESET_DEBUG |
#endif
		      0U);

	return 0;
}
