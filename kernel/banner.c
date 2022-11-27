/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <version.h>

#define CLI_INFO_BBC_MICRO_OWL_LOGO_2   "■　■　■　■　■　■　■　■　■\n"   \
                                        "　■　　　　　■　■　　　　　■　\n"   \
                                        "■　　　■　　　■　　　■　　　■\n"   \
                                        "　　　■　■　　　　　■　■　　　\n"   \
                                        "■　　　■　　　　　　　■　　　■\n"   \
                                        "　■　　　　　■　■　　　　　■　\n"   \
                                        "■　■　　　　　■　　　　　■　■\n"   \
                                        "　■　■　　　　　　　　　■　　　\n"   \
                                        "■　■　■　■　■　■　■　　　■\n"   \
                                        "　■　■　■　■　　　　　　　　　\n"   \
                                        "■　■　■　■　■　　　　　　　■\n"   \
                                        "　■　■　■　■　　　　　　　　　\n"   \
                                        "　　■　■　■　■　　　　　　　■\n"   \
                                        "　　　■　■　■　■　　　　　　　\n"   \
                                        "　　　　■　■　■　■　　　　　■\n"   \
                                        "　　　　　■　■　■　■　　　　　\n"   \
                                        "　　　　　　■　■　■　■　　　■\n"   \
                                        "　　　　　　　■　　　■　■　　　\n"   \
                                        "　　　　　　■　　　■　　　■　■\n"   \
                                        "　■　■　■　■　■　■　■　■　\n"   \
                                        "　　　　　　　　　　　　　　　　■\n"

#if defined(CONFIG_BOOT_DELAY) && (CONFIG_BOOT_DELAY > 0)
#define DELAY_STR STRINGIFY(CONFIG_BOOT_DELAY)
#define BANNER_POSTFIX " (delayed boot " DELAY_STR "ms)"
#else
#define BANNER_POSTFIX ""
#endif

#ifdef BUILD_VERSION
#define BANNER_VERSION STRINGIFY(BUILD_VERSION)
#else
#define BANNER_VERSION KERNEL_VERSION_STRING
#endif

void boot_banner(void)
{
#if defined(CONFIG_BOOT_DELAY) && (CONFIG_BOOT_DELAY > 0)
	printk("***** delaying boot " DELAY_STR "ms (per build configuration) *****\n");
	k_busy_wait(CONFIG_BOOT_DELAY * USEC_PER_MSEC);
#endif /* defined(CONFIG_BOOT_DELAY) && (CONFIG_BOOT_DELAY > 0) */

#if CONFIG_BOOT_BANNER
	printk("*** Booting Zephyr OS build " BANNER_VERSION BANNER_POSTFIX " ***\n");
    printk(CLI_INFO_BBC_MICRO_OWL_LOGO_2);
#endif /* CONFIG_BOOT_BANNER */
}
