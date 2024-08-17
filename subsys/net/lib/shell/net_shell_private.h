/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/shell/shell.h>
#include <zephyr/net/net_ip.h>

#define PR(fmt, ...)            \
    shell_fprintf_normal(sh, fmt, ##__VA_ARGS__)

#define PR_SHELL(sh, fmt, ...)  \
    shell_fprintf_normal(sh, fmt, ##__VA_ARGS__)

#define PR_ERROR(fmt, ...)      \
    shell_fprintf_error(sh, fmt, ##__VA_ARGS__)

#define PR_INFO(fmt, ...)       \
    shell_fprintf_info(sh, fmt, ##__VA_ARGS__)

#define PR_WARNING(fmt, ...)    \
    shell_fprintf_warn(sh, fmt, ##__VA_ARGS__)

#include "net_private.h"
#include "../ip/ipv6.h"

struct net_shell_user_data {
    const struct shell* sh;
    void*               user_data;
};

#if !defined(NET_VLAN_MAX_COUNT)
#define MAX_IFACE_COUNT NET_IF_MAX_CONFIGS
#else
#define MAX_IFACE_COUNT NET_VLAN_MAX_COUNT
#endif

#if defined(CONFIG_NET_IPV6) && !defined(CONFIG_NET_IPV4)
#define ADDR_LEN NET_IPV6_ADDR_LEN
#elif defined(CONFIG_NET_IPV4) && !defined(CONFIG_NET_IPV6)
#define ADDR_LEN NET_IPV4_ADDR_LEN
#else
#define ADDR_LEN NET_IPV6_ADDR_LEN
#endif

#if defined(CONFIG_NET_SHELL_DYN_CMD_COMPLETION)
#define IFACE_DYN_CMD &iface_index
#else
#define IFACE_DYN_CMD NULL
#endif /* CONFIG_NET_SHELL_DYN_CMD_COMPLETION */

char const* addrtype2str(enum net_addr_type addr_type);
char const* addrstate2str(enum net_addr_state addr_state);
void get_addresses(struct net_context const* context,
                   char addr_local[], int local_len,
                   char addr_remote[], int remote_len);
void events_enable(void);
int get_iface_idx(const struct shell* sh, char* index_str);
char const* iface2str(struct net_if* iface, char const** extra);
void ipv6_frag_cb(struct net_ipv6_reassembly* reass, void* user_data);
