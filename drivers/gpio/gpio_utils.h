/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Header where utility code can be found for GPIO drivers
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_UTILS_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_UTILS_H_

#define GPIO_PORT_PIN_MASK_FROM_NGPIOS(ngpios)			\
	((gpio_port_pins_t)(((uint64_t)1 << (ngpios)) - 1U))

#define GPIO_PORT_PIN_MASK_FROM_DT_NODE(node_id)		\
	GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_PROP(node_id, ngpios))

#define GPIO_PORT_PIN_MASK_FROM_DT_INST(inst)			\
	GPIO_PORT_PIN_MASK_FROM_NGPIOS(DT_INST_PROP(inst, ngpios))

/**
 * @brief Generic function to insert or remove a callback from a callback list
 *
 * @param callbacks A pointer to the original list of callbacks (can be NULL)
 * @param callback A pointer of the callback to insert or remove from the list
 * @param set A boolean indicating insertion or removal of the callback
 *
 * @return 0 on success, negative errno otherwise.
 */
static inline int gpio_manage_callback(sys_slist_t *callbacks,
					struct gpio_callback *callback,
					bool set)
{
	__ASSERT(callback, "No callback!");
	__ASSERT(callback->handler, "No callback handler!");

	if (!sys_slist_is_empty(callbacks)) {
		if (!sys_slist_find_and_remove(callbacks, &callback->node)) {
			if (!set) {
				return -EINVAL;
			}
		}
	}

	if (set) {
		sys_slist_prepend(callbacks, &callback->node);
	}

	return 0;
}

/**
 * @brief Generic function to go through and fire callback from a callback list
 *
 * @param list A pointer on the gpio callback list
 * @param port A pointer on the gpio driver instance
 * @param pins The actual pin mask that triggered the interrupt
 */
#if defined(_MSC_VER)                       /* #CUSTOM@NDRS, work around for "__typeof__" in gcc */
#define Z_GENLIST_CONTAINER_GPIO_CB(__ln, __cn, __n)                \
        ((__ln) ? CONTAINER_OF((__ln), struct gpio_callback, __n) : NULL)

#define Z_GENLIST_PEEK_NEXT_CONTAINER_GPIO_CB(__lname, __cn, __n)   \
        ((__cn) ? Z_GENLIST_CONTAINER_GPIO_CB(                      \
                  sys_ ## __lname ## _peek_next(&((__cn)->__n)),    \
                  __cn, __n) : NULL)

#define Z_GENLIST_PEEK_HEAD_CONTAINER_GPIO_CB(__lname, __l, __cn, __n)      \
        Z_GENLIST_CONTAINER_GPIO_CB(sys_ ## __lname ## _peek_head(__l), __cn, __n)

#define Z_GENLIST_FOR_EACH_CONTAINER_SAFE_GPIO_CB(__lname, __l, __cn, __cns, __n)   \
    for (__cn = Z_GENLIST_PEEK_HEAD_CONTAINER_GPIO_CB(__lname, __l, __cn, __n),     \
         __cns = Z_GENLIST_PEEK_NEXT_CONTAINER_GPIO_CB(__lname, __cn, __n); \
         __cn != NULL; __cn = __cns,                \
         __cns = Z_GENLIST_PEEK_NEXT_CONTAINER_GPIO_CB(__lname, __cn, __n))

#define SYS_SLIST_FOR_EACH_CONTAINER_SAFE_GPIO_CB(__sl, __cn, __cns, __n)   \
        Z_GENLIST_FOR_EACH_CONTAINER_SAFE_GPIO_CB(slist, __sl, __cn, __cns, __n)


static inline void gpio_fire_callbacks(sys_slist_t* list,
                                       const struct device* port,
                                       uint32_t pins) {
    struct gpio_callback* cb;
    struct gpio_callback* tmp;

    SYS_SLIST_FOR_EACH_CONTAINER_SAFE_GPIO_CB(list, cb, tmp, node) {
        if (cb->pin_mask & pins) {
            __ASSERT(cb->handler, "No callback handler!");
            cb->handler(port, cb, cb->pin_mask & pins);
        }
    }
}
#else
static inline void gpio_fire_callbacks(sys_slist_t *list,
                                       const struct device *port,
                                       uint32_t pins) {
	struct gpio_callback *cb, *tmp;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(list, cb, tmp, node) {
		if (cb->pin_mask & pins) {
			__ASSERT(cb->handler, "No callback handler!");
			cb->handler(port, cb, cb->pin_mask & pins);
		}
	}
}
#endif

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_UTILS_H_ */
