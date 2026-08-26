/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_s32_hse_entropy

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <Hse_Ip.h>

#define LOG_LEVEL CONFIG_ENTROPY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(entropy_nxp_s32_hse_trng);

#define ENTROPY_NXP_S32_HSE_SERVICE_TIMEOUT_TICKS 10000000
#define ENTROPY_NXP_S32_HSE_INIT_TIMEOUT_MS 10000
#define ENTROPY_NXP_S32_HSE_CHUNK_SIZE 64

/* Same MU-instance resolution trick as crypto_nxp_s32_hse.c: token-paste the devicetree
 * instance's register address against IP_MU<n>__MUB_BASE for every n in
 * [0, HSE_IP_NUM_OF_MU_INSTANCES) and keep whichever one matches.
 */
#define ENTROPY_NXP_S32_HSE_MU_INSTANCE_CHECK(indx, n)                                            \
	((DT_INST_REG_ADDR(n) == IP_MU##indx##__MUB_BASE) ? indx : 0)

#define ENTROPY_NXP_S32_HSE_MU_GET_INSTANCE(n)                                                    \
	LISTIFY(__DEBRACKET HSE_IP_NUM_OF_MU_INSTANCES,                                            \
		ENTROPY_NXP_S32_HSE_MU_INSTANCE_CHECK, (|), n)

struct entropy_nxp_s32_hse_data {
	Hse_Ip_MuStateType mu_state;
	struct k_mutex lock;
	Hse_Ip_ReqType req_type;
	hseSrvDescriptor_t srv_desc;
	uint8_t channel;
};

struct entropy_nxp_s32_hse_config {
	uint8_t mu_instance;
};

/* HSE writes the random bytes back through this host-visible staging buffer rather than directly
 * into the caller's buffer, mirroring crypto_nxp_s32_hse.c's out_buff - the caller's buffer isn't
 * guaranteed to meet the HSE core's memory attributes.
 */
static __nocache uint8_t entropy_nxp_s32_hse_chunk[ENTROPY_NXP_S32_HSE_CHUNK_SIZE];

static int entropy_nxp_s32_hse_get_entropy(const struct device *dev, uint8_t *buffer,
					    uint16_t length)
{
	const struct entropy_nxp_s32_hse_config *config = dev->config;
	struct entropy_nxp_s32_hse_data *data = dev->data;
	hseGetRandomNumSrv_t *rng_serv = &(data->srv_desc.hseSrv.getRandomNumReq);
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	while (length > 0) {
		uint16_t chunk = MIN(length, ENTROPY_NXP_S32_HSE_CHUNK_SIZE);

		rng_serv->rngClass = HSE_RNG_CLASS_DRG3;
		rng_serv->randomNumLength = chunk;
		rng_serv->pRandomNum = HSE_PTR_TO_HOST_ADDR(entropy_nxp_s32_hse_chunk);

		if (Hse_Ip_ServiceRequest(config->mu_instance, data->channel,
					  &data->req_type, &data->srv_desc) != HSE_SRV_RSP_OK) {
			ret = -EIO;
			break;
		}

		memcpy(buffer, entropy_nxp_s32_hse_chunk, chunk);
		buffer += chunk;
		length -= chunk;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static int entropy_nxp_s32_hse_init(const struct device *dev)
{
	const struct entropy_nxp_s32_hse_config *config = dev->config;
	struct entropy_nxp_s32_hse_data *data = dev->data;
	hseStatus_t status;
	k_timeout_t timeout = K_MSEC(ENTROPY_NXP_S32_HSE_INIT_TIMEOUT_MS);
	int64_t start_time = k_uptime_ticks();

	do {
		status = Hse_Ip_GetHseStatus(config->mu_instance);
	} while (!(status & (HSE_STATUS_INIT_OK | HSE_STATUS_INSTALL_OK)) &&
		 (k_uptime_ticks() - start_time < timeout.ticks));

	if (!(status & HSE_STATUS_INIT_OK) || !(status & HSE_STATUS_INSTALL_OK)) {
		LOG_ERR("HSE initialization has not been completed or MU%d is not activated",
			config->mu_instance);
		return -EIO;
	}

	if (Hse_Ip_Init(config->mu_instance, &data->mu_state) != HSE_IP_STATUS_SUCCESS) {
		LOG_ERR("Failed to initialize MU%d", config->mu_instance);
		return -EIO;
	}

	data->channel = Hse_Ip_GetFreeChannel(config->mu_instance);
	if (data->channel == HSE_IP_INVALID_MU_CHANNEL_U8) {
		LOG_ERR("No free HSE channel on MU%d", config->mu_instance);
		return -EIO;
	}

	data->req_type.eReqType = HSE_IP_REQTYPE_SYNC;
	data->req_type.u32Timeout = ENTROPY_NXP_S32_HSE_SERVICE_TIMEOUT_TICKS;
	data->srv_desc.srvId = HSE_SRV_ID_GET_RANDOM_NUM;

	k_mutex_init(&data->lock);

	return 0;
}

static DEVICE_API(entropy, entropy_nxp_s32_hse_api) = {
	.get_entropy = entropy_nxp_s32_hse_get_entropy,
};

#define ENTROPY_NXP_S32_HSE_INIT_DEVICE(n)                                                         \
	static struct entropy_nxp_s32_hse_data entropy_nxp_s32_hse_data_##n;                       \
                                                                                                   \
	static const struct entropy_nxp_s32_hse_config entropy_nxp_s32_hse_config_##n = {          \
		.mu_instance = ENTROPY_NXP_S32_HSE_MU_GET_INSTANCE(n),                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &entropy_nxp_s32_hse_init, NULL,                                  \
			      &entropy_nxp_s32_hse_data_##n, &entropy_nxp_s32_hse_config_##n,      \
			      POST_KERNEL, CONFIG_ENTROPY_INIT_PRIORITY,                           \
			      &entropy_nxp_s32_hse_api);

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_NXP_S32_HSE_INIT_DEVICE)
