/*
 * #CUSTOM@NDRS
 *
 * Force-included into the TF-M mailbox client (ifx_mtb_mailbox.c) of the CM55 build. That file includes "cybsp.h"
 * for cybsp_mtb_srf_client_context, but on a build that has the AIROC Wi-Fi driver "cybsp.h" resolves to the small
 * stub in drivers/wifi/infineon, which does not declare it. soc_pse84_m55.c defines the object.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NDRS_SRF_CLIENT_H_
#define NDRS_SRF_CLIENT_H_

#include "mtb_srf_ipc_init.h"

extern mtb_srf_ipc_client_context_t cybsp_mtb_srf_client_context;

#endif /* NDRS_SRF_CLIENT_H_ */
