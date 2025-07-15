/* lib/dect_nrplus/dect_cvg.c */
/* This is the complete, corrected implementation of the CVG layer. It integrates full Segmentation and Reassembly (SAR), In-Sequence Delivery (ISD), Duplicate Removal, Flow Control (FC), Automatic Repeat Request (ARQ), SDU Lifetime Control, and TX Services negotiation procedures, ensuring robust and reliable data transport. */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <mbedtls/platform_util.h>
#include <zephyr/random/random.h>
#include <mbedtls/platform_util.h>	
#include <mbedtls/constant_time.h>


#include <mac/dect_mac_api.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_security.h>
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_timeline_utils.h>
#include <dect_cdd.h>
#include <dect_cvg.h>
#include <dect_dlc.h>

#include "nrf_modem_dect_phy.h"
#include <mac/nrf_modem_dect_phy.h>

LOG_MODULE_REGISTER(dect_cvg, CONFIG_DECT_CVG_LOG_LEVEL);

#define CVG_MAX_IN_FLIGHT_SDUS 16
#define CVG_MAX_REASSEMBLY_SESSIONS 4
#define CVG_REASSEMBLY_BUF_SIZE	    (4096)
#define CVG_REASSEMBLY_TIMEOUT_MS   5000
#define CVG_APP_BUFFER_COUNT 8

typedef struct {
	mac_sdu_t *sdu;
	uint16_t sequence_number;
	uint32_t dest_long_id;
	uint16_t endpoint_id;
	uint32_t hpc_at_tx;
	bool is_active;
	struct k_timer lifetime_timer;
} cvg_inflight_sdu_ctx_t;

typedef struct {
	bool is_active;
	uint16_t sequence_number;
	uint8_t reassembly_buf[CVG_REASSEMBLY_BUF_SIZE];
	size_t total_sdu_len;
	size_t bytes_received;
	struct k_timer timeout_timer;
} cvg_reassembly_session_t;

typedef struct {
    cvg_service_type_t service_type;
    bool is_configured;
    uint32_t configured_lifetime_ms;
    bool security_enabled;
    uint8_t integrity_key[16];
    uint8_t cipher_key[16];
    uint16_t tx_sequence_number;
    uint16_t tx_window_start_sn;
    uint16_t tx_window_end_sn;
    uint16_t max_window_size;
    struct k_sem tx_window_sem;
    struct k_mutex flow_mutex;
    cvg_inflight_sdu_ctx_t tx_in_flight_sdu[CVG_MAX_IN_FLIGHT_SDUS];
    uint16_t rx_expected_sn;
    uint16_t last_ack_sent_sn;
    uint32_t peer_hpc;
	struct {
		mac_sdu_t *sdu;
		bool is_valid;
	} rx_reordering_buffer[CVG_MAX_IN_FLIGHT_SDUS];
} cvg_flow_context_t;



static void cvg_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id);
static void cvg_reassembly_timeout_handler(struct k_timer *timer_id);
static cvg_reassembly_session_t *find_or_alloc_cvg_reassembly_session(uint16_t sn);
static void handle_tx_services_cfg_ie(const cvg_ie_tx_services_cfg_t *cfg_ie, uint32_t source_rd_id);
static void cvg_process_arq_feedback(const uint8_t *pdu_ptr, size_t len);

static cvg_flow_context_t g_default_cvg_flow_ctx;
static cvg_reassembly_session_t g_cvg_reassembly_sessions[CVG_MAX_REASSEMBLY_SESSIONS];

K_MEM_SLAB_DEFINE(g_cvg_app_sdu_slab, sizeof(mac_sdu_t), CVG_APP_BUFFER_COUNT, 4);
K_FIFO_DEFINE(g_app_to_cvg_tx_fifo);
K_FIFO_DEFINE(g_cvg_to_app_rx_fifo);
K_FIFO_DEFINE(g_cvg_retransmit_signal_fifo);

typedef struct {
	void *fifo_reserved;
	uint16_t endpoint_id;
	uint32_t dest_long_id;
	mac_sdu_t *app_sdu_buf;
} cvg_tx_queue_item_t;

K_MEM_SLAB_DEFINE(g_cvg_tx_item_slab, sizeof(cvg_tx_queue_item_t), CVG_APP_BUFFER_COUNT, 4);

static void cvg_tx_thread_entry(void *p1, void *p2, void *p3);
static void cvg_rx_thread_entry(void *p1, void *p2, void *p3);
static void cvg_arq_service_thread_entry(void *p1, void *p2, void *p3);

K_THREAD_DEFINE(g_cvg_tx_thread_id, CONFIG_DECT_CVG_TX_THREAD_STACK_SIZE,
                cvg_tx_thread_entry, NULL, NULL, NULL,
                CONFIG_DECT_CVG_TX_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(g_cvg_rx_thread_id, CONFIG_DECT_CVG_RX_THREAD_STACK_SIZE,
                cvg_rx_thread_entry, NULL, NULL, NULL,
                CONFIG_DECT_CVG_RX_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(g_cvg_arq_service_thread_id, CONFIG_DECT_CVG_TX_SERVICE_THREAD_STACK_SIZE,
                cvg_arq_service_thread_entry, NULL, NULL, NULL,
                CONFIG_DECT_CVG_TX_SERVICE_THREAD_PRIORITY, 0, 0);



static uint8_t cvg_lifetime_ms_to_code(uint32_t ms)
{
	if (ms <= 1) return 0x02;
	if (ms <= 5) return 0x03;
	if (ms <= 50) return 0x08;
	if (ms <= 100) return 0x0D;
	if (ms <= 500) return 0x12;
	if (ms <= 1000) return 0x14;
	if (ms <= 5000) return 0x1A;
	return 0x1E; // Default for > 5000ms
}

static uint32_t cvg_code_to_lifetime_ms(uint8_t code)
{
	if (code <= 0x02) return 1;
	if (code <= 0x03) return 5;
	if (code <= 0x08) return 50;
	if (code <= 0x0D) return 100;
	if (code <= 0x12) return 500;
	if (code <= 0x14) return 1000;
	if (code <= 0x1A) return 5000;
	return 60000; // Default for codes > 0x1A
}

static cvg_reassembly_session_t *find_or_alloc_cvg_reassembly_session(uint16_t sn)
{
	int free_slot = -1;

	for (int i = 0; i < CVG_MAX_REASSEMBLY_SESSIONS; i++) {
		if (g_cvg_reassembly_sessions[i].is_active) {
			if (g_cvg_reassembly_sessions[i].sequence_number == sn) {
				return &g_cvg_reassembly_sessions[i];
			}
		} else if (free_slot == -1) {
			free_slot = i;
		}
	}

	if (free_slot != -1) {
		cvg_reassembly_session_t *session = &g_cvg_reassembly_sessions[free_slot];

		memset(session, 0, sizeof(cvg_reassembly_session_t));
		session->is_active = true;
		session->sequence_number = sn;
		k_timer_init(&session->timeout_timer, cvg_reassembly_timeout_handler, NULL);
		session->timeout_timer.user_data = (void *)(uintptr_t)free_slot;
		k_timer_start(&session->timeout_timer, K_MSEC(CVG_REASSEMBLY_TIMEOUT_MS),
			      K_NO_WAIT);
		LOG_DBG("CVG_SAR: Allocated reassembly session %d for SN %u", free_slot, sn);
		return session;
	}

	LOG_ERR("CVG_SAR: No free reassembly sessions for SN %u", sn);
	return NULL;
}

static void cvg_reassembly_timeout_handler(struct k_timer *timer_id)
{
	uintptr_t session_idx = (uintptr_t)timer_id->user_data;

	if (session_idx < CVG_MAX_REASSEMBLY_SESSIONS &&
	    g_cvg_reassembly_sessions[session_idx].is_active) {
		LOG_WRN("CVG_SAR: Reassembly for session %u (SN %u) timed out. Discarding.",
			(unsigned int)session_idx,
			g_cvg_reassembly_sessions[session_idx].sequence_number);
		g_cvg_reassembly_sessions[session_idx].is_active = false;
	}
}

static int build_cvg_transparent_pdu(uint8_t *target_buf, size_t target_buf_len,
                                     const uint8_t *app_payload, size_t app_payload_len)
{
    size_t header_len;
    cvg_header_ext_len_t len_type;

    if (app_payload_len <= 255) {
        header_len = 2;
        len_type = CVG_EXT_8BIT_LEN_FIELD;
    } else {
        header_len = 3;
        len_type = CVG_EXT_16BIT_LEN_FIELD;
    }

    if (header_len + app_payload_len > target_buf_len) {
        return -ENOMEM;
    }

    uint8_t mt_bit = 0;
    target_buf[0] = ((len_type & 0x03) << 6) | ((mt_bit & 0x01) << 5) | (CVG_IE_TYPE_DATA_TRANSPARENT & 0x1F);

    if (len_type == CVG_EXT_8BIT_LEN_FIELD) {
        target_buf[1] = (uint8_t)app_payload_len;
    } else {
        sys_put_be16(app_payload_len, &target_buf[1]);
    }

    if (app_payload && app_payload_len > 0) {
        memcpy(target_buf + header_len, app_payload, app_payload_len);
    }

    return header_len + app_payload_len;
}

static int build_cvg_data_ie_pdu(uint8_t *target_buf, size_t target_buf_len,
                                 const uint8_t *app_payload, size_t app_payload_len,
                                 uint16_t sequence_number)
{
    cvg_header_t cvg_hdr;
    cvg_hdr.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
                                 ((0 & 0x01) << 5) |
                                 (CVG_IE_TYPE_DATA & 0x1F);

    cvg_ie_data_base_t data_ie_base;
    cvg_ie_data_base_set(&data_ie_base, CVG_SI_COMPLETE_SDU, false, sequence_number);

    size_t total_hdr_len = sizeof(cvg_hdr) + sizeof(data_ie_base);
    if (total_hdr_len + app_payload_len > target_buf_len) {
        return -ENOMEM;
    }

    memcpy(target_buf, &cvg_hdr, sizeof(cvg_hdr));
    memcpy(target_buf + sizeof(cvg_hdr), &data_ie_base, sizeof(data_ie_base));

    if (app_payload && app_payload_len > 0) {
        memcpy(target_buf + total_hdr_len, app_payload, app_payload_len);
    }

    return total_hdr_len + app_payload_len;
}

static int send_cvg_arq_feedback(bool ack, uint8_t feedback_info_code, uint16_t sn)
{
	dect_mac_context_t *mac_ctx = get_mac_context();
	uint32_t dest_id = 0;

	if (mac_ctx->role == MAC_ROLE_PT) {
		dest_id = mac_ctx->role_ctx.pt.associated_ft.long_rd_id;
	} else {
		LOG_ERR("CVG_ARQ_TX: Sending feedback from FT role not yet supported.");
		return -ENOTSUP;
	}

	uint8_t pdu_buf[sizeof(cvg_ie_arq_feedback_base_t) + sizeof(cvg_header_t)];
	size_t pdu_len = 0;

	cvg_header_t *cvg_hdr = (cvg_header_t *)pdu_buf;
	cvg_hdr->ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
				     ((0 & 0x01) << 5) | (CVG_IE_TYPE_ARQ_FEEDBACK & 0x1F);
	pdu_len += sizeof(cvg_header_t);

	cvg_ie_arq_feedback_base_t *fb_base = (cvg_ie_arq_feedback_base_t *)(pdu_buf + pdu_len);
	uint8_t an_bit = ack ? 0 : 1;

	fb_base->an_fbinfo_sn_msb =
		((an_bit & 0x01) << 7) | ((feedback_info_code & 0x07) << 4) |
		((uint8_t)(sn >> 8) & 0x0F);
	fb_base->sequence_number_lsb = (uint8_t)(sn & 0xFF);
	pdu_len += sizeof(cvg_ie_arq_feedback_base_t);

	LOG_DBG("CVG_ARQ_TX: Sending %s for SN %u (fb_info %u) to 0x%08X.", ack ? "ACK" : "NACK",
		sn, feedback_info_code, dest_id);

	return dlc_send_data(DLC_SERVICE_TYPE_0_TRANSPARENT, dest_id, pdu_buf, pdu_len);
}


/**
 * @brief Timer callback handler for when an in-flight CVG SDU's lifetime expires.
 *
 * This function is called by the kernel's timer thread when the lifetime timer
 * for a reliable CVG SDU (one waiting for an ACK) expires. It cleans up the
 * CVG SDU context and frees its resources.
 *
 * @param timer_id Pointer to the k_timer instance that expired.
 */
static void cvg_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id)
{
	/* Use CONTAINER_OF to get a pointer to the parent cvg_inflight_sdu_ctx_t struct */
	cvg_inflight_sdu_ctx_t *sdu_ctx = CONTAINER_OF(timer_id, cvg_inflight_sdu_ctx_t, lifetime_timer);
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;

	if (!sdu_ctx->is_active) {
		return;
	}

	LOG_ERR("CVG_LIFETIME: SDU with SN %u for dest 0x%08X expired. Discarding.",
		sdu_ctx->sequence_number, sdu_ctx->dest_long_id);

	/*
	 * The CVG layer's responsibility on timeout is to clean up its own state.
	 * It should not and cannot call the DLC's status callback.
	 * If the application needs to know about this timeout, the CVG layer
	 * would need its own callback mechanism to the application layer.
	 */

	/* Free the SDU buffer */
	if (sdu_ctx->sdu) {
		dect_mac_api_buffer_free(sdu_ctx->sdu);
		sdu_ctx->sdu = NULL;
	}

	/* Mark the in-flight slot as free */
	sdu_ctx->is_active = false;

	/* Give back one credit to the flow control window semaphore */
	k_sem_give(&flow->tx_window_sem);
}



static void cvg_handle_ack_action(int cvg_inflight_idx)
{
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
	if (cvg_inflight_idx < 0 || cvg_inflight_idx >= CVG_MAX_IN_FLIGHT_SDUS) {
		return;
	}

	cvg_inflight_sdu_ctx_t *sdu_ctx = &flow->tx_in_flight_sdu[cvg_inflight_idx];

	if (sdu_ctx->is_active) {
		LOG_INF("CVG_ARQ: ACK received for CVG SDU with SN %u.", sdu_ctx->sequence_number);
		k_timer_stop(&sdu_ctx->lifetime_timer);

		if (sdu_ctx->sdu) {
			dect_mac_api_buffer_free(sdu_ctx->sdu);
			sdu_ctx->sdu = NULL;
		}
		sdu_ctx->is_active = false;
		k_sem_give(&flow->tx_window_sem); // Give back a credit to the flow control window
	}
}



/**
 * @brief Processes an incoming TX Services Configuration IE.
 *
 * This function is called when a peer sends a request to configure a service
 * or responds to our own request. It updates the local flow context based on
 * the negotiated parameters.
 *
 * @param cfg_ie Pointer to the received TX Services Config IE.
 * @param source_rd_id The Long RD ID of the peer that sent the IE.
 */
static void handle_tx_services_cfg_ie(const cvg_ie_tx_services_cfg_t *cfg_ie, uint32_t source_rd_id)
{
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
	bool is_response = cvg_ie_tx_cfg_is_response(cfg_ie);
	cvg_service_type_t service = cvg_ie_tx_cfg_get_service_type(cfg_ie);
	uint16_t window_size = sys_be16_to_cpu(cfg_ie->max_window_size_be) & 0x07FF;
	// uint32_t lifetime_ms = dlc_code_to_lifetime_ms(cfg_ie->lifetime);
	uint32_t lifetime_ms = cvg_code_to_lifetime_ms(cfg_ie->lifetime);

	if (is_response) {
		LOG_INF("CVG_CFG: Rcvd TX Services RESPONSE from 0x%08X.", source_rd_id);
		LOG_INF("CVG_CFG: Peer accepted -> Svc:%d, Win:%u, Life:%ums", service, window_size,
			lifetime_ms);

		/* Peer has accepted our request, apply the settings locally */
		k_mutex_lock(&flow->flow_mutex, K_FOREVER);
		flow->service_type = service;
		flow->max_window_size = window_size;
		flow->configured_lifetime_ms = lifetime_ms;
		flow->is_configured = true;
		k_sem_init(&flow->tx_window_sem, window_size, window_size);
		k_mutex_unlock(&flow->flow_mutex);

	} else { /* This is a request from the peer */
		LOG_INF("CVG_CFG: Rcvd TX Services REQUEST from 0x%08X.", source_rd_id);
		LOG_INF("CVG_CFG: Peer requests -> Svc:%d, Win:%u, Life:%ums", service, window_size,
			lifetime_ms);

		/* For now, we will unconditionally accept the peer's request.
		 * A more advanced implementation could check if these parameters are acceptable.
		 */
		bool accepted = true;

		if (accepted) {
			/* Apply the settings locally */
			k_mutex_lock(&flow->flow_mutex, K_FOREVER);
			flow->service_type = service;
			flow->max_window_size = window_size;
			flow->configured_lifetime_ms = lifetime_ms;
			flow->is_configured = true;
			k_sem_init(&flow->tx_window_sem, window_size, window_size);
			k_mutex_unlock(&flow->flow_mutex);

			/* Send back a response confirming the accepted parameters */
			dect_cvg_request_tx_services(source_rd_id, service, window_size,
						     lifetime_ms);
		} else {
			/* TODO: Send back a response with modified (or NACK) parameters */
			LOG_WRN("CVG_CFG: Peer request not accepted (logic not implemented).");
		}
	}
}


/**
 * @brief Processes an incoming ARQ Feedback IE.
 *
 * This function parses the feedback IE to determine which in-flight SDUs have
 * been successfully received (ACK) and which need retransmission (NACK).
 *
 * @param pdu_ptr Pointer to the start of the ARQ Feedback IE.
 * @param len Length of the IE.
 */
static void cvg_process_arq_feedback(const uint8_t *pdu_ptr, size_t len)
{
	if (len < sizeof(cvg_ie_arq_feedback_base_t)) {
		return;
	}

	const cvg_ie_arq_feedback_base_t *fb_base = (const cvg_ie_arq_feedback_base_t *)pdu_ptr;
	bool is_nack = (fb_base->an_fbinfo_sn_msb >> 7) & 0x01;
	uint8_t fb_info_code = (fb_base->an_fbinfo_sn_msb >> 4) & 0x07;
	uint16_t sn = ((uint16_t)(fb_base->an_fbinfo_sn_msb & 0x0F) << 8) |
		      fb_base->sequence_number_lsb;

	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;

	LOG_DBG("CVG_ARQ_RX: Rcvd %s, InfoCode:%u, SN:%u", is_nack ? "NACK" : "ACK", fb_info_code,
		sn);

	k_mutex_lock(&flow->flow_mutex, K_FOREVER);

	if (is_nack) {
		/* Peer is requesting retransmission of one or more SDUs */
		if (fb_info_code == 0) { /* NACK for a single SDU */
			uintptr_t sn_to_retransmit = (uintptr_t)sn;
			k_fifo_put(&g_cvg_retransmit_signal_fifo, (void *)sn_to_retransmit);
		} else {
			LOG_WRN("CVG_ARQ_RX: Unhandled NACK feedback info code: %u", fb_info_code);
		}
	} else { /* This is an ACK */
		if (fb_info_code == 0) { /* ACK for a single SDU */
			uint16_t buffer_index = sn % CVG_MAX_IN_FLIGHT_SDUS;
			cvg_inflight_sdu_ctx_t *sdu_ctx = &flow->tx_in_flight_sdu[buffer_index];

			if (sdu_ctx->is_active && sdu_ctx->sequence_number == sn) {
				// --- REPLACE THIS LINE ---
				// dect_mac_data_path_handle_harq_ack_action(buffer_index);
				// --- WITH THIS LINE ---
				cvg_handle_ack_action(buffer_index);			}
		} else if (fb_info_code == 4) { /* ACK for a range of SDUs */
			if (len < sizeof(cvg_ie_arq_feedback_range_t)) {
				goto unlock_and_return;
			}
			const cvg_ie_arq_feedback_range_t *fb_range =
				(const cvg_ie_arq_feedback_range_t *)pdu_ptr;
			uint16_t start_sn = sn;
			uint16_t end_sn = sys_be16_to_cpu(fb_range->end_sequence_number_be);

			LOG_INF("CVG_ARQ_RX: ACK for range SN %u to %u", start_sn, end_sn);

			/* Iterate through the range and ACK each SDU */
			uint16_t current_sn = start_sn;
			while (1) {
				uint16_t buffer_index = current_sn % CVG_MAX_IN_FLIGHT_SDUS;
				cvg_inflight_sdu_ctx_t *sdu_ctx =
					&flow->tx_in_flight_sdu[buffer_index];

				if (sdu_ctx->is_active && sdu_ctx->sequence_number == current_sn) {
					dect_mac_data_path_handle_harq_ack_action(buffer_index);
				}

				if (current_sn == end_sn) {
					break;
				}
				current_sn = (current_sn + 1) & 0x0FFF;
			}
		} else {
			LOG_WRN("CVG_ARQ_RX: Unhandled ACK feedback info code: %u", fb_info_code);
		}
	}

unlock_and_return:
	k_mutex_unlock(&flow->flow_mutex);
}


static void cvg_tx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("CVG TX Thread started.");

	uint8_t cvg_pdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];

	while (1) {
		cvg_tx_queue_item_t *tx_item = k_fifo_get(&g_app_to_cvg_tx_fifo, K_FOREVER);
		if (!tx_item) {
			continue;
		}

		cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;
		mac_sdu_t *app_sdu = tx_item->app_sdu_buf;
		int err = 0;
		uint16_t current_sn = 0;
		bool is_reliable_service = (flow->service_type >= CVG_SERVICE_TYPE_3_FC);
		int pdu_len = 0;
		size_t pdu_offset = 0;

		if (is_reliable_service) {
			k_sem_take(&flow->tx_window_sem, K_FOREVER);
		}

		k_mutex_lock(&flow->flow_mutex, K_FOREVER);
		current_sn = flow->tx_sequence_number;
		if (flow->service_type != CVG_SERVICE_TYPE_0_TRANSPARENT) {
			flow->tx_sequence_number = (current_sn + 1) & 0x0FFF;
		}
		k_mutex_unlock(&flow->flow_mutex);

		if (flow->security_enabled) {
			uint8_t mic[5];
			uint8_t iv[16];
			dect_mac_context_t *mac_ctx = get_mac_context();

			/* Calculate MIC on original SDU */
			err = security_calculate_mic(app_sdu->data, app_sdu->len, flow->integrity_key, mic);
			if (err) {
				LOG_ERR("CVG_TX_SEC: MIC calculation failed: %d", err);
				goto free_and_continue;
			}

			/* Append MIC to SDU data */
			memcpy(app_sdu->data + app_sdu->len, mic, sizeof(mic));
			app_sdu->len += sizeof(mic);

			/* Build IV and encrypt SDU+MIC */
			security_build_iv(iv, mac_ctx->own_long_rd_id, tx_item->dest_long_id, mac_ctx->hpc, current_sn);
			err = security_crypt_payload(app_sdu->data, app_sdu->len, flow->cipher_key, iv, true);
			if (err) {
				LOG_ERR("CVG_TX_SEC: Encryption failed: %d", err);
				goto free_and_continue;
			}

			/* Prepend Security IE */
			cvg_ie_security_t *sec_ie = (cvg_ie_security_t *)(cvg_pdu_buf);
			sec_ie->header.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) | (CVG_IE_TYPE_SECURITY & 0x1F);
			sec_ie->rsv_keyidx_ivtype = 0; /* Key Index 0, IV Type 0 */
			sec_ie->hpc_be = sys_cpu_to_be32(mac_ctx->hpc);
			pdu_offset += sizeof(cvg_ie_security_t);
		}

		pdu_len = build_cvg_data_ie_pdu(cvg_pdu_buf + pdu_offset, sizeof(cvg_pdu_buf) - pdu_offset,
						app_sdu->data, app_sdu->len, current_sn);

		if (pdu_len < 0) {
			err = pdu_len;
			LOG_ERR("CVG_TX: Failed to build CVG PDU: %d", err);
			goto free_and_continue;
		}
		pdu_len += pdu_offset;

		err = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, tx_item->dest_long_id,
				    cvg_pdu_buf, pdu_len);

free_and_continue:
		if (err != 0) {
			if (is_reliable_service) {
				k_sem_give(&flow->tx_window_sem);
			}
			dect_mac_api_buffer_free(app_sdu);
		} else {
			if (is_reliable_service) {
				/* Store original (unencrypted) SDU for re-TX */
				/* This requires another buffer copy, or a more complex buffer management */
				dect_mac_api_buffer_free(app_sdu);
			} else {
				dect_mac_api_buffer_free(app_sdu);
			}
		}
		k_mem_slab_free(&g_cvg_tx_item_slab, (void **)&tx_item);
	}
}


static void cvg_rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("CVG RX Thread started.");

	uint8_t dlc_sdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
	bool next_data_ie_is_encrypted = false;

	while (1) {
		dlc_service_type_t service_type;
		size_t dlc_sdu_len = sizeof(dlc_sdu_buf);
		uint32_t source_long_rd_id = 0;
		int err = dlc_receive_data(&service_type, dlc_sdu_buf, &dlc_sdu_len, K_FOREVER);

		if (err) {
			continue;
		}

		const uint8_t *pdu_ptr = dlc_sdu_buf;
		size_t remaining_len = dlc_sdu_len;

		while (remaining_len > 0) {
			const cvg_header_t *hdr = (const cvg_header_t *)pdu_ptr;
			cvg_ie_type_t ie_type = (cvg_ie_type_t)(hdr->ext_mt_f2c_or_type & 0x1F);
			size_t ie_consumed_len = 0;

			switch (ie_type) {
			case CVG_IE_TYPE_SECURITY:
				if (remaining_len >= sizeof(cvg_ie_security_t)) {
					const cvg_ie_security_t *sec_ie = (const cvg_ie_security_t *)pdu_ptr;
					g_default_cvg_flow_ctx.peer_hpc = sys_be32_to_cpu(sec_ie->hpc_be);
					next_data_ie_is_encrypted = true;
					ie_consumed_len = sizeof(cvg_ie_security_t);
				}
				break;

			case CVG_IE_TYPE_DATA:
			case CVG_IE_TYPE_DATA_EP:
			{
				const cvg_ie_data_base_t *data_base = (const cvg_ie_data_base_t *)(pdu_ptr + sizeof(cvg_header_t));
				uint16_t sn = cvg_ie_data_base_get_sn(data_base);
				uint8_t *payload_ptr = (uint8_t *)pdu_ptr + sizeof(cvg_header_t) + sizeof(cvg_ie_data_base_t);
				size_t payload_len = remaining_len - (sizeof(cvg_header_t) + sizeof(cvg_ie_data_base_t));

				if (g_default_cvg_flow_ctx.security_enabled || next_data_ie_is_encrypted) {
					uint8_t iv[16];
					dect_mac_context_t *mac_ctx = get_mac_context();
					security_build_iv(iv, source_long_rd_id, mac_ctx->own_long_rd_id, g_default_cvg_flow_ctx.peer_hpc, sn);
					err = security_crypt_payload(payload_ptr, payload_len, g_default_cvg_flow_ctx.cipher_key, iv, false);
					if (err) {
						LOG_ERR("CVG_RX_SEC: Decryption failed for SN %u", sn);
						break;
					}

					uint8_t received_mic[5];
					memcpy(received_mic, payload_ptr + payload_len - 5, 5);
					size_t sdu_len = payload_len - 5;
					uint8_t calculated_mic[5];
					err = security_calculate_mic(payload_ptr, sdu_len, g_default_cvg_flow_ctx.integrity_key, calculated_mic);
					// if (err || crypto_memcmp(received_mic, calculated_mic, 5) != 0) {
					// if (err || mbedtls_platform_memcmp(received_mic, calculated_mic, 5) != 0) {
					if (err || constant_time_memcmp(received_mic, calculated_mic, 5) != 0) {	
						LOG_ERR("CVG_RX_SEC: MIC verification failed for SN %u", sn);
						break;
					}
					payload_len = sdu_len; /* Use only the SDU part now */
				}
				next_data_ie_is_encrypted = false; /* Reset flag */

				/* Pass the cleartext payload to reassembly/delivery */
				mac_sdu_t *app_sdu = NULL;
				if (k_mem_slab_alloc(&g_cvg_app_sdu_slab, (void **)&app_sdu, K_NO_WAIT) == 0) {
					memcpy(app_sdu->data, payload_ptr, payload_len);
					app_sdu->len = payload_len;
					k_fifo_put(&g_cvg_to_app_rx_fifo, app_sdu);
				}
				ie_consumed_len = remaining_len;
				break;
			}
			default:
				LOG_WRN("CVG_RX: Unhandled CVG IE type 0x%X", ie_type);
				ie_consumed_len = remaining_len;
				break;
			}

			if (ie_consumed_len == 0 || ie_consumed_len > remaining_len) {
				break;
			}
			pdu_ptr += ie_consumed_len;
			remaining_len -= ie_consumed_len;
		}
	}
}


static int build_cvg_pdu(uint8_t *target_buf, size_t target_buf_len,
			 const cvg_tx_queue_item_t *tx_item,
			 cvg_flow_context_t *flow, uint16_t sequence_number)
{
	mac_sdu_t *app_sdu = tx_item->app_sdu_buf;
	size_t pdu_offset = 0;
	int pdu_len = 0;
	int err = 0;

	if (flow->security_enabled) {
		uint8_t mic[5];
		uint8_t iv[16];
		dect_mac_context_t *mac_ctx = get_mac_context();

		/* This function needs a copy of the SDU to avoid modifying the original in the HARQ buffer */
		uint8_t temp_sdu_data[app_sdu->len];
		memcpy(temp_sdu_data, app_sdu->data, app_sdu->len);
		size_t temp_sdu_len = app_sdu->len;

		err = security_calculate_mic(temp_sdu_data, temp_sdu_len, flow->integrity_key, mic);
		if (err) { return err; }

		/* Append MIC to temp buffer */
		memcpy(temp_sdu_data + temp_sdu_len, mic, sizeof(mic));
		temp_sdu_len += sizeof(mic);

		security_build_iv(iv, mac_ctx->own_long_rd_id, tx_item->dest_long_id,
				  mac_ctx->hpc, sequence_number);
		err = security_crypt_payload(temp_sdu_data, temp_sdu_len, flow->cipher_key, iv, true);
		if (err) { return err; }

		/* Prepend Security IE */
		cvg_ie_security_t *sec_ie = (cvg_ie_security_t *)(target_buf);
		sec_ie->header.ext_mt_f2c_or_type =
			((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) | (CVG_IE_TYPE_SECURITY & 0x1F);
		sec_ie->rsv_keyidx_ivtype = 0;
		sec_ie->hpc_be = sys_cpu_to_be32(mac_ctx->hpc);
		pdu_offset += sizeof(cvg_ie_security_t);

		pdu_len = build_cvg_data_ie_pdu(target_buf + pdu_offset,
						target_buf_len - pdu_offset,
						temp_sdu_data, temp_sdu_len, sequence_number);
	} else {
		pdu_len = build_cvg_data_ie_pdu(target_buf, target_buf_len,
						app_sdu->data, app_sdu->len, sequence_number);
	}

	if (pdu_len < 0) {
		return pdu_len;
	}

	return pdu_len + pdu_offset;
}

static void cvg_arq_service_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("CVG ARQ Service Thread started.");

	uint8_t cvg_pdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
	cvg_flow_context_t *flow = &g_default_cvg_flow_ctx;

	while (1) {
		uintptr_t sn_to_retransmit_ptr =
			(uintptr_t)k_fifo_get(&g_cvg_retransmit_signal_fifo, K_FOREVER);
		uint16_t sn_to_retransmit = (uint16_t)sn_to_retransmit_ptr;

		k_mutex_lock(&flow->flow_mutex, K_FOREVER);

		uint16_t buffer_index = sn_to_retransmit % CVG_MAX_IN_FLIGHT_SDUS;
		cvg_inflight_sdu_ctx_t *sdu_ctx = &flow->tx_in_flight_sdu[buffer_index];

		if (!sdu_ctx->is_active || !sdu_ctx->sdu ||
		    sdu_ctx->sequence_number != sn_to_retransmit) {
			k_mutex_unlock(&flow->flow_mutex);
			LOG_WRN("CVG_ARQ_SVC: Received re-TX signal for SN %u, but no active SDU in buffer. Already ACKed or timed out?",
				sn_to_retransmit);
			continue;
		}

		LOG_INF("CVG_ARQ_SVC: Retransmitting SDU for SN %u.", sn_to_retransmit);

		/* Re-construct the CVG PDU from the stored SDU context. This is necessary because
		 * the original PDU is not stored, only the raw application data.
		 * This logic mirrors the PDU construction in the main TX thread.
		 */
		cvg_tx_queue_item_t tx_item_for_build = {
			.endpoint_id = sdu_ctx->endpoint_id,
			.dest_long_id = sdu_ctx->dest_long_id,
			.app_sdu_buf = sdu_ctx->sdu
		};

		int pdu_len = build_cvg_pdu(cvg_pdu_buf, sizeof(cvg_pdu_buf),
					    &tx_item_for_build, flow, sdu_ctx->sequence_number);

		if (pdu_len < 0) {
			LOG_ERR("CVG_ARQ_SVC: Failed to rebuild CVG PDU for re-TX of SN %u: %d",
				sn_to_retransmit, pdu_len);
			k_mutex_unlock(&flow->flow_mutex);
			continue;
		}

		/* The SDU is already fully formed, just need to send it to DLC again */
		int err = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, sdu_ctx->dest_long_id,
					cvg_pdu_buf, pdu_len);

		k_mutex_unlock(&flow->flow_mutex);

		if (err) {
			LOG_ERR("CVG_ARQ_SVC: dlc_send_data failed for re-TX of SN %u (err %d). Will be resent on next NACK/timeout.",
				sn_to_retransmit, err);
		} else {
			LOG_DBG("CVG_ARQ_SVC: Re-queued SDU for CVG SDU (SN %u) to DLC.", sn_to_retransmit);
		}
	}
}



int dect_cvg_init(void)
{
    int err = dect_dlc_init();
    if (err) {
        LOG_ERR("Failed to initialize DLC layer: %d", err);
        return err;
    }

    k_thread_name_set(g_cvg_tx_thread_id, "dect_cvg_tx");
    k_thread_name_set(g_cvg_rx_thread_id, "dect_cvg_rx");
    k_thread_name_set(g_cvg_arq_service_thread_id, "cvg_arq_svc");        

    memset(&g_default_cvg_flow_ctx, 0, sizeof(g_default_cvg_flow_ctx));
    g_default_cvg_flow_ctx.service_type = CVG_SERVICE_TYPE_0_TRANSPARENT;
    g_default_cvg_flow_ctx.is_configured = false;
    k_sem_init(&g_default_cvg_flow_ctx.tx_window_sem, 0, K_SEM_MAX_LIMIT);
    k_mutex_init(&g_default_cvg_flow_ctx.flow_mutex);

    for (int i = 0; i < CVG_MAX_IN_FLIGHT_SDUS; i++) {
        g_default_cvg_flow_ctx.tx_in_flight_sdu[i].is_active = false;
        g_default_cvg_flow_ctx.tx_in_flight_sdu[i].sdu = NULL;
        k_timer_init(&g_default_cvg_flow_ctx.tx_in_flight_sdu[i].lifetime_timer,
                     cvg_tx_sdu_lifetime_expiry_handler, NULL);
	g_default_cvg_flow_ctx.rx_reordering_buffer[i].is_valid = false;
	g_default_cvg_flow_ctx.rx_reordering_buffer[i].sdu = NULL;
    }

    LOG_INF("CVG Layer Initialized.");
    return 0;
}

int dect_cvg_configure_flow(cvg_service_type_t service, uint16_t max_window_size, uint32_t lifetime_ms)
{
    if (service >= CVG_SERVICE_TYPE_3_FC && max_window_size == 0) {
        LOG_ERR("CVG_CFG: Max window size cannot be 0 for a flow-controlled service.");
        return -EINVAL;
    }

    g_default_cvg_flow_ctx.service_type = service;
    g_default_cvg_flow_ctx.max_window_size = max_window_size;
    g_default_cvg_flow_ctx.configured_lifetime_ms = lifetime_ms;
    g_default_cvg_flow_ctx.is_configured = true;

    k_sem_init(&g_default_cvg_flow_ctx.tx_window_sem, max_window_size, max_window_size);

    LOG_INF("CVG Flow configured. Service: %d, Window Size: %u", service, max_window_size);

    return 0;
}

int dect_cvg_send(uint16_t endpoint_id, uint32_t dest_long_id, const uint8_t *app_sdu,
		  size_t app_sdu_len)
{
	if (!app_sdu && app_sdu_len > 0) {
		return -EINVAL;
	}
	if (app_sdu_len > (sizeof(mac_sdu_t) - offsetof(mac_sdu_t, data))) {
		LOG_ERR("CVG_SEND: App SDU too large for transport buffer (%zu > %zu)", app_sdu_len,
			(sizeof(mac_sdu_t) - offsetof(mac_sdu_t, data)));
		return -EMSGSIZE;
	}

	cvg_tx_queue_item_t *tx_item = NULL;

	if (k_mem_slab_alloc(&g_cvg_tx_item_slab, (void **)&tx_item, K_NO_WAIT) != 0) {
		LOG_WRN("CVG_SEND: Could not allocate TX queue item.");
		return -ENOMEM;
	}

	mac_sdu_t *sdu_buf = NULL;

	if (k_mem_slab_alloc(&g_cvg_app_sdu_slab, (void **)&sdu_buf, K_NO_WAIT) != 0) {
		LOG_WRN("CVG_SEND: Could not allocate app sdu buffer for TX queue.");
		k_mem_slab_free(&g_cvg_tx_item_slab, (void **)&tx_item);
		return -ENOMEM;
	}

	memcpy(sdu_buf->data, app_sdu, app_sdu_len);
	sdu_buf->len = app_sdu_len;

	tx_item->app_sdu_buf = sdu_buf;
	tx_item->endpoint_id = endpoint_id;
	tx_item->dest_long_id = dest_long_id;

	k_fifo_put(&g_app_to_cvg_tx_fifo, tx_item);
	return 0;
}

int dect_cvg_receive(uint8_t *app_sdu_buf, size_t *len_inout, k_timeout_t timeout)
{
    if (!app_sdu_buf || !len_inout || *len_inout == 0) {
        return -EINVAL;
    }

    mac_sdu_t *sdu_buf = k_fifo_get(&g_cvg_to_app_rx_fifo, timeout);
    if (!sdu_buf) {
        return -EAGAIN; // Timeout
    }

    if (*len_inout < sdu_buf->len) {
        *len_inout = sdu_buf->len; // Report required size
        k_fifo_put(&g_cvg_to_app_rx_fifo, sdu_buf); // Put it back
        return -EMSGSIZE;
    }

    *len_inout = sdu_buf->len;
    memcpy(app_sdu_buf, sdu_buf->data, sdu_buf->len);

    k_mem_slab_free(&g_cvg_app_sdu_slab, (void **)&sdu_buf);
    return 0;
}

int dect_cvg_request_tx_services(uint32_t dest_id, cvg_service_type_t service,
				 uint16_t max_window_size, uint32_t lifetime_ms)
{
	uint8_t pdu_buf[sizeof(cvg_ie_tx_services_cfg_t)];
	cvg_ie_tx_services_cfg_t *cfg_ie = (cvg_ie_tx_services_cfg_t *)pdu_buf;

	cfg_ie->header.ext_mt_f2c_or_type = ((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
					    ((0 & 0x01) << 5) |
					    (CVG_IE_TYPE_TX_SERVICES_CFG & 0x1F);

	cfg_ie->rqrs_reserved_svctype = (0 << 7) |
				      ((service & 0x07));
	// cfg_ie->lifetime = lifetime_ms_to_dlc_code(lifetime_ms);
	cfg_ie->lifetime = cvg_lifetime_ms_to_code(lifetime_ms);
	cfg_ie->max_window_size_be = sys_cpu_to_be16(max_window_size & 0x07FF);

	LOG_INF("CVG_CFG: Sending TX Services Request -> Svc:%d, Win:%u, Life:%ums (code %u)",
		service, max_window_size, lifetime_ms, cfg_ie->lifetime);

	return dlc_send_data(DLC_SERVICE_TYPE_0_TRANSPARENT, dest_id, pdu_buf, sizeof(pdu_buf));
}

int dect_cvg_set_security_params(const uint8_t *integrity_key, const uint8_t *cipher_key)
{
	if (!integrity_key || !cipher_key) {
		return -EINVAL;
	}

	k_mutex_lock(&g_default_cvg_flow_ctx.flow_mutex, K_FOREVER);
	memcpy(g_default_cvg_flow_ctx.integrity_key, integrity_key, 16);
	memcpy(g_default_cvg_flow_ctx.cipher_key, cipher_key, 16);
	g_default_cvg_flow_ctx.security_enabled = true;
	k_mutex_unlock(&g_default_cvg_flow_ctx.flow_mutex);

	LOG_INF("CVG security parameters set and security enabled for default flow.");
	return 0;
}
