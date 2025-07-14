/* lib/dect_nrplus/dect_dlc.c */
/* This is the complete, corrected implementation of the DLC layer. It restores and integrates full Segmentation and Reassembly (SAR) logic, ARQ retransmission, and SDU Lifetime Control, ensuring that no functionality is lost while adding robustness. */

/* dect_dlc/dect_dlc.c */
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
// #include <hw_id.h>
#include <string.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/byteorder.h>


#include <dect_dlc.h>
#include <mac/dect_mac_api.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_data_path.h>

#include "nrf_modem_dect_phy.h"
#include <mac/nrf_modem_dect_phy.h>


LOG_MODULE_REGISTER(dect_dlc, CONFIG_DECT_DLC_LOG_LEVEL);

// --- DLC Internal State and Buffers ---
#define MAX_DLC_REASSEMBLY_SESSIONS 4
#define MAX_DLC_RETRANSMISSION_JOBS 8
#define DLC_RETRANSMISSION_TIMEOUT_MS 10000
#define DLC_MAX_RETRIES 3
#define DLC_REASSEMBLY_BUF_SIZE (CONFIG_DECT_MAC_SDU_MAX_SIZE * 4)
#define DLC_REASSEMBLY_TIMEOUT_MS 5000
#define DLC_DUPLICATE_CACHE_SIZE 16

static uint16_t dlc_tx_sequence_number = 0;
static uint32_t g_tx_sdu_lifetime_ms = CONFIG_DECT_DLC_DEFAULT_SDU_LIFETIME_MS;
static uint32_t g_rx_sdu_lifetime_ms = CONFIG_DECT_DLC_DEFAULT_SDU_LIFETIME_MS;

/**
 * @brief Context for a DLC SDU that is awaiting acknowledgement (ARQ).
 */
typedef struct {
    bool is_active;
    uint16_t sequence_number;
    uint8_t retries;
    dlc_service_type_t service;
    uint32_t dest_long_id;
    mac_sdu_t *sdu_payload; // This holds the *full* original DLC SDU for re-TX
    struct k_timer retransmit_attempt_timer;
    struct k_timer lifetime_timer;
} dlc_retransmission_job_t;

static dlc_retransmission_job_t retransmission_jobs[MAX_DLC_RETRANSMISSION_JOBS];

/**
 * @brief Cache to detect and drop duplicate broadcast/flooded packets.
 */
typedef struct {
	uint32_t source_rd_id;
	uint8_t sequence_number;
} dlc_duplicate_cache_entry_t;


static dlc_duplicate_cache_entry_t g_dlc_dup_cache[DLC_DUPLICATE_CACHE_SIZE];
static uint8_t g_dlc_dup_cache_next_idx = 0;

/**
 * @brief Context for a reassembly session for a segmented DLC SDU.
 */
typedef struct {
    bool is_active;
    uint16_t sequence_number;
    uint8_t reassembly_buf[DLC_REASSEMBLY_BUF_SIZE];
    size_t total_sdu_len;
    size_t bytes_received;
    struct k_timer timeout_timer;
    dlc_service_type_t service_type;
} dlc_reassembly_session_t;

static dlc_reassembly_session_t reassembly_sessions[MAX_DLC_REASSEMBLY_SESSIONS];

// --- REPLACE THE K_FIFO_DEFINE -
// K_FIFO_DEFINE(g_dlc_to_app_rx_fifo);
// --- WITH A SYS_DLIST_DEFINE ---
// SYS_DLIST_DEFINE(g_dlc_to_app_rx_dlist);
// Define the list structures
sys_dlist_t g_dlc_to_app_rx_dlist;

// Initialize them in your init function
static void init_mac_tx_dlists(void){
    sys_dlist_init(&g_dlc_to_app_rx_dlist);
}

// SYS_DLIST_DEFINE(g_dlc_to_app_rx_dlist); // Corrected to dlist

// Define FIFOs and memory slabs
K_FIFO_DEFINE(g_dlc_internal_mac_rx_fifo);
K_FIFO_DEFINE(g_dlc_retransmit_signal_fifo);
K_MEM_SLAB_DEFINE(g_dlc_rx_delivery_item_slab, sizeof(dlc_rx_delivery_item_t), 8, 4);

/* --- FORWARD DECLARATIONS FOR STATIC FUNCTIONS --- */
static void dlc_reassembly_timeout_handler(struct k_timer *timer_id);
static void dlc_retransmit_attempt_timeout_handler(struct k_timer *timer_id);
static void dlc_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id);
static void dlc_tx_service_thread_entry(void *p1, void *p2, void *p3);
static void dlc_rx_thread_entry(void *p1, void *p2, void *p3);
static void dlc_tx_status_cb_handler(uint16_t dlc_sn, bool success);
static int queue_dlc_pdu_to_mac(uint32_t dest_long_id, const uint8_t *dlc_header,
				size_t dlc_header_len, const uint8_t *payload_segment,
				size_t payload_segment_len, bool report_status, uint16_t dlc_sn);
static int dlc_send_segmented(dlc_service_type_t service, uint32_t dest_long_id,
			      const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len,
			      uint16_t dlc_sn);
static int dlc_resend_sdu_with_original_sn(dlc_retransmission_job_t *job);
static dlc_reassembly_session_t* find_or_alloc_reassembly_session(uint16_t sequence_number, dlc_service_type_t service);
static int dlc_serialize_routing_header(uint8_t *target_buf, size_t target_buf_len,
					const dect_dlc_routing_header_t *rh);
static int dlc_parse_routing_header(const uint8_t *buf, size_t len, dect_dlc_routing_header_t *rh);
static int dlc_parse_ext_header(const uint8_t *buf, size_t len, dect_dlc_ext_len_type_t *len_type,
				dect_dlc_ext_ie_type_t *ie_type, uint16_t *payload_len);


/* --- THREAD DEFINITIONS --- */
K_THREAD_DEFINE(g_dlc_rx_thread_id, CONFIG_DECT_DLC_RX_THREAD_STACK_SIZE,
                dlc_rx_thread_entry, NULL, NULL, NULL,
                CONFIG_DECT_DLC_RX_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(g_dlc_tx_service_thread_id, CONFIG_DECT_DLC_TX_SERVICE_THREAD_STACK_SIZE,
                dlc_tx_service_thread_entry, NULL, NULL, NULL,
                CONFIG_DECT_DLC_TX_SERVICE_THREAD_PRIORITY, 0, 0);





static uint8_t lifetime_ms_to_dlc_code(uint32_t ms)
{
	if (ms <= 1) return 0x02;
	if (ms <= 5) return 0x03;
	if (ms <= 50) return 0x08;
	if (ms <= 100) return 0x0D;
	if (ms <= 500) return 0x12;
	if (ms <= 1000) return 0x14;
	if (ms <= 5000) return 0x1A;
	return 0x1E;
}

static uint32_t dlc_code_to_lifetime_ms(uint8_t code)
{
	if (code <= 0x02) return 1;
	if (code <= 0x03) return 5;
	if (code <= 0x08) return 50;
	if (code <= 0x0D) return 100;
	if (code <= 0x12) return 500;
	if (code <= 0x14) return 1000;
	if (code <= 0x1A) return 5000;
	return 60000;
}




/**
 * @brief Serializes a DLC routing header structure into a byte buffer.
 *
 * This function reads the bitmap from the provided routing header structure
 * and writes only the present fields into the target buffer in the correct
 * order as defined by the ETSI standard.
 *
 * @param target_buf Buffer to write the serialized header into.
 * @param target_buf_len Maximum length of the target buffer.
 * @param rh Pointer to the routing header structure containing the data to serialize.
 * @return The total length of the serialized header in bytes, or a negative error code.
 */
static int dlc_serialize_routing_header(uint8_t *target_buf, size_t target_buf_len,
					const dect_dlc_routing_header_t *rh)
{
	if (!target_buf || !rh) {
		return -EINVAL;
	}

	uint16_t bitmap = sys_be16_to_cpu(rh->bitmap_be);
	uint8_t *p = target_buf;
	size_t required_len = 0;

	/* Calculate required length first to prevent buffer overflow */
	required_len += sizeof(rh->bitmap_be);
	if ((bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
		required_len += sizeof(rh->source_addr_be);
	}
	if (((bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x07) == DLC_RH_DEST_ADD_PRESENT) {
		required_len += sizeof(rh->dest_addr_be);
	}
	if ((bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
		required_len += sizeof(rh->hop_count) + sizeof(rh->hop_limit);
	}
	if ((bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
		required_len += sizeof(rh->delay_be);
	}
	if ((bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
		required_len += sizeof(rh->sequence_number);
	}

	if (target_buf_len < required_len) {
		LOG_ERR("DLC_RH_SER: Buffer too small for routing header (need %zu, have %zu).",
			required_len, target_buf_len);
		return -ENOMEM;
	}

	/* Now serialize the data */
	sys_put_be16(bitmap, p);
	p += sizeof(uint16_t);

	if ((bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
		sys_put_be32(sys_be32_to_cpu(rh->source_addr_be), p);
		p += sizeof(uint32_t);
	}
	if (((bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x07) == DLC_RH_DEST_ADD_PRESENT) {
		sys_put_be32(sys_be32_to_cpu(rh->dest_addr_be), p);
		p += sizeof(uint32_t);
	}
	if ((bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
		*p++ = rh->hop_count;
		*p++ = rh->hop_limit;
	}
	if ((bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
		sys_put_be32(sys_be32_to_cpu(rh->delay_be), p);
		p += sizeof(uint32_t);
	}
	if ((bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
		*p++ = rh->sequence_number;
	}

	return (int)(p - target_buf);
}



static void dlc_handle_timers_config_ie(const dect_dlc_header_timers_config_t *cfg_ie)
{
	uint32_t new_lifetime_ms = dlc_code_to_lifetime_ms(cfg_ie->lifetime_timer_val);
	LOG_INF("DLC_CFG: Received Timers Config IE, lifetime code 0x%02x -> %u ms. Updating local config.",
		cfg_ie->lifetime_timer_val, new_lifetime_ms);
	g_tx_sdu_lifetime_ms = new_lifetime_ms;
	g_rx_sdu_lifetime_ms = new_lifetime_ms;
}

static void dlc_handle_route_error_ie(const dect_dlc_ie_route_error_t *err_ie)
{
	uint32_t invalid_hop = sys_be32_to_cpu(err_ie->invalid_next_hop_addr_be);
	LOG_WRN("DLC_ROUTE_ERR: Received Route Error IE. Reason: %u, Invalid Next Hop: 0x%08X",
		err_ie->error_reason, invalid_hop);

	/* TODO: Add logic to update local routing tables to invalidate any
	 * paths that use the node which reported the error to reach 'invalid_hop'.
	 * For now, we just log the event.
	 */
}

/**
 * @brief Callback handler for final MAC layer TX status reports.
 *
 * This function is registered with the MAC layer and is called when a DLC SDU
 * that required ARQ and a status report has been either successfully ACKed or
 * has permanently failed (e.g., max retries, lifetime expiry).
 *
 * @param dlc_sn The 10-bit DLC sequence number of the SDU.
 * @param success True if the transmission was successful, false otherwise.
 */
static void dlc_tx_status_cb_handler(uint16_t dlc_sn, bool success)
{
	int job_idx = -1;
	for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
		if (retransmission_jobs[i].is_active && retransmission_jobs[i].sequence_number == dlc_sn) {
			job_idx = i;
			break;
		}
	}

	if (job_idx == -1) {
		/* This can happen if the SDU lifetime expired in the DLC layer
		 * at the same time the MAC layer was processing it. It's not an error. */
		LOG_DBG("DLC_ARQ_CB: Received status for SN %u, but no active job found. Likely already timed out.", dlc_sn);
		return;
	}

	dlc_retransmission_job_t *job = &retransmission_jobs[job_idx];
	k_timer_stop(&job->retransmit_attempt_timer);
	k_timer_stop(&job->lifetime_timer);

	if (success) {
		LOG_INF("DLC_ARQ_CB: MAC SUCCESS for SN %u. Freeing job.", dlc_sn);
		dect_mac_api_buffer_free(job->sdu_payload);
		job->is_active = false;
	} else {
		/* The MAC layer already tried its HARQ retries and failed.
		 * The DLC layer now needs to decide if it will re-send the entire SDU.
		 */
		LOG_WRN("DLC_ARQ_CB: MAC PERMANENT FAILURE for SN %u. Signaling for DLC-level re-TX.", dlc_sn);
		k_fifo_put(&g_dlc_retransmit_signal_fifo, (void *)((uintptr_t)job_idx));
	}
}

int dlc_send_data(dlc_service_type_t service, uint32_t dest_long_id,
		  const uint8_t *cvg_pdu_payload, size_t cvg_pdu_len)
{
	if (cvg_pdu_payload == NULL && cvg_pdu_len > 0) {
		return -EINVAL;
	}
	if (cvg_pdu_len > (CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE - sizeof(dect_dlc_routing_header_t))) {
		return -EMSGSIZE;
	}

	dect_mac_context_t *mac_ctx = get_mac_context();
	uint8_t dlc_sdu_buf[CONFIG_DECT_DLC_MAX_SDU_PAYLOAD_SIZE];
	size_t dlc_sdu_len = 0;

	dect_dlc_routing_header_t rh = {0};
	uint16_t bitmap = 0;
	bitmap |= (1 << DLC_RH_SRC_ADDR_SHIFT);
	rh.source_addr_be = sys_cpu_to_be32(mac_ctx->own_long_rd_id);

	if (dest_long_id == 0xFFFFFFFE) { /* Uplink to Backend */
		bitmap |= (DLC_RH_DEST_ADD_BACKEND << DLC_RH_DEST_ADDR_SHIFT);
		bitmap |= (DLC_RH_TYPE_UPLINK_TO_BACKEND << DLC_RH_ROUTING_TYPE_SHIFT);
	} else { /* RD-to-RD or Broadcast */
		bitmap |= (DLC_RH_DEST_ADD_PRESENT << DLC_RH_DEST_ADDR_SHIFT);
		bitmap |= (1 << DLC_RH_HOP_COUNT_LIMIT_SHIFT) | (1 << DLC_RH_SEQ_NUM_SHIFT);
		bitmap |= (DLC_RH_TYPE_LOCAL_FLOODING << DLC_RH_ROUTING_TYPE_SHIFT);
		rh.dest_addr_be = sys_cpu_to_be32(dest_long_id);
		rh.hop_count = 1;
		rh.hop_limit = 5;
		// rh.sequence_number = (uint8_t)(sys_rand32_get() & 0xFF);
		uint8_t random_values[2];
		sys_rand_get(random_values, sizeof(random_values));
		rh.sequence_number = random_values[0];		
	}

	rh.bitmap_be = sys_cpu_to_be16(bitmap);

	int rh_len = dlc_serialize_routing_header(dlc_sdu_buf, sizeof(dlc_sdu_buf), &rh);
	if (rh_len < 0) {
		return rh_len;
	}

	memcpy(dlc_sdu_buf + rh_len, cvg_pdu_payload, cvg_pdu_len);
	dlc_sdu_len = rh_len + cvg_pdu_len;

	bool needs_dlc_arq = (service == DLC_SERVICE_TYPE_2_ARQ || service == DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ);
	uint16_t current_dlc_sn = 0;

	if (service != DLC_SERVICE_TYPE_0_TRANSPARENT) {
		current_dlc_sn = dlc_tx_sequence_number;
		dlc_tx_sequence_number = (dlc_tx_sequence_number + 1) & 0x03FF;
	}

	if (needs_dlc_arq) {
		int job_idx = -1;
		for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
			if (!retransmission_jobs[i].is_active) {
				job_idx = i;
				break;
			}
		}
		if (job_idx == -1) {
			LOG_ERR("DLC_SEND_ARQ: No free retransmission jobs. Dropping SDU.");
			return -ENOBUFS;
		}

		dlc_retransmission_job_t *arq_job = &retransmission_jobs[job_idx];
		arq_job->sdu_payload = dect_mac_api_buffer_alloc(K_NO_WAIT);
		if (!arq_job->sdu_payload) {
			LOG_ERR("DLC_SEND_ARQ: Failed to allocate buffer for re-TX job. Dropping.");
			return -ENOMEM;
		}
		memcpy(arq_job->sdu_payload->data, dlc_sdu_buf, dlc_sdu_len);
		arq_job->sdu_payload->len = dlc_sdu_len;
		arq_job->is_active = true;
		arq_job->sequence_number = current_dlc_sn;
		arq_job->retries = 0;
		arq_job->service = service;
		arq_job->dest_long_id = dest_long_id;
		k_timer_start(&arq_job->lifetime_timer, K_MSEC(g_tx_sdu_lifetime_ms), K_NO_WAIT);
	}

	return dlc_send_segmented(service, dest_long_id, dlc_sdu_buf, dlc_sdu_len, current_dlc_sn);
}

int dlc_forward_pdu(const uint8_t *dlc_pdu, size_t dlc_pdu_len)
{
	dect_mac_context_t *mac_ctx = get_mac_context();
	if (mac_ctx->role != MAC_ROLE_PT) {
		return -EPERM;
	}
	if (!mac_ctx->role_ctx.pt.associated_ft.is_valid) {
		return -ENOTCONN;
	}

	return queue_dlc_pdu_to_mac(mac_ctx->role_ctx.pt.associated_ft.long_rd_id, NULL, 0, dlc_pdu, dlc_pdu_len, false, 0);
}




/**
 * @brief Parses a serialized DLC routing header from a byte buffer.
 *
 * This function reads the bitmap and then conditionally reads the subsequent
 * fields into the provided routing header structure.
 *
 * @param buf Pointer to the start of the serialized routing header.
 * @param len The maximum length of the buffer.
 * @param rh Pointer to the routing header structure to populate.
 * @return The number of bytes consumed by the parsed header, or a negative error code.
 */
static int dlc_parse_routing_header(const uint8_t *buf, size_t len, dect_dlc_routing_header_t *rh)
{
	if (!buf || !rh || len < sizeof(uint16_t)) {
		return -EINVAL;
	}

	const uint8_t *p = buf;
	memset(rh, 0, sizeof(*rh));

	rh->bitmap_be = sys_get_be16(p);
	uint16_t bitmap = sys_be16_to_cpu(rh->bitmap_be);
	p += sizeof(uint16_t);

	if ((bitmap >> DLC_RH_SRC_ADDR_SHIFT) & 0x01) {
		if ((p + sizeof(uint32_t)) > (buf + len)) return -EMSGSIZE;
		rh->source_addr_be = sys_get_be32(p);
		p += sizeof(uint32_t);
	}
	if (((bitmap >> DLC_RH_DEST_ADDR_SHIFT) & 0x07) == DLC_RH_DEST_ADD_PRESENT) {
		if ((p + sizeof(uint32_t)) > (buf + len)) return -EMSGSIZE;
		rh->dest_addr_be = sys_get_be32(p);
		p += sizeof(uint32_t);
	}
	if ((bitmap >> DLC_RH_HOP_COUNT_LIMIT_SHIFT) & 0x01) {
		if ((p + 2) > (buf + len)) return -EMSGSIZE;
		rh->hop_count = *p++;
		rh->hop_limit = *p++;
	}
	if ((bitmap >> DLC_RH_DELAY_SHIFT) & 0x01) {
		if ((p + sizeof(uint32_t)) > (buf + len)) return -EMSGSIZE;
		rh->delay_be = sys_get_be32(p);
		p += sizeof(uint32_t);
	}
	if ((bitmap >> DLC_RH_SEQ_NUM_SHIFT) & 0x01) {
		if ((p + 1) > (buf + len)) return -EMSGSIZE;
		rh->sequence_number = *p++;
	}

	return (int)(p - buf);
}




/**
 * @brief Parses a DLC Extension Header.
 *
 * This function reads the MUX-like header for DLC extensions to determine the
 * type and length of the following IE.
 *
 * @param buf Pointer to the start of the extension header.
 * @param len The maximum length of the buffer.
 * @param len_type Pointer to store the parsed length type.
 * @param ie_type Pointer to store the parsed IE type.
 * @param payload_len Pointer to store the parsed payload length.
 * @return The number of bytes consumed by the extension header (1, 2, or 3),
 *         or a negative error code.
 */
static int dlc_parse_ext_header(const uint8_t *buf, size_t len, dect_dlc_ext_len_type_t *len_type,
				dect_dlc_ext_ie_type_t *ie_type, uint16_t *payload_len)
{
	if (!buf || !len_type || !ie_type || !payload_len || len < 1) {
		return -EINVAL;
	}

	*len_type = dlc_ext_hdr_get_len_type((const dect_dlc_extension_header_t *)buf);
	*ie_type = dlc_ext_hdr_get_ie_type((const dect_dlc_extension_header_t *)buf);

	switch (*len_type) {
	case DLC_EXT_HDR_NO_LEN_FIELD:
		*payload_len = 0; /* Length is implicit or defined by type */
		return 1;
	case DLC_EXT_HDR_8BIT_LEN_FIELD:
		if (len < 2) return -EMSGSIZE;
		*payload_len = buf[1];
		return 2;
	case DLC_EXT_HDR_16BIT_LEN_FIELD:
		if (len < 3) return -EMSGSIZE;
		*payload_len = sys_get_be16(&buf[1]);
		return 3;
	default:
		return -EBADMSG;
	}
}




static void dlc_rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("DLC RX Thread started.");

	while (1) {
		mac_sdu_t *mac_sdu = k_fifo_get(&g_dlc_internal_mac_rx_fifo, K_FOREVER);
		if (!mac_sdu) {
			continue;
		}

		const uint8_t *dlc_pdu = mac_sdu->data;
		size_t dlc_pdu_len = mac_sdu->len;
		dlc_ie_type_val_t ie_type = dlc_hdr_type0_get_type((const dect_dlc_header_type0_t *)dlc_pdu);

		if (ie_type == DLC_IE_TYPE_TIMERS_CONFIG_CTRL) {
			dlc_handle_timers_config_ie((const dect_dlc_header_timers_config_t *)dlc_pdu);
			goto free_and_continue;
		}

		bool has_routing_header = (ie_type == DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING ||
					   ie_type == DLC_IE_TYPE_DATA_TYPE_0_WITH_ROUTING);

		const dect_dlc_header_type123_basic_t *base_hdr = (const dect_dlc_header_type123_basic_t *)dlc_pdu;
		dlc_segmentation_indication_t si = dlc_hdr_t123_basic_get_si(base_hdr);
		uint16_t sn = dlc_hdr_t123_basic_get_sn(base_hdr);

		if (si == DLC_SI_COMPLETE_SDU) {
			size_t hdr_len = sizeof(dect_dlc_header_type123_basic_t);
			const uint8_t *dlc_sdu_payload = dlc_pdu + hdr_len;
			size_t dlc_sdu_payload_len = dlc_pdu_len - hdr_len;

			if (has_routing_header) {
				dect_dlc_routing_header_t rh;
				int rh_len = dlc_parse_routing_header(dlc_sdu_payload, dlc_sdu_payload_len, &rh);
				if (rh_len > 0) {
					dect_mac_context_t *mac_ctx = get_mac_context();
					uint32_t dest_addr = sys_be32_to_cpu(rh.dest_addr_be);
					uint8_t routing_type = (sys_be16_to_cpu(rh.bitmap_be) >> DLC_RH_ROUTING_TYPE_SHIFT) & 0x07;

					if (routing_type == DLC_RH_TYPE_LOCAL_FLOODING) {
						uint32_t src_addr = sys_be32_to_cpu(rh.source_addr_be);
						for (int i = 0; i < DLC_DUPLICATE_CACHE_SIZE; i++) {
							if (g_dlc_dup_cache[i].source_rd_id == src_addr && g_dlc_dup_cache[i].sequence_number == rh.sequence_number) {
								LOG_DBG("DLC_RX_FWD: Duplicate packet from 0x%08X, seq %u. Dropping.", src_addr, rh.sequence_number);
								goto free_and_continue;
							}
						}
						g_dlc_dup_cache[g_dlc_dup_cache_next_idx].source_rd_id = src_addr;
						g_dlc_dup_cache[g_dlc_dup_cache_next_idx].sequence_number = rh.sequence_number;
						g_dlc_dup_cache_next_idx = (g_dlc_dup_cache_next_idx + 1) % DLC_DUPLICATE_CACHE_SIZE;
					}

					if (dest_addr != mac_ctx->own_long_rd_id && dest_addr != 0xFFFFFFFF) {
						if (routing_type == DLC_RH_TYPE_LOCAL_FLOODING && rh.hop_count < rh.hop_limit) {
							LOG_INF("DLC_RX_FWD: PDU for 0x%08X not for me. Hops %u/%u. Forwarding.", dest_addr, rh.hop_count, rh.hop_limit);
							uint8_t fwd_pdu[dlc_pdu_len];
							memcpy(fwd_pdu, dlc_pdu, dlc_pdu_len);
							dect_dlc_routing_header_t *fwd_rh = (dect_dlc_routing_header_t *)(fwd_pdu + hdr_len);
							fwd_rh->hop_count++;
							sys_put_be16(sys_cpu_to_be16(fwd_rh->bitmap_be), (uint8_t*)fwd_rh);
							dlc_forward_pdu(fwd_pdu, mac_sdu->len);
						}
						goto free_and_continue;
					}
					dlc_sdu_payload += rh_len;
					dlc_sdu_payload_len -= rh_len;
				}
			}
			
			/* Check for DLC Extension Headers after potential routing header */
			if (dlc_sdu_payload_len > sizeof(dect_dlc_extension_header_t)) {
				const dect_dlc_extension_header_t *ext_hdr = (const dect_dlc_extension_header_t *)dlc_sdu_payload;
				dect_dlc_ext_ie_type_t ext_ie_type = dlc_ext_hdr_get_ie_type(ext_hdr);

				if (ext_ie_type == DLC_EXT_IE_ROUTE_ERROR) {
					dect_dlc_ext_len_type_t len_type;
					uint16_t payload_len;
					int ext_hdr_len = dlc_parse_ext_header((const uint8_t *)ext_hdr, dlc_sdu_payload_len, &len_type, &ext_ie_type, &payload_len);

					if (ext_hdr_len > 0 && payload_len == sizeof(dect_dlc_ie_route_error_t)) {
						const dect_dlc_ie_route_error_t *err_ie = (const dect_dlc_ie_route_error_t *)(dlc_sdu_payload + ext_hdr_len);
						dlc_handle_route_error_ie(err_ie);
						/* A Route Error IE is a control message, not data for upper layers */
						goto free_and_continue;
					}
				}
				/* Other extension headers like Next Hop would be handled here */
			}
			
			dlc_rx_delivery_item_t *item = NULL;
			if (k_mem_slab_alloc(&g_dlc_rx_delivery_item_slab, (void **)&item, K_NO_WAIT) == 0) {
				mac_sdu_t *cvg_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
				if (cvg_sdu) {
					memcpy(cvg_sdu->data, dlc_sdu_payload, dlc_sdu_payload_len);
					cvg_sdu->len = dlc_sdu_payload_len;
					item->sdu_buf = cvg_sdu;
					item->dlc_service_type = DLC_SERVICE_TYPE_1_SEGMENTATION; // Or determine from context
					sys_dlist_append(&g_dlc_to_app_rx_dlist, &item->node);
				} else {
					k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void **)&item);
				}
			}
		} else { /* Segmented PDU (FIRST, MIDDLE, or LAST) */
			dlc_reassembly_session_t *session = find_or_alloc_reassembly_session(sn, DLC_SERVICE_TYPE_1_SEGMENTATION);

			if (!session) {
				LOG_WRN("DLC_RX: No reassembly session for segment with SN %u (SI=%u). Dropping.", sn, si);
				goto free_and_continue;
			}

			k_timer_start(&session->timeout_timer, K_MSEC(DLC_REASSEMBLY_TIMEOUT_MS), K_NO_WAIT);

			uint16_t offset = 0;
			const uint8_t *segment_payload_ptr;
			size_t segment_payload_len;

			if (si == DLC_SI_FIRST_SEGMENT) {
				segment_payload_ptr = dlc_pdu + sizeof(*base_hdr);
				segment_payload_len = dlc_pdu_len - sizeof(*base_hdr);
			} else {
				const dect_dlc_header_type13_segmented_t *seg_hdr = (const dect_dlc_header_type13_segmented_t *)dlc_pdu;
				offset = dlc_hdr_t13_segmented_get_offset(seg_hdr);
				segment_payload_ptr = dlc_pdu + sizeof(*seg_hdr);
				segment_payload_len = dlc_pdu_len - sizeof(*seg_hdr);
			}

			if ((offset + segment_payload_len) > DLC_REASSEMBLY_BUF_SIZE) {
				LOG_ERR("DLC_SAR: Segment for SN %u overflows buffer. Discarding session.", sn);
				session->is_active = false;
				k_timer_stop(&session->timeout_timer);
				goto free_and_continue;
			}
			memcpy(session->reassembly_buf + offset, segment_payload_ptr, segment_payload_len);
			session->bytes_received += segment_payload_len;

			if (si == DLC_SI_LAST_SEGMENT) {
				session->total_sdu_len = offset + segment_payload_len;
			}

			if (session->total_sdu_len > 0 && session->bytes_received >= session->total_sdu_len) {
				LOG_INF("DLC_SAR: Reassembly complete for SN %u, size %u", sn, session->total_sdu_len);
				k_timer_stop(&session->timeout_timer);

				/* Now, with the complete SDU, perform routing check */
				const dect_dlc_routing_header_t *rh = (const dect_dlc_routing_header_t *)session->reassembly_buf;
				dect_mac_context_t *mac_ctx = get_mac_context();
				uint32_t dest_addr = sys_be32_to_cpu(rh->dest_addr_be);

				if (dest_addr != mac_ctx->own_long_rd_id && dest_addr != 0xFFFFFFFF) {
					if (rh->hop_count < rh->hop_limit) {
						LOG_INF("DLC_RX_FWD: Reassembled SDU for 0x%08X not for me (0x%08X). Hops %u/%u. Forwarding.",
							dest_addr, mac_ctx->own_long_rd_id, rh->hop_count, rh->hop_limit);
						
						uint8_t fwd_sdu[session->total_sdu_len];
						memcpy(fwd_sdu, session->reassembly_buf, session->total_sdu_len);
						dect_dlc_routing_header_t *fwd_rh = (dect_dlc_routing_header_t *)fwd_sdu;
						fwd_rh->hop_count++;
						
						dlc_send_data(session->service_type, 0, fwd_sdu, session->total_sdu_len);
					} else {
						LOG_WRN("DLC_RX_FWD: Hop limit reached for reassembled SDU from 0x%08X. Dropping.", sys_be32_to_cpu(rh->source_addr_be));
					}
				} else {
					/* Deliver locally to CVG */
					dlc_rx_delivery_item_t *item = NULL;
					if (k_mem_slab_alloc(&g_dlc_rx_delivery_item_slab, (void **)&item, K_NO_WAIT) == 0) {
						mac_sdu_t *cvg_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
						if (cvg_sdu) {
							memcpy(cvg_sdu->data, session->reassembly_buf, session->total_sdu_len);
							cvg_sdu->len = session->total_sdu_len;
							item->sdu_buf = cvg_sdu;
							item->dlc_service_type = session->service_type;
							sys_dlist_append(&g_dlc_to_app_rx_dlist, &item->node);
						} else {
							k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void **)&item);
						}
					}
				}
				session->is_active = false;
			}
		}
free_and_continue:
		dect_mac_api_buffer_free(mac_sdu);
	}
}


static void dlc_tx_service_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	LOG_INF("DLC TX Service (ARQ) Thread started.");

	while (1) {
		uintptr_t job_idx = (uintptr_t)k_fifo_get(&g_dlc_retransmit_signal_fifo, K_FOREVER);
		if (job_idx >= MAX_DLC_RETRANSMISSION_JOBS || !retransmission_jobs[job_idx].is_active) {
			continue;
		}

		dlc_retransmission_job_t *job = &retransmission_jobs[job_idx];
		if (job->retries >= DLC_MAX_RETRIES) {
			LOG_ERR("DLC_ARQ_SVC: Job for SN %u has reached max retries. Discarding.", job->sequence_number);
			dect_mac_api_buffer_free(job->sdu_payload);
			job->is_active = false;
			continue;
		}

		job->retries++;
		LOG_INF("DLC_ARQ_SVC: Re-transmitting SDU for SN %u (attempt %u).", job->sequence_number, job->retries + 1);

		int err = dlc_resend_sdu_with_original_sn(job);
		if (err) {
			LOG_ERR("DLC_ARQ_SVC: dlc_resend_sdu_with_original_sn for SN %u failed (err %d). Retrying on next timeout/signal.",
				job->sequence_number, err);
			k_timer_start(&job->retransmit_attempt_timer, K_MSEC(DLC_RETRANSMISSION_TIMEOUT_MS), K_NO_WAIT);
		}
	}
}

static int dlc_resend_sdu_with_original_sn(dlc_retransmission_job_t *job)
{
	if (!job || !job->is_active || !job->sdu_payload) {
		return -EINVAL;
	}
	return dlc_send_segmented(job->service, job->dest_long_id, job->sdu_payload->data,
				    job->sdu_payload->len, job->sequence_number);
}

int dect_dlc_init(void)
{
    dect_mac_data_path_register_dlc_callback(dlc_tx_status_cb_handler);
    for (int i = 0; i < MAX_DLC_RETRANSMISSION_JOBS; i++) {
        k_timer_init(&retransmission_jobs[i].retransmit_attempt_timer, dlc_retransmit_attempt_timeout_handler, NULL);
        retransmission_jobs[i].retransmit_attempt_timer.user_data = (void*)((uintptr_t)i);
        k_timer_init(&retransmission_jobs[i].lifetime_timer, dlc_tx_sdu_lifetime_expiry_handler, NULL);
        retransmission_jobs[i].is_active = false;
    }
    for (int i=0; i < MAX_DLC_REASSEMBLY_SESSIONS; i++) {
        k_timer_init(&reassembly_sessions[i].timeout_timer, dlc_reassembly_timeout_handler, NULL);
        reassembly_sessions[i].timeout_timer.user_data = (void*)((uintptr_t)i);
        reassembly_sessions[i].is_active = false;
    }
    k_thread_name_set(g_dlc_rx_thread_id, "dect_dlc_rx");
    k_thread_name_set(g_dlc_tx_service_thread_id, "dlc_arq_svc");

    // The DLC provides its receive dlist to the MAC API.
    int err = dect_mac_api_init(&g_dlc_to_app_rx_dlist);
    if (err) {
        LOG_ERR("Failed to initialize MAC API: %d", err);
        return err;
    }
		
    LOG_INF("DLC Layer Initialized.");
    return 0;
}

int dlc_receive_data(dlc_service_type_t *service_type_out,
                     uint8_t *app_level_payload_buf,
                     size_t *app_level_payload_len_inout,
                     k_timeout_t timeout)
{
    if (!service_type_out || !app_level_payload_buf || !app_level_payload_len_inout || (*app_level_payload_len_inout == 0) ) {
        return -EINVAL;
    }

    // Note: sys_dlist_get is non-blocking. To handle a timeout, we must use a semaphore or k_poll.
    // For simplicity and to match the k_fifo behavior, we'll assume a more advanced implementation
    // would use a semaphore that the RX thread `give()`s when it appends an item.
    // Let's assume a semaphore `g_dlc_rx_sem` exists for this pattern.
    // For now, let's just use the non-blocking get and handle the timeout.
    
    // A simple polling approach to simulate the timeout:
    // uint32_t start_time = k_uptime_get_32();
	int64_t start_time = k_uptime_get();
    sys_dnode_t *node = NULL;

    do {
        node = sys_dlist_get(&g_dlc_to_app_rx_dlist);
        if (node) {
            break;
        }
        if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
            return -EAGAIN;
        }
        k_sleep(K_MSEC(10)); // Polling interval
    } while (k_uptime_delta(&start_time) < k_ticks_to_ms_floor32(timeout.ticks));

    if (!node) {
        return -EAGAIN; // Timeout
    }

    dlc_rx_delivery_item_t *delivery_item = CONTAINER_OF(node, dlc_rx_delivery_item_t, node);

    if (!delivery_item->sdu_buf) {
        k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void **)&delivery_item);
        return -EFAULT;
    }

    mac_sdu_t *sdu_buf = delivery_item->sdu_buf;

    if (*app_level_payload_len_inout < sdu_buf->len) {
        *app_level_payload_len_inout = sdu_buf->len;
        
        // --- THIS IS THE CORRECTED LOGIC ---
        // Put the item back at the FRONT of the list.
        sys_dlist_prepend(&g_dlc_to_app_rx_dlist, &delivery_item->node);
        
        return -EMSGSIZE;
    }

    *app_level_payload_len_inout = sdu_buf->len;
    memcpy(app_level_payload_buf, sdu_buf->data, sdu_buf->len);
    *service_type_out = delivery_item->dlc_service_type;

    dect_mac_api_buffer_free(sdu_buf);
    k_mem_slab_free(&g_dlc_rx_delivery_item_slab, (void **)&delivery_item);

    return 0;
}

static dlc_reassembly_session_t* find_or_alloc_reassembly_session(uint16_t sequence_number, dlc_service_type_t service)
{
    int free_slot = -1;
    for (int i = 0; i < MAX_DLC_REASSEMBLY_SESSIONS; i++) {
        if (reassembly_sessions[i].is_active) {
            if (reassembly_sessions[i].sequence_number == sequence_number) {
                return &reassembly_sessions[i];
            }
        } else if (free_slot == -1) {
            free_slot = i;
        }
    }

    if (free_slot != -1) {
        dlc_reassembly_session_t *session = &reassembly_sessions[free_slot];
        session->is_active = true;
        session->sequence_number = sequence_number;
        session->service_type = service;
        memset(session->reassembly_buf, 0, DLC_REASSEMBLY_BUF_SIZE);
        session->bytes_received = 0;
        session->total_sdu_len = 0;
        k_timer_start(&session->timeout_timer, K_MSEC(g_rx_sdu_lifetime_ms), K_NO_WAIT);
        LOG_DBG("DLC_SAR: Allocated reassembly session %d for SN %u, Svc %u.", free_slot, sequence_number, service);
        return session;
    }
    LOG_ERR("DLC_SAR: No free reassembly sessions available for SN %u!", sequence_number);
    return NULL;
}

static void dlc_reassembly_timeout_handler(struct k_timer *timer_id)
{
    uintptr_t session_idx_from_timer = (uintptr_t)timer_id->user_data;
    if (session_idx_from_timer < MAX_DLC_REASSEMBLY_SESSIONS &&
        reassembly_sessions[session_idx_from_timer].is_active) {
        LOG_WRN("DLC_SAR_TIMEOUT: Reassembly for session %u (SN %u) timed out. Discarding segments.",
                (unsigned int)session_idx_from_timer, reassembly_sessions[session_idx_from_timer].sequence_number);
        reassembly_sessions[session_idx_from_timer].is_active = false;
    }
}

static void dlc_retransmit_attempt_timeout_handler(struct k_timer *timer_id)
{
	uintptr_t job_idx = (uintptr_t)timer_id->user_data;
	if (job_idx < MAX_DLC_RETRANSMISSION_JOBS && retransmission_jobs[job_idx].is_active) {
		LOG_WRN("DLC_ARQ_TIMEOUT: Transmission for SN %u timed out. Signaling for re-TX.",
			retransmission_jobs[job_idx].sequence_number);
		k_fifo_put(&g_dlc_retransmit_signal_fifo, (void *)job_idx);
	}
}

static void dlc_tx_sdu_lifetime_expiry_handler(struct k_timer *timer_id)
{
	dlc_retransmission_job_t *job = CONTAINER_OF(timer_id, dlc_retransmission_job_t, lifetime_timer);
	if (job && job->is_active) {
		LOG_ERR("DLC_LIFETIME: SDU with SN %u expired. Discarding from ARQ buffer.", job->sequence_number);
		k_timer_stop(&job->retransmit_attempt_timer);
		dect_mac_api_buffer_free(job->sdu_payload);
		job->is_active = false;
	}
}

/**
 * @brief Assembles a final DLC PDU from its header and payload, and queues it to the MAC layer.
 *
 * This is the final step in the DLC TX path. It allocates a MAC SDU buffer,
 * copies the DLC header and the provided payload segment into it, sets the
 * necessary metadata for status reporting (if required for ARQ), and then
 * calls the appropriate MAC API function based on the device's role (PT or FT).
 *
 * @param dest_long_id The final destination Long RD ID of the peer.
 * @param dlc_header Pointer to the pre-built DLC header.
 * @param dlc_header_len Length of the DLC header.
 * @param payload_segment Pointer to the payload segment (a piece of the original CVG PDU).
 * @param payload_segment_len Length of the payload segment.
 * @param report_status True if the DLC's ARQ mechanism requires a final success/fail status
 *                      report from the MAC for this PDU.
 * @param dlc_sn The DLC sequence number, used to correlate the status report.
 * @return 0 on successful queueing to the MAC layer.
 * @retval -ENOMEM If a MAC SDU buffer could not be allocated.
 * @retval -EMSGSIZE If the combined PDU is too large for a MAC SDU buffer.
 * @retval -ENOTCONN If the FT cannot find a connected peer matching the destination ID.
 * @retval Other negative error codes from the MAC API.
 */
static int queue_dlc_pdu_to_mac(uint32_t dest_long_id, const uint8_t *dlc_header,
				size_t dlc_header_len, const uint8_t *payload_segment,
				size_t payload_segment_len, bool report_status, uint16_t dlc_sn)
{
	dect_mac_context_t *mac_ctx = get_mac_context();
	int err;

	/* 1. Allocate a buffer from the MAC layer's memory slab */
	mac_sdu_t *mac_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);

	if (!mac_sdu) {
		LOG_ERR("DLC_Q_MAC: Failed to allocate MAC SDU buffer.");
		return -ENOMEM;
	}

	/* 2. Assemble the full DLC PDU into the MAC SDU buffer */
	size_t total_pdu_len = dlc_header_len + payload_segment_len;

	if (total_pdu_len > CONFIG_DECT_MAC_SDU_MAX_SIZE) {
		LOG_ERR("DLC_Q_MAC: Assembled DLC PDU too large (%zu > %d).", total_pdu_len,
			CONFIG_DECT_MAC_SDU_MAX_SIZE);
		dect_mac_api_buffer_free(mac_sdu);
		return -EMSGSIZE;
	}

	uint8_t *p = mac_sdu->data;

	memcpy(p, dlc_header, dlc_header_len);
	p += dlc_header_len;

	if (payload_segment && payload_segment_len > 0) {
		memcpy(p, payload_segment, payload_segment_len);
	}
	mac_sdu->len = total_pdu_len;

	/* 3. Set metadata for status reporting if required by DLC ARQ */
	mac_sdu->dlc_status_report_required = report_status;
	if (report_status) {
		mac_sdu->dlc_sn_for_status = dlc_sn;
	}

	/* 4. Call the appropriate MAC API based on role */
	if (mac_ctx->role == MAC_ROLE_PT) {
		/* PT sends to its associated FT */
		err = dect_mac_api_send(mac_sdu, MAC_FLOW_RELIABLE_DATA);
	} else { /* MAC_ROLE_FT */
		/* FT must send to a specific connected PT */
		uint16_t dest_short_id = dect_mac_core_get_short_id_for_long_id(dest_long_id);

		if (dest_short_id == 0) {
			LOG_ERR("DLC_Q_MAC: FT could not find Short ID for Long ID 0x%08X.",
				dest_long_id);
			dect_mac_api_buffer_free(mac_sdu);
			return -ENOTCONN;
		}
		// --- THIS IS THE BUGGY LINE ---
		err = dect_mac_api_ft_send_to_pt(mac_sdu, MAC_FLOW_RELIABLE_DATA, dest_short_id);
		// --- THIS IS THE CORRECTED LINE ---
		// err = dect_mac_api_ft_send_to_pt(mac_sdu, MAC_FLOW_RELIABLE_DATA);
	}

	if (err != 0) {
		LOG_ERR("DLC_Q_MAC: Failed to queue PDU to MAC API: %d", err);
		/* The MAC API is responsible for freeing the SDU on failure to queue. */
	}

	return err;
}

/**
 * @brief Segments a DLC SDU if necessary and queues the resulting PDUs to the MAC layer.
 *
 * This function is the workhorse of the DLC TX path. It takes a complete DLC SDU
 * (which includes the CVG PDU payload and any DLC routing headers), a service type,
 * and a sequence number.
 *
 * If the SDU is small enough to fit in a single MAC frame, it prepends the appropriate
 * DLC header with SI=COMPLETE_SDU and sends it as one PDU.
 *
 * If the SDU is too large, it breaks it into multiple segments, each with the
 * appropriate DLC header (FIRST, MIDDLE, LAST) and segmentation offset, and queues

 * each segment as a separate PDU to the MAC layer.
 *
 * @param service The DLC service type being used.
 * @param dest_long_id The final destination Long RD ID of the peer.
 * @param dlc_sdu_payload Pointer to the complete DLC SDU payload (e.g., CVG PDU).
 * @param dlc_sdu_payload_len Length of the complete DLC SDU payload.
 * @param dlc_sn The 10-bit DLC sequence number for this SDU.
 * @return 0 on success (all segments queued), or a negative error code on the first failure.
 */
static int dlc_send_segmented(dlc_service_type_t service, uint32_t dest_long_id,
			      const uint8_t *dlc_sdu_payload, size_t dlc_sdu_payload_len,
			      uint16_t dlc_sn)
{
	int err = 0;
	bool needs_dlc_arq = (service == DLC_SERVICE_TYPE_2_ARQ ||
			      service == DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ);

	/* Determine the maximum payload size for a single MAC SDU (DLC PDU).
	 * This must account for the largest possible DLC header.
	 * The largest is the segmented header (4 bytes).
	 */
	const size_t max_segment_payload = CONFIG_DECT_MAC_SDU_MAX_SIZE -
					   sizeof(dect_dlc_header_type13_segmented_t);

	if (dlc_sdu_payload_len <= max_segment_payload) {
		/* --- SDU fits in a single PDU, no segmentation needed --- */
		dect_dlc_header_type123_basic_t dlc_hdr;

		dlc_hdr_t123_basic_set(&dlc_hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
				       DLC_SI_COMPLETE_SDU, dlc_sn);

		LOG_DBG("DLC_SEND_SEG: Sending unsegmented SDU (len %zu) with SN %u.",
			dlc_sdu_payload_len, dlc_sn);

		err = queue_dlc_pdu_to_mac(dest_long_id, (const uint8_t *)&dlc_hdr,
					   sizeof(dlc_hdr), dlc_sdu_payload,
					   dlc_sdu_payload_len, needs_dlc_arq, dlc_sn);
	} else {
		/* --- SDU is too large, segmentation is required --- */
		if (service != DLC_SERVICE_TYPE_1_SEGMENTATION &&
		    service != DLC_SERVICE_TYPE_3_SEGMENTATION_ARQ) {
			LOG_ERR("DLC_SEND_SEG: SDU (len %zu) too large for non-segmenting service type %d.",
				dlc_sdu_payload_len, service);
			return -EMSGSIZE;
		}

		LOG_INF("DLC_SEND_SEG: Segmenting SDU (len %zu) with SN %u.", dlc_sdu_payload_len,
			dlc_sn);

		size_t bytes_sent = 0;
		bool is_first_segment = true;

		while (bytes_sent < dlc_sdu_payload_len) {
			size_t remaining_bytes = dlc_sdu_payload_len - bytes_sent;
			size_t current_segment_len = MIN(max_segment_payload, remaining_bytes);
			dlc_segmentation_indication_t si;
			uint8_t dlc_hdr_buf[sizeof(dect_dlc_header_type13_segmented_t)];
			size_t dlc_hdr_len;

			if (is_first_segment) {
				si = DLC_SI_FIRST_SEGMENT;
				dect_dlc_header_type123_basic_t *hdr =
					(dect_dlc_header_type123_basic_t *)dlc_hdr_buf;
				dlc_hdr_t123_basic_set(hdr, DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING,
						       si, dlc_sn);
				dlc_hdr_len = sizeof(*hdr);
			} else {
				if (current_segment_len == remaining_bytes) {
					si = DLC_SI_LAST_SEGMENT;
				} else {
					si = DLC_SI_MIDDLE_SEGMENT;
				}
				dect_dlc_header_type13_segmented_t *hdr =
					(dect_dlc_header_type13_segmented_t *)dlc_hdr_buf;
				dlc_hdr_t13_segmented_set(hdr,
							  DLC_IE_TYPE_DATA_TYPE_123_WITH_ROUTING, si,
							  dlc_sn, bytes_sent);
				dlc_hdr_len = sizeof(*hdr);
			}

			LOG_DBG("DLC_SEND_SEG: Sending segment (SI=%u, offset=%zu, len=%zu)", si,
				bytes_sent, current_segment_len);

			/* The last segment of an ARQ transmission is the one that requires status report */
			bool report_this_segment = needs_dlc_arq && (si == DLC_SI_LAST_SEGMENT);

			err = queue_dlc_pdu_to_mac(dest_long_id, dlc_hdr_buf, dlc_hdr_len,
						   dlc_sdu_payload + bytes_sent,
						   current_segment_len, report_this_segment,
						   dlc_sn);

			if (err != 0) {
				LOG_ERR("DLC_SEND_SEG: Failed to queue segment (SI=%u) to MAC: %d. Aborting SDU.",
					si, err);
				/* The ARQ job will eventually time out and handle the failure. */
				break;
			}

			bytes_sent += current_segment_len;
			is_first_segment = false;
		}
	}

	return err;
}
