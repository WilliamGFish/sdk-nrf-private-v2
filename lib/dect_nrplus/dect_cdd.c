/* lib/dect_nrplus/dect_cdd.c */
// Overview: New file implementing the core logic for the CDD service.

#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>
#include <dect_cdd.h>
#include <dect_cvg.h>
#include <dect_dlc.h>

#include "nrf_modem_dect_phy.h"
#include <mac/nrf_modem_dect_phy.h>

// #include "../../drivers/net/dect_nrplus/dect_nrplus.h" /* For dect_nrplus_l2_set_sixlowpan_context */
#include <dect_nrplus.h> /* For dect_nrplus_l2_set_sixlowpan_context */

LOG_MODULE_REGISTER(dect_cdd, CONFIG_DECT_CDD_LOG_LEVEL);


static struct {
	cdd_content_pdu_t content;
	bool is_valid;
	cdd_ipv6_prefix_handler_t prefix_handler;
} g_cdd_ctx;

static void pt_send_cdd_request(void)
{
	dect_mac_context_t *mac_ctx = get_mac_context();

	if (!mac_ctx->role_ctx.pt.associated_ft.is_valid) {
		LOG_ERR("CDD_REQ: Cannot send, PT not associated.");
		return;
	}

	uint8_t pdu_buf[sizeof(cvg_ie_ep_mux_t) + sizeof(cdd_request_pdu_t)];
	cvg_ie_ep_mux_t *ep_mux = (cvg_ie_ep_mux_t *)pdu_buf;

	ep_mux->header.ext_mt_f2c_or_type =
		((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) | (CVG_IE_TYPE_EP_MUX & 0x1F);
	ep_mux->endpoint_mux_be = sys_cpu_to_be16(CVG_EP_MANAGEMENT_CDD);

	cdd_request_pdu_t *req = (cdd_request_pdu_t *)(pdu_buf + sizeof(cvg_ie_ep_mux_t));

	req->reserved_type = CDD_REQUEST_TYPE_COMPLETE_CONFIG;

	LOG_INF("CDD_REQ: Sending Configuration Data Request to FT.");
	dect_cvg_send(CVG_SERVICE_TYPE_1_SEQ_NUM, mac_ctx->role_ctx.pt.associated_ft.long_rd_id,
		      pdu_buf, sizeof(pdu_buf));
}


/**
 * @brief Builds the FT's own configuration data to be served via CDD.
 *
 * This function should be called by the FT when it initializes. It populates
 * the CDD content with network parameters, like the IPv6 prefix.
 */
/**
 * @brief Builds the FT's own configuration data to be served via CDD.
 *
 * This function should be called by the FT when it initializes. It populates
 * the CDD content with network parameters, like the IPv6 prefix. This version
 * uses fixed-size buffers to prevent overflows and includes bounds checking.
 */
void dect_cdd_ft_build_own_config(void)
{
	dect_mac_context_t *mac_ctx = get_mac_context();

	if (mac_ctx->role != MAC_ROLE_FT) {
		return;
	}

	LOG_INF("CDD_FT: Building own configuration data.");

	g_cdd_ctx.content.type = 0; /* Full Configuration Data Content PDU */
	g_cdd_ctx.content.sink_addr_be = sys_cpu_to_be32(mac_ctx->own_long_rd_id);
	g_cdd_ctx.content.app_seq_num = 1; /* Initial sequence number */
	g_cdd_ctx.content.num_data_items = 0;

	/* --- Add IPv6 Prefix Data Item --- */
	/* This simulates the FT getting its prefix from an upstream router. */
	/* For a real product, this would be dynamically configured. */
	struct in6_addr prefix;
	uint8_t context_id = 1; /* Use Context ID 1 for this prefix */

	net_addr_pton(AF_INET6, CONFIG_NET_CONFIG_MY_IPV6_PREFIX, &prefix);

	/* Check if there is space for a new item in the fixed-size array */
	if (g_cdd_ctx.content.num_data_items >= DECT_CDD_MAX_ITEMS) {
		LOG_ERR("CDD_FT: Cannot add new data item, CDD content is full.");
		return;
	}

	cdd_data_item_t *item = &g_cdd_ctx.content.items[g_cdd_ctx.content.num_data_items];

	sys_put_be16(CVG_EP_IPV6_PROFILE, item->ep_address);

	/* Check if the required payload will fit in the item's fixed-size payload buffer */
	size_t required_payload_len = sizeof(cdd_ipv6_addr_element_hdr_t) + 8; /* 8 bytes for a /64 prefix */

	if (required_payload_len > DECT_CDD_MAX_ITEM_PAYLOAD) {
		LOG_ERR("CDD_FT: IPv6 prefix data (%zu bytes) is too large for item payload buffer (%d bytes).",
			required_payload_len, DECT_CDD_MAX_ITEM_PAYLOAD);
		return;
	}

	cdd_ipv6_addr_element_hdr_t *hdr = (cdd_ipv6_addr_element_hdr_t *)item->payload;
	hdr->element_type = 1;   /* IPv6 Address Element */
	hdr->element_version = 0;
	hdr->prefix_type = 0;    /* 64-bit prefix */
	hdr->context_usage = 1;  /* Used for header compression */
	hdr->context_id = context_id;
	hdr->service_id = 0;     /* RFU */

	uint8_t *prefix_ptr = item->payload + sizeof(*hdr);

	memcpy(prefix_ptr, &prefix.s6_addr, 8); /* Copy 64-bit /64 prefix */

	item->length = required_payload_len; /* Set the actual length used in the payload */
	g_cdd_ctx.content.num_data_items++;
	g_cdd_ctx.is_valid = true;

	// LOG_INF("CDD_FT: Added IPv6 prefix %s to CDD content for CID %u.",
	// 	net_sprint_ipv6_addr(&prefix), context_id);

	LOG_INF("CDD_FT: Added IPv6 prefix to CDD content for CID %u.", context_id);
	LOG_HEXDUMP_DBG(&prefix, sizeof(prefix), "IPv6 prefix:");
	
}


void dect_cdd_init(void)
{
	memset(&g_cdd_ctx, 0, sizeof(g_cdd_ctx));
	LOG_INF("CDD Service Initialized.");
}

void dect_cdd_register_prefix_handler(cdd_ipv6_prefix_handler_t handler)
{
	g_cdd_ctx.prefix_handler = handler;
}

int dect_cdd_ft_set_data(const cdd_content_pdu_t *data)
{
	if (!data) {
		return -EINVAL;
	}
	memcpy(&g_cdd_ctx.content, data, sizeof(cdd_content_pdu_t));
	g_cdd_ctx.content.app_seq_num++; /* Increment sequence on new data */
	g_cdd_ctx.is_valid = true;
	LOG_INF("CDD_FT: New configuration data set. AppSeqNum is now %u.",
		g_cdd_ctx.content.app_seq_num);
	return 0;
}

void dect_cdd_pt_process_beacon_info(uint32_t sink_addr, uint8_t app_seq_num)
{
	if (!g_cdd_ctx.is_valid || g_cdd_ctx.content.sink_addr_be != sys_cpu_to_be32(sink_addr) ||
	    g_cdd_ctx.content.app_seq_num != app_seq_num) {
		LOG_INF("CDD_PT: Stale or missing config data. Local(valid:%d, sink:0x%08X, seq:%u), Beacon(sink:0x%08X, seq:%u). Requesting update.",
			g_cdd_ctx.is_valid, sys_be32_to_cpu(g_cdd_ctx.content.sink_addr_be),
			g_cdd_ctx.content.app_seq_num, sink_addr, app_seq_num);
		pt_send_cdd_request();
	}
}

void dect_cdd_handle_incoming_pdu(const uint8_t *data, size_t len, uint32_t source_rd_id)
{
	dect_mac_context_t *mac_ctx = get_mac_context();

	if (mac_ctx->role == MAC_ROLE_FT) {
		/* This must be a request from a PT */
		if (len == sizeof(cdd_request_pdu_t)) {
			const cdd_request_pdu_t *req = (const cdd_request_pdu_t *)data;

			if ((req->reserved_type & 0x1F) == CDD_REQUEST_TYPE_COMPLETE_CONFIG) {
				LOG_INF("CDD_FT: Rcvd Config Request from 0x%08X. Sending response.",
					source_rd_id);

				if (g_cdd_ctx.is_valid) {
					/* Calculate actual content size based on variable item lengths */
					size_t data_items_total_len = 0;

					for (int i = 0; i < g_cdd_ctx.content.num_data_items;
					     i++) {
						/* Each item's size is header + actual payload length */
						data_items_total_len +=
							offsetof(cdd_data_item_t, payload) +
							g_cdd_ctx.content.items[i].length;
					}

					size_t content_len =
						offsetof(cdd_content_pdu_t, items) +
						data_items_total_len;
					size_t pdu_len = sizeof(cvg_ie_ep_mux_t) + content_len;

					/* Use a VLA on the stack. Assumes stack is large enough. */
					/* A more robust solution might use a dedicated memory pool. */
					if (pdu_len > 256) { /* Safety check */
						LOG_ERR("CDD_FT: Response PDU too large for stack VLA (%zu).", pdu_len);
						return;
					}
					uint8_t pdu_buf[pdu_len];

					cvg_ie_ep_mux_t *ep_mux = (cvg_ie_ep_mux_t *)pdu_buf;

					ep_mux->header.ext_mt_f2c_or_type =
						((CVG_EXT_NO_LEN_FIELD & 0x03) << 6) |
						(CVG_IE_TYPE_EP_MUX & 0x1F);
					ep_mux->endpoint_mux_be =
						sys_cpu_to_be16(CVG_EP_MANAGEMENT_CDD);

					memcpy(pdu_buf + sizeof(cvg_ie_ep_mux_t),
					       &g_cdd_ctx.content, content_len);

					dect_cvg_send(CVG_SERVICE_TYPE_1_SEQ_NUM, source_rd_id,
						      pdu_buf, pdu_len);

					// k_free(pdu_buf);
				} else {
					LOG_WRN("CDD_FT: Rcvd request, but no valid CDD data to send.");
				}
			}
		}
	} else { /* PT Role */
		/* This must be a response from the FT */
		if (len >= offsetof(cdd_content_pdu_t, items)) {
			const cdd_content_pdu_t *resp = (const cdd_content_pdu_t *)data;

			LOG_INF("CDD_PT: Rcvd Config Content from FT. Sink:0x%08X, Seq:%u, Items:%u",
				sys_be32_to_cpu(resp->sink_addr_be), resp->app_seq_num,
				resp->num_data_items);

			/* Store the new configuration metadata */
			g_cdd_ctx.content.sink_addr_be = resp->sink_addr_be;
			g_cdd_ctx.content.app_seq_num = resp->app_seq_num;
			g_cdd_ctx.is_valid = true;

			/* Parse items and invoke callback if an IPv6 prefix is found */
			const cdd_data_item_t *item = resp->items;
			size_t remaining_len = len - offsetof(cdd_content_pdu_t, items);

			for (int i = 0; i < resp->num_data_items; i++) {
				if (remaining_len < offsetof(cdd_data_item_t, payload)) {
					LOG_ERR("CDD_PT: Malformed PDU, not enough data for item %d header.", i);
					break;
				}
				if (remaining_len < offsetof(cdd_data_item_t, payload) + item->length) {
					LOG_ERR("CDD_PT: Malformed PDU, item %d length %u exceeds remaining data %zu.",
						i, item->length, remaining_len);
					break;
				}

				uint16_t ep = sys_be16_to_cpu(*(uint16_t *)item->ep_address);

				if (ep == CVG_EP_IPV6_PROFILE) {
					const cdd_ipv6_addr_element_hdr_t *hdr =
						(const cdd_ipv6_addr_element_hdr_t *)item->
						payload;
					size_t prefix_bytes = (hdr->prefix_type == 0) ? 8 : 16;
					size_t expected_item_len = sizeof(*hdr) + prefix_bytes;

					if (item->length < expected_item_len) {
						LOG_WRN("CDD_PT: IPv6 data item too short (len %u).",
							item->length);
						continue;
					}

					if (hdr->context_usage == 1 && hdr->prefix_type == 0) {
						if (g_cdd_ctx.prefix_handler) {
							struct in6_addr prefix;
							uint8_t prefix_len = 64;
							uint8_t context_id = hdr->context_id;
							const uint8_t *prefix_ptr =
								item->payload + sizeof(*hdr);

							memset(&prefix, 0, sizeof(prefix));
							memcpy(&prefix.s6_addr, prefix_ptr, 8);

							LOG_INF("CDD_PT: Found IPv6 prefix for 6LoWPAN context CID %u.",
								context_id);
							g_cdd_ctx.prefix_handler(&prefix,
									       prefix_len,
									       context_id);
						}
					}
				}

				size_t current_item_size =
					offsetof(cdd_data_item_t, payload) + item->length;
				item = (const cdd_data_item_t *)((const uint8_t *)item +
								 current_item_size);
				remaining_len -= current_item_size;
			}
		}
	}
}