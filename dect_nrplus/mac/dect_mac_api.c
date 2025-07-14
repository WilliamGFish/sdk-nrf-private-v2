/* dect_mac/dect_mac_api.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h> // For memset if used, though k_mem_slab_alloc often returns zeroed or uninit memory
#include <zephyr/sys/dlist.h>

#include <mac/dect_mac_api.h>
#include <mac/dect_mac_sm.h>        // For MAC_EVENT_CMD_ENTER_PAGING_MODE (event type)
#include <mac/dect_mac_main_dispatcher.h> // For mac_event_msgq (external from dect_mac_phy_if.c)
#include <mac/dect_mac_core.h>      // For get_mac_context() to check role
#include <mac/dect_mac_context.h>   // For dect_mac_context_t and role_ctx for FT FIFOs

LOG_MODULE_REGISTER(dect_mac_api, CONFIG_DECT_MAC_API_LOG_LEVEL);


// Define the memory slab for mac_sdu_t buffers
K_MEM_SLAB_DEFINE(g_mac_sdu_slab, sizeof(mac_sdu_t), MAX_MAC_SDU_BUFFERS_CONFIG, 4);

sys_dlist_t *g_dlc_rx_sdu_dlist_ptr = NULL;


// Define the list structures
sys_dlist_t g_mac_tx_dlist_high_priority;
sys_dlist_t g_mac_tx_dlist_reliable_data;
sys_dlist_t g_mac_tx_dlist_best_effort;

sys_dlist_t * const mac_tx_dlists[MAC_FLOW_COUNT] = {
    &g_mac_tx_dlist_high_priority,
    &g_mac_tx_dlist_reliable_data,
    &g_mac_tx_dlist_best_effort
};

// External message queue for sending commands to the MAC thread
// Defined in dect_mac_phy_if.c
extern struct k_msgq mac_event_msgq;


int dect_mac_api_init(sys_dlist_t *rx_dlist_from_dlc)
{
    if (!rx_dlist_from_dlc) {
        LOG_ERR("DLC RX dlist pointer cannot be NULL for MAC API init.");
        return -EINVAL;
    }

    g_dlc_rx_sdu_dlist_ptr = rx_dlist_from_dlc;

    LOG_INF("MAC API Initialized. DLC RX dlist registered.");
    return 0;
}

mac_sdu_t* dect_mac_api_buffer_alloc(k_timeout_t timeout)
{
    mac_sdu_t *sdu = NULL;
    int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void **)&sdu, timeout);
    if (ret != 0) {                
        if (ret == -ENOMEM) {
            if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
                LOG_DBG("Failed to allocate MAC SDU buffer (no wait), slab empty.");
            } else {
                LOG_WRN("Failed to allocate MAC SDU buffer (timeout), slab empty.");
            }
        } else {
            LOG_ERR("Failed to allocate MAC SDU buffer, unexpected err: %d", ret);
        }
        return NULL;
    }
    // This memset correctly initializes the new boolean field to false and the SN to 0.
    // memset(sdu, 0, sizeof(mac_sdu_t));
    sdu->len = 0;
    sdu->target_peer_short_rd_id = 0;
    sdu->dlc_status_report_required = false;
    sdu->dlc_sn_for_status = 0;
    return sdu;
}

void dect_mac_api_buffer_free(mac_sdu_t *sdu)
{
    if (sdu == NULL) {
        LOG_WRN("Attempted to free a NULL SDU buffer.");
        return;
    }
    k_mem_slab_free(&g_mac_sdu_slab, (void **)&sdu);
}

int dect_mac_api_send(mac_sdu_t *sdu, mac_flow_id_t flow)
{
	dect_mac_context_t *ctx = get_mac_context();
	if (ctx->role == MAC_ROLE_FT) {
		LOG_ERR("Generic dect_mac_api_send() called by FT. Use dect_mac_api_ft_send_to_pt() instead.");
		if (sdu) {
			dect_mac_api_buffer_free(sdu);
		}
		return -EPERM;
	}

	if (sdu == NULL) {
		return -EINVAL;
	}
	if (flow >= MAC_FLOW_COUNT) {
		dect_mac_api_buffer_free(sdu);
		return -EINVAL;
	}

	if (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING) {
		LOG_DBG("PT_SEND_API: Handover in progress. Buffering SDU (len %u) to holding queue.", sdu->len);
		sys_dlist_append(&ctx->role_ctx.pt.handover_tx_holding_dlist, &sdu->node);
		return 0;
	}

	if (ctx->state < MAC_STATE_ASSOCIATED) {
		LOG_WRN("PT_SEND_API: Cannot send, not associated. Dropping SDU.");
		dect_mac_api_buffer_free(sdu);
		return -ENETDOWN;
	}

	LOG_DBG("PT_SEND_API: Queueing SDU (len %u) to generic MAC TX Flow %d (for associated FT)", sdu->len, flow);
	// k_fifo_put(mac_tx_fifos[flow], sdu);
    // // Use sys_dlist_append to add to the back of the queue
    // sys_dlist_append(mac_tx_dlists[flow], &sdu->node);     
        // Add SDU to the appropriate TX dlist based on flow
    sys_dlist_append(mac_tx_dlists[flow], &sdu->node);
	return 0;
}

int dect_mac_api_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id)
{
    dect_mac_context_t *ctx = get_mac_context();

    if (ctx->role != MAC_ROLE_FT) {
        LOG_ERR("FT_SEND_API: Not in FT role. Cannot send to specific PT.");
        if (sdu) dect_mac_api_buffer_free(sdu);
        return -EPERM;
    }
    if (sdu == NULL) {
        LOG_ERR("FT_SEND_API: Cannot send NULL SDU.");
        return -EINVAL;
    }
    if (flow >= MAC_FLOW_COUNT) {
        LOG_ERR("FT_SEND_API: Invalid MAC flow ID: %d", flow);
        dect_mac_api_buffer_free(sdu);
        return -EINVAL;
    }
    if (target_pt_short_rd_id == 0 || target_pt_short_rd_id == 0xFFFF) {
        LOG_ERR("FT_SEND_API: Invalid target PT ShortID: 0x%04X", target_pt_short_rd_id);
        dect_mac_api_buffer_free(sdu);
        return -EINVAL;
    }
    if (sdu->len == 0 || sdu->len > CONFIG_DECT_MAC_SDU_MAX_SIZE) {
        LOG_ERR("FT_SEND_API: Invalid SDU length for TX: %u", sdu->len);
        dect_mac_api_buffer_free(sdu);
        return -EMSGSIZE;
    }
    // sdu->target_peer_short_rd_id should already be set by caller or here.
    // Let's enforce it or set it if not already.
    sdu->target_peer_short_rd_id = target_pt_short_rd_id;

    int target_peer_slot_idx = -1;
    for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
        if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
            ctx->role_ctx.ft.connected_pts[i].short_rd_id == target_pt_short_rd_id) {
            target_peer_slot_idx = i;
            break;
        }
    }

    if (target_peer_slot_idx == -1) {
        LOG_ERR("FT_SEND_API: Target PT ShortID 0x%04X not found or not connected.", target_pt_short_rd_id);
        dect_mac_api_buffer_free(sdu);
        return -ENOTCONN;
    }

    // // from k_fifo to sys_dlist_t.
    // sys_dlist_t *target_dlist_ptr = NULL;
    // switch (flow) {
    //     case MAC_FLOW_HIGH_PRIORITY:
    //         target_dlist_ptr = &ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx].high_priority_dlist;
    //         break;
    //     case MAC_FLOW_RELIABLE_DATA:
    //         target_dlist_ptr = &ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx].reliable_data_dlist;
    //         break;
    //     case MAC_FLOW_BEST_EFFORT:
    //         target_dlist_ptr = &ctx->role_ctx.ft.peer_tx_data_dlists[target_peer_slot_idx].best_effort_dlist;
    //         break;
    //     default:
    //         dect_mac_api_buffer_free(sdu);
    //         return -EINVAL;
    // }

    LOG_DBG("FT_SEND_API: Queueing SDU (len %u) to PT Slot %d (ShortID 0x%04X), Flow %d",
            sdu->len, target_peer_slot_idx, target_pt_short_rd_id, flow);

    // // Use sys_dlist_append to add the SDU's node to the tail of the per-peer list.
    // sys_dlist_append(target_dlist_ptr, &sdu->node);


    if (!sdu || flow >= MAC_FLOW_COUNT || sdu->len > CONFIG_DECT_MAC_SDU_MAX_SIZE || !target_pt_short_rd_id) {
        return -EINVAL;
    }
    // Check if MAC is in FT role and PT is connected (implementation-specific)
    // Add SDU to TX dlist with target PT info
    sdu->target_peer_short_rd_id = target_pt_short_rd_id;
    sys_dlist_append(mac_tx_dlists[flow], &sdu->node);

    return 0;    
}

int dect_mac_api_enter_paging_mode(void)
{
    dect_mac_context_t *ctx = get_mac_context();
    if (ctx->role != MAC_ROLE_PT) {
        LOG_WRN("PAGING_CMD_API: Paging mode request ignored, not in PT role.");
        return -EPERM;
    }
    // Further state checks (e.g., must be associated) will be done by the PT state machine.

    struct dect_mac_event_msg msg = { .type = MAC_EVENT_CMD_ENTER_PAGING_MODE };
    // msg.modem_time_of_event = k_uptime_get(); // Or not strictly needed for CMD events
    int ret = k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("PAGING_CMD_API: Failed to queue CMD_ENTER_PAGING_MODE to MAC thread: %d", ret);
        return -EIO;
    }
    LOG_INF("PAGING_CMD_API: CMD_ENTER_PAGING_MODE queued to MAC thread.");
    return 0;
}

int dect_mac_api_ft_page_pt(uint32_t target_pt_long_rd_id)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->role != MAC_ROLE_FT) {
		LOG_ERR("PAGE_API: Not in FT role.");
		return -EPERM;
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
		    ctx->role_ctx.ft.connected_pts[i].long_rd_id == target_pt_long_rd_id) {
			LOG_INF("PAGE_API: Flagging PT 0x%08X (slot %d) for paging.",
				target_pt_long_rd_id, i);
			ctx->role_ctx.ft.connected_pts[i].paging_pending = true;
			return 0;
		}
	}

	LOG_WRN("PAGE_API: Could not find associated PT with Long ID 0x%08X to page.",
		target_pt_long_rd_id);
	return -ENOTCONN;
}
