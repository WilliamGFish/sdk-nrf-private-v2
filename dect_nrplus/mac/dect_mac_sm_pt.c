/* dect_mac/dect_mac_sm_pt.c */
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <zephyr/sys/util.h>

#include <mac/dect_mac_main.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_ctrl.h>
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_api.h>       // For g_mac_sdu_slab
#include <dect_cdd.h>
#include <dect_dlc.h>
#include <mac/dect_mac_security.h>  // For security functions
#include <mac/dect_mac_timeline_utils.h>


LOG_MODULE_REGISTER(dect_mac_sm_pt, CONFIG_DECT_MAC_SM_PT_LOG_LEVEL);

#ifndef HPC_RX_WINDOW_SIZE
#define HPC_RX_WINDOW_SIZE 64
#endif
#ifndef HPC_RX_FORWARD_WINDOW_MAX_ADVANCE
// Allow a jump of up to 1024
#define HPC_RX_FORWARD_WINDOW_MAX_ADVANCE 1024
#endif

// --- Static Globals for this Module ---
// Store the PCC associated with an incoming PDC for context
static struct {
    struct nrf_modem_dect_phy_pcc_event pcc_data;
    uint64_t pcc_event_modem_time; // Store the modem time of the PCC event
    bool is_valid;
} last_relevant_pcc_for_pt;


// --- Static Helper Function Prototypes ---
static void pt_handle_phy_op_complete_internal(const struct nrf_modem_dect_phy_op_complete_event *event, pending_op_type_t completed_op_type);
static void pt_handle_phy_pcc_internal(const struct nrf_modem_dect_phy_pcc_event *event, uint64_t pcc_event_time);
static void pt_handle_phy_pdc_internal(const struct nrf_modem_dect_phy_pdc_event *pdc_event, const struct nrf_modem_dect_phy_pcc_event *assoc_pcc_event, uint64_t pcc_reception_modem_time);
static void pt_handle_phy_rssi_internal(const struct nrf_modem_dect_phy_rssi_event *event);
static void pt_update_mobility_candidate(uint16_t carrier, int16_t rssi, uint32_t long_id, uint16_t short_id);

static void pt_process_identified_beacon_and_attempt_assoc(dect_mac_context_t *ctx,
                                                           const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
                                                           const dect_mac_rach_info_ie_fields_t *rach_fields,
                                                           uint32_t ft_long_id, uint16_t ft_short_id, int16_t rssi,
                                                           uint16_t beacon_rx_carrier, uint64_t beacon_pcc_rx_time);
static void pt_send_association_request_action(void);
static void pt_process_association_response_pdu(const uint8_t *mac_sdu_area_data, size_t mac_sdu_area_len,
                                                uint32_t ft_tx_long_rd_id, uint64_t assoc_resp_pcc_rx_time);
static void pt_send_keep_alive_action(void);
static void pt_process_page_indication(void);
static void pt_paging_cycle_timer_expired_action(void);

// static void pt_initiate_background_mobility_scan_action(void); // TODO
static void pt_start_authentication_with_ft_action(dect_mac_context_t *ctx); // Simplified for PSK
static void pt_authentication_complete_action(dect_mac_context_t* ctx, bool success); // Called after key derivation


static void pt_send_auth_initiate_action(void);
static void pt_send_auth_response_action(void);
static void pt_process_auth_success(dect_mac_context_t *ctx, const uint8_t *ft_mac);
static void pt_initiate_authentication_handshake(
	dect_mac_context_t *ctx, const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
	const dect_mac_rach_info_ie_fields_t *rach_fields, uint32_t ft_long_id,
	uint16_t ft_short_id, int16_t rssi_q7_1, uint16_t beacon_rx_carrier,
	uint64_t beacon_pcc_rx_time);
static void pt_process_group_assignment_ie(const uint8_t *payload, uint16_t len);
static void pt_evaluate_mobility_candidate(dect_mac_context_t *ctx,
					   const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
					   uint32_t ft_long_id, uint16_t ft_short_id,
					   int16_t rssi_q7_1, uint16_t beacon_rx_carrier);
static void pt_initiate_handover_action(dect_mac_context_t *ctx,
					dect_mobility_candidate_t *candidate);
static void pt_send_association_release_action(dect_mac_peer_info_t *old_ft_info);


// Helpers from data_path or utils (ensure they are accessible)
extern uint32_t get_subslot_duration_ticks(dect_mac_context_t *ctx);
extern uint32_t modem_us_to_ticks(uint32_t us, uint32_t tick_rate_khz);
extern void update_next_occurrence(dect_mac_context_t *ctx, dect_mac_schedule_t *schedule, uint64_t current_modem_time);
// extern uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time, uint8_t anchor_sfn_val, uint8_t target_sfn_val, uint16_t target_subslot_idx);
extern uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time, uint8_t sfn_of_anchor_relevance, uint8_t target_sfn_val, uint16_t target_subslot_idx, uint8_t link_mu_code, uint8_t link_beta_code);

// --- PT Timer Expiry Action Functions (called by dispatcher) ---
void pt_rach_backoff_timer_expired_action(void) {
    dect_mac_context_t* ctx = get_mac_context();
    LOG_INF("PT SM: RACH Backoff timer expired.");
    if (ctx->role_ctx.pt.target_ft.is_valid && ctx->role_ctx.pt.target_ft.is_fully_identified) {
        LOG_INF("PT SM: Retrying Association Request to FT ShortID 0x%04X.", ctx->role_ctx.pt.target_ft.short_rd_id);
        // State should already be PT_RACH_BACKOFF, send_association_request changes to PT_ASSOCIATING
        pt_send_association_request_action();
    } else {
        LOG_ERR("PT SM: RACH backoff expired, but no valid/fully_identified target FT. Restarting scan.");
        dect_mac_sm_pt_start_operation();
    }
}




static void pt_requeue_held_packets(dect_mac_context_t *ctx)
{
	mac_sdu_t *sdu;
	sys_dnode_t *node;
	int count = 0;

	while ((node = sys_dlist_get(&ctx->role_ctx.pt.handover_tx_holding_dlist)) != NULL) 
	{
		sdu = CONTAINER_OF(node, mac_sdu_t, node);
		
        /* Prepend to the reliable queue to send them out first */
		sys_dlist_prepend(mac_tx_dlists[MAC_FLOW_RELIABLE_DATA], &sdu->node);
		count++;
	}

	if (count > 0) {
		LOG_INF("MOBILITY: Re-queued %d held packets for transmission to new/reverted FT.", count);
	}
}


static void pt_revert_to_old_ft_after_handover_failure(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	LOG_ERR("MOBILITY: Handover to FT 0x%04X failed. Reverting to old FT 0x%04X.",
		ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.associated_ft.short_rd_id);

    pt_requeue_held_packets(ctx);

	/* Clear the failed target */
	memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
	ctx->role_ctx.pt.target_ft.is_valid = false;

	/* Cancel any active HARQ processes that were for the failed handover attempt.
	 * Re-queue their SDUs to be sent to the original FT.
	 */
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];

		if (harq_p->is_active) {
			LOG_WRN("MOBILITY: Cancelling active HARQ process %d (for PSN %u) due to handover failure.",
				i, harq_p->original_psn);
			k_timer_stop(&harq_p->retransmission_timer);
			if (harq_p->sdu) {
				/* Prepend to the reliable queue to ensure it's sent first */
				sys_dlist_prepend(mac_tx_dlists[MAC_FLOW_RELIABLE_DATA], &harq_p->sdu->node);
								harq_p->sdu = NULL;
			}
			/* Reset the HARQ process */
			memset(harq_p, 0, sizeof(dect_harq_tx_process_t));
			k_timer_init(&harq_p->retransmission_timer,
				     dect_mac_data_path_harq_timer_expired, NULL);
			harq_p->retransmission_timer.user_data = (void *)((uintptr_t)i);
		}
	}

	/* Revert state and restart timers for the old (and now current) FT */
	dect_mac_change_state(MAC_STATE_ASSOCIATED);
	LOG_INF("MOBILITY: State -> ASSOCIATED. Data TX resumed to old FT.");

	k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
		      K_MSEC(ctx->config.keep_alive_period_ms),
		      K_MSEC(ctx->config.keep_alive_period_ms));
	if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
		k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
			      K_MSEC(ctx->config.mobility_scan_interval_ms),
			      K_MSEC(ctx->config.mobility_scan_interval_ms));
	}
}


static void pt_paging_cycle_timer_expired_action(void)
{
    dect_mac_context_t* ctx = get_mac_context();
    if (ctx->state != MAC_STATE_PT_PAGING) {
        LOG_WRN("PT_PAGING: Paging timer fired in unexpected state %s. Stopping timer.",
                dect_mac_state_to_str(ctx->state));
        k_timer_stop(&ctx->role_ctx.pt.paging_cycle_timer);
        return;
    }

    if (ctx->pending_op_type != PENDING_OP_NONE) {
        LOG_WRN("PT_PAGING: Paging listen time, but op %s pending. Will retry shortly.",
                dect_pending_op_to_str(ctx->pending_op_type));
        k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer, K_MSEC(100), K_NO_WAIT); // Quick retry
        return;
    }

    LOG_INF("PT_PAGING: Waking up to listen for page (on FT carrier %u).",
            ctx->role_ctx.pt.associated_ft.operating_carrier);

    // uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
    uint32_t listen_duration_modem_units = get_subslot_duration_ticks(ctx) *
                                           SUB_SLOTS_PER_ETSI_SLOT * 2;

    int ret = dect_mac_phy_ctrl_start_rx(
        ctx->role_ctx.pt.associated_ft.operating_carrier,
        listen_duration_modem_units,
        NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
        phy_op_handle,
        0xFFFF,
        PENDING_OP_PT_PAGING_LISTEN);

    if (ret != 0) {
        LOG_ERR("PT_PAGING: Failed to schedule RX for paging listen: %d. Retrying shortly.", ret);
        k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer, K_MSEC(200), K_NO_WAIT);
    }
}


void pt_rach_response_window_timer_expired_action(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	k_timer_stop(&ctx->rach_context.rach_response_window_timer);

	bool was_handover = (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING);

	if (ctx->state != MAC_STATE_PT_WAIT_ASSOC_RESP && !was_handover) {
		LOG_WRN("PT_RACH_RESP_TIMEOUT: Timer expired in unexpected state %s. Ignoring.",
			dect_mac_state_to_str(ctx->state));
		return;
	}

	LOG_WRN("PT_RACH_RESP_TIMEOUT: No Association Response from target FT 0x%04X (L:0x%08X).",
		ctx->role_ctx.pt.target_ft.short_rd_id, ctx->role_ctx.pt.target_ft.long_rd_id);

	if (was_handover) {
		pt_revert_to_old_ft_after_handover_failure();
		return;
	}

	ctx->role_ctx.pt.current_assoc_retries++;
	if (ctx->role_ctx.pt.target_ft.is_valid &&
	    ctx->role_ctx.pt.current_assoc_retries < ctx->config.max_assoc_retries) {
		LOG_INF("PT_RACH_RESP_TIMEOUT: Retrying association to FT 0x%04X (attempt %u / %u).",
			ctx->role_ctx.pt.target_ft.short_rd_id,
			ctx->role_ctx.pt.current_assoc_retries + 1, ctx->config.max_assoc_retries);

		uint8_t ft_cwmin_sig_code =
			ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields
				.cwmin_sig_code;
		uint8_t ft_cwmax_sig_code =
			ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields
				.cwmax_sig_code;

		if (ctx->rach_context.rach_cw_current_idx < ft_cwmin_sig_code) {
			ctx->rach_context.rach_cw_current_idx = ft_cwmin_sig_code;
		}
		if (ctx->rach_context.rach_cw_current_idx < ft_cwmax_sig_code) {
			ctx->rach_context.rach_cw_current_idx++;
			LOG_DBG("PT_RACH_RESP_TIMEOUT: Increased CW index to %u for next attempt.",
				ctx->rach_context.rach_cw_current_idx);
		} else {
			LOG_DBG("PT_RACH_RESP_TIMEOUT: CW index already at max (%u) from FT.",
				ft_cwmax_sig_code);
		}
		pt_send_association_request_action();

	} else {
		LOG_ERR("PT_RACH_RESP_TIMEOUT: Max association retries (%u) for FT 0x%04X or no valid target. Restarting scan.",
			ctx->config.max_assoc_retries, ctx->role_ctx.pt.target_ft.short_rd_id);
		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.target_ft.is_valid = false;
		dect_mac_sm_pt_start_operation();
	}
}

void dect_mac_sm_pt_keep_alive_timer_expired_action(void) {
    dect_mac_context_t* ctx = get_mac_context();
    if (ctx->state == MAC_STATE_ASSOCIATED) {
        if (ctx->pending_op_type == PENDING_OP_NONE) {
            pt_send_keep_alive_action();
        } else {
            LOG_WRN("PT SM: Keep-alive time, but op %s pending. Will retry on next expiry.",
                    dect_pending_op_to_str(ctx->pending_op_type));
            // Periodic timer will fire again.
        }
    }
}

void dect_mac_sm_pt_mobility_scan_timer_expired_action(void)
{
    dect_mac_context_t* ctx = get_mac_context();
    if (ctx->state != MAC_STATE_ASSOCIATED) { // Only scan for mobility if associated
        LOG_DBG("PT SM: Mobility scan timer fired but not associated. Restarting general scan.");
        dect_mac_sm_pt_start_operation();
        return;
    }

    if (ctx->pending_op_type != PENDING_OP_NONE) {
        LOG_WRN("PT SM: Mobility scan time, but op %s pending. Deferring scan.",
                dect_pending_op_to_str(ctx->pending_op_type));
        // The timer will fire again later.
        return;
    }

    // Simple channel selection logic: scan the next channel.
    // A production system would use a more sophisticated channel hopping sequence.
    uint32_t current_carrier = ctx->role_ctx.pt.associated_ft.operating_carrier;
    uint32_t scan_carrier = (current_carrier != 0) ? (current_carrier + 1) : DEFAULT_DECT_CARRIER;
    // TODO: Add logic to wrap around the valid channel range.

    LOG_INF("PT SM: Starting mobility background RSSI scan on carrier %u.", scan_carrier);

    // uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
    // A short scan, e.g., for one or two slots duration.
    uint32_t scan_duration_modem_units = get_subslot_duration_ticks(ctx) * SUB_SLOTS_PER_ETSI_SLOT * 2;

    int ret = dect_mac_phy_ctrl_start_rssi_scan(
        scan_carrier,
        scan_duration_modem_units,
        NRF_MODEM_DECT_PHY_RSSI_INTERVAL_24_SLOTS, // Get one report for this short scan
        phy_op_handle,
        PENDING_OP_PT_MOBILITY_SCAN);

    if (ret != 0) {
        LOG_ERR("PT SM: Failed to start mobility RSSI scan: %d.", ret);
        // The periodic timer will try again on its next cycle.
    }
}


// --- PT Public Functions ---
void dect_mac_sm_pt_start_operation(void)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_change_state(MAC_STATE_PT_SCANNING);
	uint32_t scan_carrier = DEFAULT_DECT_CARRIER;
	LOG_INF("PT SM: Starting scan for FT beacons on carrier %u.", scan_carrier);

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
	ctx->role_ctx.pt.current_assoc_retries = 0;
	memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
	memset(&ctx->role_ctx.pt.associated_ft, 0, sizeof(dect_mac_peer_info_t));
	ctx->keys_provisioned = false;

	int ret = dect_mac_phy_ctrl_start_rx(scan_carrier, 0,
					   NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
					   phy_op_handle, 0xFFFF,
					   PENDING_OP_PT_SCAN);
	if (ret != 0) {
		dect_mac_enter_error_state("Failed to start initial PT scan");
	}
}

static void pt_handle_phy_pdc_pt(const struct nrf_modem_dect_phy_pdc_event *pdc_event, uint64_t event_modem_time) {
    // This is the wrapper that calls pt_handle_phy_pdc_internal
    if (last_relevant_pcc_for_pt.is_valid && last_relevant_pcc_for_pt.pcc_data.transaction_id == pdc_event->transaction_id) {
        pt_handle_phy_pdc_internal(pdc_event, &last_relevant_pcc_for_pt.pcc_data, last_relevant_pcc_for_pt.pcc_event_modem_time);
        last_relevant_pcc_for_pt.is_valid = false;
    } else {
        LOG_WRN("PT_SM_PDC_WRAP: PDC (TID %u) but no matching valid PCC stored (LastPCC TID %u, valid %d). Discarding.",
                pdc_event->transaction_id,
                last_relevant_pcc_for_pt.is_valid ? last_relevant_pcc_for_pt.pcc_data.transaction_id : 0,
                last_relevant_pcc_for_pt.is_valid);
    }
}

void dect_mac_sm_pt_handle_event(const struct dect_mac_event_msg *msg)
{
	dect_mac_context_t *ctx = get_mac_context();

	switch (ctx->state) {
	case MAC_STATE_IDLE:
		/* In IDLE, the only action is to start scanning, which is not event-driven */
		break;

	case MAC_STATE_PT_SCANNING:
	case MAC_STATE_PT_BEACON_PDC_WAIT:
		if (msg->type == MAC_EVENT_PHY_PCC) {
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
		} else if (msg->type == MAC_EVENT_PHY_PDC) {
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
		} else if (msg->type == MAC_EVENT_PHY_OP_COMPLETE) {
			pending_op_type_t op = dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete);
			if (op == PENDING_OP_PT_SCAN) {
				pt_handle_phy_op_complete_internal(&msg->data.op_complete, op);
			}
		}
		break;

	case MAC_STATE_PT_ASSOCIATING:
	case MAC_STATE_PT_RACH_BACKOFF:
	case MAC_STATE_PT_WAIT_ASSOC_RESP:
	case MAC_STATE_PT_AUTHENTICATING:
	case MAC_STATE_PT_WAIT_AUTH_CHALLENGE:
	case MAC_STATE_PT_WAIT_AUTH_SUCCESS:
	case MAC_STATE_PT_HANDOVER_ASSOCIATING:
		switch (msg->type) {
		case MAC_EVENT_PHY_OP_COMPLETE:
			pt_handle_phy_op_complete_internal(&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
			break;
		case MAC_EVENT_PHY_PCC:
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PDC:
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_TIMER_EXPIRED_RACH_BACKOFF:
			pt_rach_backoff_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_RACH_RESP_WINDOW:
			pt_rach_response_window_timer_expired_action();
			break;
		default:
			break; /* Ignore other events */
		}
		break;

	case MAC_STATE_ASSOCIATED:
	case MAC_STATE_PT_PAGING:
		switch (msg->type) {
		case MAC_EVENT_PHY_OP_COMPLETE:
			pt_handle_phy_op_complete_internal(&msg->data.op_complete,
				dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete));
			break;
		case MAC_EVENT_PHY_PCC:
			pt_handle_phy_pcc_internal(&msg->data.pcc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PDC:
			pt_handle_phy_pdc_pt(&msg->data.pdc, msg->modem_time_of_event);
			break;
		case MAC_EVENT_PHY_PCC_ERROR:
		case MAC_EVENT_PHY_PDC_ERROR:
			/* These are logged by the dispatcher, but could trigger link quality logic here */
			break;
		case MAC_EVENT_TIMER_EXPIRED_KEEPALIVE:
			dect_mac_sm_pt_keep_alive_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_MOBILITY_SCAN:
			dect_mac_sm_pt_mobility_scan_timer_expired_action();
			break;
		case MAC_EVENT_TIMER_EXPIRED_HARQ:
			dect_mac_data_path_handle_harq_nack_action(msg->data.timer_data.id);
			break;
		case MAC_EVENT_CMD_ENTER_PAGING_MODE:
			if (ctx->state == MAC_STATE_ASSOCIATED) {
				dect_mac_change_state(MAC_STATE_PT_PAGING);
				k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
				k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);
				k_timer_start(&ctx->role_ctx.pt.paging_cycle_timer, K_MSEC(CONFIG_DECT_MAC_PT_PAGING_CYCLE_MS), K_MSEC(CONFIG_DECT_MAC_PT_PAGING_CYCLE_MS));
			}
			break;
		case MAC_EVENT_TIMER_EXPIRED_PAGING_CYCLE:
			pt_paging_cycle_timer_expired_action();
			break;
		default:
			break; /* Ignore other events */
		}
		break;

	default:
		LOG_WRN("PT SM: Event %s received in unhandled state %s. Discarding.",
			dect_mac_event_to_str(msg->type), dect_mac_state_to_str(ctx->state));
		break;
	}
}

// All static helper functions (pt_handle_phy_op_complete_internal, pt_handle_phy_pcc_internal,
// pt_handle_phy_pdc_internal, pt_process_identified_beacon_and_attempt_assoc,
// pt_send_association_request_action, pt_process_association_response_pdu,
// pt_send_keep_alive_action, pt_start_authentication_with_ft_action,
// pt_authentication_complete_action) are included below with their full implementations.

// --- PT Static Helper Implementations ---
static void pt_handle_phy_op_complete_internal(const struct nrf_modem_dect_phy_op_complete_event *event,
                                               pending_op_type_t completed_op_type) {
    dect_mac_context_t* ctx = get_mac_context();
    switch (completed_op_type) {
        case PENDING_OP_PT_MOBILITY_SCAN:
            LOG_DBG("PT SM: Mobility scan op completed (err %d).", event->err);
            break;
        case PENDING_OP_PT_PAGING_LISTEN:
            if (ctx->state == MAC_STATE_PT_PAGING) {
                LOG_DBG("PT_PAGING: Paging listen RX window complete (err %d).", event->err);
            }
            break;
        case PENDING_OP_PT_SCAN:
            if (event->err == NRF_MODEM_DECT_PHY_ERR_OP_CANCELED) {
                LOG_INF("PT SM: Scan successfully canceled (Hdl %u). Presuming association attempt follows.", event->handle);
                if (ctx->state == MAC_STATE_PT_ASSOCIATING &&
                    ctx->role_ctx.pt.target_ft.is_valid &&
                    ctx->role_ctx.pt.target_ft.is_fully_identified) {
                    pt_send_association_request_action();
                } else {
                    LOG_WRN("PT_SM: Scan op cancelled but state is %s or no valid target. Restarting scan.",
                        dect_mac_state_to_str(ctx->state));
                    dect_mac_sm_pt_start_operation();
                }
            } else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_ERR("PT_SM: Scan PHY op failed (err %d, %s). Restarting scan after delay.", event->err, nrf_modem_dect_phy_err_to_str(event->err));
                // k_sleep(K_MSEC(1000 + (sys_rand32_get() % 1000)));
				uint8_t random_byte;
				sys_rand_get(&random_byte, sizeof(random_byte));
				k_sleep(K_MSEC(1000 + (random_byte % 100)));  // 0-99ms additional delay				
                dect_mac_sm_pt_start_operation();
            } else {
                LOG_INF("PT_SM: Scan PHY op completed (Hdl %u), but no suitable FT found during PDC parsing. Restarting scan.", event->handle);
                dect_mac_sm_pt_start_operation();
            }
            break;

       case PENDING_OP_PT_RACH_ASSOC_REQ:
            if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_INF("PT_SM: Association Request TX successful (Hdl %u). Waiting for Response.", event->handle);
                dect_mac_change_state(MAC_STATE_PT_WAIT_ASSOC_RESP);

                uint32_t resp_win_ms = ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us / 1000;
                if (resp_win_ms < 10) resp_win_ms = 50;
                if (resp_win_ms == 0 && ctx->config.rach_response_window_ms > 0) {
                    resp_win_ms = ctx->config.rach_response_window_ms;
                } else if (resp_win_ms == 0) {
                    resp_win_ms = 200;
                }
                if (resp_win_ms > 5000) resp_win_ms = 5000;

                k_timer_start(&ctx->rach_context.rach_response_window_timer, K_MSEC(resp_win_ms), K_NO_WAIT);

                // uint32_t phy_rx_op_handle = sys_rand32_get();
				uint32_t phy_rx_op_handle;
				sys_rand_get(&phy_rx_op_handle, sizeof(uint32_t));
                uint32_t rx_duration_modem_units = modem_us_to_ticks( (resp_win_ms + 50) * 1000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ );

                int ret = dect_mac_phy_ctrl_start_rx(
                    ctx->role_ctx.pt.target_ft.operating_carrier,
                    rx_duration_modem_units,
                    NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
                    phy_rx_op_handle,
                    ctx->own_short_rd_id,
                    PENDING_OP_PT_WAIT_ASSOC_RESP);
                if (ret != 0) {
                    LOG_ERR("PT_SM: Failed to schedule RX for AssocResp: %d. Resp timer will timeout.", ret);
                } else {
                    LOG_INF("PT_SM: RX scheduled (Hdl %u) for Association Response from FT 0x%04X.",
                            phy_rx_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
                }

            } else if (event->err == NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY) {
                LOG_WRN("PT_SM: RACH TX LBT busy for AssocReq (Hdl %u). Increasing CW and backing off.", event->handle);
                dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);

                if (ctx->role_ctx.pt.current_assoc_retries >= ctx->config.max_assoc_retries) {
                    LOG_ERR("PT_SM_RACH_LBT_BUSY: Max association retries reached for FT 0x%04X after LBT busy. Restarting scan.",
                            ctx->role_ctx.pt.target_ft.short_rd_id);
                    dect_mac_sm_pt_start_operation();
                    return;
                }

                uint8_t ft_cwmin_sig_code = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.cwmin_sig_code;
                uint8_t ft_cwmax_sig_code = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.cwmax_sig_code;

                if (ctx->rach_context.rach_cw_current_idx < ft_cwmin_sig_code) {
                    ctx->rach_context.rach_cw_current_idx = ft_cwmin_sig_code;
                }

                if (ctx->rach_context.rach_cw_current_idx < ft_cwmax_sig_code) {
                    ctx->rach_context.rach_cw_current_idx++;
                }

                uint16_t current_cw_value = 8 * (1U << ctx->rach_context.rach_cw_current_idx);

                if (current_cw_value < ctx->role_ctx.pt.current_ft_rach_params.cw_min_val) {
                    current_cw_value = ctx->role_ctx.pt.current_ft_rach_params.cw_min_val;
                }
                if (current_cw_value > ctx->role_ctx.pt.current_ft_rach_params.cw_max_val &&
                    ctx->role_ctx.pt.current_ft_rach_params.cw_max_val > 0 ) {
                    current_cw_value = ctx->role_ctx.pt.current_ft_rach_params.cw_max_val;
                }

                // uint32_t backoff_slots_to_wait = (current_cw_value > 0) ? (sys_rand32_get() % current_cw_value) : 0;
				uint32_t backoff_slots_to_wait = 0;
				if (current_cw_value > 0) {
					uint32_t random_value;
					sys_rand_get(&random_value, sizeof(random_value));
					backoff_slots_to_wait = random_value % current_cw_value;
				}				
                uint8_t ft_mu_for_rach = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.mu_value_for_ft_beacon;
                if (ft_mu_for_rach > 7) {
                    ft_mu_for_rach = 0;
                }
                uint32_t rach_contention_slot_ticks = get_subslot_duration_ticks_for_mu(ft_mu_for_rach);
                if (rach_contention_slot_ticks == 0) {
                    rach_contention_slot_ticks = NRF_MODEM_DECT_LBT_PERIOD_MIN;
                }

                uint32_t backoff_duration_ticks = backoff_slots_to_wait * rach_contention_slot_ticks;
                uint32_t backoff_ms = (backoff_duration_ticks * 1000U) / NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ;
                if (backoff_ms == 0 && backoff_slots_to_wait > 0) backoff_ms = 1;
                if (backoff_ms == 0 && backoff_slots_to_wait == 0) backoff_ms = 2;

                k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(MAX(10, backoff_ms)), K_NO_WAIT);
            } else {
                pt_rach_response_window_timer_expired_action();
            }
            break;

        case PENDING_OP_PT_WAIT_ASSOC_RESP:
        case PENDING_OP_PT_WAIT_AUTH_CHALLENGE:
        case PENDING_OP_PT_WAIT_AUTH_SUCCESS:
            LOG_DBG("PT SM: RX op for %s completed (Hdl %u, err %d). If no PDC, timer will expire.",
                    dect_pending_op_to_str(completed_op_type), event->handle, event->err);
            break;

        case PENDING_OP_PT_AUTH_MSG_TX:
            if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_INF("PT_SM: Auth message TX successful (Hdl %u). Waiting for response.",
                        event->handle);
                // uint32_t phy_rx_op_handle = sys_rand32_get();
				uint32_t phy_rx_op_handle;
				sys_rand_get(&phy_rx_op_handle, sizeof(uint32_t));				
                uint32_t rx_duration_modem_units = modem_us_to_ticks(
                    (ctx->config.rach_response_window_ms + 50) * 1000,
                    NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

                pending_op_type_t next_op = PENDING_OP_NONE;

                if (ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE) {
                    next_op = PENDING_OP_PT_WAIT_AUTH_CHALLENGE;
                } else if (ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS) {
                    next_op = PENDING_OP_PT_WAIT_AUTH_SUCCESS;
                }

                if (next_op != PENDING_OP_NONE) {
                    dect_mac_phy_ctrl_start_rx(
                        ctx->role_ctx.pt.target_ft.operating_carrier,
                        rx_duration_modem_units, NRF_MODEM_DECT_PHY_RX_MODE_SEMICONTINUOUS,
                        phy_rx_op_handle, ctx->own_short_rd_id, next_op);
                }
            } else {
                LOG_ERR("PT_SM: Auth message TX failed (err %d). Restarting scan.",
                        event->err);
                dect_mac_sm_pt_start_operation();
            }
            break;

        case PENDING_OP_PT_KEEP_ALIVE:
            if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_ERR("PT_SM: Keep Alive TX failed (Hdl %u, err %d, %s).",
                        event->handle, event->err, nrf_modem_dect_phy_err_to_str(event->err));
            } else {
                LOG_DBG("PT_SM: Keep Alive TX successful (Hdl %u).", event->handle);
            }
            break;
        case PENDING_OP_PT_DATA_TX_HARQ0:
        case PENDING_OP_PT_DATA_TX_HARQ_MAX:
            {
                int harq_idx = completed_op_type - PENDING_OP_PT_DATA_TX_HARQ0;
                if (harq_idx >= 0 && harq_idx < MAX_HARQ_PROCESSES) {
                    if (event->err == NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY) {
                        LOG_WRN("PT SM: Data TX HARQ %d LBT busy. Data Path will re-TX.", harq_idx);
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                        LOG_ERR("PT SM: Data TX HARQ %d failed (err %d, %s). Data Path will re-TX/discard.",
                                harq_idx, event->err, nrf_modem_dect_phy_err_to_str(event->err));
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else {
                        LOG_DBG("PT SM: Data TX HARQ %d PHY op complete. Awaiting feedback.", harq_idx);
                    }
                } else {
                     LOG_ERR("PT SM: OP_COMPLETE for invalid PT_DATA_TX_HARQ op type: %d", completed_op_type);
                }
            }
            break;
        default:
             LOG_WRN("PT SM: OP_COMPLETE for unhandled PT op type: %s (Hdl %u), err %d (%s)",
                    dect_pending_op_to_str(completed_op_type), event->handle,
                    event->err, nrf_modem_dect_phy_err_to_str(event->err));
            break;
    }
}

// All other static helper functions (pt_handle_phy_pcc_internal, pt_handle_phy_pdc_internal,
// pt_process_identified_beacon_and_attempt_assoc, pt_send_association_request_action,
// pt_process_association_response_pdu, pt_send_keep_alive_action,
// pt_start_authentication_with_ft_action, pt_authentication_complete_action)
// are now included below with their full implementations as per our latest discussions.

static void pt_handle_phy_pcc_internal(const struct nrf_modem_dect_phy_pcc_event *pcc_event, uint64_t pcc_event_time) {
    dect_mac_context_t* ctx = get_mac_context();

    // Clear previous stored PCC before evaluating the new one
    last_relevant_pcc_for_pt.is_valid = false;

    if (pcc_event->header_status == NRF_MODEM_DECT_PHY_HDR_STATUS_VALID) {
        // Store this valid PCC and its reception time for potential correlation with a subsequent PDC
        memcpy(&last_relevant_pcc_for_pt.pcc_data, pcc_event, sizeof(struct nrf_modem_dect_phy_pcc_event));
        last_relevant_pcc_for_pt.pcc_event_modem_time = pcc_event_time;
        last_relevant_pcc_for_pt.is_valid = true;

        uint16_t pcc_tx_short_id = 0; // Transmitter of this PCC (the FT)
        bool is_type2_pcc = false;

        if (pcc_event->phy_type == 0) { // nRF PHY Type 0 (ETSI PCC Type 1) - Beacon
            pcc_tx_short_id = sys_be16_to_cpu(
                (uint16_t)((pcc_event->hdr.hdr_type_1.transmitter_id_hi << 8) |
                            pcc_event->hdr.hdr_type_1.transmitter_id_lo));

            if (ctx->state == MAC_STATE_PT_SCANNING) {
                LOG_INF("PT_SM_PCC: Beacon PCC (Type1) received from FT ShortID 0x%04X in SCANNING. TID: %u. Waiting for PDC.",
                        pcc_tx_short_id, pcc_event->transaction_id);
                // Further processing happens when the PDC arrives.
                // PT might transition to MAC_STATE_PT_BEACON_PDC_WAIT here if desired,
                // or simply let the SCANNING state's PDC handler do the work.
            } else {
                // Unlikely to get a beacon PCC in other states unless it's a neighbor for mobility
                LOG_DBG("PT_SM_PCC: Beacon PCC (Type1) from FT 0x%04X in state %s. TID %u.",
                        pcc_tx_short_id, dect_mac_state_to_str(ctx->state), pcc_event->transaction_id);
            }

        } else if (pcc_event->phy_type == 1) { // nRF PHY Type 1 (ETSI PCC Type 2) - Unicast/Data
            is_type2_pcc = true;
            pcc_tx_short_id = sys_be16_to_cpu(
                (uint16_t)((pcc_event->hdr.hdr_type_2.transmitter_id_hi << 8) |
                            pcc_event->hdr.hdr_type_2.transmitter_id_lo));
            uint16_t pcc_rx_short_id_on_pt = sys_be16_to_cpu(
                (uint16_t)((pcc_event->hdr.hdr_type_2.receiver_id_hi << 8) |
                            pcc_event->hdr.hdr_type_2.receiver_id_lo));

            if (pcc_rx_short_id_on_pt != ctx->own_short_rd_id) {
                LOG_DBG("PT_SM_PCC: Unicast PCC (Type2) not for this PT (RxID 0x%04X vs Own 0x%04X). TID: %u. Ignoring.",
                        pcc_rx_short_id_on_pt, ctx->own_short_rd_id, pcc_event->transaction_id);
                last_relevant_pcc_for_pt.is_valid = false; // Don't process its PDC
                return;
            }

            if (ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP) {
                if (pcc_tx_short_id == ctx->role_ctx.pt.target_ft.short_rd_id) {
                    LOG_INF("PT_SM_PCC: PCC (Type2) from target FT 0x%04X while WAITING_ASSOC_RESP. TID %u. Waiting for PDC.",
                            pcc_tx_short_id, pcc_event->transaction_id);
                    // Association Response is typically unsecure initially, or first secure packet.
                    // HARQ feedback for PT's AssocReq itself is not standard via this PCC's feedback field.
                    // If FT *did* include feedback for the AssocReq (unlikely), it would be processed here.
                    // For now, assume no HARQ feedback processing specific to the AssocReq itself.
                } else {
                    LOG_WRN("PT_SM_PCC: PCC (Type2) from unexpected FT 0x%04X while WAITING_ASSOC_RESP. Ignoring. TID %u",
                            pcc_tx_short_id, pcc_event->transaction_id);
                    last_relevant_pcc_for_pt.is_valid = false;
                }
            } else if (ctx->state == MAC_STATE_ASSOCIATED) {
                if (pcc_tx_short_id == ctx->role_ctx.pt.associated_ft.short_rd_id) {
                    LOG_DBG("PT_SM_PCC: PCC (Type2) from associated FT 0x%04X. TID %u. Processing feedback.",
                            pcc_tx_short_id, pcc_event->transaction_id);
                    // Process HARQ feedback sent by the FT for PT's previous transmissions
                    dect_mac_data_path_process_harq_feedback(&pcc_event->hdr.hdr_type_2.feedback, pcc_tx_short_id);
                } else {
                    LOG_WRN("PT_SM_PCC: PCC (Type2) from unexpected FT 0x%04X while ASSOCIATED. Ignoring. TID %u",
                            pcc_tx_short_id, pcc_event->transaction_id);
                    last_relevant_pcc_for_pt.is_valid = false;
                }
            } else {
                 LOG_DBG("PT_SM_PCC: PCC (Type2) from FT 0x%04X in unexpected state %s. TID %u",
                        pcc_tx_short_id, dect_mac_state_to_str(ctx->state), pcc_event->transaction_id);
                 // Store it anyway, PDC handler will re-check state.
            }
        } else {
            LOG_ERR("PT_SM_PCC: Unknown nRF PHY header type in PCC: %d. TID %u", pcc_event->phy_type, pcc_event->transaction_id);
            last_relevant_pcc_for_pt.is_valid = false;
        }
    } else { // PCC HeaderStatus not VALID
        last_relevant_pcc_for_pt.is_valid = false;
        LOG_WRN("PT_SM_PCC: Received PCC with invalid status %d. TID %u. Op Hdl %u.",
                pcc_event->header_status, pcc_event->transaction_id, pcc_event->handle);
        // If PT was waiting for a specific response (e.g. AssocResp on pending_op_handle) and PCC is invalid,
        // the response timer (e.g. rach_response_window_timer) should eventually handle the timeout.
        // No PDC will follow this invalid PCC.
    }
}



static void pt_handle_phy_pdc_internal(const struct nrf_modem_dect_phy_pdc_event *pdc_event,
				       const struct nrf_modem_dect_phy_pcc_event *assoc_pcc_event,
				       uint64_t pcc_reception_modem_time)
{
	dect_mac_context_t *ctx = get_mac_context();
	uint8_t mac_pdc_payload_copy[CONFIG_DECT_MAC_PDU_MAX_SIZE];
	uint16_t pdc_payload_len = pdc_event->len;

	if (pdc_payload_len > sizeof(mac_pdc_payload_copy)) {
		LOG_ERR("PT_SM_PDC: PDC payload from PHY (%u bytes) too large for copy buffer (%zu). Discarding.",
			pdc_payload_len, sizeof(mac_pdc_payload_copy));
		return;
	}
	memcpy(mac_pdc_payload_copy, pdc_event->data, pdc_payload_len);

	dect_mac_header_type_octet_t mac_hdr_type_octet;
	memcpy(&mac_hdr_type_octet, &mac_pdc_payload_copy[0], sizeof(dect_mac_header_type_octet_t));

	uint8_t *common_hdr_start_in_payload = mac_pdc_payload_copy + sizeof(dect_mac_header_type_octet_t);
	size_t common_hdr_actual_len = 0;
	uint8_t *sdu_area_after_common_hdr = NULL;
	size_t sdu_area_plus_mic_len_in_payload = 0;

	uint16_t ft_sender_short_id_from_pcc = 0;
	if (assoc_pcc_event->phy_type == 0) { /* Beacon */
		ft_sender_short_id_from_pcc = sys_be16_to_cpu(
			(uint16_t)((assoc_pcc_event->hdr.hdr_type_1.transmitter_id_hi << 8) |
				   assoc_pcc_event->hdr.hdr_type_1.transmitter_id_lo));
	} else if (assoc_pcc_event->phy_type == 1) { /* Unicast/Data */
		ft_sender_short_id_from_pcc = sys_be16_to_cpu(
			(uint16_t)((assoc_pcc_event->hdr.hdr_type_2.transmitter_id_hi << 8) |
				   assoc_pcc_event->hdr.hdr_type_2.transmitter_id_lo));
	}

	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
		common_hdr_actual_len = sizeof(dect_mac_beacon_header_t);
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		common_hdr_actual_len = sizeof(dect_mac_unicast_header_t);
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU) {
		common_hdr_actual_len = sizeof(dect_mac_data_pdu_header_t);
	} else {
		return;
	}

	if (pdc_payload_len < (sizeof(dect_mac_header_type_octet_t) + common_hdr_actual_len)) {
		LOG_ERR("PT_SM_PDC: PDU too short for its Common Hdr. Len %u, HdrLen %zu",
			pdc_payload_len, common_hdr_actual_len);
		return;
	}
	sdu_area_after_common_hdr = common_hdr_start_in_payload + common_hdr_actual_len;
	sdu_area_plus_mic_len_in_payload = pdc_payload_len - sizeof(dect_mac_header_type_octet_t) - common_hdr_actual_len;

	/* --- Beacon Processing Logic --- */
	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_BEACON) {
		const dect_mac_beacon_header_t *bch = (const dect_mac_beacon_header_t *)common_hdr_start_in_payload;
		uint32_t ft_long_id = sys_be32_to_cpu(bch->transmitter_long_rd_id_be);
		dect_mac_cluster_beacon_ie_fields_t cb_fields_parsed;
		dect_mac_rach_info_ie_fields_t rach_fields_parsed;
		dect_mac_rd_capability_ie_t ft_caps_parsed;
		dect_mac_resource_alloc_ie_fields_t res_alloc_fields;
		bool cb_found = false, rach_found = false, ft_cap_found = false, res_alloc_found = false;

		const uint8_t *sdu_area_ptr = sdu_area_after_common_hdr;
		size_t sdu_area_len = sdu_area_plus_mic_len_in_payload;

		while (sdu_area_len > 0) {
			uint8_t ie_type;
			uint16_t ie_len;
			const uint8_t *ie_payload;
			
            // The parse_mac_mux_header function now correctly handles all length calculations.
			int mux_hdr_len = parse_mac_mux_header(sdu_area_ptr, sdu_area_len, &ie_type, &ie_len, &ie_payload);
			
            if (mux_hdr_len <= 0) { break; }

			// if (ie_len == 0 && ((sdu_area_ptr[0] >> 6) & 0x03) == 0b00) {
			// 	ie_len = get_fixed_ie_payload_len(ie_type);
			// }

			if (sdu_area_len < (size_t)mux_hdr_len + ie_len) break;

			if (ie_type == IE_TYPE_RD_CAPABILITY) {
				if (parse_rd_capability_ie_payload(ie_payload, ie_len, &ft_caps_parsed) == 0) {
					ft_cap_found = true;
				}
			} else if (ie_type == IE_TYPE_CLUSTER_BEACON) {
				if (parse_cluster_beacon_ie_payload(ie_payload, ie_len, &cb_fields_parsed) == 0) cb_found = true;
			} else if (ie_type == IE_TYPE_RACH_INFO) {
				uint8_t mu_code = ft_cap_found ? ft_caps_parsed.phy_variants[0].mu_value : 0;
				if (parse_rach_info_ie_payload(ie_payload, ie_len, mu_code, &rach_fields_parsed) == 0) rach_found = true;
			} else if (ie_type == IE_TYPE_RES_ALLOC) {
				uint8_t mu_code = ft_cap_found ? ft_caps_parsed.phy_variants[0].mu_value : 0;
				if (parse_resource_alloc_ie_payload(ie_payload, ie_len, mu_code, &res_alloc_fields) == 0) {
					if (res_alloc_fields.repeat_val == RES_ALLOC_REPEAT_FRAMES_GROUP || res_alloc_fields.repeat_val == RES_ALLOC_REPEAT_SUBSLOTS_GROUP) {
						res_alloc_found = true;
					}
				}
			} else if (ie_type == IE_TYPE_BROADCAST_IND) {
				if (ctx->state == MAC_STATE_PT_PAGING && ie_len >= 2) {
					uint16_t paged_short_id = sys_be16_to_cpu(*((const uint16_t*)ie_payload));
					if (paged_short_id == ctx->own_short_rd_id) {
						pt_process_page_indication();
					}
				}
			}
			else if (ie_type == IE_TYPE_GROUP_ASSIGNMENT) {
				if (ctx->state == MAC_STATE_ASSOCIATED) {
					pt_process_group_assignment_ie(ie_payload, ie_len);
				}
			}

			size_t consumed = mux_hdr_len + ie_len;
			if (consumed == 0) {
				LOG_WRN("PT_PDC_BEACON: Consumed 0 bytes, breaking loop.");
				break;
			}
			if (sdu_area_len >= consumed) {
				sdu_area_len -= consumed;
				sdu_area_ptr += consumed;
			} else {
				sdu_area_len = 0;
			}
		}

        uint16_t beacon_rx_carrier = ctx->current_rx_op_carrier;

		if (res_alloc_found) {
			dect_mac_schedule_t *group_sched = &ctx->role_ctx.pt.group_schedule;
			group_sched->is_active = false;
			group_sched->alloc_type = res_alloc_fields.alloc_type_val;
			group_sched->ul_start_subslot = res_alloc_fields.start_subslot_val_res1;
			group_sched->ul_duration_subslots = res_alloc_fields.length_val_res1 + 1;
			group_sched->ul_length_is_slots = res_alloc_fields.length_type_is_slots_res1;
			group_sched->repeat_type = res_alloc_fields.repeat_val;
			group_sched->repetition_value = res_alloc_fields.repetition_value;
			group_sched->validity_value = res_alloc_fields.validity_value;
			// group_sched->channel = res_alloc_fields.channel_present ? res_alloc_fields.channel_val : assoc_pcc_event->pcc_params_from_modem.carrier;
			group_sched->channel = res_alloc_fields.channel_present ? res_alloc_fields.channel_val : beacon_rx_carrier;			
		}

		if (cb_found && rach_found) {
			if (ctx->state == MAC_STATE_PT_SCANNING || ctx->state == MAC_STATE_PT_BEACON_PDC_WAIT) {
				pt_process_identified_beacon_and_attempt_assoc(ctx, &cb_fields_parsed, &rach_fields_parsed, ft_long_id, ft_sender_short_id_from_pcc, assoc_pcc_event->rssi_2, beacon_rx_carrier, pcc_reception_modem_time);
			} else if (ctx->state == MAC_STATE_ASSOCIATED) {
				pt_evaluate_mobility_candidate(ctx, &cb_fields_parsed, ft_long_id, ft_sender_short_id_from_pcc, assoc_pcc_event->rssi_2, beacon_rx_carrier);
			}
		}

		return;
	}

	/* --- Unicast PDU Processing Logic --- */
	dect_mac_peer_info_t *active_ft_peer_ctx = NULL;
	if ((ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP || ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE || ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS) && ctx->role_ctx.pt.target_ft.is_valid && ft_sender_short_id_from_pcc == ctx->role_ctx.pt.target_ft.short_rd_id) {
		active_ft_peer_ctx = &ctx->role_ctx.pt.target_ft;
	} else if (ctx->state >= MAC_STATE_ASSOCIATED && ctx->role_ctx.pt.associated_ft.is_valid && ft_sender_short_id_from_pcc == ctx->role_ctx.pt.associated_ft.short_rd_id) {
		active_ft_peer_ctx = &ctx->role_ctx.pt.associated_ft;
	} else {
		LOG_WRN("PT_SM_PDC: Unicast from FT 0x%04X in unexpected state %s or from unexpected FT. Discarding.", ft_sender_short_id_from_pcc, dect_mac_state_to_str(ctx->state));
		return;
	}

	bool pdc_process_ok_for_feedback = true;
	uint8_t *sdu_area_for_data_path = sdu_area_after_common_hdr;
	size_t sdu_area_len_for_data_path = sdu_area_plus_mic_len_in_payload;

	/* Security processing block would go here, updating pdc_process_ok_for_feedback */

	/* HARQ Feedback Generation for Downlink Data */
	if (active_ft_peer_ctx && ctx->state == MAC_STATE_ASSOCIATED) {
		uint8_t harq_proc_in_ft_tx = assoc_pcc_event->hdr.hdr_type_2.df_harq_process_num;

		if (active_ft_peer_ctx->num_pending_feedback_items < 2) {
			int fb_idx = active_ft_peer_ctx->num_pending_feedback_items++;

			active_ft_peer_ctx->pending_feedback_to_send[fb_idx].valid = true;
			active_ft_peer_ctx->pending_feedback_to_send[fb_idx].is_ack =
				pdc_process_ok_for_feedback;
			active_ft_peer_ctx->pending_feedback_to_send[fb_idx]
				.harq_process_num_for_peer = harq_proc_in_ft_tx;

			LOG_DBG("PT_SM_HARQ_RX: Stored %s for FT's HARQ_Proc %u.",
				pdc_process_ok_for_feedback ? "ACK" : "NACK",
				harq_proc_in_ft_tx);
		} else {
			LOG_WRN("PT_SM_PDC: Feedback buffer full for FT 0x%04X",
				ft_sender_short_id_from_pcc);
		}
	}

	if (!pdc_process_ok_for_feedback) {
		return;
	}

	uint32_t sender_long_id_final = 0;

	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		const dect_mac_unicast_header_t *uch =
			(const dect_mac_unicast_header_t *)common_hdr_start_in_payload;
		sender_long_id_final = sys_be32_to_cpu(uch->transmitter_long_rd_id_be);

		uint8_t ie_type;
		uint16_t ie_len;
		const uint8_t *ie_payload;
		int mux_hdr_len = parse_mac_mux_header(sdu_area_for_data_path,
						       sdu_area_len_for_data_path, &ie_type,
						       &ie_len, &ie_payload);

		if (mux_hdr_len > 0) {
			switch (ie_type) {
			case IE_TYPE_ASSOC_RESP:
				if (ctx->state == MAC_STATE_PT_WAIT_ASSOC_RESP) {
					pt_process_association_response_pdu(sdu_area_for_data_path, sdu_area_len_for_data_path, sender_long_id_final, pcc_reception_modem_time);
				}
				break;
			case IE_TYPE_ASSOC_RELEASE:
				if (ctx->state >= MAC_STATE_ASSOCIATED) {
					dect_mac_assoc_release_ie_t release_fields;
					if (parse_assoc_release_ie_payload(ie_payload, ie_len, &release_fields) == 0) {
						LOG_WRN("PT SM: Received Association Release from FT (cause: %u). Disconnecting.", release_fields.cause);
						dect_mac_sm_pt_start_operation(); /* Restart full scan */
					}
				}
				break;
			case IE_TYPE_AUTH_CHALLENGE:
				if (ctx->state == MAC_STATE_PT_WAIT_AUTH_CHALLENGE) {
					if (ie_len == sizeof(dect_mac_auth_challenge_ie_t)) {
						const dect_mac_auth_challenge_ie_t *chal = (const dect_mac_auth_challenge_ie_t *)ie_payload;
						ctx->role_ctx.pt.target_ft.ft_nonce = sys_be32_to_cpu(chal->ft_nonce_be);
						pt_send_auth_response_action();
					}
				}
				break;
			case IE_TYPE_AUTH_SUCCESS:
				if (ctx->state == MAC_STATE_PT_WAIT_AUTH_SUCCESS) {
					if (ie_len == sizeof(dect_mac_auth_success_ie_t)) {
						const dect_mac_auth_success_ie_t *succ = (const dect_mac_auth_success_ie_t *)ie_payload;
						pt_process_auth_success(ctx, succ->ft_mac);
					}
				}
				break;
			default:
				if (active_ft_peer_ctx && active_ft_peer_ctx->is_valid) {
					dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path,
									 sdu_area_len_for_data_path,
									 sender_long_id_final);
				} else {
					LOG_WRN("PT_SM_PDC: Unicast from unknown FT 0x%04X or unexpected state. Discarding.",
						ft_sender_short_id_from_pcc);
				}
				break;
			}
		}
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU &&
		   active_ft_peer_ctx && active_ft_peer_ctx->is_valid) {
		sender_long_id_final = active_ft_peer_ctx->long_rd_id;
		dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path,
						 sdu_area_len_for_data_path, sender_long_id_final);
	} else {
		LOG_WRN("PT_SM_PDC: Received PDC with unhandled/unexpected MAC Common Header Type %u.",
			mac_hdr_type_octet.mac_header_type);
	}
}

static void pt_initiate_authentication_handshake(
	dect_mac_context_t *ctx, const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
	const dect_mac_rach_info_ie_fields_t *rach_fields, uint32_t ft_long_id,
	uint16_t ft_short_id, int16_t rssi_q7_1, uint16_t beacon_rx_carrier,
	uint64_t beacon_pcc_rx_time)
{
	/* Store target FT info and RACH params, similar to legacy assoc */
	pt_process_identified_beacon_and_attempt_assoc(ctx, cb_fields, rach_fields, ft_long_id,
						       ft_short_id, rssi_q7_1, beacon_rx_carrier,
						       beacon_pcc_rx_time);

	/* Generate and store PT nonce */
	// ctx->role_ctx.pt.target_ft.pt_nonce = sys_rand32_get();
	uint32_t random_nonce;
	sys_rand_get(&random_nonce, sizeof(random_nonce));
	ctx->role_ctx.pt.target_ft.pt_nonce = random_nonce;	

	/* Cancel scan and send Auth Initiate */
	if (ctx->pending_op_type == PENDING_OP_PT_SCAN && ctx->pending_op_handle != 0) {
		dect_mac_phy_ctrl_cancel_op(ctx->pending_op_handle);
		/* The actual send will happen in the OP_COMPLETE handler for the cancelled scan */
	} else {
		pt_send_auth_initiate_action();
	}
}


static void pt_process_identified_beacon_and_attempt_assoc(dect_mac_context_t *ctx,
                                                           const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
                                                           const dect_mac_rach_info_ie_fields_t *rach_fields,
                                                           uint32_t ft_long_id,
                                                           uint16_t ft_short_id,
                                                           int16_t rssi_q7_1,
                                                           uint16_t beacon_rx_carrier,
                                                           uint64_t beacon_pcc_rx_time)
{
    if (!ctx || !cb_fields || !rach_fields) {
        LOG_ERR("PT_BEACON_PROC: NULL arguments.");
        return;
    }

    bool select_this_ft = false;
    uint8_t new_ft_cost = 255;

    /* Find the Route Info IE to get the cost */
    /* This logic is now inside pt_handle_phy_pdc_internal, which needs to be updated */
    /* For now, assume route info is passed in or looked up differently */
    /* Simplified logic: assume mesh is not primary factor for now */
    if (!ctx->role_ctx.pt.target_ft.is_valid) {
        select_this_ft = true;
        LOG_INF("PT_BEACON_PROC: No current target. Selecting FT 0x%04X (RSSI:%.1f).", ft_short_id, (float)rssi_q7_1 / 2.0f);
    } else if (rssi_q7_1 > (ctx->role_ctx.pt.target_ft.rssi_2 + (RSSI_HYSTERESIS_DB * 2))) {
        select_this_ft = true;
        LOG_INF("PT_BEACON_PROC: New FT 0x%04X has better RSSI (%.1f > %.1f). Switching.",
                ft_short_id, (float)rssi_q7_1 / 2.0f, (float)ctx->role_ctx.pt.target_ft.rssi_2 / 2.0f);
    }


    if (select_this_ft) {
        LOG_INF("PT_BEACON_PROC: Selected FT LongID:0x%08X, ShortID:0x%04X, BeaconRxCarrier:%u, RSSI2:%.1f dBm",
                ft_long_id, ft_short_id, beacon_rx_carrier, (float)rssi_q7_1 / 2.0f);

        dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);
        
        ctx->role_ctx.pt.target_ft.is_valid = true;
        ctx->role_ctx.pt.target_ft.is_fully_identified = (ft_long_id != 0);
        ctx->role_ctx.pt.target_ft.long_rd_id = ft_long_id;
        ctx->role_ctx.pt.target_ft.short_rd_id = ft_short_id;
        ctx->role_ctx.pt.target_ft.rssi_2 = rssi_q7_1;

        if (cb_fields->next_channel_present && cb_fields->next_cluster_channel_val != 0 &&
            cb_fields->next_cluster_channel_val != 0xFFFF) {
            ctx->role_ctx.pt.target_ft.operating_carrier = cb_fields->next_cluster_channel_val;
        } else {
            ctx->role_ctx.pt.target_ft.operating_carrier = beacon_rx_carrier;
        }
        LOG_INF("PT_BEACON_PROC: Target FT operating_carrier set to: %u", ctx->role_ctx.pt.target_ft.operating_carrier);

        uint32_t frame_duration_ticks = (uint32_t)FRAME_DURATION_MS_NOMINAL *
                                        (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);

        uint64_t new_sfn0_estimate = beacon_pcc_rx_time -
                                     ((uint64_t)cb_fields->sfn * frame_duration_ticks);

        if (ctx->ft_sfn_zero_modem_time_anchor == 0 || select_this_ft) {
            ctx->ft_sfn_zero_modem_time_anchor = new_sfn0_estimate;
        } else {
            ctx->ft_sfn_zero_modem_time_anchor = (ctx->ft_sfn_zero_modem_time_anchor + new_sfn0_estimate) / 2;
        }
        ctx->current_sfn_at_anchor_update = cb_fields->sfn;
        LOG_DBG("PT_BEACON_PROC: FT SFN0 Anchor: %llu (Beacon SFN %u @ %llu)",
                ctx->ft_sfn_zero_modem_time_anchor, cb_fields->sfn, beacon_pcc_rx_time);

        memcpy(&ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields,
               rach_fields, sizeof(dect_mac_rach_info_ie_fields_t));

        if (rach_fields->channel_field_present &&
            rach_fields->channel_abs_freq_num != 0 &&
            rach_fields->channel_abs_freq_num != 0xFFFF) {
            ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel = rach_fields->channel_abs_freq_num;
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel = ctx->role_ctx.pt.target_ft.operating_carrier;
        }

        if (rach_fields->cwmin_sig_code <= 7) {
             ctx->role_ctx.pt.current_ft_rach_params.cw_min_val = 8 * (1U << rach_fields->cwmin_sig_code);
        } else {
             ctx->role_ctx.pt.current_ft_rach_params.cw_min_val = 8 * (1U << ctx->config.rach_cw_min_idx);
        }
        if (rach_fields->cwmax_sig_code <= 7) {
            ctx->role_ctx.pt.current_ft_rach_params.cw_max_val = 8 * (1U << rach_fields->cwmax_sig_code);
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.cw_max_val = 8 * (1U << ctx->config.rach_cw_max_idx);
        }

        uint32_t resp_win_subslots_actual = rach_fields->response_window_subslots_val_minus_1 + 1;
        uint8_t ft_mu_code_from_beacon = rach_fields->mu_value_for_ft_beacon;
        if (ft_mu_code_from_beacon > 7) {
            ft_mu_code_from_beacon = 0;
        }
        uint32_t ft_subslot_duration_ticks = get_subslot_duration_ticks_for_mu(ft_mu_code_from_beacon);

        if (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ > 0 && ft_subslot_duration_ticks > 0) {
            ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us =
                (resp_win_subslots_actual * ft_subslot_duration_ticks * 1000U) / NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ;
        } else {
            ctx->role_ctx.pt.current_ft_rach_params.response_window_duration_us = ctx->config.rach_response_window_ms * 1000;
        }

        ctx->role_ctx.pt.current_assoc_retries = 0;
        ctx->rach_context.rach_cw_current_idx = ctx->config.rach_cw_min_idx;

        if (ctx->pending_op_type == PENDING_OP_PT_SCAN && ctx->pending_op_handle != 0) {
            LOG_INF("PT_BEACON_PROC: Cancelling ongoing scan (handle %u) to associate with FT 0x%04X.",
                    ctx->pending_op_handle, ft_short_id);
            dect_mac_phy_ctrl_cancel_op(ctx->pending_op_handle);
        } else {
            LOG_INF("PT_BEACON_PROC: No active scan to cancel. Directly attempting association with FT 0x%04X.", ft_short_id);
            pt_send_association_request_action();
        }
    } else {
        LOG_DBG("PT_BEACON_PROC: Beacon from FT 0x%04X (L:0x%08X, RSSI:%.1f) not better than current target 0x%04X (RSSI:%.1f). Continuing scan.",
                ft_short_id, ft_long_id, (float)rssi_q7_1 / 2.0f,
                ctx->role_ctx.pt.target_ft.short_rd_id, (float)ctx->role_ctx.pt.target_ft.rssi_2 / 2.0f);
    }
}

static void pt_send_auth_initiate_action(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (!ctx->role_ctx.pt.target_ft.is_valid) {
		LOG_ERR("PT_AUTH_INIT: No valid target FT. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_change_state(MAC_STATE_PT_WAIT_AUTH_CHALLENGE);

	uint8_t sdu_area_buf[32];
	int sdu_area_len = build_auth_initiate_ie_muxed(
		sdu_area_buf, sizeof(sdu_area_buf), ctx->role_ctx.pt.target_ft.pt_nonce);

	if (sdu_area_len < 0) {
		LOG_ERR("PT_AUTH_INIT: Failed to build Auth Initiate IE: %d", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_header_type_octet_t hdr_type = {.version = 0,
						 .mac_security = MAC_SECURITY_NONE,
						 .mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST};
	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(ctx->role_ctx.pt.target_ft.long_rd_id);

	mac_sdu_t *pdu_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("FT_AUTH_CHAL: Failed to alloc PDU buffer.");
		return;
	}

	uint16_t pdu_len;
	int ret;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						   &hdr_type, &common_hdr, sizeof(common_hdr),
						   sdu_area_buf, (size_t)sdu_area_len, &pdu_len);

	if (ret != 0) {
		dect_mac_api_buffer_free(pdu_sdu);
		dect_mac_sm_pt_start_operation();
		return;
	}

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, pdu_sdu->data, pdu_len,
		ctx->role_ctx.pt.target_ft.short_rd_id, false, phy_op_handle,
		PENDING_OP_PT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);

	dect_mac_api_buffer_free(pdu_sdu);

	if (ret != 0) {
		LOG_ERR("PT_AUTH_INIT: Failed to schedule Auth Initiate TX: %d. Entering backoff.", ret);
		dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);
		// uint32_t backoff_ms = 20 + (sys_rand32_get() % 50);
		uint32_t random_value;
		sys_rand_get(&random_value, sizeof(random_value));
		uint32_t backoff_ms = 20 + (random_value % 50);  // Range: 20-69ms		
		k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(backoff_ms), K_NO_WAIT);
	} else {
		LOG_INF("PT_AUTH_INIT: Auth Initiate TX scheduled (Hdl %u) to FT 0x%04X.",
			phy_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
	}
}


// Brief Overview: Enhances pt_send_association_request_action to fully populate
// the dect_mac_assoc_req_ie_t structure with all relevant ETSI fields,
// including HARQ parameters, and placeholder logic for requested Flow IDs and FT Mode parameters.
// Also populates the PT's RD Capability IE more completely.
static void pt_send_association_request_action(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (!ctx->role_ctx.pt.target_ft.is_valid || !ctx->role_ctx.pt.target_ft.is_fully_identified) {
		LOG_ERR("PT_SM_ASSOC_REQ: No valid or not fully identified target FT. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}
	if (ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0 ||
	    ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel == 0xFFFF) {
		LOG_ERR("PT_SM_ASSOC_REQ: Target FT RACH operating channel invalid. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}
	uint8_t ft_max_rach_len_actual_units = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.max_rach_pdu_len_units + 1;
	if (ft_max_rach_len_actual_units == 0 || ft_max_rach_len_actual_units > 128) { /* Max N-1 is 127 for 7 bits */
		LOG_ERR("PT_SM_ASSOC_REQ: Target FT RACH max PDU length invalid (%u units). Cannot send. Restarting scan.", ft_max_rach_len_actual_units);
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_change_state(MAC_STATE_PT_ASSOCIATING);

	LOG_INF("PT_SM_ASSOC_REQ: Attempting Association Request to FT 0x%04X on RACH carrier %u (Attempt %u).",
		ctx->role_ctx.pt.target_ft.short_rd_id,
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel,
		ctx->role_ctx.pt.current_assoc_retries + 1);

	uint8_t sdu_area_buf[128];
	dect_mac_assoc_req_ie_t assoc_req_fields;
	memset(&assoc_req_fields, 0, sizeof(assoc_req_fields));
	dect_mac_rd_capability_ie_t rd_cap_fields; /* PT's own capabilities */
	memset(&rd_cap_fields, 0, sizeof(rd_cap_fields));

	/* --- Populate Association Request IE Fields (dect_mac_assoc_req_ie_t) --- */
	assoc_req_fields.setup_cause_val = ASSOC_CAUSE_INITIAL_ASSOCIATION;
	assoc_req_fields.power_const_active = false;
	assoc_req_fields.ft_mode_capable = IS_ENABLED(CONFIG_DECT_MAC_PT_CAN_BE_FT);

	assoc_req_fields.number_of_flows_val = 0;

	assoc_req_fields.harq_params_present = true;
	assoc_req_fields.harq_processes_tx_val = CONFIG_DECT_MAC_PT_HARQ_TX_PROC_CODE & 0x07;
	assoc_req_fields.max_harq_re_tx_delay_code = CONFIG_DECT_MAC_PT_HARQ_RETX_DELAY_PT_CODE & 0x1F;
	assoc_req_fields.harq_processes_rx_val = CONFIG_DECT_MAC_PT_HARQ_RX_PROC_CODE & 0x07;
	assoc_req_fields.max_harq_re_rx_delay_code = CONFIG_DECT_MAC_PT_HARQ_RERX_DELAY_PT_CODE & 0x1F;

	// Use preprocessor directives to conditionally compile the code
	// that depends on the conditional Kconfig options.
#if IS_ENABLED(CONFIG_DECT_MAC_PT_CAN_BE_FT)
	assoc_req_fields.ft_mode_capable = true;
		assoc_req_fields.ft_beacon_periods_octet_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_SIGNAL_PERIODS);
		if (assoc_req_fields.ft_beacon_periods_octet_present) {
			assoc_req_fields.ft_network_beacon_period_code = CONFIG_DECT_MAC_PT_FT_MODE_NET_BEACON_PERIOD_CODE & 0x0F;
			assoc_req_fields.ft_cluster_beacon_period_code = CONFIG_DECT_MAC_PT_FT_MODE_CLUS_BEACON_PERIOD_CODE & 0x0F;
		}

		assoc_req_fields.ft_next_channel_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_NEXT_CHAN_PRESENT);
		assoc_req_fields.ft_time_to_next_present = IS_ENABLED(CONFIG_DECT_MAC_PT_FT_MODE_TIME_TO_NEXT_PRESENT);
		assoc_req_fields.ft_current_channel_present = false;

		if (assoc_req_fields.ft_next_channel_present || assoc_req_fields.ft_time_to_next_present || assoc_req_fields.ft_current_channel_present) {
			assoc_req_fields.ft_param_flags_octet_present = true;
		} else {
			assoc_req_fields.ft_param_flags_octet_present = false;
		}

		if (assoc_req_fields.ft_next_channel_present) {
			assoc_req_fields.ft_next_cluster_channel_val = CONFIG_DECT_MAC_PT_FT_MODE_NEXT_CLUSTER_CHANNEL_VAL & 0x1FFF;
		}
		if (assoc_req_fields.ft_time_to_next_present) {
			assoc_req_fields.ft_time_to_next_us_val = CONFIG_DECT_MAC_PT_FT_MODE_TIME_TO_NEXT_US_VAL;
		}
#else
		// This is the "PT-only" case.
		assoc_req_fields.ft_mode_capable = false;
		assoc_req_fields.ft_beacon_periods_octet_present = false;
		assoc_req_fields.ft_param_flags_octet_present = false;
#endif

	/* --- Populate PT's RD Capability IE Fields from common Kconfig values --- */
	rd_cap_fields.release_version = CONFIG_DECT_MAC_CAP_RELEASE_VERSION;
	rd_cap_fields.num_phy_capabilities = 1; /* We are sending one explicit set */

	rd_cap_fields.supports_group_assignment = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_GROUP_ASSIGNMENT);
	rd_cap_fields.supports_paging = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_PAGING);
	rd_cap_fields.operating_modes_code = assoc_req_fields.ft_mode_capable
						 ? DECT_MAC_OP_MODE_BOTH
						 : DECT_MAC_OP_MODE_PT_ONLY;
	rd_cap_fields.supports_mesh = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_MESH);
	rd_cap_fields.supports_sched_data = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_SCHED_DATA);
	rd_cap_fields.mac_security_modes_code = IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)
						    ? DECT_MAC_SECURITY_SUPPORT_MODE1
						    : DECT_MAC_SECURITY_SUPPORT_NONE;

	dect_mac_phy_capability_set_t *pt_phy_set0 = &rd_cap_fields.phy_variants[0];
	pt_phy_set0->dlc_service_type_support_code = CONFIG_DECT_MAC_CAP_DLC_SERVICE_SUPPORT_CODE;
	pt_phy_set0->rx_for_tx_diversity_code = CONFIG_DECT_MAC_CAP_RX_TX_DIVERSITY_CODE;
	pt_phy_set0->mu_value = ctx->own_phy_params.mu;
	pt_phy_set0->beta_value = ctx->own_phy_params.beta;
	pt_phy_set0->max_nss_for_rx_code = CONFIG_DECT_MAC_CAP_MAX_NSS_RX_CODE;
	pt_phy_set0->max_mcs_code = CONFIG_DECT_MAC_CAP_MAX_MCS_CODE;
	pt_phy_set0->harq_soft_buffer_size_code = CONFIG_DECT_MAC_CAP_HARQ_BUFFER_CODE;
	pt_phy_set0->num_harq_processes_code = CONFIG_DECT_MAC_CAP_NUM_HARQ_PROC_CODE;
	pt_phy_set0->harq_feedback_delay_code = CONFIG_DECT_MAC_CAP_HARQ_FEEDBACK_DELAY_CODE;
	pt_phy_set0->supports_dect_delay = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_DECT_DELAY);
	pt_phy_set0->supports_half_duplex = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_HALF_DUPLEX);

	/* --- Build SDU Area and MAC PDU --- */
	int sdu_area_len = build_assoc_req_ies_area(sdu_area_buf, sizeof(sdu_area_buf),
                                               &assoc_req_fields, &rd_cap_fields);
	if (sdu_area_len < 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to build Association Request SDU area: %d. Restarting scan.", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_header_type_octet_t hdr_type_octet;
	hdr_type_octet.version = 0;
	hdr_type_octet.mac_security = MAC_SECURITY_NONE;
	hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST;

	dect_mac_unicast_header_t common_hdr;
	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1 /*reset*/);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(ctx->role_ctx.pt.target_ft.long_rd_id);

	uint8_t *full_mac_pdu_for_phy_slab = NULL;
    int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab, K_NO_WAIT);

    if(ret != 0 || full_mac_pdu_for_phy_slab == NULL) {
        dect_mac_enter_error_state("Failed to alloc PDU for AssocReq");
        return;
    }
	uint8_t * const full_mac_pdu_for_phy = full_mac_pdu_for_phy_slab;

	uint16_t pdu_len;
	ret = dect_mac_phy_ctrl_assemble_final_pdu(full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE,
                                         &hdr_type_octet, &common_hdr, sizeof(common_hdr),
                                         sdu_area_buf, (size_t)sdu_area_len,
                                         &pdu_len);
	if (ret != 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to assemble Association Request PDU: %d. Restarting scan.", ret);
		k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t pcc_pkt_len_f, pcc_mcs_f_ignored, pcc_pkt_len_type_f;
	uint8_t rach_tx_mcs = 0;
	uint8_t ft_mu_for_rach_timing = ctx->role_ctx.pt.current_ft_rach_params.advertised_beacon_ie_fields.mu_value_for_ft_beacon;
	if (ft_mu_for_rach_timing > 7) ft_mu_for_rach_timing = 0;
	uint8_t ft_beta_for_rach_timing = 0;

	dect_mac_phy_ctrl_calculate_pcc_params(pdu_len - sizeof(dect_mac_header_type_octet_t),
                                           ft_mu_for_rach_timing,
                                           ft_beta_for_rach_timing,
                                           &pcc_pkt_len_f, &rach_tx_mcs, &pcc_pkt_len_type_f);
	uint32_t assoc_req_tx_duration_actual_units = pcc_pkt_len_f + 1;
	if (pcc_pkt_len_type_f == 1) {
		assoc_req_tx_duration_actual_units *= get_subslots_per_etsi_slot_for_mu(ft_mu_for_rach_timing);
	}

	if (assoc_req_tx_duration_actual_units > ft_max_rach_len_actual_units) {
		LOG_ERR("PT_SM_ASSOC_REQ: Assembled AssocReq PDU needs %u units, but FT RACH max is %u. Cannot send. Restarting scan.",
			assoc_req_tx_duration_actual_units, ft_max_rach_len_actual_units);
		k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab);
		dect_mac_sm_pt_start_operation();
		return;
	}

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel,
		full_mac_pdu_for_phy, pdu_len,
		ctx->role_ctx.pt.target_ft.short_rd_id,
		false, phy_op_handle, PENDING_OP_PT_RACH_ASSOC_REQ,
		true, 0, ctx->own_phy_params.mu, NULL);

	dect_mac_api_buffer_free((mac_sdu_t *)full_mac_pdu_for_phy);

	if (ret != 0) {
		LOG_ERR("PT_SM_ASSOC_REQ: Failed to schedule AssocReq TX: %d. Entering backoff.", ret);
		dect_mac_change_state(MAC_STATE_PT_RACH_BACKOFF);
		// uint32_t backoff_ms = 20 + (sys_rand32_get() % 50);
		uint32_t random_value;
		sys_rand_get(&random_value, sizeof(random_value));
		uint32_t backoff_ms = 20 + (random_value % 50);  // Range: 20-69ms		
		k_timer_start(&ctx->rach_context.rach_backoff_timer, K_MSEC(backoff_ms), K_NO_WAIT);
	} else {
		LOG_INF("PT_SM_ASSOC_REQ: Association Request TX scheduled (Hdl %u) to FT 0x%04X.",
			phy_op_handle, ctx->role_ctx.pt.target_ft.short_rd_id);
	}
}




static void pt_process_association_response_pdu(const uint8_t *mac_sdu_area_data,
						size_t mac_sdu_area_len,
						uint32_t ft_tx_long_rd_id,
						uint64_t assoc_resp_pcc_rx_time)
{
	dect_mac_context_t *ctx = get_mac_context();

	k_timer_stop(&ctx->rach_context.rach_response_window_timer);

	if (ft_tx_long_rd_id != ctx->role_ctx.pt.target_ft.long_rd_id) {
		LOG_WRN("PT_SM_ASSOC_RESP: From unexpected FT L:0x%08X (expected L:0x%08X). Ignoring.",
			ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.long_rd_id);
		return;
	}

	dect_mac_assoc_resp_ie_t resp_fields;
	dect_mac_rd_capability_ie_t ft_cap_fields;
	dect_mac_resource_alloc_ie_fields_t res_alloc_fields;
	bool resp_ie_found = false;
	bool ft_cap_found = false;
	bool res_alloc_found = false;

	const uint8_t *current_ie_ptr = mac_sdu_area_data;
	size_t remaining_len = mac_sdu_area_len;

	LOG_INF("PT_SM_ASSOC_RESP: Processing AssocResp from FT L:0x%08X (S:0x%04X)",
		ft_tx_long_rd_id, ctx->role_ctx.pt.target_ft.short_rd_id);

	while (remaining_len > 0) {
		uint8_t ie_type;
		uint16_t ie_payload_len;
		const uint8_t *ie_payload_ptr;
		
        // The parse_mac_mux_header function now correctly handles all length calculations.
		int mux_hdr_len = parse_mac_mux_header(current_ie_ptr, remaining_len, &ie_type,
						       &ie_payload_len, &ie_payload_ptr);

		if (mux_hdr_len <= 0) {
			break;
		}
        
        /* --- THIS BLOCK IS REMOVED --- */
		// if (ie_payload_len == 0 && ((current_ie_ptr[0] >> 6) & 0x03) == 0b00) {
		// 	ie_payload_len = get_fixed_ie_payload_len(ie_type);
		// }
        
		if (remaining_len < (size_t)mux_hdr_len + ie_payload_len) {
			break;
		}

		if (ie_type == IE_TYPE_ASSOC_RESP) {
			if (parse_assoc_resp_ie_payload(ie_payload_ptr, ie_payload_len,
							&resp_fields) == 0) {
				resp_ie_found = true;
			}
		} else if (ie_type == IE_TYPE_RD_CAPABILITY) {
			if (parse_rd_capability_ie_payload(ie_payload_ptr, ie_payload_len,
							   &ft_cap_fields) == 0) {
				ft_cap_found = true;
			}
		} else if (ie_type == IE_TYPE_RES_ALLOC) {
			uint8_t ft_mu_code =
				(ft_cap_found && ft_cap_fields.num_phy_capabilities >= 1)
					? ft_cap_fields.phy_variants[0].mu_value
					: 0;
			if (parse_resource_alloc_ie_payload(ie_payload_ptr, ie_payload_len,
							    ft_mu_code, &res_alloc_fields) == 0) {
				res_alloc_found = true;
			}
		}

		size_t consumed = mux_hdr_len + ie_payload_len;
		if (consumed == 0) {
			LOG_WRN("PT_ASSOC_RESP: Consumed 0 bytes, breaking loop.");
			break;
		}
		if (remaining_len >= consumed) {
			remaining_len -= consumed;
			current_ie_ptr += consumed;
		} else {
			remaining_len = 0;
		}
	}

	if (!resp_ie_found) {
		LOG_ERR("PT_SM_ASSOC_RESP: Association Response IE missing. Restarting scan.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	if (resp_fields.ack_nack) {
		if (!ft_cap_found || !res_alloc_found) {
			LOG_ERR("PT_SM_ASSOC_RESP: ACK received but mandatory IEs missing (Cap:%d, Res:%d). Restarting scan.",
				ft_cap_found, res_alloc_found);
			dect_mac_sm_pt_start_operation();
			return;
		}

		bool is_handover = (ctx->state == MAC_STATE_PT_HANDOVER_ASSOCIATING);

		if (is_handover) {
			LOG_INF("MOBILITY: Handover association to FT 0x%04X ACCEPTED.",
				ctx->role_ctx.pt.target_ft.short_rd_id);
			dect_mac_peer_info_t old_ft_info = ctx->role_ctx.pt.associated_ft;

			memcpy(&ctx->role_ctx.pt.associated_ft, &ctx->role_ctx.pt.target_ft,
			       sizeof(dect_mac_peer_info_t));

			if (old_ft_info.is_valid) {
				pt_send_association_release_action(&old_ft_info);
			}
		} else {
			LOG_INF("PT_SM: Association ACCEPTED by FT L:0x%08X (S:0x%04X).",
				ctx->role_ctx.pt.target_ft.long_rd_id,
				ctx->role_ctx.pt.target_ft.short_rd_id);
			memcpy(&ctx->role_ctx.pt.associated_ft, &ctx->role_ctx.pt.target_ft,
			       sizeof(dect_mac_peer_info_t));
		}
        pt_requeue_held_packets(ctx);
		ctx->role_ctx.pt.associated_ft.is_valid = true;

		if (ft_cap_found) {
			ctx->role_ctx.pt.associated_ft.num_phy_variants = ft_cap_fields.actual_num_phy_variants_parsed;
			if (ctx->role_ctx.pt.associated_ft.num_phy_variants > 0) {
				for (int i = 0; i < ctx->role_ctx.pt.associated_ft.num_phy_variants; i++) {
					memcpy(&ctx->role_ctx.pt.associated_ft.phy_variants[i], &ft_cap_fields.phy_variants[i],
					       sizeof(dect_mac_phy_capability_set_t));
				}
				ctx->role_ctx.pt.associated_ft.peer_mu = ft_cap_fields.phy_variants[0].mu_value;
				ctx->role_ctx.pt.associated_ft.peer_beta = ft_cap_fields.phy_variants[0].beta_value;
				ctx->role_ctx.pt.associated_ft.peer_phy_params_known = true;
			}
		}

		/* Store the negotiated parameters from the FT's response */
		if (resp_fields.harq_mod_present) {
			LOG_INF("PT_SM: FT negotiated HARQ params -> PT_TX(FT_RX) Procs: %u, Delay: %u; PT_RX(FT_TX) Procs: %u, Delay: %u",
				resp_fields.harq_processes_rx_val_ft, /* FT's RX is our TX */
				resp_fields.max_harq_re_rx_delay_code_ft,
				resp_fields.harq_processes_tx_val_ft, /* FT's TX is our RX */
				resp_fields.max_harq_re_tx_delay_code_ft);

			ctx->role_ctx.pt.associated_ft.pt_req_harq_procs_tx = resp_fields.harq_processes_rx_val_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_max_harq_retx_delay = resp_fields.max_harq_re_rx_delay_code_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_harq_procs_rx = resp_fields.harq_processes_tx_val_ft;
			ctx->role_ctx.pt.associated_ft.pt_req_max_harq_rerx_delay = resp_fields.max_harq_re_tx_delay_code_ft;
		}

		if (resp_fields.group_assignment_active) {
			ctx->role_ctx.pt.associated_ft.group_id = resp_fields.group_id_val;
			ctx->role_ctx.pt.associated_ft.resource_tag = resp_fields.resource_tag_val;
			LOG_INF("PT_SM: Assigned to Group ID %u with Resource Tag %u by FT.",
				resp_fields.group_id_val, resp_fields.resource_tag_val);
		}

		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.target_ft.is_valid = false;

		if (res_alloc_found) {
			LOG_INF("PT_SM: Storing schedule from FT 0x%04X.",
				ctx->role_ctx.pt.associated_ft.short_rd_id);

			uint32_t frame_duration_ticks_val =
				(uint32_t)FRAME_DURATION_MS_NOMINAL *
				(NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
			if (frame_duration_ticks_val == 0) {
				LOG_ERR("PT_SCHED: Frame duration ticks is 0!");
				return;
			}

			uint8_t ft_mu_code = ctx->role_ctx.pt.associated_ft.peer_phy_params_known
						     ? ctx->role_ctx.pt.associated_ft.peer_mu
						     : 0;
			uint8_t subslots_per_ft_slot = get_subslots_per_etsi_slot_for_mu(ft_mu_code);
			uint16_t schedule_channel =
				res_alloc_fields.channel_present
					? res_alloc_fields.channel_val
					: ctx->role_ctx.pt.associated_ft.operating_carrier;

			/* Populate Downlink Schedule */
			dect_mac_schedule_t *dl_sched = &ctx->role_ctx.pt.dl_schedule;
			memset(dl_sched, 0, sizeof(dect_mac_schedule_t));
			dl_sched->is_active = true;
			dl_sched->alloc_type = RES_ALLOC_TYPE_DOWNLINK;
			dl_sched->res1_is_9bit_subslot = res_alloc_fields.res1_is_9bit_subslot;
			dl_sched->dl_start_subslot = res_alloc_fields.start_subslot_val_res1;
			dl_sched->dl_length_is_slots = res_alloc_fields.length_type_is_slots_res1;
			dl_sched->dl_duration_subslots = res_alloc_fields.length_val_res1 + 1;
			if (dl_sched->dl_length_is_slots) {
				dl_sched->dl_duration_subslots *= subslots_per_ft_slot;
			}

			dl_sched->repeat_type = res_alloc_fields.repeat_val;
			dl_sched->repetition_value = res_alloc_fields.repetition_value;
			dl_sched->validity_value = res_alloc_fields.validity_value;
			dl_sched->channel = schedule_channel;
			dl_sched->schedule_init_modem_time = assoc_resp_pcc_rx_time;

			if (res_alloc_fields.sfn_present) {
				dl_sched->sfn_of_initial_occurrence = res_alloc_fields.sfn_val;
				dl_sched->next_occurrence_modem_time = calculate_target_modem_time(
					ctx, ctx->ft_sfn_zero_modem_time_anchor,
					ctx->current_sfn_at_anchor_update, res_alloc_fields.sfn_val,
					dl_sched->dl_start_subslot, ft_mu_code,
					ctx->role_ctx.pt.associated_ft.peer_beta);
			} else {
				dl_sched->sfn_of_initial_occurrence =
					ctx->current_sfn_at_anchor_update;
				uint64_t now_plus_processing_delay =
					assoc_resp_pcc_rx_time +
					modem_us_to_ticks(
						CONFIG_DECT_MAC_SCHEDULE_PROCESSING_DELAY_US,
						NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
				uint64_t current_frame_start_approx =
					(now_plus_processing_delay / frame_duration_ticks_val) *
					frame_duration_ticks_val;
				if (current_frame_start_approx < ctx->ft_sfn_zero_modem_time_anchor) {
					current_frame_start_approx = ctx->ft_sfn_zero_modem_time_anchor;
				}
				uint32_t ft_subslot_duration = get_subslot_duration_ticks_for_mu(ft_mu_code);
				uint64_t candidate_time =
					current_frame_start_approx +
					(uint64_t)dl_sched->dl_start_subslot * ft_subslot_duration;
				while (candidate_time <= now_plus_processing_delay) {
					candidate_time += frame_duration_ticks_val;
				}
				dl_sched->next_occurrence_modem_time = candidate_time;
			}
			update_next_occurrence(ctx, dl_sched, ctx->last_known_modem_time);
			LOG_INF("PT_SM: DL Schedule Init: NextOcc @ %llu, StartSS %u, Dur %u subslots",
				dl_sched->next_occurrence_modem_time, dl_sched->dl_start_subslot,
				dl_sched->dl_duration_subslots);

			/* Populate Uplink Schedule */
			if (res_alloc_fields.alloc_type_val == RES_ALLOC_TYPE_BIDIR) {
				dect_mac_schedule_t *ul_sched = &ctx->role_ctx.pt.ul_schedule;
				memset(ul_sched, 0, sizeof(dect_mac_schedule_t));
				ul_sched->is_active = true;
				ul_sched->alloc_type = RES_ALLOC_TYPE_UPLINK;
				ul_sched->res1_is_9bit_subslot = res_alloc_fields.res2_is_9bit_subslot;
				ul_sched->ul_start_subslot = res_alloc_fields.start_subslot_val_res2;
				ul_sched->ul_length_is_slots =
					res_alloc_fields.length_type_is_slots_res2;
				ul_sched->ul_duration_subslots = res_alloc_fields.length_val_res2 + 1;
				if (ul_sched->ul_length_is_slots) {
					ul_sched->ul_duration_subslots *= subslots_per_ft_slot;
				}
				ul_sched->repeat_type = res_alloc_fields.repeat_val;
				ul_sched->repetition_value = res_alloc_fields.repetition_value;
				ul_sched->validity_value = res_alloc_fields.validity_value;
				ul_sched->channel = schedule_channel;
				ul_sched->schedule_init_modem_time = assoc_resp_pcc_rx_time;

				if (res_alloc_fields.sfn_present) {
					ul_sched->sfn_of_initial_occurrence = res_alloc_fields.sfn_val;
					ul_sched->next_occurrence_modem_time = calculate_target_modem_time(
						ctx, ctx->ft_sfn_zero_modem_time_anchor,
						ctx->current_sfn_at_anchor_update,
						res_alloc_fields.sfn_val, ul_sched->ul_start_subslot,
						ft_mu_code, ctx->role_ctx.pt.associated_ft.peer_beta);
				} else {
					ul_sched->sfn_of_initial_occurrence =
						ctx->current_sfn_at_anchor_update;
					uint64_t now_plus_processing_delay =
						assoc_resp_pcc_rx_time +
						modem_us_to_ticks(
							CONFIG_DECT_MAC_SCHEDULE_PROCESSING_DELAY_US,
							NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
					uint64_t current_frame_start_approx =
						(now_plus_processing_delay /
						 frame_duration_ticks_val) *
						frame_duration_ticks_val;
					if (current_frame_start_approx <
					    ctx->ft_sfn_zero_modem_time_anchor) {
						current_frame_start_approx =
							ctx->ft_sfn_zero_modem_time_anchor;
					}
					uint32_t ft_subslot_duration = get_subslot_duration_ticks_for_mu(ft_mu_code);
					uint64_t candidate_time =
						current_frame_start_approx +
						(uint64_t)ul_sched->ul_start_subslot *
							ft_subslot_duration;
					while (candidate_time <= now_plus_processing_delay) {
						candidate_time += frame_duration_ticks_val;
					}
					ul_sched->next_occurrence_modem_time = candidate_time;
				}
				update_next_occurrence(ctx, ul_sched, ctx->last_known_modem_time);
				LOG_INF("PT_SM: UL Schedule Init: NextOcc @ %llu, StartSS %u, Dur %u subslots",
					ul_sched->next_occurrence_modem_time,
					ul_sched->ul_start_subslot, ul_sched->ul_duration_subslots);
			} else {
				ctx->role_ctx.pt.ul_schedule.is_active = false;
			}
		}

		bool attempt_security = IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE);

		if (attempt_security) {
			dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);
			dect_mac_sm_pt_initiate_authentication_protocol();
		} else {
			ctx->keys_provisioned = false;
			ctx->role_ctx.pt.associated_ft.is_secure = false;
			dect_mac_change_state(MAC_STATE_ASSOCIATED);
			k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
				      K_MSEC(ctx->config.keep_alive_period_ms),
				      K_MSEC(ctx->config.keep_alive_period_ms));
			if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
				k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
					      K_MSEC(ctx->config.mobility_scan_interval_ms),
					      K_MSEC(ctx->config.mobility_scan_interval_ms));
			}
			ctx->role_ctx.pt.current_assoc_retries = 0;
		}
	} else {
		LOG_WRN("PT_SM_ASSOC_RESP: Association REJECTED by FT 0x%04X. Cause: %u, Timer Code: %u.",
			ctx->role_ctx.pt.target_ft.short_rd_id, resp_fields.reject_cause,
			resp_fields.reject_timer_code);
		memset(&ctx->role_ctx.pt.target_ft, 0, sizeof(dect_mac_peer_info_t));
		ctx->role_ctx.pt.target_ft.is_valid = false;
		dect_mac_sm_pt_start_operation();
	}
}


static void pt_send_auth_response_action(void)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_peer_info_t *target_ft = &ctx->role_ctx.pt.target_ft;
	uint8_t pt_mac[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t k_auth[16];
	int ret;

	LOG_INF("PT_AUTH_RESP: Generating Auth Response for FT 0x%04X.", target_ft->short_rd_id);

	ret = security_derive_auth_key(ctx->master_psk, k_auth);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to derive K_auth: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	ret = security_generate_auth_mac(k_auth, target_ft->pt_nonce, target_ft->ft_nonce,
					 ctx->own_long_rd_id, target_ft->long_rd_id, pt_mac);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to generate PT MAC: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	uint8_t sdu_area_buf[32];
	int sdu_area_len =
		build_auth_response_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf), pt_mac);
	if (sdu_area_len < 0) {
		LOG_ERR("PT_AUTH_RESP: Failed to build Auth Response IE: %d", sdu_area_len);
		dect_mac_sm_pt_start_operation();
		return;
	}

	dect_mac_header_type_octet_t hdr_type = {.version = 0,
						 .mac_security = MAC_SECURITY_NONE,
						 .mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST};
	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(target_ft->long_rd_id);

	mac_sdu_t *pdu_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	if (!pdu_sdu) {
		LOG_ERR("FT_AUTH_CHAL: Failed to alloc PDU buffer.");
		return;
	}

	uint16_t pdu_len;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						   &hdr_type, &common_hdr, sizeof(common_hdr),
						   sdu_area_buf, (size_t)sdu_area_len, &pdu_len);
	if (ret != 0) {
		dect_mac_api_buffer_free(pdu_sdu);
		return;
	}

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.pt.current_ft_rach_params.rach_operating_channel, pdu_sdu->data, pdu_len,
		target_ft->short_rd_id, false, phy_op_handle, PENDING_OP_PT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);

	dect_mac_api_buffer_free(pdu_sdu);

	if (ret == 0) {
		dect_mac_change_state(MAC_STATE_PT_WAIT_AUTH_SUCCESS);
		/* TODO: Start a response timer for the success message */
	} else {
		/* TODO: Handle RACH busy/backoff */
		dect_mac_sm_pt_start_operation();
	}
}


static void pt_process_auth_success(dect_mac_context_t *ctx, const uint8_t *ft_mac)
{
	dect_mac_peer_info_t *target_ft = &ctx->role_ctx.pt.target_ft;
	uint8_t k_auth[16];
	uint8_t expected_ft_mac[DECT_MAC_AUTH_MAC_SIZE];
	int ret;

	LOG_INF("PT_AUTH_SUCC: Received Auth Success from FT 0x%04X.", target_ft->short_rd_id);

	ret = security_derive_auth_key(ctx->master_psk, k_auth);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Failed to derive K_auth: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/* Generate the expected FT MAC for verification */
	ret = security_generate_auth_mac(k_auth, target_ft->ft_nonce, target_ft->pt_nonce,
					 target_ft->long_rd_id, ctx->own_long_rd_id,
					 expected_ft_mac);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Failed to generate expected FT MAC: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	if (constant_time_memcmp(ft_mac, expected_ft_mac, DECT_MAC_AUTH_MAC_SIZE) != 0) {
		LOG_ERR("PT_AUTH_SUCC: FT MAC verification FAILED! Aborting association.");
		dect_mac_sm_pt_start_operation();
		return;
	}

	LOG_INF("PT_AUTH_SUCC: FT MAC verified. Deriving session keys.");

	/* Derive final session keys using the now-trusted K_auth */
	ret = security_derive_session_keys(k_auth, ctx->own_long_rd_id, target_ft->long_rd_id,
					   ctx->integrity_key, ctx->cipher_key);
	if (ret != 0) {
		LOG_ERR("PT_AUTH_SUCC: Session key derivation failed: %d", ret);
		dect_mac_sm_pt_start_operation();
		return;
	}

	/* Authentication is fully complete */
	pt_authentication_complete_action(ctx, true);
}

static void pt_process_group_assignment_ie(const uint8_t *payload, uint16_t len)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_peer_info_t *ft = &ctx->role_ctx.pt.associated_ft;

	if (!ft->is_valid || ft->group_id == 0) {
		return; /* Not part of a group */
	}

	uint8_t group_id = (payload[0] >> 0) & 0x3F;

	if (group_id != ft->group_id) {
		return; /* Not for our group */
	}

	uint8_t num_tags = len - 1;

	for (int i = 0; i < num_tags; i++) {
		uint8_t tag = payload[i + 1] & 0x7F;

		if (tag == ft->resource_tag) {
			/* This is our slot repetition */
			dect_mac_schedule_t *ul_sched = &ctx->role_ctx.pt.ul_schedule;

			if (ctx->role_ctx.pt.group_schedule.is_active &&
			    (ctx->role_ctx.pt.group_schedule.repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP ||
			     ctx->role_ctx.pt.group_schedule.repeat_type == RES_ALLOC_REPEAT_SUBSLOTS_GROUP)) {
				
				/* Activate the main UL schedule from the group template */
				memcpy(ul_sched, &ctx->role_ctx.pt.group_schedule, sizeof(dect_mac_schedule_t));
				ul_sched->is_active = true;

				uint64_t repetition_ticks = 0;
				uint32_t frame_ticks = (uint32_t)FRAME_DURATION_MS_NOMINAL * (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
				if (ul_sched->repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP) {
					repetition_ticks = (uint64_t)ul_sched->repetition_value * frame_ticks;
				} else {
					repetition_ticks = (uint64_t)ul_sched->repetition_value *
						get_subslot_duration_ticks_for_mu(ft->peer_mu);
				}
				/* Our specific slot is the i-th repetition */
				ul_sched->next_occurrence_modem_time =
					ul_sched->schedule_init_modem_time + (i * repetition_ticks);
				
				update_next_occurrence(ctx, ul_sched, ctx->last_known_modem_time);

				LOG_INF("GROUP_ASSIGN: Found our slot! Tag %u is repetition %d. Activating UL schedule. New UL time: %llu",
					tag, i, ul_sched->next_occurrence_modem_time);
			}
			break;
		}
	}
}

static void pt_send_keep_alive_action(void) {
    dect_mac_context_t* ctx = get_mac_context();

    if (ctx->state != MAC_STATE_ASSOCIATED || !ctx->role_ctx.pt.associated_ft.is_valid) {
        LOG_WRN("PT_SM_KA: Cannot send Keep Alive, not in associated state or no valid FT.");
        k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
        return;
    }

    if (ctx->pending_op_type != PENDING_OP_NONE) {
        LOG_DBG("PT_SM_KA: PHY op %s pending, deferring Keep Alive. Will retry on next timer expiry.",
                dect_pending_op_to_str(ctx->pending_op_type));
        return;
    }

    LOG_INF("PT_SM_KA: Sending Keep Alive to associated FT 0x%04X.", ctx->role_ctx.pt.associated_ft.short_rd_id);

    uint8_t sdu_area_buf[20]; // KeepAlive IE is small, SecIE is 6 bytes.
    size_t current_sdu_area_len = 0;
    int ie_len_written_val;
    size_t len_of_muxed_sec_ie_for_crypto_calc = 0; // Initialize

    bool secure_this_pdu = ctx->role_ctx.pt.associated_ft.is_secure && ctx->keys_provisioned;
    bool include_mac_sec_info_ie_for_ka = false;
    uint8_t sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;

    if (secure_this_pdu) {
	    dect_mac_peer_info_t *assoc_ft_ctx = &ctx->role_ctx.pt.associated_ft;

	    if (assoc_ft_ctx->self_needs_to_request_hpc_from_peer) {
		    include_mac_sec_info_ie_for_ka = true;
		    sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE;
	    } else if (assoc_ft_ctx->peer_requested_hpc_resync || ctx->send_mac_sec_info_ie_on_next_tx) {
		    include_mac_sec_info_ie_for_ka = true;
		    sec_iv_type_for_ka_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
	    }
    }


    // 1. MAC Header Type Octet
    dect_mac_header_type_octet_t hdr_type_octet;
    hdr_type_octet.version = 0;
    hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST;
    if (secure_this_pdu) {
        hdr_type_octet.mac_security = include_mac_sec_info_ie_for_ka ? MAC_SECURITY_USED_WITH_IE : MAC_SECURITY_USED_NO_IE;
    } else {
        hdr_type_octet.mac_security = MAC_SECURITY_NONE;
    }

    // 2. MAC Common Unicast Header
    dect_mac_unicast_header_t common_hdr;
    increment_psn_and_hpc(ctx); 
    uint16_t current_psn_for_tx = ctx->psn;
    uint32_t current_hpc_for_iv = ctx->hpc;

    common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((current_psn_for_tx >> 8) & 0x0F, 1);
    common_hdr.sequence_num_low = current_psn_for_tx & 0xFF;
    common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
    common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(ctx->role_ctx.pt.associated_ft.long_rd_id);

    // 3. SDU Area Construction
    if (include_mac_sec_info_ie_for_ka) {
        ie_len_written_val = build_mac_security_info_ie_muxed(
            sdu_area_buf + current_sdu_area_len,
            sizeof(sdu_area_buf) - current_sdu_area_len,
            0, ctx->current_key_index,
            sec_iv_type_for_ka_ie,
            ctx->hpc); 
        if (ie_len_written_val < 0) { LOG_ERR("PT_SM_KA: Build MAC Sec Info IE failed: %d", ie_len_written_val); return; }
        current_sdu_area_len += ie_len_written_val;
        len_of_muxed_sec_ie_for_crypto_calc = ie_len_written_val;
        if (sec_iv_type_for_ka_ie == SEC_IV_TYPE_MODE1_HPC_PROVIDED) {
             LOG_DBG("PT_SM_KA: Including MAC Sec Info IE (PT_HPC: %u) as PROVIDED.", ctx->hpc);
        } else {
             LOG_DBG("PT_SM_KA: Including MAC Sec Info IE (PT_HPC: %u) as RESYNC_INITIATE.", ctx->hpc);
        }
    }

    ie_len_written_val = build_keep_alive_ie_muxed(sdu_area_buf + current_sdu_area_len,
                                               sizeof(sdu_area_buf) - current_sdu_area_len);
    if (ie_len_written_val < 0) { LOG_ERR("PT_SM_KA: Build Keep Alive IE failed: %d", ie_len_written_val); return; }
    current_sdu_area_len += ie_len_written_val;

    // 4. Assemble PDU
    uint8_t *full_mac_pdu_for_phy_slab = NULL;
    int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab, K_NO_WAIT);
    if(ret != 0 || full_mac_pdu_for_phy_slab == NULL) {
        LOG_ERR("PT_SM_KA: Failed to alloc PDU buf for Keep Alive.");
        return;
    }
    uint8_t * const full_mac_pdu_for_phy = full_mac_pdu_for_phy_slab;

    uint16_t assembled_pdu_len_pre_mic;
    ret = dect_mac_phy_ctrl_assemble_final_pdu(
              full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE,
              &hdr_type_octet,
              &common_hdr, sizeof(common_hdr),
              sdu_area_buf, current_sdu_area_len,
              &assembled_pdu_len_pre_mic);

    if (ret != 0) {
        LOG_ERR("PT_SM_KA: Assemble PDU failed: %d", ret);
        k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab);
        return;
    }

    // 5. Apply Security if active
    uint16_t final_tx_pdu_len = assembled_pdu_len_pre_mic; // Length before MIC

    if (secure_this_pdu) { // secure_this_pdu was set earlier based on link state
        uint8_t iv[16];
        // current_hpc_for_iv and current_psn_for_tx were determined earlier
        security_build_iv(iv, ctx->own_long_rd_id, ctx->role_ctx.pt.associated_ft.long_rd_id,
                          current_hpc_for_iv, current_psn_for_tx);

        // MIC Calculation: Covers MAC Common Header + entire MAC SDU Area (cleartext).
        uint8_t *mic_calculation_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t);
        size_t mic_calculation_length = sizeof(common_hdr) + current_sdu_area_len;

        if ((assembled_pdu_len_pre_mic + 5) > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
            LOG_ERR("PT_SM_KA: No space for MIC in PDU.");
            k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab); return;
        }
        uint8_t *mic_location_ptr = full_mac_pdu_for_phy + assembled_pdu_len_pre_mic;
        ret = security_calculate_mic(mic_calculation_start_ptr, mic_calculation_length,
                                   ctx->integrity_key, mic_location_ptr);
        if (ret != 0) {
            LOG_ERR("PT_SM_KA: MIC calculation failed: %d", ret);
            k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab); return;
        }
        final_tx_pdu_len = assembled_pdu_len_pre_mic + 5;

        // Encryption:
        uint8_t *encryption_start_ptr;
        size_t encryption_length;
        // len_of_muxed_sec_ie_for_crypto_calc was determined when sdu_area_buf was populated

        if (hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) { // i.e. include_mac_sec_info_ie_for_ka was true
            // Encrypt: (Rest of MAC SDU Area, i.e., SDU Area - MUXed SecIE) + MIC
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) +
                                   sizeof(common_hdr) + len_of_muxed_sec_ie_for_crypto_calc;
            encryption_length = (current_sdu_area_len - len_of_muxed_sec_ie_for_crypto_calc) + 5; // (Rest of SDU Area) + MIC
        } else { // MAC_SECURITY_USED_NO_IE
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) + sizeof(common_hdr);
            encryption_length = current_sdu_area_len + 5; // Full SDU Area + MIC
        }

        if (encryption_length > 0) {
             uint8_t* pdu_buffer_end_with_mic = full_mac_pdu_for_phy + final_tx_pdu_len;
             if (encryption_start_ptr < full_mac_pdu_for_phy || (encryption_start_ptr + encryption_length) > pdu_buffer_end_with_mic ) {
                 LOG_ERR("PT_SM_KA: Encryption range error. Start %p + Len %zu > PDU End %p",
                         encryption_start_ptr, encryption_length, pdu_buffer_end_with_mic);
                 k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab); return;
             }
             ret = security_crypt_payload(encryption_start_ptr, encryption_length, ctx->cipher_key, iv, true /*encrypt*/);
             if (ret != 0) {
                 LOG_ERR("PT_SM_KA: Encryption failed: %d", ret);
                 k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab); return;
             }
        }
        LOG_DBG("PT_SM_KA: Keep Alive PDU secured. Final len %u. Mode: %s, MUXSecIELen: %zu",
                final_tx_pdu_len,
                (hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) ? "WITH_SEC_IE" : "NO_SEC_IE",
                len_of_muxed_sec_ie_for_crypto_calc);
    }

    // 6. Schedule TX
    // uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
    ret = dect_mac_phy_ctrl_start_tx_assembled(
        ctx->role_ctx.pt.associated_ft.operating_carrier,
        full_mac_pdu_for_phy, 
		final_tx_pdu_len,
        ctx->role_ctx.pt.associated_ft.short_rd_id,
        false, /* is_beacon */
        phy_op_handle,
        PENDING_OP_PT_KEEP_ALIVE,
        true, /* use_lbt for unicast control PDU */
        0,                                  // Target start time = 0 for immediate send
		ctx->own_phy_params.mu,             // **Use the FT's own mu_code**
        NULL                                // No piggybacked HARQ feedback
    );


    // --- This is the block for clearing HPC sync flags after successful scheduling ---
    if (ret == 0 && secure_this_pdu /* && !is_retransmission_placeholder_ka - KA is new */) {
        dect_mac_peer_info_t *assoc_ft_ctx_flags = &ctx->role_ctx.pt.associated_ft;
        if (include_mac_sec_info_ie_for_ka) { // If SecIE was actually sent
            if (sec_iv_type_for_ka_ie == SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE) {
                if (assoc_ft_ctx_flags->self_needs_to_request_hpc_from_peer) {
                    LOG_DBG("PT_SM_KA: Cleared self_needs_to_request_hpc_from_peer for FT after sending INITIATE.");
                    assoc_ft_ctx_flags->self_needs_to_request_hpc_from_peer = false;
                }
            } else { // SEC_IV_TYPE_MODE1_HPC_PROVIDED was sent
                bool cleared_peer_req = false;
                if (assoc_ft_ctx_flags->peer_requested_hpc_resync) {
                    LOG_DBG("PT_SM_KA: Cleared peer_requested_hpc_resync for FT after sending PROVIDED.");
                    assoc_ft_ctx_flags->peer_requested_hpc_resync = false;
                    cleared_peer_req = true;
                }
                // Only clear global send_mac_sec_info_ie_on_next_tx if it was the *sole* reason for sending PROVIDED
                if (ctx->send_mac_sec_info_ie_on_next_tx && !cleared_peer_req &&
                    !(sec_iv_type_for_ka_ie == SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE) ) { // Double check it wasn't initiate
                    LOG_DBG("PT_SM_KA: Cleared global send_mac_sec_info_ie_on_next_tx after sending PROVIDED for own HPC wrap.");
                    ctx->send_mac_sec_info_ie_on_next_tx = false;
                }
            }
        }
    }
    // --- End of HPC sync flag clearing block ---

    k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_for_phy_slab); // Free slab buffer after PHY call

    if (ret != 0) {
        LOG_ERR("PT_SM_KA: Failed to schedule Keep Alive TX: %d", ret);
    } else {
        LOG_INF("PT_SM_KA: Keep Alive TX scheduled (Hdl %u).", phy_op_handle);
    }
}







static void pt_start_authentication_with_ft_action(dect_mac_context_t *ctx) {
    if (!ctx) {
        LOG_ERR("PT_AUTH_START: NULL context provided.");
        return;
    }

    if (!ctx->role_ctx.pt.associated_ft.is_valid) {
        LOG_ERR("PT_AUTH_START: No valid associated FT to authenticate with. Aborting auth.");
        // This state should not normally be reached if called correctly after processing AssocResp(ACK)
        dect_mac_change_state(MAC_STATE_IDLE); // Go back to idle to rescan
        dect_mac_sm_pt_start_operation();      // Trigger rescan
        return;
    }

    LOG_INF("PT SM: Starting Authentication (PSK-based key derivation) with FT 0x%04X (L:0x%08X).",
            ctx->role_ctx.pt.associated_ft.short_rd_id,
            ctx->role_ctx.pt.associated_ft.long_rd_id);

    dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);

    // For PSK-based "authentication", the main action is local key derivation.
    // No PDUs are exchanged for this simplified model.
    // A real authentication would involve sending an Auth Request, receiving Challenge, sending Response etc.

    if (ctx->master_psk_provisioned) {
        LOG_DBG("PT_AUTH_START: Master PSK is provisioned. Deriving session keys.");
        int kdf_err = security_derive_session_keys(
            ctx->master_psk,
            ctx->own_long_rd_id,                                // PT is local
            ctx->role_ctx.pt.associated_ft.long_rd_id,          // FT is peer
            ctx->integrity_key,
            ctx->cipher_key);

        if (kdf_err == 0) {
            LOG_INF("PT_AUTH_START: Session keys successfully derived from PSK.");
            // Call the completion action with success
            pt_authentication_complete_action(ctx, true);
        } else {
            LOG_ERR("PT_AUTH_START: Failed to derive session keys (err %d). Authentication failed.", kdf_err);
            // Call the completion action with failure
            pt_authentication_complete_action(ctx, false);
        }
    } else {
        LOG_WRN("PT_AUTH_START: Master PSK not provisioned. Cannot perform PSK-based authentication. Authentication 'fails' (link remains unsecure).");
        // Call the completion action with failure for security establishment,
        // but the MAC link might still be considered associated (unsecure).
        pt_authentication_complete_action(ctx, false);
    }
}

/**
 * @brief pt_authentication_complete_action
 * 
 * @param ctx 
 * @param success 
 */
static void pt_authentication_complete_action(dect_mac_context_t* ctx, bool success) {
    if (!ctx) {
        LOG_ERR("PT_AUTH_COMPLETE: NULL context provided.");
        return;
    }

    if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
        LOG_WRN("PT_AUTH_COMPLETE: Called in unexpected state %s. Current FT ShortID: 0x%04X",
                dect_mac_state_to_str(ctx->state), ctx->role_ctx.pt.associated_ft.short_rd_id);
        // If not in authenticating, perhaps an old/stale completion.
        // If already associated, do nothing more.
        // If in another state, it might be an error. For now, just log.
        if (ctx->state == MAC_STATE_ASSOCIATED && ctx->role_ctx.pt.associated_ft.is_valid) {
            return; // Already successfully associated.
        }
        // Otherwise, if some error led here, might need to reset.
    }

    if (success && ctx->role_ctx.pt.associated_ft.is_valid) {
        // This 'success' specifically means keys were derived and security context is ready.
        ctx->keys_provisioned = true; // PT's global session keys are now set for this FT
        ctx->role_ctx.pt.associated_ft.is_secure = true;

        // Reset/Initialize HPCs for this new secure session:
        // PT's own transmit HPC for communication with this FT.
        ctx->hpc = 1; // Start own HPC at 1 (or a random value, but 1 is fine for new session)
        // PT's tracking of the FT's transmit HPC. Assume FT also starts/resets its HPC for PT.
        ctx->role_ctx.pt.associated_ft.hpc = 1; // Initial assumption of FT's TX HPC
        // PT should send its HPC in the first secured PDU to the FT.
        ctx->send_mac_sec_info_ie_on_next_tx = true;

        dect_mac_change_state(MAC_STATE_ASSOCIATED);
        LOG_INF("PT_AUTH_COMPLETE: Authentication successful with FT 0x%04X (L:0x%08X). Link is SECURE.",
                ctx->role_ctx.pt.associated_ft.short_rd_id, ctx->role_ctx.pt.associated_ft.long_rd_id);
        LOG_INF("PT_AUTH_COMPLETE: OwnTX_HPC=%u, Tracking FT_TX_HPC=%u. Will send SecIE.",
                ctx->hpc, ctx->role_ctx.pt.associated_ft.hpc);

        // Start periodic timers for an active link
        k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
                      K_MSEC(ctx->config.keep_alive_period_ms),  // Initial delay
                      K_MSEC(ctx->config.keep_alive_period_ms)); // Period

        if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) { // Enable via Kconfig
            k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
                          K_MSEC(ctx->config.mobility_scan_interval_ms),
                          K_MSEC(ctx->config.mobility_scan_interval_ms));
        }
        // Reset association attempt counter for any future (re-)associations
        ctx->role_ctx.pt.current_assoc_retries = 0;

        // After successful association, notify the CDD service about the new link.
        // The CDD service will then decide if a configuration request is needed.
        // We assume the initial App Sequence Number from the beacon was 0 or unknown.
        LOG_INF("PT_AUTH_COMPLETE: Notifying CDD service of new association.");
        dect_cdd_pt_process_beacon_info(ctx->role_ctx.pt.associated_ft.long_rd_id, 0);

    } else { // Authentication failed or was skipped (e.g. no PSK)
        if (ctx->role_ctx.pt.associated_ft.is_valid) { // Still associated, but unsecure
            ctx->keys_provisioned = false;
            ctx->role_ctx.pt.associated_ft.is_secure = false;
            dect_mac_change_state(MAC_STATE_ASSOCIATED); // Proceed to associated state, but unsecure
            LOG_WRN("PT_AUTH_COMPLETE: Authentication failed or skipped for FT 0x%04X. Link is UNSECURE.",
                    ctx->role_ctx.pt.associated_ft.short_rd_id);
            k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
                          K_MSEC(ctx->config.keep_alive_period_ms),
                          K_MSEC(ctx->config.keep_alive_period_ms));
            if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
                k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
                              K_MSEC(ctx->config.mobility_scan_interval_ms),
                              K_MSEC(ctx->config.mobility_scan_interval_ms));
            }
        } else { // No valid associated FT (e.g. if assoc was rejected prior to auth attempt)
            LOG_ERR("PT_AUTH_COMPLETE: Authentication failed and no valid associated FT. Restarting scan.");
            dect_mac_sm_pt_start_operation(); // Go back to scanning
        }
    }
}


void dect_mac_sm_pt_initiate_authentication_protocol(void)
{
	dect_mac_context_t* ctx = get_mac_context();

	if (!ctx->role_ctx.pt.associated_ft.is_valid) {
		LOG_ERR("PT_AUTH_INIT: No valid associated FT to authenticate with. Aborting.");
		dect_mac_change_state(MAC_STATE_IDLE);
		dect_mac_sm_pt_start_operation();
		return;
	}

	if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
		LOG_WRN("PT_AUTH_INIT: Called in state %s, expected AUTHENTICATING. Transitioning.", dect_mac_state_to_str(ctx->state));
		dect_mac_change_state(MAC_STATE_PT_AUTHENTICATING);
	}

	LOG_INF("PT_AUTH_INIT: Initiating authentication handshake with FT 0x%04X.",
		ctx->role_ctx.pt.associated_ft.short_rd_id);

	pt_send_auth_initiate_action();
}

void dect_mac_sm_pt_handle_auth_pdu(const uint8_t *pdu_data, size_t pdu_len)
{
    ARG_UNUSED(pdu_data);
    ARG_UNUSED(pdu_len);
    dect_mac_context_t* ctx = get_mac_context();

    if (ctx->state != MAC_STATE_PT_AUTHENTICATING) {
        LOG_WRN("PT_AUTH_HANDLE_PDU: Received Auth PDU in unexpected state %s. Ignoring.", dect_mac_state_to_str(ctx->state));
        return;
    }

    LOG_INF("PT_AUTH_HANDLE_PDU: Received (stubbed) Auth PDU from FT 0x%04X (len %zu). No action for current PSK model.",
            ctx->role_ctx.pt.associated_ft.is_valid ? ctx->role_ctx.pt.associated_ft.short_rd_id : 0xFFFF,
            pdu_len);

    // For a real multi-step protocol, this would parse pdu_data.
    // If it's the final confirmation from FT:
    // pt_authentication_complete_action(ctx, true_if_auth_ok_else_false);
}




static void pt_update_mobility_candidate(uint16_t carrier, int16_t rssi, uint32_t long_id,
					 uint16_t short_id)
{
	dect_mac_context_t *ctx = get_mac_context();
	int free_slot = -1;
	int existing_slot = -1;

	/* Check if this candidate (by Long ID) already exists */
	for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
		if (ctx->role_ctx.pt.mobility_candidates[i].is_valid) {
			if (ctx->role_ctx.pt.mobility_candidates[i].long_rd_id == long_id) {
				existing_slot = i;
				break;
			}
		} else if (free_slot == -1) {
			free_slot = i;
		}
	}

	int target_slot = -1;

	if (existing_slot != -1) {
		target_slot = existing_slot;
		LOG_DBG("MOBILITY: Updating existing candidate in slot %d.", target_slot);
	} else if (free_slot != -1) {
		target_slot = free_slot;
		LOG_INF("MOBILITY: Adding new candidate in slot %d.", target_slot);
	} else {
		/* No free slots. Find the weakest candidate to replace. */
		int16_t weakest_rssi = 0; /* RSSI is negative, so 0 is very strong */
		int weakest_slot = 0;

		for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
			if (ctx->role_ctx.pt.mobility_candidates[i].rssi_2 < weakest_rssi) {
				weakest_rssi = ctx->role_ctx.pt.mobility_candidates[i].rssi_2;
				weakest_slot = i;
			}
		}
		if (rssi > weakest_rssi) {
			target_slot = weakest_slot;
			LOG_INF("MOBILITY: Evicting weakest candidate (slot %d, RSSI %.1f) for new one (RSSI %.1f).",
				target_slot, (float)weakest_rssi / 2.0f, (float)rssi / 2.0f);
		} else {
			LOG_DBG("MOBILITY: New candidate (RSSI %.1f) not stronger than weakest in full list (RSSI %.1f). Ignoring.",
				(float)rssi / 2.0f, (float)weakest_rssi / 2.0f);
			return;
		}
	}

	/* Update the target slot with the new information */
	dect_mobility_candidate_t *cand = &ctx->role_ctx.pt.mobility_candidates[target_slot];

	cand->is_valid = true;
	cand->long_rd_id = long_id;
	cand->short_rd_id = short_id;
	cand->operating_carrier = carrier;
	cand->rssi_2 = rssi;
	/* Reset trigger count when a candidate is first added or updated */
	cand->trigger_count_remaining = ctx->role_ctx.pt.initial_count_to_trigger;
}


static void pt_process_page_indication(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	LOG_INF("PT_PAGING: Page received from FT! Responding and transitioning to Associated state.");

	/* Stop the paging cycle timer */
	k_timer_stop(&ctx->role_ctx.pt.paging_cycle_timer);

	/*
	 * Per ETSI 5.12.3, PT shall transmit a "Paging response".
	 * If config is ok, this is a Keep alive IE.
	 * If config needs modification, it's an Association Request.
	 * We implement the Keep alive IE response here.
	 */
	pt_send_keep_alive_action();

	/* Transition back to the normal connected state */
	dect_mac_change_state(MAC_STATE_ASSOCIATED);

	/* Restart normal link supervision */
	k_timer_start(&ctx->role_ctx.pt.keep_alive_timer,
		      K_MSEC(ctx->config.keep_alive_period_ms),
		      K_MSEC(ctx->config.keep_alive_period_ms));
	if (IS_ENABLED(CONFIG_DECT_MAC_PT_MOBILITY_ENABLE)) {
		k_timer_start(&ctx->role_ctx.pt.mobility_scan_timer,
			      K_MSEC(ctx->config.mobility_scan_interval_ms),
			      K_MSEC(ctx->config.mobility_scan_interval_ms));
	}
}


static void pt_handle_phy_rssi_internal(const struct nrf_modem_dect_phy_rssi_event *event)
{
    if (event == NULL || event->meas_len == 0) {
        return;
    }

    // For mobility, we are just interested in the average channel energy.
    // If we find a quiet channel, we could schedule a brief RX on it to listen for beacons.
    int32_t rssi_sum = 0;
    int valid_count = 0;
    for (uint16_t i = 0; i < event->meas_len; ++i) {
        if (event->meas[i] != NRF_MODEM_DECT_PHY_RSSI_NOT_MEASURED) {
            rssi_sum += event->meas[i];
            valid_count++;
        }
    }

    if (valid_count > 0) {
        int16_t avg_rssi = rssi_sum / valid_count;
        LOG_DBG("MOBILITY: Scan on carrier %u result: avg RSSI %.1f dBm.", event->carrier, (float)avg_rssi / 2.0f);

        // TODO: Here you would decide if this channel is "interesting" enough to
        // do a follow-up RX listen for a beacon. For now, this RSSI scan is just a placeholder.
    }
}

static void pt_evaluate_mobility_candidate(dect_mac_context_t *ctx,
					   const dect_mac_cluster_beacon_ie_fields_t *cb_fields,
					   uint32_t ft_long_id, uint16_t ft_short_id,
					   int16_t rssi_q7_1, uint16_t beacon_rx_carrier)
{
	if (ft_long_id == ctx->role_ctx.pt.associated_ft.long_rd_id) {
		/* This is a beacon from our current FT, update its info */
		ctx->role_ctx.pt.associated_ft.rssi_2 = rssi_q7_1;
		/* Update the trigger count based on the latest beacon from our FT */
		ctx->role_ctx.pt.initial_count_to_trigger = cb_fields->count_to_trigger_code;
		return;
	}

	pt_update_mobility_candidate(beacon_rx_carrier, rssi_q7_1, ft_long_id, ft_short_id);

	/* Find the updated candidate to evaluate it */
	for (int i = 0; i < MAX_MOBILITY_CANDIDATES; i++) {
		dect_mobility_candidate_t *cand = &ctx->role_ctx.pt.mobility_candidates[i];

		if (cand->is_valid && cand->long_rd_id == ft_long_id) {
			/* RelQuality code (0-7) maps to 0,3,6,9... dB. Multiply by 2 for Q7.1 format. */
			int16_t rssi_hysteresis = (int16_t)(cb_fields->rel_quality_code * 3 * 2);

			if (cand->rssi_2 > (ctx->role_ctx.pt.associated_ft.rssi_2 + rssi_hysteresis)) {
				if (cand->trigger_count_remaining > 0) {
					cand->trigger_count_remaining--;
					LOG_INF("MOBILITY: Candidate FT 0x%04X is stronger. Trigger count now %u.",
						cand->short_rd_id, cand->trigger_count_remaining);
				}

				if (cand->trigger_count_remaining == 0) {
					LOG_INF("MOBILITY: Handover triggered for candidate FT 0x%04X!",
						cand->short_rd_id);
					pt_initiate_handover_action(ctx, cand);
				}
			} else {
				/* Not significantly better, reset its trigger count */
				if (cand->trigger_count_remaining !=
				    ctx->role_ctx.pt.initial_count_to_trigger) {
					LOG_DBG("MOBILITY: Candidate FT 0x%04X not strong enough. Resetting trigger count.",
						cand->short_rd_id);
					cand->trigger_count_remaining =
						ctx->role_ctx.pt.initial_count_to_trigger;
				}
			}
			break;
		}
	}
}

static void pt_initiate_handover_action(dect_mac_context_t *ctx,
					dect_mobility_candidate_t *candidate)
{
	if (ctx->state != MAC_STATE_ASSOCIATED) {
		LOG_WRN("MOBILITY: Handover initiated in unexpected state %s",
			dect_mac_state_to_str(ctx->state));
		return;
	}

	/* Set the new target FT from the winning candidate */
	ctx->role_ctx.pt.target_ft.is_valid = true;
	ctx->role_ctx.pt.target_ft.is_fully_identified = true;
	ctx->role_ctx.pt.target_ft.long_rd_id = candidate->long_rd_id;
	ctx->role_ctx.pt.target_ft.short_rd_id = candidate->short_rd_id;
	ctx->role_ctx.pt.target_ft.operating_carrier = candidate->operating_carrier;
	ctx->role_ctx.pt.target_ft.rssi_2 = candidate->rssi_2;
	memcpy(&ctx->role_ctx.pt.current_ft_rach_params, &candidate->rach_params_from_beacon,
	       sizeof(dect_ft_rach_params_t));

	/* Invalidate the candidate so we don't try to switch to it again immediately */
	candidate->is_valid = false;

	/* Stop timers related to the old FT before attempting to switch */
	k_timer_stop(&ctx->role_ctx.pt.keep_alive_timer);
	k_timer_stop(&ctx->role_ctx.pt.mobility_scan_timer);

	dect_mac_change_state(MAC_STATE_PT_HANDOVER_ASSOCIATING);
	LOG_INF("MOBILITY: State -> HANDOVER_ASSOCIATING. Data TX paused.");

	/* Start the association process with the new target */
	pt_send_association_request_action();
}



static void pt_send_association_release_action(dect_mac_peer_info_t *old_ft_info)
{
	if (!old_ft_info || !old_ft_info->is_valid) {
		return;
	}

	LOG_INF("MOBILITY: Sending Association Release to old FT 0x%08X", old_ft_info->long_rd_id);

	uint8_t sdu_area_buf[16];
	dect_mac_assoc_release_ie_t release_fields = {.cause = ASSOC_RELEASE_CAUSE_MOBILITY};

	int sdu_area_len = build_assoc_release_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf),
							&release_fields);
	if (sdu_area_len < 0) {
		LOG_ERR("MOBILITY: Failed to build Assoc Release IE: %d", sdu_area_len);
		return;
	}

	/* This is a fire-and-forget message, sent unreliably. */
	/* A full implementation might use a reliable service. */
	dect_mac_context_t *ctx = get_mac_context();

	mac_sdu_t *sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
	if (!sdu) {
		LOG_ERR("MOBILITY: Failed to alloc SDU for Assoc Release.");
		return;
	}

	dect_mac_header_type_octet_t hdr_type = {.version = 0,
						 .mac_security = MAC_SECURITY_NONE,
						 .mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST};
	dect_mac_unicast_header_t common_hdr;

	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(old_ft_info->long_rd_id);

	uint16_t pdu_len;

	int ret = dect_mac_phy_ctrl_assemble_final_pdu(sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE,
						       &hdr_type, &common_hdr, sizeof(common_hdr),
						       sdu_area_buf, (size_t)sdu_area_len,
						       &pdu_len);
	if (ret != 0) {
		dect_mac_api_buffer_free(sdu);
		return;
	}
	sdu->len = pdu_len;

	/* This should be sent on the OLD FT's RACH resources, which we no longer track.
	 * For simplicity, we send it via the generic API, which will use the NEW FT's
	 * resources. This is a simplification; a full implementation would need to
	 * temporarily use the old link's parameters for this one message.
	 */
	dect_mac_api_send(sdu, MAC_FLOW_HIGH_PRIORITY);
}




