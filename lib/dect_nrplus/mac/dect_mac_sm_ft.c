/* dect_mac/dect_mac_sm_ft.c */

#include <stdlib.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/dlist.h>


#include <nrf_modem_dect_phy.h>

#include <mac/dect_mac_main.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_pdu.h>
#include <mac/dect_mac_phy_ctrl.h>
#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_main_dispatcher.h>
#include <mac/dect_mac_api.h>       // For g_mac_sdu_slab
#include <dect_cdd.h>
#include <mac/dect_mac_security.h>  // For security functions
#include <mac/dect_mac_timeline_utils.h>

LOG_MODULE_REGISTER(dect_mac_sm_ft, CONFIG_DECT_MAC_SM_FT_LOG_LEVEL);

// --- Static Globals for this Module ---
static struct nrf_modem_dect_phy_pcc_event ft_last_relevant_pcc;
static bool ft_last_relevant_pcc_is_valid = false;

// --- Static Helper Function Prototypes (defined below) ---
static void ft_handle_phy_op_complete_ft(const struct nrf_modem_dect_phy_op_complete_event *event, pending_op_type_t completed_op_type);
static void ft_handle_phy_pcc_ft(const struct nrf_modem_dect_phy_pcc_event *event);
static void ft_handle_phy_pdc_ft(const struct nrf_modem_dect_phy_pdc_event *event);
static void ft_handle_phy_rssi_ft(const struct nrf_modem_dect_phy_rssi_event *event);
static void ft_select_operating_carrier_and_start_beaconing(const struct nrf_modem_dect_phy_rssi_event *rssi_event_data);
static void ft_start_beaconing_actions(void);
static void ft_send_beacon_action(void);
static void ft_schedule_rach_listen_action(void);
static void ft_process_association_request_pdu(const uint8_t *mac_sdu_area_data, size_t mac_sdu_area_len,
                                               uint16_t pt_tx_short_rd_id, uint32_t pt_tx_long_rd_id, int16_t rssi_from_pcc);

static int  ft_find_and_init_peer_slot(uint32_t pt_long_id, uint16_t pt_short_id, int16_t rssi);
static void ft_update_pt_schedule_and_signal(int peer_slot_idx, const dect_mac_resource_alloc_ie_fields_t *new_schedule_fields);
static void ft_evaluate_and_update_pt_schedules(void);
static int ft_get_peer_slot_idx(dect_mac_context_t* ctx, uint16_t pt_short_id);
static void populate_cb_fields_from_ctx(dect_mac_context_t *ctx, dect_mac_cluster_beacon_ie_fields_t *cb_fields);
static void ft_send_reject_response_action(uint32_t pt_long_id, uint16_t pt_short_id, const dect_mac_assoc_resp_ie_t *reject_fields);
static void ft_send_association_response_action(int peer_slot_idx, const dect_mac_assoc_resp_ie_t *resp_fields, const dect_mac_rd_capability_ie_t *ft_cap_fields,const dect_mac_resource_alloc_ie_fields_t *res_alloc_fields);
// static uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time, uint8_t anchor_sfn_val, uint8_t target_sfn_val, uint16_t target_subslot_idx);
uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time, uint8_t sfn_of_anchor_relevance, uint8_t target_sfn_val, uint16_t target_subslot_idx, uint8_t link_mu_code, uint8_t link_beta_code);


static void ft_send_auth_challenge_action(int peer_slot_idx);
static void ft_send_auth_success_action(int peer_slot_idx, const uint8_t *pt_mac);

// Helpers (ensure these are defined or made extern if in another file)
uint32_t get_subslot_duration_ticks(dect_mac_context_t *ctx);
uint32_t modem_us_to_ticks(uint32_t us, uint32_t tick_rate_khz);


// --- FT Timer Expiry Action Function (called by dispatcher) ---
void dect_mac_sm_ft_beacon_timer_expired_action(void) {
    dect_mac_context_t* ctx = get_mac_context();
    if (ctx->pending_op_type == PENDING_OP_NONE) {
        ft_send_beacon_action();
    } else {
        LOG_WRN("FT SM: Beacon time, but op %s pending. Skipping this beacon.",
                dect_pending_op_to_str(ctx->pending_op_type));
        uint32_t short_delay_ms = ctx->config.ft_cluster_beacon_period_ms / 4;
        if (short_delay_ms < 10) short_delay_ms = 10; // Minimum sensible retry delay
        if (short_delay_ms == 0 && ctx->config.ft_cluster_beacon_period_ms > 0) {
            short_delay_ms = ctx->config.ft_cluster_beacon_period_ms; // If period is very short, retry at full period
        } else if (short_delay_ms == 0) {
             short_delay_ms = 100; // Fallback if period was 0
        }
        k_timer_start(&ctx->role_ctx.ft.beacon_timer, K_MSEC(short_delay_ms), K_NO_WAIT);
    }
}

// --- FT Public Functions ---
void dect_mac_sm_ft_start_operation(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	dect_mac_data_path_register_scheduler_hook(ft_service_schedules);

	dect_mac_change_state(MAC_STATE_FT_SCANNING);

	ctx->role_ctx.ft.dcs_current_channel_scan_index = 0;
	ctx->role_ctx.ft.dcs_scan_complete = false;
	ctx->role_ctx.ft.dcs_num_valid_candidate_channels = 0;
	for (int i = 0; i < CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN; i++) {
		ctx->role_ctx.ft.dcs_candidate_channels[i] = 0;
		ctx->role_ctx.ft.dcs_candidate_rssi_avg[i] = NRF_MODEM_DECT_PHY_RSSI_NOT_MEASURED;
		ctx->role_ctx.ft.dcs_candidate_busy_percent[i] = 101;
	}

	if (CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN > 0) {
		const char *chan_list_str = CONFIG_DECT_MAC_DCS_CHANNEL_LIST;
		char *next_chan_str;
		char *search_start = (char *)chan_list_str;
		int parsed_count = 0;

		while (parsed_count < CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN && *search_start != '\0') {
			/* Skip leading delimiters */
			while (*search_start != '\0' && (*search_start == ',' || *search_start == ' ')) {
				search_start++;
			}
			if (*search_start == '\0') {
				break;
			}

			long chan_val = strtol(search_start, &next_chan_str, 10);

			if (search_start == next_chan_str) {
				LOG_WRN("FT_DCS_INIT: Invalid character in Kconfig channel list. Stopping parse.");
				break;
			}

			if (chan_val >= 100000 && chan_val <= 3000000) {
				ctx->role_ctx.ft.dcs_candidate_channels[parsed_count] =
					(uint16_t)chan_val;
				LOG_DBG("FT_DCS_INIT: Added candidate channel %u kHz to scan list (idx %d).",
					(uint16_t)chan_val, parsed_count);
				parsed_count++;
			} else {
				LOG_WRN("FT_DCS_INIT: Parsed channel value %ld kHz from Kconfig list is out of plausible range. Skipping.",
					chan_val);
			}
			search_start = next_chan_str;
		}
		ctx->role_ctx.ft.dcs_num_valid_candidate_channels = parsed_count;
		LOG_INF("FT_DCS_INIT: Parsed %u valid candidate channels from Kconfig for DCS.",
			parsed_count);
	}

	if (ctx->role_ctx.ft.dcs_num_valid_candidate_channels == 0) {
		LOG_WRN("FT SM: DCS: No valid channels to scan. Defaulting to carrier %u and starting beaconing.",
			CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CARRIER_KHZ);
		ctx->role_ctx.ft.operating_carrier =
			CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CARRIER_KHZ;
		if (ctx->role_ctx.ft.operating_carrier == 0) {
			ctx->role_ctx.ft.operating_carrier = 1881792;
		}
		ft_start_beaconing_actions();
		return;
	}

	ctx->role_ctx.ft.dcs_current_channel_scan_index = 0;
	uint16_t scan_carrier = ctx->role_ctx.ft.dcs_candidate_channels[0];

	LOG_INF("FT SM: Starting DCS scan, 1/%u on carrier %u.",
		ctx->role_ctx.ft.dcs_num_valid_candidate_channels, scan_carrier);

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
	uint32_t scan_duration_total_subslots =
		SCAN_MEAS_DURATION_SLOTS_CONFIG *
		get_subslots_per_etsi_slot_for_mu(ctx->own_phy_params.mu);
	uint32_t subslot_ticks = get_subslot_duration_ticks_for_mu(ctx->own_phy_params.mu);
	uint32_t scan_duration_modem_units = scan_duration_total_subslots * subslot_ticks;

	if (subslot_ticks == 0) {
		scan_duration_modem_units =
			modem_us_to_ticks(10000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
		LOG_WRN("FT_DCS_INIT: Subslot duration calc error, using default %u TU.",
			scan_duration_modem_units);
	}

	int ret = dect_mac_phy_ctrl_start_rssi_scan(
		scan_carrier, scan_duration_modem_units, NRF_MODEM_DECT_PHY_RSSI_INTERVAL_24_SLOTS,
		phy_op_handle, PENDING_OP_FT_INITIAL_SCAN);

	if (ret != 0) {
		LOG_ERR("FT SM: Failed to start initial RSSI scan for DCS (channel %u): %d. Retrying after delay.",
			scan_carrier, ret);
		k_timer_start(&ctx->role_ctx.ft.beacon_timer, K_SECONDS(1), K_NO_WAIT);
		dect_mac_change_state(MAC_STATE_IDLE);
	}
}



void dect_mac_sm_ft_handle_event(const struct dect_mac_event_msg *msg) {
    dect_mac_context_t* ctx = get_mac_context();

    switch (msg->type) {
        case MAC_EVENT_PHY_OP_COMPLETE:
            {
                pending_op_type_t completed_op = dect_mac_phy_ctrl_handle_op_complete(&msg->data.op_complete);
                if (completed_op != PENDING_OP_NONE) {
                    ft_handle_phy_op_complete_ft(&msg->data.op_complete, completed_op);
                } else {
                     LOG_DBG("FT SM: OP_COMPLETE for handle %u (err %d), but no matching/active MAC pending_op_type.",
                            msg->data.op_complete.handle, msg->data.op_complete.err);
                }
            }
            break;
        case MAC_EVENT_PHY_PCC:
            ft_handle_phy_pcc_ft(&msg->data.pcc);
            break;
        case MAC_EVENT_PHY_PDC:
            ft_handle_phy_pdc_ft(&msg->data.pdc);
            break;
        case MAC_EVENT_PHY_PCC_ERROR:
             LOG_WRN("FT SM: Received PCC_ERROR for handle %u (TID %u).", msg->data.pcc_crc_err.handle, msg->data.pcc_crc_err.transaction_id);
             if (ft_last_relevant_pcc_is_valid && ft_last_relevant_pcc.transaction_id == msg->data.pcc_crc_err.transaction_id) {
                ft_last_relevant_pcc_is_valid = false;
             }
            break;
        case MAC_EVENT_PHY_PDC_ERROR:
            LOG_WRN("FT SM: Received PDC_ERROR for handle %u (TID %u).", msg->data.pdc_crc_err.handle, msg->data.pdc_crc_err.transaction_id);
            if (ft_last_relevant_pcc_is_valid && ft_last_relevant_pcc.transaction_id == msg->data.pdc_crc_err.transaction_id) {
                uint16_t pt_short_id = sys_be16_to_cpu(
                    (uint16_t)((ft_last_relevant_pcc.hdr.hdr_type_2.transmitter_id_hi << 8) |
                                ft_last_relevant_pcc.hdr.hdr_type_2.transmitter_id_lo));
                int peer_idx = ft_get_peer_slot_idx(ctx, pt_short_id);
                if (peer_idx != -1 && ctx->role_ctx.ft.connected_pts[peer_idx].is_secure) {
                    uint8_t harq_proc_in_pt_tx = ft_last_relevant_pcc.hdr.hdr_type_2.df_harq_process_num;
                    dect_mac_peer_info_t *pt_ctx_peer = &ctx->role_ctx.ft.connected_pts[peer_idx];
                    if (pt_ctx_peer->num_pending_feedback_items < 2) {
                        int fb_idx = pt_ctx_peer->num_pending_feedback_items++;
                        pt_ctx_peer->pending_feedback_to_send[fb_idx].valid = true;
                        pt_ctx_peer->pending_feedback_to_send[fb_idx].is_ack = false;
                        pt_ctx_peer->pending_feedback_to_send[fb_idx].harq_process_num_for_peer = harq_proc_in_pt_tx;
                        LOG_WRN("FT_SM_HARQ_RX: Stored NACK for PT 0x%04X's HARQ_Proc %u due to PDC_ERROR.", pt_short_id, harq_proc_in_pt_tx);
                    } else { LOG_WRN("FT_SM_PDC_ERR: Feedback buffer full for PT 0x%04X", pt_short_id); }
                }
                ft_last_relevant_pcc_is_valid = false;
            }
            break;
        case MAC_EVENT_PHY_RSSI_RESULT:
            ft_handle_phy_rssi_ft(&msg->data.rssi);
            break;
        case MAC_EVENT_TIMER_EXPIRED_BEACON:
            if (ctx->state == MAC_STATE_FT_BEACONING ||
                (ctx->state == MAC_STATE_FT_SCANNING && ctx->pending_op_type == PENDING_OP_NONE) ) {
                dect_mac_sm_ft_beacon_timer_expired_action();
            }
            break;
        case MAC_EVENT_TIMER_EXPIRED_HARQ:
            dect_mac_data_path_handle_harq_nack_action(msg->data.timer_data.id);
            break;
        default:
            LOG_DBG("FT SM: Unhandled event type %s in state %s",
                    dect_mac_event_to_str(msg->type), dect_mac_state_to_str(ctx->state));
            break;
    }
    /* After handling an event, evaluate if schedules need updating */
    if (ctx->state == MAC_STATE_FT_BEACONING || ctx->state == MAC_STATE_ASSOCIATED) {
        ft_evaluate_and_update_pt_schedules();
    }    
}


/**
 * @brief Evaluates resource needs for connected PTs and potentially updates/sends schedules.
 *
 * This is a placeholder for a more complex FT scheduler.
 * For now, it checks if any PT has DL data pending in FT's queues and grants a basic schedule.
 */
static void ft_evaluate_and_update_pt_schedules(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->state != MAC_STATE_FT_BEACONING && ctx->state != MAC_STATE_ASSOCIATED) {
		return; /* Only schedule when FT is operational */
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (!ctx->role_ctx.ft.connected_pts[i].is_valid) {
			continue;
		}

		dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[i];
		dect_mac_schedule_t *current_pt_schedule = &ctx->role_ctx.ft.peer_schedules[i];
		
        // --- THIS IS THE CORRECTED LOGIC ---
        // Use sys_dlist_is_empty on the correct dlist members.
		bool dl_data_exists =
			!sys_dlist_is_empty(&ctx->role_ctx.ft.peer_tx_data_dlists[i].high_priority_dlist) ||
			!sys_dlist_is_empty(&ctx->role_ctx.ft.peer_tx_data_dlists[i].reliable_data_dlist) ||
			!sys_dlist_is_empty(&ctx->role_ctx.ft.peer_tx_data_dlists[i].best_effort_dlist);

		pt_peer_ctx->dl_data_pending_for_pt = dl_data_exists;

		/* Grant a new schedule if DL data exists or UL is requested, AND no active schedule exists */
		if ((pt_peer_ctx->dl_data_pending_for_pt || pt_peer_ctx->ul_resource_request_pending_from_pt) &&
		    !current_pt_schedule->is_active) {
			LOG_INF("FT_SCHED: PT %d (0x%04X) needs resources (DL:%d, UL_Req:%d). Granting schedule.", i,
				pt_peer_ctx->short_rd_id, pt_peer_ctx->dl_data_pending_for_pt,
				pt_peer_ctx->ul_resource_request_pending_from_pt);

			dect_mac_resource_alloc_ie_fields_t new_alloc_fields;
			memset(&new_alloc_fields, 0, sizeof(new_alloc_fields));

			/* Grant a basic bidirectional schedule */
			new_alloc_fields.alloc_type_val = RES_ALLOC_TYPE_BIDIR;
			new_alloc_fields.id_present = true;
			new_alloc_fields.short_rd_id_val = pt_peer_ctx->short_rd_id;
			new_alloc_fields.repeat_val = RES_ALLOC_REPEAT_FRAMES;
			new_alloc_fields.sfn_present = true;
			new_alloc_fields.res1_is_9bit_subslot = (pt_peer_ctx->peer_mu > 2);
			new_alloc_fields.res2_is_9bit_subslot = (pt_peer_ctx->peer_mu > 2);
			
			/* Simple static slot assignment for now */
			new_alloc_fields.start_subslot_val_res1 = (i * 4 + 10) % MAX_SUBSLOTS_IN_FRAME_NOMINAL; // DL
			new_alloc_fields.length_val_res1 = 2 - 1; // 2 subslots
			new_alloc_fields.start_subslot_val_res2 = (i * 4 + 12) % MAX_SUBSLOTS_IN_FRAME_NOMINAL; // UL
			new_alloc_fields.length_val_res2 = 2 - 1; // 2 subslots

			new_alloc_fields.repetition_value = CONFIG_DECT_MAC_FT_DEFAULT_SCHEDULE_REPEAT_FRAMES;
			new_alloc_fields.validity_value = CONFIG_DECT_MAC_FT_DEFAULT_SCHEDULE_VALIDITY_FRAMES;
			new_alloc_fields.sfn_val = (ctx->role_ctx.ft.sfn + CONFIG_DECT_MAC_FT_SCHEDULE_START_SFN_OFFSET) & 0xFF;

			ft_update_pt_schedule_and_signal(i, &new_alloc_fields);
			pt_peer_ctx->ul_resource_request_pending_from_pt = false; // Request is being served

		} else if (!pt_peer_ctx->dl_data_pending_for_pt && !pt_peer_ctx->ul_resource_request_pending_from_pt &&
			   current_pt_schedule->is_active) {
			/* No data to send and no UL request, release the schedule */
			LOG_INF("FT_SCHED: No pending data/requests for PT %d (0x%04X). Releasing schedule.", i,
				pt_peer_ctx->short_rd_id);
			dect_mac_resource_alloc_ie_fields_t release_alloc_fields;
			memset(&release_alloc_fields, 0, sizeof(release_alloc_fields));
			release_alloc_fields.alloc_type_val = RES_ALLOC_TYPE_RELEASE_ALL;
			release_alloc_fields.id_present = true;
			release_alloc_fields.short_rd_id_val = pt_peer_ctx->short_rd_id;
			
			ft_update_pt_schedule_and_signal(i, &release_alloc_fields);
		}
	}
}


/**
 * @brief Updates a PT's schedule in context and signals it (e.g., via unicast MAC message).
 */
static void ft_update_pt_schedule_and_signal(int peer_slot_idx,
					     const dect_mac_resource_alloc_ie_fields_t *new_schedule_fields)
{
	dect_mac_context_t *ctx = get_mac_context();
	int ret;

	if (peer_slot_idx < 0 || peer_slot_idx >= MAX_PEERS_PER_FT ||
	    !ctx->role_ctx.ft.connected_pts[peer_slot_idx].is_valid) {
		LOG_ERR("FT_SCHED_SIG: Invalid peer_slot_idx %d", peer_slot_idx);
		return;
	}
	dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];
	dect_mac_schedule_t *pt_sched = &ctx->role_ctx.ft.peer_schedules[peer_slot_idx];

	/* Store/Update the schedule in FT's context */
	if (new_schedule_fields->alloc_type_val == RES_ALLOC_TYPE_RELEASE_ALL) {
		pt_sched->is_active = false;
		memset(pt_sched, 0, sizeof(dect_mac_schedule_t));
		LOG_INF("FT_SCHED_SIG: Schedule released for PT %d (0x%04X).", peer_slot_idx,
			pt_peer_ctx->short_rd_id);
	} else {
		pt_sched->is_active = true;
		pt_sched->alloc_type = new_schedule_fields->alloc_type_val;
		pt_sched->dl_start_subslot = new_schedule_fields->start_subslot_val_res1;
		pt_sched->dl_duration_subslots = new_schedule_fields->length_val_res1 + 1;
		pt_sched->dl_length_is_slots = new_schedule_fields->length_type_is_slots_res1;
		pt_sched->res1_is_9bit_subslot = new_schedule_fields->res1_is_9bit_subslot;

		if (new_schedule_fields->alloc_type_val == RES_ALLOC_TYPE_BIDIR) {
			pt_sched->ul_start_subslot = new_schedule_fields->start_subslot_val_res2;
			pt_sched->ul_duration_subslots = new_schedule_fields->length_val_res2 + 1;
			pt_sched->ul_length_is_slots =
				new_schedule_fields->length_type_is_slots_res2;
			pt_sched->res2_is_9bit_subslot = new_schedule_fields->res2_is_9bit_subslot;
		} else {
			if (new_schedule_fields->alloc_type_val == RES_ALLOC_TYPE_DOWNLINK) {
				pt_sched->ul_duration_subslots = 0;
			} else {
				pt_sched->dl_duration_subslots = 0;
			}
		}
		pt_sched->repeat_type = new_schedule_fields->repeat_val;
		pt_sched->repetition_value = new_schedule_fields->repetition_value;
		pt_sched->validity_value = new_schedule_fields->validity_value;
		pt_sched->channel = new_schedule_fields->channel_present
					    ? new_schedule_fields->channel_val
					    : ctx->role_ctx.ft.operating_carrier;
		pt_sched->schedule_init_modem_time = ctx->last_known_modem_time;
		pt_sched->sfn_of_initial_occurrence = new_schedule_fields->sfn_val;

		uint16_t primary_start_ss = (pt_sched->dl_duration_subslots > 0)
						  ? pt_sched->dl_start_subslot
						  : pt_sched->ul_start_subslot;

		pt_sched->next_occurrence_modem_time = calculate_target_modem_time(
			ctx, ctx->ft_sfn_zero_modem_time_anchor, 0, new_schedule_fields->sfn_val,
			primary_start_ss, ctx->own_phy_params.mu, ctx->own_phy_params.beta);
		update_next_occurrence(ctx, pt_sched, ctx->last_known_modem_time);
		
		LOG_INF("FT_SCHED_SIG: Schedule updated for PT %d (0x%04X). Next primary op @ %llu",
			peer_slot_idx, pt_peer_ctx->short_rd_id,
			pt_sched->next_occurrence_modem_time);
	}

	/* Build and send the unicast Resource Allocation PDU */
	uint8_t sdu_area_buf[64];
	int sdu_area_len = build_resource_alloc_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf),
							 new_schedule_fields);

	if (sdu_area_len < 0) {
		LOG_ERR("FT_SCHED_SIG: Failed to build ResAlloc IE: %d", sdu_area_len);
		return;
	}

	dect_mac_header_type_octet_t hdr_type_octet;
	hdr_type_octet.version = 0;
	hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST;
	hdr_type_octet.mac_security = MAC_SECURITY_NONE; /* TODO: Secure control messages */

	dect_mac_unicast_header_t common_hdr;
	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv =
		SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(pt_peer_ctx->long_rd_id);

	mac_sdu_t *mac_pdu_to_send = dect_mac_api_buffer_alloc(K_NO_WAIT);
	if (!mac_pdu_to_send) {
		LOG_ERR("FT_SCHED_SIG: Failed to alloc MAC PDU buffer.");
		return;
	}

	uint16_t assembled_pdu_len;
	ret = dect_mac_phy_ctrl_assemble_final_pdu(
		mac_pdu_to_send->data, CONFIG_DECT_MAC_PDU_MAX_SIZE, &hdr_type_octet, &common_hdr,
		sizeof(common_hdr), sdu_area_buf, (size_t)sdu_area_len, &assembled_pdu_len);
	if (ret != 0) {
		dect_mac_api_buffer_free(mac_pdu_to_send);
		return;
	}
	mac_pdu_to_send->len = assembled_pdu_len;

	ret = dect_mac_api_ft_send_to_pt(mac_pdu_to_send, MAC_FLOW_HIGH_PRIORITY,
					 pt_peer_ctx->short_rd_id);
	if (ret != 0) {
		LOG_ERR("FT_SCHED_SIG: Failed to queue ResAlloc PDU to PT 0x%04X: %d",
			pt_peer_ctx->short_rd_id, ret);
		/* The API should free the buffer on error */
	} else {
		LOG_INF("FT_SCHED_SIG: Queued unicast Resource Allocation IE to PT 0x%04X.",
			pt_peer_ctx->short_rd_id);
	}
}


// --- FT Static Helper Implementations ---
static int ft_get_peer_slot_idx(dect_mac_context_t* ctx, uint16_t pt_short_id) {
    for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
        if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
            ctx->role_ctx.ft.connected_pts[i].short_rd_id == pt_short_id) {
            return i;
        }
    }
    return -1;
}

static void ft_send_auth_challenge_action(int peer_slot_idx)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_peer_info_t *pt_peer = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];

	/* Generate and store FT nonce */
	// pt_peer->ft_nonce = sys_rand32_get();
	sys_rand_get(&pt_peer->ft_nonce, sizeof(uint32_t));

	LOG_INF("FT_AUTH_CHAL: Sending challenge to PT 0x%04X (PT_N:0x%08X, FT_N:0x%08X)",
		pt_peer->short_rd_id, pt_peer->pt_nonce, pt_peer->ft_nonce);

	uint8_t sdu_area_buf[64];
	int sdu_area_len = build_auth_challenge_ie_muxed(
		sdu_area_buf, sizeof(sdu_area_buf), pt_peer->pt_nonce, pt_peer->ft_nonce);

	if (sdu_area_len < 0) {
		LOG_ERR("FT_AUTH_CHAL: Failed to build Auth Challenge IE: %d", sdu_area_len);
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
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(pt_peer->long_rd_id);

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
		return;
	}

	    // Assign the final length to the SDU struct member
    pdu_sdu->len = pdu_len;

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.ft.operating_carrier, 
		pdu_sdu->data, pdu_sdu->len, pt_peer->short_rd_id, false,
		phy_op_handle, PENDING_OP_FT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);		
		
	dect_mac_api_buffer_free(pdu_sdu);

	if (ret == 0) {
		/* TODO: Transition peer sub-state to WAIT_AUTH_RESP and start a timer */
	}
}

static void ft_send_auth_success_action(int peer_slot_idx, const uint8_t *pt_mac)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_peer_info_t *pt_peer = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];
	uint8_t k_auth[16];
	uint8_t expected_pt_mac[DECT_MAC_AUTH_MAC_SIZE];
	int ret;

	ret = security_derive_auth_key(ctx->master_psk, k_auth);
	if (ret != 0) {
		LOG_ERR("FT_AUTH_SUCC: Failed to derive K_auth: %d", ret);
		return;
	}

	ret = security_generate_auth_mac(k_auth, pt_peer->pt_nonce, pt_peer->ft_nonce,
					 pt_peer->long_rd_id, ctx->own_long_rd_id,
					 expected_pt_mac);
	if (ret != 0) {
		LOG_ERR("FT_AUTH_SUCC: Failed to generate expected PT MAC: %d", ret);
		return;
	}

	if (constant_time_memcmp(pt_mac, expected_pt_mac, DECT_MAC_AUTH_MAC_SIZE) != 0) {
		LOG_ERR("FT_AUTH_SUCC: PT MAC verification failed! Disconnecting PT 0x%04X.",
			pt_peer->short_rd_id);
		pt_peer->is_valid = false;
		return;
	}

	LOG_INF("FT_AUTH_SUCC: PT MAC verified. Sending Auth Success to PT 0x%04X.",
		pt_peer->short_rd_id);

	ret = security_derive_session_keys(k_auth, ctx->own_long_rd_id, pt_peer->long_rd_id,
					   ctx->role_ctx.ft.peer_integrity_keys[peer_slot_idx],
					   ctx->role_ctx.ft.peer_cipher_keys[peer_slot_idx]);
	if (ret != 0) {
		LOG_ERR("FT_AUTH_SUCC: Session key derivation failed: %d", ret);
		return;
	}
	ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx] = true;
	pt_peer->is_secure = true;
	pt_peer->hpc = 1;

	uint8_t ft_mac[DECT_MAC_AUTH_MAC_SIZE];

	ret = security_generate_auth_mac(k_auth, pt_peer->ft_nonce, pt_peer->pt_nonce,
					 ctx->own_long_rd_id, pt_peer->long_rd_id, ft_mac);
	if (ret != 0) {
		LOG_ERR("FT_AUTH_SUCC: Failed to generate FT MAC: %d", ret);
		return;
	}

	uint8_t sdu_area_buf[32];
	int sdu_area_len =
		build_auth_success_ie_muxed(sdu_area_buf, sizeof(sdu_area_buf), ft_mac);
	if (sdu_area_len < 0) {
		LOG_ERR("FT_AUTH_SUCC: Failed to build Auth Success IE: %d", sdu_area_len);
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
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(pt_peer->long_rd_id);

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

    // Assign the final length to the SDU struct member
    pdu_sdu->len = pdu_len;

	// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.ft.operating_carrier, pdu_sdu->data, pdu_sdu->len, pt_peer->short_rd_id, false,
		phy_op_handle, PENDING_OP_FT_AUTH_MSG_TX, true, 0, ctx->own_phy_params.mu, NULL);
		
	dect_mac_api_buffer_free(pdu_sdu);

	if (ret == 0) {
		LOG_INF("FT_AUTH_SUCC: Auth Success PDU scheduled for PT 0x%04X.",
			pt_peer->short_rd_id);
	}
}

void dect_mac_sm_ft_handle_auth_pdu(uint16_t pt_short_id, uint32_t pt_long_id, const uint8_t *pdu_data, size_t pdu_len)
{
    ARG_UNUSED(pdu_data);
    ARG_UNUSED(pdu_len);
    dect_mac_context_t* ctx = get_mac_context();
    int peer_idx = ft_get_peer_slot_idx(ctx, pt_short_id);

    if (peer_idx == -1 || !ctx->role_ctx.ft.connected_pts[peer_idx].is_valid ||
        ctx->role_ctx.ft.connected_pts[peer_idx].long_rd_id != pt_long_id) {
        LOG_WRN("FT_AUTH_HANDLE_PDU: Received Auth PDU from unknown/invalid PT S:0x%04X L:0x%08X. Ignoring.",
                pt_short_id, pt_long_id);
        return;
    }

    // A real auth protocol might involve the FT being in a specific auth sub-state for this peer.
    // if (ctx->role_ctx.ft.connected_pts[peer_idx].auth_sub_state != EXPECTING_PT_AUTH_RESPONSE) {
    //     LOG_WRN("FT_AUTH_HANDLE_PDU: Received Auth PDU from PT S:0x%04X in unexpected auth sub-state. Ignoring.");
    //     return;
    // }

    LOG_INF("FT_AUTH_HANDLE_PDU: Received (stubbed) Auth PDU from PT S:0x%04X L:0x%08X (slot %d, len %zu). No action for current PSK model.",
            pt_short_id, pt_long_id, peer_idx, pdu_len);

    // For a real multi-step protocol:
    // 1. Parse pdu_data (e.g., PT's response to FT's challenge).
    // 2. If valid and final step for FT:
    //    ctx->role_ctx.ft.connected_pts[peer_idx].is_secure = true; // Mark link as secure
    //    LOG_INF("FT_AUTH_HANDLE_PDU: Authentication with PT 0x%04X complete. Link SECURE.");
    //    // Optionally send a final "Secure Link Confirm" PDU to PT.
    // 3. If valid and more steps for FT (e.g., FT needs to send another message):
    //    // Build and send next auth PDU to PT.
    // 4. If invalid:
    //    // Send Auth Fail PDU and/or release PT.
    //
    // The FT's PSK-based key derivation and setting of `is_secure` happens in `ft_process_association_request_pdu`.
}


// Brief Overview: This is the complete populate_cb_fields_from_ctx function.
// It accurately populates all fields for the dect_mac_cluster_beacon_ie_fields_t structure,
// including deriving ETSI codes for Network and Cluster Beacon periods from Kconfig
// millisecond values, and using Kconfig for Count To Trigger, Rel Quality, and Min Quality codes.

static void populate_cb_fields_from_ctx(dect_mac_context_t *ctx, dect_mac_cluster_beacon_ie_fields_t *cb_fields) {
    if (!ctx || !cb_fields) {
        LOG_ERR("POP_CB_FIELDS: NULL context or cb_fields pointer.");
        return;
    }
    memset(cb_fields, 0, sizeof(dect_mac_cluster_beacon_ie_fields_t));

    cb_fields->sfn = ctx->role_ctx.ft.sfn;
    cb_fields->tx_power_present = true; 
    cb_fields->clusters_max_tx_power_code = ctx->config.default_tx_power_code; 
    cb_fields->power_constraints_active = false; 

    // Frame Offset field handling
    cb_fields->frame_offset_present = IS_ENABLED(CONFIG_DECT_MAC_FT_USE_FRAME_OFFSET); // Example Kconfig
    if (cb_fields->frame_offset_present) {
        // ETSI TS 103 636-4, Table 6.4.2.3-1: Frame Offset field is 8 bits if mu <= 4, 16 bits if mu > 4.
        // mu_code: 0(mu=1), 1(mu=2), 2(mu=4) -> 8-bit FO.
        // mu_code: 3(mu=8) or higher -> 16-bit FO.
        // So, 16-bit if mu_code > 2 (i.e., actual mu > 4).
        if (ctx->own_phy_params.is_valid) {
            cb_fields->frame_offset_is_16bit = (ctx->own_phy_params.mu > 2);
             // cb_fields->frame_offset_value = ctx->role_ctx.ft.current_frame_offset_subslots; // Requires this field in ft_context_t
             // For now, if present, set a placeholder value.
             cb_fields->frame_offset_value = 0; // Placeholder
             LOG_DBG("POP_CB_FIELDS: Frame Offset present, %d-bit, val %u (mu_code %u)",
                     cb_fields->frame_offset_is_16bit ? 16:8,
                     cb_fields->frame_offset_value,
                     ctx->own_phy_params.mu);
        } else {
            LOG_WRN("POP_CB_FIELDS: Frame Offset present requested, but own_phy_params.mu not valid. Assuming 8-bit FO.");
            cb_fields->frame_offset_is_16bit = false;
            cb_fields->frame_offset_value = 0;
        }
    } else {
        cb_fields->frame_offset_is_16bit = false; // Not relevant if not present
        cb_fields->frame_offset_value = 0;
    }

    // Next Cluster Channel / Time To Next (Advanced Features - currently disabled)
    cb_fields->next_channel_present = false; // Set to true if FT wants to signal this
    // if (cb_fields->next_channel_present) {
    //    cb_fields->next_cluster_channel_val = ctx->role_ctx.ft.next_beacon_carrier_val_config; // Kconfig or dynamic
    // }
    cb_fields->time_to_next_present = false; // Set to true if FT wants to signal this
    // if (cb_fields->time_to_next_present) {
    //    cb_fields->time_to_next_us = ctx->role_ctx.ft.time_to_next_beacon_us_config; // Kconfig or dynamic
    // }

    // Populate beacon period codes from Kconfig (logic from previous update)
    uint32_t net_period_ms = ctx->config.ft_network_beacon_period_ms;
    if (net_period_ms <= 50) cb_fields->network_beacon_period_code = 0;
    else if (net_period_ms <= 100) cb_fields->network_beacon_period_code = 1;
    else if (net_period_ms <= 500) cb_fields->network_beacon_period_code = 2;
    else if (net_period_ms <= 1000) cb_fields->network_beacon_period_code = 3;
    else if (net_period_ms <= 1500) cb_fields->network_beacon_period_code = 4;
    else if (net_period_ms <= 2000) cb_fields->network_beacon_period_code = 5;
    else if (net_period_ms <= 4000) cb_fields->network_beacon_period_code = 6;
    else { cb_fields->network_beacon_period_code = 3; /* Default */ }

    uint32_t clus_period_ms = ctx->config.ft_cluster_beacon_period_ms;
    if (clus_period_ms <= 10) cb_fields->cluster_beacon_period_code = 0;
    else if (clus_period_ms <= 50) cb_fields->cluster_beacon_period_code = 1;
    else if (clus_period_ms <= 100) cb_fields->cluster_beacon_period_code = 2;
    else if (clus_period_ms <= 500) cb_fields->cluster_beacon_period_code = 3;
    else if (clus_period_ms <= 1000) cb_fields->cluster_beacon_period_code = 4;
    else if (clus_period_ms <= 1500) cb_fields->cluster_beacon_period_code = 5;
    else if (clus_period_ms <= 2000) cb_fields->cluster_beacon_period_code = 6;
    else if (clus_period_ms <= 4000) cb_fields->cluster_beacon_period_code = 7;
    else if (clus_period_ms <= 8000) cb_fields->cluster_beacon_period_code = 8;
    else if (clus_period_ms <= 16000) cb_fields->cluster_beacon_period_code = 9;
    else if (clus_period_ms <= 32000) cb_fields->cluster_beacon_period_code = 10;
    else { cb_fields->cluster_beacon_period_code = 2; /* Default */ }

    cb_fields->count_to_trigger_code = CONFIG_DECT_MAC_FT_COUNT_TO_TRIGGER_CODE & 0x07;
    cb_fields->rel_quality_code = CONFIG_DECT_MAC_FT_REL_QUALITY_CODE & 0x07;
    cb_fields->min_quality_code = CONFIG_DECT_MAC_FT_MIN_QUALITY_CODE & 0x03;

    // Current Cluster Channel field logic:
    // The serializer (serialize_cluster_beacon_ie_payload) should handle the actual inclusion
    // of the "Current Cluster Channel" field based on these flags and values.
    // This function just prepares the values for the serializer.
    // For now, next_channel_present is false, so current channel field is not applicable.
    // if (cb_fields->next_channel_present &&
    //     (cb_fields->next_cluster_channel_val != ctx->role_ctx.ft.operating_carrier)) {
    //     // cb_fields->current_channel_val = ctx->role_ctx.ft.operating_carrier; // Requires this field in struct
    //     // cb_fields->current_channel_field_present_flag_for_serializer = true; // Helper for serializer
    // }
}


static void ft_select_operating_carrier_and_start_beaconing(
	const struct nrf_modem_dect_phy_rssi_event *optional_last_rssi_event_data)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->state != MAC_STATE_FT_SCANNING) {
		LOG_WRN("FT_DCS_SEL: Not in SCANNING state (%s), ignoring request to select carrier.",
			dect_mac_state_to_str(ctx->state));
		if (ctx->pending_op_type == PENDING_OP_NONE) {
			k_timer_start(&ctx->role_ctx.ft.beacon_timer, K_SECONDS(1), K_NO_WAIT);
		}
		return;
	}
	ARG_UNUSED(optional_last_rssi_event_data);

	int16_t best_rssi_found = INT16_MAX;
	int selected_idx = -1;

	LOG_INF("FT_DCS_SEL: Evaluating %u scanned channels. Busy <= %d%%, Noisy < %ddBm.",
		ctx->role_ctx.ft.dcs_num_valid_candidate_channels,
		CONFIG_DECT_MAC_DCS_ACCEPTABLE_BUSY_PERCENT,
		CONFIG_DECT_MAC_DCS_NOISY_THRESHOLD_DBM);

	/* First pass: Find the quietest channel among the "good" ones */
	for (int i = 0; i < ctx->role_ctx.ft.dcs_num_valid_candidate_channels; i++) {
		if (ctx->role_ctx.ft.dcs_candidate_rssi_avg[i] ==
			    NRF_MODEM_DECT_PHY_RSSI_NOT_MEASURED ||
		    ctx->role_ctx.ft.dcs_candidate_busy_percent[i] > 100) {
			continue;
		}

		LOG_INF("  Candidate %d: C%u, AvgRSSI: %.1f dBm, Busy: %u%%", i,
			ctx->role_ctx.ft.dcs_candidate_channels[i],
			(double)ctx->role_ctx.ft.dcs_candidate_rssi_avg[i] / 2.0f,
			ctx->role_ctx.ft.dcs_candidate_busy_percent[i]);

		if (ctx->role_ctx.ft.dcs_candidate_busy_percent[i] <=
			    CONFIG_DECT_MAC_DCS_ACCEPTABLE_BUSY_PERCENT &&
		    ctx->role_ctx.ft.dcs_candidate_rssi_avg[i] <
			    (CONFIG_DECT_MAC_DCS_NOISY_THRESHOLD_DBM * 2)) {
			if (ctx->role_ctx.ft.dcs_candidate_rssi_avg[i] < best_rssi_found) {
				best_rssi_found = ctx->role_ctx.ft.dcs_candidate_rssi_avg[i];
				selected_idx = i;
			}
		}
	}

	/* Second pass (fallback): If no "good" channels, find the least busy one */
	if (selected_idx == -1) {
		LOG_WRN("FT_DCS_SEL: No ideal channels found. Falling back to least busy channel.");
		uint8_t least_busy_percent = 101;

		for (int i = 0; i < ctx->role_ctx.ft.dcs_num_valid_candidate_channels; i++) {
			if (ctx->role_ctx.ft.dcs_candidate_busy_percent[i] <= 100) {
				if (ctx->role_ctx.ft.dcs_candidate_busy_percent[i] <
				    least_busy_percent) {
					least_busy_percent =
						ctx->role_ctx.ft.dcs_candidate_busy_percent[i];
					selected_idx = i;
				}
			}
		}
	}

	if (selected_idx != -1) {
		ctx->role_ctx.ft.operating_carrier =
			ctx->role_ctx.ft.dcs_candidate_channels[selected_idx];
		LOG_INF("FT_DCS_SEL: Best carrier selected: C%u (idx %d) with Avg RSSI %.1f dBm, Busy %u%%.",
			ctx->role_ctx.ft.operating_carrier, selected_idx,
			(double)ctx->role_ctx.ft.dcs_candidate_rssi_avg[selected_idx] / 2.0f,
			ctx->role_ctx.ft.dcs_candidate_busy_percent[selected_idx]);
	} else {
		LOG_ERR("FT_DCS_SEL: No scannable channels found any usable signal. Defaulting to C%u.",
			CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CARRIER_KHZ);
		ctx->role_ctx.ft.operating_carrier =
			CONFIG_DECT_MAC_FT_DEFAULT_OPERATING_CARRIER_KHZ;
		if (ctx->role_ctx.ft.operating_carrier == 0) {
			ctx->role_ctx.ft.operating_carrier = 1881792;
		}
	}

	ctx->role_ctx.ft.advertised_rach_params.rach_operating_channel =
		ctx->role_ctx.ft.operating_carrier;
	ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_abs_freq_num =
		ctx->role_ctx.ft.operating_carrier;
	ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields.channel_field_present =
		true;

	ft_start_beaconing_actions();
}

static void ft_start_beaconing_actions(void) {
    dect_mac_context_t* ctx = get_mac_context();
    dect_mac_change_state(MAC_STATE_FT_BEACONING);
    LOG_INF("FT SM: Entered BEACONING state on carrier %u.", ctx->role_ctx.ft.operating_carrier);
    
    // /* Create a default group schedule to advertise */
    // dect_mac_resource_alloc_ie_fields_t *group_sched = &ctx->role_ctx.ft.group_schedule_fields;
    // memset(group_sched, 0, sizeof(*group_sched));
    // group_sched->alloc_type_val = RES_ALLOC_TYPE_UPLINK;
    // group_sched->id_present = true; /* ID is the Group ID */
    // group_sched->short_rd_id_val = 1; /* Group ID 1 */
    // group_sched->repeat_val = RES_ALLOC_REPEAT_FRAMES_GROUP;
    // group_sched->start_subslot_val_res1 = 20;
    // group_sched->length_val_res1 = 4 - 1; /* 4 subslots */
    // group_sched->repetition_value = 10; /* Repeats every 10 frames */
    // group_sched->validity_value = 0xFF; /* Permanent until released */

    /* Create a default group schedule to advertise */
    dect_mac_schedule_t *group_sched = &ctx->role_ctx.ft.group_schedule;
    memset(group_sched, 0, sizeof(*group_sched));
    group_sched->is_active = true; // Mark the template as active
    group_sched->alloc_type = RES_ALLOC_TYPE_UPLINK;
    group_sched->ul_start_subslot = 20;
    group_sched->ul_duration_subslots = 4;
    group_sched->repeat_type = RES_ALLOC_REPEAT_FRAMES_GROUP;
    group_sched->repetition_value = 10; /* Repeats every 10 frames */
    group_sched->validity_value = 0xFF; /* Permanent until released */

    /* Initialize own routing info. A standalone FT acts as its own sink. */
    ctx->sink_long_rd_id = ctx->own_long_rd_id;
    ctx->own_route_cost = 0;

    /* Build the configuration data that this FT will serve */
	dect_cdd_ft_build_own_config();

    ctx->role_ctx.ft.sfn = 0; // FT starts its SFN count from 0
    uint32_t first_beacon_tx_attempt_delay_ms = 50; // Small delay before trying to send the first beacon

    // Initialize ft_sfn_zero_modem_time_anchor to 0.
    // It will be set accurately when the first beacon (SFN 0) is scheduled for transmission
    // in ft_send_beacon_action, based on its target PHY operation start time.
    ctx->ft_sfn_zero_modem_time_anchor = 0;
    // current_sfn_at_anchor_update for FT will effectively be 0 when anchor is first set.
    // This field is primarily for the PT to know the SFN context of the anchor time it receives.
    // For the FT, its anchor is *defined* at SFN 0.
    ctx->current_sfn_at_anchor_update = 0; 

    LOG_DBG("FT_START_BEACONING: SFN anchor will be established with first beacon TX. Initial SFN set to 0.");

    // Start the periodic beacon timer.
    // The first timer expiry will trigger ft_send_beacon_action, which will then establish the SFN anchor.
    uint32_t beacon_period_ms = ctx->config.ft_cluster_beacon_period_ms;
    if (beacon_period_ms == 0) { // Fallback if Kconfig is 0
        beacon_period_ms = 100;
        LOG_WRN("FT_START_BEACONING: ft_cluster_beacon_period_ms is 0, using fallback %ums for timer.", beacon_period_ms);
    }
    // The first timer event will trigger the first beacon send attempt.
    k_timer_start(&ctx->role_ctx.ft.beacon_timer, 
                  K_MSEC(first_beacon_tx_attempt_delay_ms), // Initial delay for the first beacon attempt
                  K_MSEC(beacon_period_ms));               // Subsequent period
}


static void ft_send_beacon_action(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->state != MAC_STATE_FT_BEACONING) {
		LOG_WRN("FT SM: Beacon TX attempt, but not in BEACONING state (%s). Aborting.",
			dect_mac_state_to_str(ctx->state));
		return;
	}

	if (ctx->pending_op_type != PENDING_OP_NONE) {
		LOG_WRN("FT SM: Beacon TX time, but op %s pending. Delaying beacon.",
			dect_pending_op_to_str(ctx->pending_op_type));
		uint32_t short_delay_ms = ctx->config.ft_cluster_beacon_period_ms / 10;

		if (short_delay_ms < 20) {
			short_delay_ms = 20;
		}
		if (short_delay_ms > 100) {
			short_delay_ms = 100;
		}
		k_timer_start(&ctx->role_ctx.ft.beacon_timer, K_MSEC(short_delay_ms),
			      K_MSEC(ctx->config.ft_cluster_beacon_period_ms));
		return;
	}

	uint8_t mac_sdu_area_buf[128];
	int sdu_area_len;

	dect_mac_cluster_beacon_ie_fields_t cb_fields;

	populate_cb_fields_from_ctx(ctx, &cb_fields);

	dect_mac_rach_info_ie_fields_t *rach_adv_fields =
		&ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields;

	uint8_t ft_operational_mu_code = 0;

	if (ctx->own_phy_params.is_valid && ctx->own_phy_params.mu <= 7) {
		ft_operational_mu_code = ctx->own_phy_params.mu;
	} else {
		LOG_WRN("FT_BEACON_ACT: FT's own operational mu_code not valid in context. Defaulting to mu_code=0 for RACH IE.");
	}
	rach_adv_fields->mu_value_for_ft_beacon = ft_operational_mu_code;

	if (rach_adv_fields->channel_abs_freq_num != ctx->role_ctx.ft.operating_carrier ||
	    !rach_adv_fields->channel_field_present) {
		rach_adv_fields->channel_abs_freq_num = ctx->role_ctx.ft.operating_carrier;
		rach_adv_fields->channel_field_present = true;
	}

	sdu_area_len = build_beacon_sdu_area_content(mac_sdu_area_buf, sizeof(mac_sdu_area_buf),
						   &cb_fields, rach_adv_fields);
	if (sdu_area_len < 0) {
		LOG_ERR("FT SM: Failed to build beacon SDU area content: %d. Skipping this beacon.",
			sdu_area_len);
		return;
	}

	if (ctx->own_route_cost != 255) {
		dect_mac_route_info_ie_t route_info_fields = {
			.sink_address_be = sys_cpu_to_be32(ctx->sink_long_rd_id),
			.route_cost = ctx->own_route_cost,
			.app_sequence_number = dect_cdd_get_app_seq_num()
		};
		int route_info_len =
			build_route_info_ie_muxed(mac_sdu_area_buf + sdu_area_len,
						  sizeof(mac_sdu_area_buf) - sdu_area_len,
						  &route_info_fields);
		if (route_info_len > 0) {
			sdu_area_len += route_info_len;
			LOG_DBG("BEACON_SDU_AREA: Added Route Info IE (len %d).", route_info_len);
		}
	}

	if (IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE)) {
		int join_ie_len = build_joining_information_ie_muxed(
			mac_sdu_area_buf + sdu_area_len, sizeof(mac_sdu_area_buf) - sdu_area_len,
			NULL, 0);
		if (join_ie_len > 0) {
			sdu_area_len += join_ie_len;
			LOG_DBG("FT_BEACON: Added Joining Information IE (len %d).", join_ie_len);
		}
	}

	/* Add Group Assignment IE if enabled and pending */
	if (IS_ENABLED(CONFIG_DECT_MAC_FT_SUPPORTS_GROUP_ASSIGNMENT) && ctx->role_ctx.ft.group_assignment_pending) {
		int group_ie_len = build_group_assignment_ie_muxed(
			mac_sdu_area_buf + sdu_area_len,
			sizeof(mac_sdu_area_buf) - sdu_area_len,
            &ctx->role_ctx.ft.group_assignment_fields); // Pass a pointer to the correct struct

		if (group_ie_len > 0) {
			sdu_area_len += group_ie_len;
			LOG_DBG("FT_BEACON: Added Group Assignment IE (len %d).", group_ie_len);
		}
		ctx->role_ctx.ft.group_assignment_pending = false; /* Send only once per trigger */
	}

	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
		    ctx->role_ctx.ft.connected_pts[i].paging_pending) {
			LOG_INF("FT_BEACON: Paging PT 0x%04X in this beacon.",
				ctx->role_ctx.ft.connected_pts[i].short_rd_id);

			int page_ie_len = build_broadcast_indication_ie_muxed(
				mac_sdu_area_buf + sdu_area_len,
				sizeof(mac_sdu_area_buf) - sdu_area_len,
				ctx->role_ctx.ft.connected_pts[i].short_rd_id);

			if (page_ie_len > 0) {
				sdu_area_len += page_ie_len;
				ctx->role_ctx.ft.connected_pts[i].paging_pending = false;
			} else {
				LOG_ERR("FT_BEACON: Failed to add Broadcast Indication IE for paging.");
			}
			break;
		}
	}

	dect_mac_header_type_octet_t hdr_type_octet;

	hdr_type_octet.version = 0;
	hdr_type_octet.mac_security = MAC_SECURITY_NONE;
	hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_BEACON;

	dect_mac_beacon_header_t common_beacon_hdr;

	common_beacon_hdr.network_id_ms24[0] = (uint8_t)((ctx->network_id_32bit >> 24) & 0xFF);
	common_beacon_hdr.network_id_ms24[1] = (uint8_t)((ctx->network_id_32bit >> 16) & 0xFF);
	common_beacon_hdr.network_id_ms24[2] = (uint8_t)((ctx->network_id_32bit >> 8) & 0xFF);
	common_beacon_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);

	uint8_t *full_mac_pdu_for_phy_slab = NULL;
	int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void **)&full_mac_pdu_for_phy_slab,
				   K_MSEC(10));

	if (ret != 0 || full_mac_pdu_for_phy_slab == NULL) {
		LOG_ERR("FT SM: Failed to alloc PDU buf for beacon TX: %d. Skipping this beacon.",
			ret);
		return;
	}
	uint8_t *const full_mac_pdu_for_phy = full_mac_pdu_for_phy_slab;

	uint16_t cleartext_pdu_len;

	ret = dect_mac_phy_ctrl_assemble_final_pdu(
		full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE, &hdr_type_octet,
		&common_beacon_hdr, sizeof(common_beacon_hdr), mac_sdu_area_buf, (size_t)sdu_area_len,
		&cleartext_pdu_len);

	if (ret != 0) {
		LOG_ERR("FT SM: Failed to assemble final beacon PDU: %d", ret);
		k_mem_slab_free(&g_mac_sdu_slab, (void **)&full_mac_pdu_for_phy_slab);
		return;
	}

	// uint32_t phy_op_handle = sys_rand32_get();
uint32_t phy_op_handle;
sys_rand_get(&phy_op_handle, sizeof(uint32_t));

	ctx->role_ctx.ft.sfn_for_last_beacon_tx = ctx->role_ctx.ft.sfn;
	uint64_t beacon_target_start_time;
	uint32_t frame_duration_ticks_val = 0;

	if (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ > 0) {
		frame_duration_ticks_val =
			(uint32_t)FRAME_DURATION_MS_NOMINAL *
			(NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
	}
	if (frame_duration_ticks_val == 0) {
		LOG_ERR("FT_BEACON_ACT: Frame duration ticks is zero! Cannot schedule beacon precisely. Sending immediate-ish.");
		beacon_target_start_time = 0;
	} else if (ctx->ft_sfn_zero_modem_time_anchor == 0) {
		uint32_t initial_tx_delay_ticks =
			modem_us_to_ticks(5000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
		beacon_target_start_time =
			(ctx->last_known_modem_time > 0
				 ? ctx->last_known_modem_time
				 : modem_us_to_ticks(1000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ)) +
			initial_tx_delay_ticks;
	} else {
		beacon_target_start_time = calculate_target_modem_time(
			ctx, ctx->ft_sfn_zero_modem_time_anchor, 0, ctx->role_ctx.ft.sfn, 0,
			ctx->own_phy_params.mu, ctx->own_phy_params.beta);
	}

	if (beacon_target_start_time != 0) {
		uint32_t min_prep_time_ticks = modem_us_to_ticks(
			ctx->phy_latency.idle_to_active_tx_us +
				ctx->phy_latency.scheduled_operation_startup_us,
			NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
		if (ctx->last_known_modem_time > 0 &&
		    beacon_target_start_time < (ctx->last_known_modem_time + min_prep_time_ticks)) {
			LOG_WRN("FT_BEACON_ACT: Target start %llu for SFN %u is too soon (current %llu, prep %u). Adjusting.",
				beacon_target_start_time, ctx->role_ctx.ft.sfn,
				ctx->last_known_modem_time, min_prep_time_ticks);

			uint64_t earliest_possible_start_from_now =
				ctx->last_known_modem_time + min_prep_time_ticks;
			uint64_t frames_since_anchor = 0;

			if (earliest_possible_start_from_now >= ctx->ft_sfn_zero_modem_time_anchor) {
				frames_since_anchor =
					(earliest_possible_start_from_now -
					 ctx->ft_sfn_zero_modem_time_anchor +
					 frame_duration_ticks_val - 1) /
					frame_duration_ticks_val;
			}

			ctx->role_ctx.ft.sfn = (uint8_t)((ctx->current_sfn_at_anchor_update +
							frames_since_anchor) %
						       256);
			beacon_target_start_time = ctx->ft_sfn_zero_modem_time_anchor +
						 (frames_since_anchor * frame_duration_ticks_val);
			ctx->role_ctx.ft.sfn_for_last_beacon_tx = ctx->role_ctx.ft.sfn;
			LOG_INF("FT_BEACON_ACT: Adjusted target SFN to %u, start_time %llu.",
				ctx->role_ctx.ft.sfn, beacon_target_start_time);
		}
	}

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.ft.operating_carrier, full_mac_pdu_for_phy, cleartext_pdu_len, 0xFFFF,
		true, phy_op_handle, PENDING_OP_FT_BEACON, false, beacon_target_start_time,
		ctx->own_phy_params.mu, NULL);

	k_mem_slab_free(&g_mac_sdu_slab, (void **)&full_mac_pdu_for_phy_slab);

	if (ret != 0) {
		dect_mac_enter_error_state("Failed to schedule beacon TX");
	} else {
		LOG_INF("FT SM: Beacon SFN %u TX scheduled on C%u (Hdl %u), TargetStart %llu",
			ctx->role_ctx.ft.sfn, ctx->role_ctx.ft.operating_carrier, phy_op_handle,
			beacon_target_start_time);

		/* Only set anchor and increment SFN on successful scheduling */
		if (ctx->ft_sfn_zero_modem_time_anchor == 0) {
			ctx->ft_sfn_zero_modem_time_anchor = beacon_target_start_time;
			ctx->current_sfn_at_anchor_update = ctx->role_ctx.ft.sfn;
			LOG_INF("FT_BEACON_ACT: SFN0 anchor established at %llu for SFN %u.",
				ctx->ft_sfn_zero_modem_time_anchor, ctx->current_sfn_at_anchor_update);
		}
		ctx->role_ctx.ft.sfn = (ctx->role_ctx.ft.sfn + 1) & 0xFF;
	}
}
/**
 * @brief Schedules a PHY RX operation for the FT to listen on its advertised RACH resources.
 */
static void ft_schedule_rach_listen_action(void) {
    dect_mac_context_t* ctx = get_mac_context();

    if (ctx->state != MAC_STATE_FT_BEACONING && ctx->state != MAC_STATE_ASSOCIATED) {
        LOG_DBG("FT_RACH_LSN: Not in a state to listen for RACH (%s).", dect_mac_state_to_str(ctx->state));
        return;
    }
    if (ctx->pending_op_type != PENDING_OP_NONE) {
        LOG_DBG("FT_RACH_LSN: PHY op %s pending. Deferring RACH listen.", dect_pending_op_to_str(ctx->pending_op_type));
        return;
    }

    const dect_mac_rach_info_ie_fields_t *rach_adv_fields = &ctx->role_ctx.ft.advertised_rach_params.advertised_beacon_ie_fields;

    // Point 1: Uses Advertised Parameters - YES (rach_adv_fields)

    uint16_t rach_carrier = rach_adv_fields->channel_field_present ?
                            rach_adv_fields->channel_abs_freq_num :
                            ctx->role_ctx.ft.operating_carrier;
    if (rach_carrier == 0 || rach_carrier == 0xFFFF) {
        LOG_ERR("FT_RACH_LSN: Invalid RACH carrier 0x%04X configured. Cannot listen.", rach_carrier);
        return;
    }

    // Point 4 & 5 (partially): Duration and mu-awareness for timing
    uint8_t ft_mu_for_rach = rach_adv_fields->mu_value_for_ft_beacon;
    if (ft_mu_for_rach > 7) { 
        LOG_WRN("FT_RACH_LSN: Invalid mu_code (%u) in FT's advertised RACH IE. Defaulting to mu_code=0 (mu=1).", ft_mu_for_rach);
        ft_mu_for_rach = 0; 
    }
    
    uint32_t ft_subslot_duration_ticks = get_subslot_duration_ticks_for_mu(ft_mu_for_rach);
    if (ft_subslot_duration_ticks == 0) {
        LOG_ERR("FT_RACH_LSN: Calculated FT subslot duration is 0 for mu_code %u. Cannot proceed.", ft_mu_for_rach);
        return;
    }

    uint32_t rach_resource_len_actual_units = rach_adv_fields->num_subslots_or_slots;
    uint32_t rach_resource_len_subslots = rach_resource_len_actual_units;

    if (rach_adv_fields->length_type_is_slots) {
        uint8_t subslots_per_etsi_slot_for_ft_mu = get_subslots_per_etsi_slot_for_mu(ft_mu_for_rach);
        rach_resource_len_subslots *= subslots_per_etsi_slot_for_ft_mu;
    }
    if (rach_resource_len_subslots == 0) {
        LOG_ERR("FT_RACH_LSN: Advertised RACH resource length is 0 subslots. Cannot listen.");
        return;
    }

    // Point 2: SFN Alignment (Target SFN Calculation)
    uint8_t target_sfn_for_rach;
    uint8_t sfn_of_initial_rach_advertisement; 

    if (rach_adv_fields->sfn_validity_present) {
        sfn_of_initial_rach_advertisement = rach_adv_fields->sfn_value;
        target_sfn_for_rach = sfn_of_initial_rach_advertisement; 

        uint8_t repetition_interval_frames = 1U << rach_adv_fields->repetition_code; 
        if (repetition_interval_frames == 0 || repetition_interval_frames > 8) { // Code 0-3 maps to 1,2,4,8
            LOG_WRN("FT_RACH_LSN: Invalid repetition_code %u, defaulting interval to 1 frame.", rach_adv_fields->repetition_code);
            repetition_interval_frames = 1; 
        }
        
        uint8_t sfn_loop_guard = 0; // To prevent potential infinite loop with bad params
        while (sfn_loop_guard < 256) { // Max SFN cycle
            int16_t sfn_diff_to_current = (int16_t)target_sfn_for_rach - (int16_t)ctx->role_ctx.ft.sfn;
            if (sfn_diff_to_current > 128) sfn_diff_to_current -=256; 
            else if (sfn_diff_to_current < -128) sfn_diff_to_current += 256;

            if (sfn_diff_to_current >= 0) { 
                int16_t frames_from_initial_adv = (int16_t)target_sfn_for_rach - (int16_t)sfn_of_initial_rach_advertisement;
                if (frames_from_initial_adv < 0) frames_from_initial_adv += 256;

                if ((uint8_t)frames_from_initial_adv % repetition_interval_frames == 0) {
                    break; 
                }
            }
            target_sfn_for_rach = (target_sfn_for_rach + 1) & 0xFF; // Check next SFN
            sfn_loop_guard++;
        }
        if (sfn_loop_guard >= 256) {
             LOG_ERR("FT_RACH_LSN: Could not find aligned SFN for SFN-based RACH. Check repetition/validity. Skipping.");
             return;
        }


        int16_t frames_from_initial_validity_sfn = (int16_t)target_sfn_for_rach - (int16_t)sfn_of_initial_rach_advertisement;
        if (frames_from_initial_validity_sfn < 0) frames_from_initial_validity_sfn += 256; 

        if (rach_adv_fields->validity_frames != 0xFF && (uint8_t)frames_from_initial_validity_sfn >= rach_adv_fields->validity_frames) {
            LOG_WRN("FT_RACH_LSN: Advertised RACH validity expired for target SFN %u. Not listening.", target_sfn_for_rach);
            return;
        }
    } else { 
        target_sfn_for_rach = ctx->role_ctx.ft.sfn; // Listen in current SFN cycle
        sfn_of_initial_rach_advertisement = ctx->role_ctx.ft.sfn_for_last_beacon_tx; 
        // TODO: If repetition_code is non-zero and sfn_validity_present is false, how is the repetition anchored?
        // Assuming for now it means "every X frames/subslots starting from this beacon's frame context".
        // The "too soon" loop will handle advancing if current SFN is already past the immediate opportunity.
        LOG_DBG("FT_RACH_LSN: SFN not in RACH IE, targeting current SFN %u, repetition relative to beacon SFN %u",
                target_sfn_for_rach, sfn_of_initial_rach_advertisement);
    }

    // Point 3 & 5: Subslot Alignment & Modem Time Calculation
    uint64_t rach_listen_start_time = calculate_target_modem_time(ctx,
                                                                  ctx->ft_sfn_zero_modem_time_anchor,
                                                                  ctx->current_sfn_at_anchor_update, 
                                                                  target_sfn_for_rach,
                                                                  rach_adv_fields->start_subslot_index,
                                                                  ft_mu_for_rach, /* FT's own mu for its RACH */
                                                                  ctx->own_phy_params.beta /* FT's own beta */);
    
    // Point 4: Duration
    uint32_t listen_duration_subslots = rach_resource_len_subslots + 2; 
    uint32_t listen_duration_modem_units = listen_duration_subslots * ft_subslot_duration_ticks;

    // Point 6: "Too Soon" Handling
    uint32_t min_prep_time_ticks = modem_us_to_ticks(ctx->phy_latency.idle_to_active_rx_us +
                                                     ctx->phy_latency.scheduled_operation_startup_us,
                                                     NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
    uint8_t initial_target_sfn_for_log = target_sfn_for_rach;
    uint8_t advance_loop_guard = 0; // Prevent potential infinite loop in "too soon"

    while (ctx->last_known_modem_time > 0 && rach_listen_start_time < (ctx->last_known_modem_time + min_prep_time_ticks) && advance_loop_guard < 255) {
        advance_loop_guard++;
        LOG_WRN("FT_RACH_LSN: Target RACH listen SFN %u, SS %u at time %llu is too soon. Advancing by repetition.",
                target_sfn_for_rach, rach_adv_fields->start_subslot_index, rach_listen_start_time);

        uint8_t repetition_interval_frames = 1U << rach_adv_fields->repetition_code;
        if (repetition_interval_frames == 0) repetition_interval_frames = 1; 

        target_sfn_for_rach = (target_sfn_for_rach + repetition_interval_frames) & 0xFF;

        if (rach_adv_fields->sfn_validity_present && rach_adv_fields->validity_frames != 0xFF) {
            int16_t frames_from_initial_validity_sfn = (int16_t)target_sfn_for_rach - (int16_t)sfn_of_initial_rach_advertisement;
            if (frames_from_initial_validity_sfn < 0) frames_from_initial_validity_sfn += 256;
            if ((uint8_t)frames_from_initial_validity_sfn >= rach_adv_fields->validity_frames) {
                LOG_WRN("FT_RACH_LSN: Next RACH repetition (SFN %u) would exceed validity. Stopping listen.", target_sfn_for_rach);
                return;
            }
        }
        rach_listen_start_time = calculate_target_modem_time(ctx,
                                                              ctx->ft_sfn_zero_modem_time_anchor,
                                                              ctx->current_sfn_at_anchor_update,
                                                              target_sfn_for_rach,
                                                              rach_adv_fields->start_subslot_index,
                                                              ft_mu_for_rach,
                                                              ctx->own_phy_params.beta);
        LOG_INF("FT_RACH_LSN: Advanced to next RACH opp: SFN %u, new start_time %llu",
                target_sfn_for_rach, rach_listen_start_time);
    }
    if (advance_loop_guard >= 255) {
        LOG_ERR("FT_RACH_LSN: Cycled SFNs fully while advancing for 'too soon'. Check RACH params/timing. Skipping.");
        return;
    }


    if (listen_duration_modem_units == 0) {
        LOG_ERR("FT_RACH_LSN: Calculated listen duration is 0 for mu_code %u. Aborting.", ft_mu_for_rach);
        return;
    }

    LOG_INF("FT_RACH_LSN: Scheduling RX on RACH C%u for SFN %u (initial target was SFN %u), StartSS %u (len %u actual units, %u subslots). RXDur:%u TU, TargetStart:%llu",
            rach_carrier, target_sfn_for_rach, initial_target_sfn_for_log,
            rach_adv_fields->start_subslot_index,
            rach_resource_len_actual_units, rach_resource_len_subslots,
            listen_duration_modem_units, rach_listen_start_time);

    // uint32_t phy_op_handle = sys_rand32_get();
uint32_t phy_op_handle;
sys_rand_get(&phy_op_handle, sizeof(uint32_t));
    int ret = dect_mac_phy_ctrl_start_rx(
        rach_carrier,
        listen_duration_modem_units,
        NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
        phy_op_handle,
        ctx->own_short_rd_id,
        PENDING_OP_FT_RACH_RX_WINDOW);

    if (ret != 0) {
        LOG_ERR("FT_SM: Failed to schedule RACH RX window (SFN %u): %d", target_sfn_for_rach, ret);
    }
}


static int  ft_find_and_init_peer_slot(uint32_t pt_long_id, uint16_t pt_short_id, int16_t rssi) {
    dect_mac_context_t* ctx = get_mac_context();
    // Check if PT with this Long ID already exists
    for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
        if (ctx->role_ctx.ft.connected_pts[i].is_valid && ctx->role_ctx.ft.connected_pts[i].long_rd_id == pt_long_id) {
            LOG_WRN("FT_SM_PEER: PT LongID 0x%08X already in slot %d (OldShort:0x%04X). Updating ShortID to 0x%04X and RSSI.",
                    pt_long_id, i, ctx->role_ctx.ft.connected_pts[i].short_rd_id, pt_short_id);
            ctx->role_ctx.ft.connected_pts[i].short_rd_id = pt_short_id;
            ctx->role_ctx.ft.connected_pts[i].rssi_2 = rssi;
            ctx->role_ctx.ft.connected_pts[i].is_secure = false; // Re-association requires re-authentication
            ctx->role_ctx.ft.connected_pts[i].hpc = 1;       // Reset tracked HPC for peer
            ctx->role_ctx.ft.keys_provisioned_for_peer[i] = false;
            ctx->role_ctx.ft.connected_pts[i].num_pending_feedback_items = 0;
            memset(ctx->role_ctx.ft.connected_pts[i].pending_feedback_to_send, 0, sizeof(ctx->role_ctx.ft.connected_pts[i].pending_feedback_to_send));
            // Keep existing schedule for now, or clear it:
            // memset(&ctx->role_ctx.ft.peer_schedules[i], 0, sizeof(dect_mac_schedule_t));
            // ctx->role_ctx.ft.peer_schedules[i].is_active = false;
            return i;
        }
    }
    // Find a new free slot
    for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
        if (!ctx->role_ctx.ft.connected_pts[i].is_valid) {
            ctx->role_ctx.ft.connected_pts[i].is_valid = true;
            ctx->role_ctx.ft.connected_pts[i].long_rd_id = pt_long_id;
            ctx->role_ctx.ft.connected_pts[i].short_rd_id = pt_short_id;
            ctx->role_ctx.ft.connected_pts[i].rssi_2 = rssi;
            ctx->role_ctx.ft.connected_pts[i].is_secure = false;
            ctx->role_ctx.ft.connected_pts[i].hpc = 1; // FT's initial assumption of PT's TX HPC
            ctx->role_ctx.ft.keys_provisioned_for_peer[i] = false;
            ctx->role_ctx.ft.connected_pts[i].num_pending_feedback_items = 0;
            memset(ctx->role_ctx.ft.connected_pts[i].pending_feedback_to_send, 0, sizeof(ctx->role_ctx.ft.connected_pts[i].pending_feedback_to_send));
            memset(&ctx->role_ctx.ft.peer_schedules[i], 0, sizeof(dect_mac_schedule_t));
            ctx->role_ctx.ft.peer_schedules[i].is_active = false;
            LOG_INF("FT SM: PT 0x%08X (S:0x%04X) assigned to new peer slot %d.", pt_long_id, pt_short_id, i);
            return i;
        }
    }
    LOG_WRN("FT_SM: No free peer slots for PT LongID 0x%08X.", pt_long_id);
    return -1;
}

static void ft_handle_phy_op_complete_ft(const struct nrf_modem_dect_phy_op_complete_event *event, pending_op_type_t completed_op_type) {
    dect_mac_context_t* ctx = get_mac_context();
    switch (completed_op_type) {
        case PENDING_OP_FT_INITIAL_SCAN:
            if (ctx->role_ctx.ft.dcs_current_channel_scan_index < ctx->role_ctx.ft.dcs_num_valid_candidate_channels) {
                 LOG_INF("FT SM: DCS Scan for channel %u (idx %u of %u) completed (err %d).",
                        ctx->role_ctx.ft.dcs_candidate_channels[ctx->role_ctx.ft.dcs_current_channel_scan_index],
                        ctx->role_ctx.ft.dcs_current_channel_scan_index + 1, // 1-based for logging
                        ctx->role_ctx.ft.dcs_num_valid_candidate_channels,
                        event->err);
            } // else, this might be a stale completion if dcs_current_channel_scan_index was reset.

            if (event->err != NRF_MODEM_DECT_PHY_SUCCESS && event->err != NRF_MODEM_DECT_PHY_ERR_OP_CANCELED) {
                if (ctx->role_ctx.ft.dcs_current_channel_scan_index < ctx->role_ctx.ft.dcs_num_valid_candidate_channels) {
                    LOG_ERR("FT_DCS: Scan op for C%u failed (err %d). Marking as unusable.",
                            ctx->role_ctx.ft.dcs_candidate_channels[ctx->role_ctx.ft.dcs_current_channel_scan_index], event->err);
                    ctx->role_ctx.ft.dcs_candidate_rssi_avg[ctx->role_ctx.ft.dcs_current_channel_scan_index] = INT16_MAX; // Mark as very noisy
                    ctx->role_ctx.ft.dcs_candidate_busy_percent[ctx->role_ctx.ft.dcs_current_channel_scan_index] = 100;
                }
            }

            ctx->role_ctx.ft.dcs_current_channel_scan_index++;
            if (ctx->role_ctx.ft.dcs_current_channel_scan_index < ctx->role_ctx.ft.dcs_num_valid_candidate_channels) {
                uint16_t next_scan_carrier = ctx->role_ctx.ft.dcs_candidate_channels[ctx->role_ctx.ft.dcs_current_channel_scan_index];
                LOG_INF("FT SM: Starting DCS scan %u/%u on carrier %u.",
                        ctx->role_ctx.ft.dcs_current_channel_scan_index + 1,
                        ctx->role_ctx.ft.dcs_num_valid_candidate_channels, next_scan_carrier);
						
                // uint32_t phy_op_handle = sys_rand32_get();
				uint32_t phy_op_handle;
				sys_rand_get(&phy_op_handle, sizeof(uint32_t));
                uint32_t scan_duration_total_subslots = SCAN_MEAS_DURATION_SLOTS_CONFIG * get_subslots_per_etsi_slot_for_mu(ctx->own_phy_params.mu);
                uint32_t subslot_ticks = get_subslot_duration_ticks_for_mu(ctx->own_phy_params.mu);
                uint32_t scan_duration_modem_units = scan_duration_total_subslots * subslot_ticks;
                if (subslot_ticks == 0 || scan_duration_modem_units < scan_duration_total_subslots) { // Use local var for safety
                     scan_duration_modem_units = modem_us_to_ticks(10000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
                }

                int ret = dect_mac_phy_ctrl_start_rssi_scan(
                    next_scan_carrier, scan_duration_modem_units,
                    NRF_MODEM_DECT_PHY_RSSI_INTERVAL_24_SLOTS,
                    phy_op_handle, PENDING_OP_FT_INITIAL_SCAN);
                if (ret != 0) {
                    LOG_ERR("FT SM: Failed to start next DCS scan (C%u): %d. Proceeding with selection based on current results.", next_scan_carrier, ret);
                    ctx->role_ctx.ft.dcs_scan_complete = true;
                    ft_select_operating_carrier_and_start_beaconing(NULL);
                }
            } else {
                LOG_INF("FT SM: DCS scan sequence complete (%u channels scanned).", ctx->role_ctx.ft.dcs_num_valid_candidate_channels);
                ctx->role_ctx.ft.dcs_scan_complete = true;
                ft_select_operating_carrier_and_start_beaconing(NULL);
            }
            break;



        case PENDING_OP_FT_BEACON:
            if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_ERR("FT SM: Beacon TX failed (err %d). Next beacon by timer.", event->err);
            } else {
                LOG_DBG("FT SM: Beacon TX SFN %u successful.", ctx->role_ctx.ft.sfn_for_last_beacon_tx);
            }
            if (ctx->state == MAC_STATE_FT_BEACONING) {
                ft_schedule_rach_listen_action();
            }
            break;
        case PENDING_OP_FT_RACH_RX_WINDOW:
             LOG_DBG("FT SM: RACH RX window op completed (err %d). Next RACH listen after next beacon cycle.", event->err);
            // Next RACH listen will be scheduled after next beacon if FT is still in beaconing state.
            break;
        case PENDING_OP_FT_ASSOC_RESP:
             if (event->err == NRF_MODEM_DECT_PHY_SUCCESS) {
                LOG_INF("FT SM: Association Response sent successfully to PT ShortID 0x%04X.",
                        ctx->role_ctx.ft.last_assoc_resp_pt_short_id);
             } else {
                LOG_ERR("FT SM: Association Response TX failed (err %d) for PT ShortID 0x%04X.",
                        event->err, ctx->role_ctx.ft.last_assoc_resp_pt_short_id);
                int peer_idx = ft_get_peer_slot_idx(ctx, ctx->role_ctx.ft.last_assoc_resp_pt_short_id);
                if (peer_idx != -1 && ctx->role_ctx.ft.connected_pts[peer_idx].is_valid) {
                    if (ctx->role_ctx.ft.connected_pts[peer_idx].long_rd_id != 0) { // Check if it was an actual acceptance attempt
                        LOG_INF("FT SM: Invalidating peer slot %d for PT 0x%04X due to failed AssocResp TX.",
                                peer_idx, ctx->role_ctx.ft.last_assoc_resp_pt_short_id);
                        // Don't fully clear, just mark not valid, so no new PT takes this slot immediately if it was a temp issue
                        ctx->role_ctx.ft.connected_pts[peer_idx].is_valid = false; // Or a more nuanced state like "assoc_pending_retry"
                        ctx->role_ctx.ft.keys_provisioned_for_peer[peer_idx] = false;
                    }
                }
            }
            ctx->role_ctx.ft.last_assoc_resp_pt_short_id = 0; // Clear
            break;
        case PENDING_OP_FT_DATA_TX_HARQ0:
        case PENDING_OP_FT_DATA_TX_HARQ_MAX: // Covers range with fallthrough
            {
                int harq_idx = completed_op_type - PENDING_OP_FT_DATA_TX_HARQ0;
                if (harq_idx >= 0 && harq_idx < MAX_HARQ_PROCESSES) {
                    if (event->err == NRF_MODEM_DECT_PHY_ERR_LBT_CHANNEL_BUSY) {
                        LOG_WRN("FT SM: Data TX HARQ %d LBT busy. Data Path will re-TX.", harq_idx);
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else if (event->err != NRF_MODEM_DECT_PHY_SUCCESS) {
                        LOG_ERR("FT SM: Data TX HARQ %d failed (err %d). Data Path will re-TX/discard.", harq_idx, event->err);
                        dect_mac_data_path_handle_harq_nack_action(harq_idx);
                    } else {
                        LOG_DBG("FT SM: Data TX HARQ %d PHY op complete. Awaiting feedback.", harq_idx);
                    }
                } else {
                     LOG_ERR("FT SM: OP_COMPLETE for invalid FT_DATA_TX_HARQ op type: %d", completed_op_type);
                }
            }
            break;
        default:
            LOG_WRN("FT SM: OP_COMPLETE for unhandled FT op type: %s, err %d",
                    dect_pending_op_to_str(completed_op_type), event->err);
            break;
    }
}


static void ft_handle_phy_rssi_ft(const struct nrf_modem_dect_phy_rssi_event *rssi_event) {
    dect_mac_context_t* ctx = get_mac_context();
    if (ctx->state != MAC_STATE_FT_SCANNING || ctx->pending_op_type != PENDING_OP_FT_INITIAL_SCAN || rssi_event->handle != ctx->pending_op_handle) {
        LOG_WRN("FT_RSSI: Received RSSI result for unexpected op type %s, handle %u, or state %s.",
                dect_pending_op_to_str(ctx->pending_op_type), rssi_event->handle, dect_mac_state_to_str(ctx->state));
        // Clear pending op if it was this handle to avoid stall
        if (rssi_event->handle == ctx->pending_op_handle) {
             dect_mac_phy_ctrl_handle_op_complete(&(struct nrf_modem_dect_phy_op_complete_event){.handle = rssi_event->handle, .err = NRF_MODEM_DECT_PHY_ERR_OP_CANCELED});
        }
        return;
    }

    // pending_op is cleared by dect_mac_phy_ctrl_handle_op_complete called from dispatcher before this.
    // No, this handler is called directly for the RSSI event itself by the dispatcher.
    // The PENDING_OP_FT_INITIAL_SCAN is completed with its own NRF_MODEM_DECT_PHY_EVT_COMPLETED event.
    // This RSSI event is an intermediate report *during* PENDING_OP_FT_INITIAL_SCAN.

    uint8_t current_scan_idx = ctx->role_ctx.ft.dcs_current_channel_scan_index;
    uint16_t scanned_carrier = rssi_event->carrier;

    if (scanned_carrier != ctx->role_ctx.ft.dcs_candidate_channels[current_scan_idx]) {
        LOG_WRN("FT_RSSI: RSSI report for carrier %u, but expected scan for %u (idx %u). Ignoring.",
                scanned_carrier, ctx->role_ctx.ft.dcs_candidate_channels[current_scan_idx], current_scan_idx);
        return;
    }

    if (rssi_event->meas_len > 0) {
        int32_t rssi_sum_q71 = 0; // Renamed to avoid conflict and indicate Q7.1
        int valid_sample_count = 0;
        // New calculation including busy percentage:
        int busy_sample_count = 0;
        int16_t busy_threshold_q71 = ctx->config.rssi_threshold_max_dbm * 2; // Convert dBm to Q7.1

        for (uint16_t i = 0; i < rssi_event->meas_len; ++i) {
            if (rssi_event->meas[i] != NRF_MODEM_DECT_PHY_RSSI_NOT_MEASURED) {
                rssi_sum_q71 += rssi_event->meas[i];
                valid_sample_count++;
                if (rssi_event->meas[i] > busy_threshold_q71) {
                    busy_sample_count++;
                }
            }
        }

        if (valid_sample_count > 0) {
            ctx->role_ctx.ft.dcs_candidate_rssi_avg[current_scan_idx] = rssi_sum_q71 / valid_sample_count;
            ctx->role_ctx.ft.dcs_candidate_busy_percent[current_scan_idx] = (busy_sample_count * 100) / valid_sample_count;
            LOG_INF("FT_DCS: Scan %u/%u on C%u: AvgRSSI %.1f dBm, Busy %u%% (%d/%d samples).",
                    current_scan_idx + 1, CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN,
                    scanned_carrier,
                    (double)ctx->role_ctx.ft.dcs_candidate_rssi_avg[current_scan_idx] / 2.0f,
                    ctx->role_ctx.ft.dcs_candidate_busy_percent[current_scan_idx],
                    busy_sample_count, valid_sample_count);
        } else {
            LOG_WRN("FT_DCS: Scan %u/%u on C%u: No valid RSSI samples.",
                    current_scan_idx + 1, CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN, scanned_carrier);
            ctx->role_ctx.ft.dcs_candidate_rssi_avg[current_scan_idx] = 0; // Or INT16_MAX to mark as unusable
            ctx->role_ctx.ft.dcs_candidate_busy_percent[current_scan_idx] = 101; // Mark as unscanned/invalid
        }


    } else {
        LOG_WRN("FT_DCS: Scan %u/%u on C%u: RSSI event with no measurements.",
                current_scan_idx + 1, CONFIG_DECT_MAC_DCS_NUM_CHANNELS_TO_SCAN, scanned_carrier);
    }

    // This RSSI event is just a report. The actual PENDING_OP_FT_INITIAL_SCAN
    // will complete with NRF_MODEM_DECT_PHY_EVT_COMPLETED.
    // The logic to scan the *next* channel or select the best should happen
    // in the handler for NRF_MODEM_DECT_PHY_EVT_COMPLETED for PENDING_OP_FT_INITIAL_SCAN.
}


// Include the full ft_handle_phy_pcc_ft from "Phase 5 P5.3"
static void ft_handle_phy_pcc_ft(const struct nrf_modem_dect_phy_pcc_event *pcc_event) {
    dect_mac_context_t* ctx = get_mac_context();
    ft_last_relevant_pcc_is_valid = false; 

    if (pcc_event->header_status == NRF_MODEM_DECT_PHY_HDR_STATUS_VALID) {
        uint16_t pcc_rx_id_on_ft_antenna = 0;
        uint16_t pcc_tx_id_from_pt = 0;

        if (pcc_event->phy_type == 1) { // nRF PHY Type 1 (ETSI PCC Type 2)
            pcc_rx_id_on_ft_antenna = sys_be16_to_cpu(
                (uint16_t)((pcc_event->hdr.hdr_type_2.receiver_id_hi << 8) |
                            pcc_event->hdr.hdr_type_2.receiver_id_lo));
            pcc_tx_id_from_pt = sys_be16_to_cpu(
                (uint16_t)((pcc_event->hdr.hdr_type_2.transmitter_id_hi << 8) |
                            pcc_event->hdr.hdr_type_2.transmitter_id_lo));

            if (pcc_rx_id_on_ft_antenna == ctx->own_short_rd_id) {
                memcpy(&ft_last_relevant_pcc, pcc_event, sizeof(struct nrf_modem_dect_phy_pcc_event));
                // ft_last_relevant_pcc.pcc_event_modem_time = pcc_event->time; // If storing time in wrapper
                ft_last_relevant_pcc_is_valid = true;

                if (ctx->pending_op_type == PENDING_OP_FT_RACH_RX_WINDOW) {
                    LOG_INF("FT_SM_PCC: PCC (Type2) on RACH from PT 0x%04X. TID:%u. Waiting PDC.",
                            pcc_tx_id_from_pt, pcc_event->transaction_id);
                    ctx->role_ctx.ft.last_rach_pt_short_id = pcc_tx_id_from_pt;
                    ctx->role_ctx.ft.last_rach_rssi2 = pcc_event->rssi_2;
                } else { 
                    int peer_idx = ft_get_peer_slot_idx(ctx, pcc_tx_id_from_pt);
                    if (peer_idx != -1) {
                        LOG_DBG("FT_SM_PCC: PCC (Type2) from connected PT 0x%04X (slot %d). TID:%u.",
                                pcc_tx_id_from_pt, peer_idx, pcc_event->transaction_id);
                        dect_mac_data_path_process_harq_feedback(&pcc_event->hdr.hdr_type_2.feedback, pcc_tx_id_from_pt);
                    } else {
                        LOG_WRN("FT_SM_PCC: PCC (Type2) from unknown PT 0x%04X. TID:%u. Ignoring.",
                                pcc_tx_id_from_pt, pcc_event->transaction_id);
                        ft_last_relevant_pcc_is_valid = false;
                    }
                }
            } else { 
                LOG_DBG("FT_SM_PCC: PCC (Type2) not for this FT (RxID 0x%04X vs Own 0x%04X). TID: %u",
                        pcc_rx_id_on_ft_antenna, ctx->own_short_rd_id, pcc_event->transaction_id);
            }
        } else if (pcc_event->phy_type == 0) {
             LOG_WRN("FT_SM_PCC: Received Type 1 PCC (Beacon). Normally not processed by FT. TID: %u", pcc_event->transaction_id);
        } else { 
             LOG_ERR("FT_SM_PCC: Unknown nRF PHY header type in PCC: %d. TID: %u", pcc_event->phy_type, pcc_event->transaction_id);
        }
    } else { LOG_WRN("FT_SM_PCC: Invalid PCC status %d. TID:%u.", pcc_event->header_status, pcc_event->transaction_id); }
}

static void ft_handle_phy_pdc_ft(const struct nrf_modem_dect_phy_pdc_event *pdc_event)
{
	dect_mac_context_t *ctx = get_mac_context();
	uint8_t mac_pdc_payload_copy[CONFIG_DECT_MAC_PDU_MAX_SIZE];
	uint16_t pdc_payload_len = pdc_event->len;

	if (!ft_last_relevant_pcc_is_valid ||
	    ft_last_relevant_pcc.transaction_id != pdc_event->transaction_id) {
		LOG_WRN("FT_SM_PDC: PDC (TID %u) without matching valid PCC. Discarding.",
			pdc_event->transaction_id);
		return;
	}
	struct nrf_modem_dect_phy_pcc_event current_pcc_data = ft_last_relevant_pcc;

	ft_last_relevant_pcc_is_valid = false;

	if (pdc_payload_len == 0 && pdc_event->transaction_id != 0) {
		LOG_DBG("FT_SM_PDC: Empty PDC (TID %u). Ignoring.", pdc_event->transaction_id);
		return;
	}
	if (pdc_payload_len > sizeof(mac_pdc_payload_copy)) {
		LOG_ERR("FT_SM_PDC: PDC payload from PHY (%u bytes) too large for copy buffer (%zu). Discarding.",
			pdc_payload_len, sizeof(mac_pdc_payload_copy));
		return;
	}
	memcpy(mac_pdc_payload_copy, pdc_event->data, pdc_payload_len);

	dect_mac_header_type_octet_t mac_hdr_type_octet;

	/* The MAC Header Type is the FIRST byte of the PDC payload (the MAC PDU). */
	memcpy(&mac_hdr_type_octet, &mac_pdc_payload_copy[0],
	       sizeof(dect_mac_header_type_octet_t));

	uint8_t *pdu_content_start = mac_pdc_payload_copy;
	uint16_t pdu_content_len = pdc_payload_len;

	uint16_t pt_sender_short_id_from_pcc = 0;

	if (current_pcc_data.phy_type == 1) { /* Unicast/Data */
		pt_sender_short_id_from_pcc = sys_be16_to_cpu(
			(uint16_t)((current_pcc_data.hdr.hdr_type_2.transmitter_id_hi << 8) |
				   current_pcc_data.hdr.hdr_type_2.transmitter_id_lo));
	} else {
		LOG_ERR("FT_SM_PDC: PDC for non-unicast PCC phy_type %d. Discarding.",
			current_pcc_data.phy_type);
		return;
	}

	int peer_slot_idx = ft_get_peer_slot_idx(ctx, pt_sender_short_id_from_pcc);
	dect_mac_peer_info_t *pt_peer_ctx =
		(peer_slot_idx != -1) ? &ctx->role_ctx.ft.connected_pts[peer_slot_idx] : NULL;

	bool security_applied_by_sender = (mac_hdr_type_octet.mac_security != MAC_SECURITY_NONE);
	bool link_is_expected_to_be_secure =
		(pt_peer_ctx && pt_peer_ctx->is_valid && pt_peer_ctx->is_secure &&
		 ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx]);
	bool pdc_process_ok_for_feedback = true;

	uint8_t *common_hdr_start_in_payload = pdu_content_start + sizeof(dect_mac_header_type_octet_t);
	size_t common_hdr_actual_len = 0;
	uint8_t *sdu_area_after_common_hdr = NULL;
	size_t sdu_area_plus_mic_len_in_payload = 0;

	if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_UNICAST) {
		common_hdr_actual_len = sizeof(dect_mac_unicast_header_t);
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU) {
		common_hdr_actual_len = sizeof(dect_mac_data_pdu_header_t);
	} else {
		LOG_WRN("FT_SM_PDC: Received PDC with MAC Hdr Type %u, not Unicast/Data from PT. TID %u",
			mac_hdr_type_octet.mac_header_type, pdc_event->transaction_id);
		return;
	}

	if (pdu_content_len < (sizeof(dect_mac_header_type_octet_t) + common_hdr_actual_len)) {
		LOG_ERR("FT_SM_PDC: PDU too short (%u) for Common Hdr type %u (len %zu).",
			pdu_content_len, mac_hdr_type_octet.mac_header_type, common_hdr_actual_len);
		return;
	}
	sdu_area_after_common_hdr = common_hdr_start_in_payload + common_hdr_actual_len;
	sdu_area_plus_mic_len_in_payload = pdu_content_len - sizeof(dect_mac_header_type_octet_t) - common_hdr_actual_len;

	uint8_t *sdu_area_for_data_path = sdu_area_after_common_hdr;
	size_t sdu_area_len_for_data_path = sdu_area_plus_mic_len_in_payload;

	if (security_applied_by_sender) {
		if (!pt_peer_ctx || !link_is_expected_to_be_secure) {
			LOG_WRN("FT_SM_PDC_SEC: Secured PDU from PT 0x%04X, but no valid secure context/peer_ctx. Discarding.",
				pt_sender_short_id_from_pcc);
			return;
		}
		if (sdu_area_plus_mic_len_in_payload < 5) {
			LOG_ERR("FT_SM_PDC_SEC: Secured PDU from PT 0x%04X too short for MIC (SDUArea+MIC len %zu). Discarding.",
				pt_sender_short_id_from_pcc, sdu_area_plus_mic_len_in_payload);
			return;
		}

		const dect_mac_unicast_header_t *uch_ptr =
			(const dect_mac_unicast_header_t *)common_hdr_start_in_payload;
		uint16_t received_psn =
			((uch_ptr->sequence_num_high_reset_rsv >> 4) & 0x0F) << 8 |
			uch_ptr->sequence_num_low;
		uint32_t pt_tx_long_id_from_hdr =
			sys_be32_to_cpu(uch_ptr->transmitter_long_rd_id_be);

		if (pt_tx_long_id_from_hdr != pt_peer_ctx->long_rd_id) {
			LOG_WRN("FT_SM_PDC_SEC: Secured PDU LongID 0x%08X mismatch for PT 0x%04X (expected 0x%08X). Discarding.",
				pt_tx_long_id_from_hdr, pt_sender_short_id_from_pcc,
				pt_peer_ctx->long_rd_id);
			return;
		}

		uint8_t *payload_to_decrypt_start = NULL;
		size_t payload_to_decrypt_len = 0;
		size_t cleartext_sec_ie_mux_len = 0;

		if (mac_hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) {
			uint8_t ie_type_sec;
			uint16_t ie_len_sec;
			const uint8_t *ie_payload_sec;
			int mux_hdr_len_sec =
				parse_mac_mux_header(sdu_area_after_common_hdr,
						     sdu_area_plus_mic_len_in_payload, &ie_type_sec,
						     &ie_len_sec, &ie_payload_sec);

			if (mux_hdr_len_sec > 0 && ie_type_sec == IE_TYPE_MAC_SECURITY_INFO) {
				if (sdu_area_plus_mic_len_in_payload <
				    (size_t)mux_hdr_len_sec + ie_len_sec + 5) {
					pdc_process_ok_for_feedback = false;
					goto process_feedback_ft_pdc_secure_rx_path;
				}
				cleartext_sec_ie_mux_len = mux_hdr_len_sec + ie_len_sec;
				uint8_t ver, kidx, secivtype_from_ie;
				uint32_t hpc_from_ie;

				if (parse_mac_security_info_ie_payload(
					    ie_payload_sec, ie_len_sec, &ver, &kidx,
					    &secivtype_from_ie, &hpc_from_ie) == 0) {
					bool hpc_accepted_for_iv = false;

					if (secivtype_from_ie == SEC_IV_TYPE_MODE1_HPC_PROVIDED) {
						uint32_t forward_diff;

						if (hpc_from_ie >=
						    pt_peer_ctx->highest_rx_peer_hpc) {
							forward_diff =
								hpc_from_ie -
								pt_peer_ctx->highest_rx_peer_hpc;
						} else {
							forward_diff =
								(UINT32_MAX -
								 pt_peer_ctx->highest_rx_peer_hpc) +
								hpc_from_ie + 1;
						}

						if (forward_diff <=
						    CONFIG_DECT_MAC_HPC_RX_FORWARD_WINDOW_MAX_ADVANCE) {
							pt_peer_ctx->highest_rx_peer_hpc =
								hpc_from_ie;
							pt_peer_ctx->hpc = hpc_from_ie;
							hpc_accepted_for_iv = true;
						} else {
							pdc_process_ok_for_feedback = false;
						}
					} else if (secivtype_from_ie ==
						   SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE) {
						pt_peer_ctx->peer_requested_hpc_resync = true;
						pt_peer_ctx->hpc = hpc_from_ie;
						hpc_accepted_for_iv = true;
					} else {
						pdc_process_ok_for_feedback = false;
					}

					if (!hpc_accepted_for_iv) {
						pdc_process_ok_for_feedback = false;
					}
				} else {
					pdc_process_ok_for_feedback = false;
				}

				payload_to_decrypt_start =
					sdu_area_after_common_hdr + cleartext_sec_ie_mux_len;
				payload_to_decrypt_len = sdu_area_plus_mic_len_in_payload -
							 cleartext_sec_ie_mux_len;
			} else {
				pdc_process_ok_for_feedback = false;
			}
		} else {
			cleartext_sec_ie_mux_len = 0;
			payload_to_decrypt_start = sdu_area_after_common_hdr;
			payload_to_decrypt_len = sdu_area_plus_mic_len_in_payload;
		}

		if (!pdc_process_ok_for_feedback) {
			goto process_feedback_ft_pdc_secure_rx_path;
		}
		if (payload_to_decrypt_len < 5) {
			pdc_process_ok_for_feedback = false;
			goto process_feedback_ft_pdc_secure_rx_path;
		}

		uint8_t iv[16];

		security_build_iv(iv, pt_tx_long_id_from_hdr, ctx->own_long_rd_id,
				  pt_peer_ctx->hpc, received_psn);

		if (security_crypt_payload(payload_to_decrypt_start, payload_to_decrypt_len,
					   ctx->role_ctx.ft.peer_cipher_keys[peer_slot_idx], iv,
					   false) != 0) {
			pdc_process_ok_for_feedback = false;
			goto process_feedback_ft_pdc_secure_rx_path;
		}

		uint8_t *cleartext_mic_ptr = payload_to_decrypt_start + payload_to_decrypt_len - 5;
		uint8_t calculated_mic[5];
		size_t data_for_mic_len = common_hdr_actual_len +
					(payload_to_decrypt_len - 5) + cleartext_sec_ie_mux_len;

		if (security_calculate_mic(common_hdr_start_in_payload, data_for_mic_len,
					   ctx->role_ctx.ft.peer_integrity_keys[peer_slot_idx],
					   calculated_mic) != 0) {
			pdc_process_ok_for_feedback = false;
			goto process_feedback_ft_pdc_secure_rx_path;
		}

		if (constant_time_memcmp(cleartext_mic_ptr, calculated_mic, 5) != 0) {
			pt_peer_ctx->consecutive_mic_failures++;
			if (pt_peer_ctx->consecutive_mic_failures >=
			    CONFIG_DECT_MAC_MAX_MIC_FAILURES_BEFORE_HPC_RESYNC) {
				pt_peer_ctx->self_needs_to_request_hpc_from_peer = true;
				pt_peer_ctx->consecutive_mic_failures = 0;
			}
			pdc_process_ok_for_feedback = false;
		} else { /* MIC OK */
			LOG_DBG("FT_SM_PDC_SEC: MIC OK from PT 0x%04X (PSN %u, PeerHPC %u).",
				pt_sender_short_id_from_pcc, received_psn, pt_peer_ctx->hpc);
			pt_peer_ctx->consecutive_mic_failures = 0;
			sdu_area_for_data_path =
				sdu_area_after_common_hdr + cleartext_sec_ie_mux_len;
			sdu_area_len_for_data_path = payload_to_decrypt_len - 5;
		}
	}

process_feedback_ft_pdc_secure_rx_path:; /* Label must have a statement */

	/* HARQ Feedback logic now runs for both secure and unsecure data packets */
	if (pt_peer_ctx && pt_peer_ctx->is_valid && current_pcc_data.phy_type == 1) {
		uint8_t harq_proc_in_pt_tx = current_pcc_data.hdr.hdr_type_2.df_harq_process_num;

		if (pt_peer_ctx->num_pending_feedback_items < 2) {
			int fb_idx = pt_peer_ctx->num_pending_feedback_items++;

			pt_peer_ctx->pending_feedback_to_send[fb_idx].valid = true;
			pt_peer_ctx->pending_feedback_to_send[fb_idx].is_ack =
				pdc_process_ok_for_feedback;
			pt_peer_ctx->pending_feedback_to_send[fb_idx].harq_process_num_for_peer =
				harq_proc_in_pt_tx;
		} else {
			LOG_WRN("FT_SM_PDC: Feedback buffer full for PT 0x%04X",
				pt_sender_short_id_from_pcc);
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
			case IE_TYPE_ASSOC_REQ:
				if (ctx->pending_op_type == PENDING_OP_FT_RACH_RX_WINDOW && pt_peer_ctx == NULL) {
					ft_process_association_request_pdu(
						sdu_area_for_data_path, sdu_area_len_for_data_path,
						pt_sender_short_id_from_pcc, sender_long_id_final,
						current_pcc_data.rssi_2);
				}
				break;
			case IE_TYPE_ASSOC_RELEASE:
				if (pt_peer_ctx) {
					dect_mac_assoc_release_ie_t release_fields;
					if (parse_assoc_release_ie_payload(ie_payload, ie_len, &release_fields) == 0) {
						LOG_WRN("FT SM: Received Association Release from PT 0x%04X (cause: %u). Invalidating peer slot %d.",
							pt_sender_short_id_from_pcc, release_fields.cause, peer_slot_idx);
						pt_peer_ctx->is_valid = false;
						/* TODO: Also cancel any pending schedules for this peer */
					}
				}
				break;
			case IE_TYPE_AUTH_INITIATE:
				if (ie_len == sizeof(dect_mac_auth_initiate_ie_t)) {
					const dect_mac_auth_initiate_ie_t *auth_init_ie = (const dect_mac_auth_initiate_ie_t *)ie_payload;
					int peer_idx = ft_find_and_init_peer_slot(sender_long_id_final, pt_sender_short_id_from_pcc, current_pcc_data.rssi_2);
					if (peer_idx != -1) {
						ctx->role_ctx.ft.connected_pts[peer_idx].pt_nonce = sys_be32_to_cpu(auth_init_ie->pt_nonce_be);
						ft_send_auth_challenge_action(peer_idx);
					}
				}
				break;
			case IE_TYPE_AUTH_RESPONSE:
				if (ie_len == sizeof(dect_mac_auth_response_ie_t) && pt_peer_ctx) {
					const dect_mac_auth_response_ie_t *auth_resp_ie = (const dect_mac_auth_response_ie_t *)ie_payload;
					ft_send_auth_success_action(peer_slot_idx, auth_resp_ie->pt_mac);
				}
				break;
			default:
				if (pt_peer_ctx && pt_peer_ctx->is_valid) {
					dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path,
									 sdu_area_len_for_data_path,
									 sender_long_id_final);
				} else {
					LOG_WRN("FT_SM_PDC: Unicast from unknown PT 0x%04X or unexpected state. Discarding.",
						pt_sender_short_id_from_pcc);
				}
				break;
			}
		}
	} else if (mac_hdr_type_octet.mac_header_type == MAC_COMMON_HEADER_TYPE_DATA_PDU &&
		   pt_peer_ctx && pt_peer_ctx->is_valid) {
		sender_long_id_final = pt_peer_ctx->long_rd_id;
		dect_mac_data_path_handle_rx_sdu(sdu_area_for_data_path,
						 sdu_area_len_for_data_path, sender_long_id_final);
	} else {
		LOG_WRN("FT_SM_PDC: Received PDC with unhandled/unexpected MAC Common Header Type %u after security.",
			mac_hdr_type_octet.mac_header_type);
	}
}

static void ft_process_association_request_pdu(const uint8_t *mac_sdu_area_data, size_t mac_sdu_area_len,
                                               uint16_t pt_tx_short_rd_id, uint32_t pt_tx_long_rd_id, int16_t rssi_from_pcc)
{
	dect_mac_context_t *ctx = get_mac_context();
	dect_mac_assoc_req_ie_t req_fields;
	dect_mac_rd_capability_ie_t pt_cap_fields;
	bool assoc_req_ie_found = false;
	bool pt_cap_ie_found = false;

	LOG_INF("FT_SM_ASSOC: Processing Association Request from PT LongID:0x%08X, ShortID:0x%04X, RSSI:%.1f dBm",
		pt_tx_long_rd_id, pt_tx_short_rd_id, (double)rssi_from_pcc / 2.0f);

	const uint8_t *current_ie_ptr = mac_sdu_area_data;
	size_t remaining_len = mac_sdu_area_len;

	while (remaining_len > 0) {
		uint8_t ie_type;
		uint16_t ie_payload_len;
		const uint8_t *ie_payload_ptr;
		int mux_hdr_len = parse_mac_mux_header(current_ie_ptr, remaining_len, &ie_type,
						       &ie_payload_len, &ie_payload_ptr);
		if (mux_hdr_len <= 0) {
			break;
		}
		// if (ie_payload_len == 0 && ((current_ie_ptr[0] >> 6) & 0x03) == 0b00) {
		// 	ie_payload_len = get_fixed_ie_payload_len(ie_type);
		// }
		if (remaining_len < (size_t)mux_hdr_len + ie_payload_len) {
			break;
		}

		if (ie_type == IE_TYPE_ASSOC_REQ) {
			if (parse_assoc_req_ie_payload(ie_payload_ptr, ie_payload_len, &req_fields) == 0) {
				assoc_req_ie_found = true;
			}
		} else if (ie_type == IE_TYPE_RD_CAPABILITY) {
			if (parse_rd_capability_ie_payload(ie_payload_ptr, ie_payload_len, &pt_cap_fields) == 0) {
				pt_cap_ie_found = true;
			}
		}

		size_t consumed = mux_hdr_len + ie_payload_len;
		if (remaining_len >= consumed) {
			remaining_len -= consumed;
			current_ie_ptr += consumed;
		} else {
			remaining_len = 0;
		}
	}

	if (!assoc_req_ie_found) {
		LOG_WRN("FT_SM_ASSOC: No Association Request IE found in PDU from PT 0x%04X. Ignoring.", pt_tx_short_rd_id);
		return;
	}

	int peer_slot_idx = ft_find_and_init_peer_slot(pt_tx_long_rd_id, pt_tx_short_rd_id, rssi_from_pcc);
	if (peer_slot_idx < 0) {
		// This block handles the "FT is full" case.
		LOG_WRN("FT is full. Rejecting association from PT 0x%08X.", pt_tx_long_rd_id);
		dect_mac_assoc_resp_ie_t reject_fields = {
			.ack_nack = false,
			.reject_cause = ASSOC_REJECT_CAUSE_OTHER, // Or a specific "resources unavailable" cause
			.reject_timer_code = 5,
		};
		
        // To send a reject, we still need to build a PDU. We can't use the main sender
        // function as it requires a valid peer_slot_idx. We need a temporary sender.
        // Let's create a helper for this specific case.
        ft_send_reject_response_action(pt_tx_long_rd_id, pt_tx_short_rd_id, &reject_fields);		
		// ft_send_association_response_action(peer_slot_idx, pt_tx_long_rd_id, pt_tx_short_rd_id, &reject_fields, NULL, NULL);
		return;
	}

	/* Store parsed fields in the peer context */
	dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];

	/* Act on Setup Cause */
	switch (req_fields.setup_cause_val) {
	case ASSOC_CAUSE_INITIAL_ASSOCIATION:
		LOG_DBG("FT_ASSOC: Processing as Initial Association.");
		break;
	case ASSOC_CAUSE_MOBILITY:
		LOG_INF("FT_ASSOC: PT 0x%04X is associating due to mobility.", pt_tx_short_rd_id);
		break;
	case ASSOC_CAUSE_REASSOC_AFTER_ERROR:
		LOG_WRN("FT_ASSOC: PT 0x%04X is re-associating after an error.", pt_tx_short_rd_id);
		break;
	default:
		LOG_DBG("FT_ASSOC: Unhandled setup cause %u.", req_fields.setup_cause_val);
		break;
	}

	/* Store PT's requested HARQ capabilities for negotiation */
	pt_peer_ctx->pt_requested_harq_params_valid = req_fields.harq_params_present;
	if (req_fields.harq_params_present) {
		pt_peer_ctx->pt_req_harq_procs_tx = req_fields.harq_processes_tx_val;
		pt_peer_ctx->pt_req_max_harq_retx_delay = req_fields.max_harq_re_tx_delay_code;
		pt_peer_ctx->pt_req_harq_procs_rx = req_fields.harq_processes_rx_val;
		pt_peer_ctx->pt_req_max_harq_rerx_delay = req_fields.max_harq_re_rx_delay_code;
	}

	dect_mac_assoc_resp_ie_t resp_fields;
	memset(&resp_fields, 0, sizeof(resp_fields));

	if (ctx->config.ft_policy_secure_on_assoc && !ctx->master_psk_provisioned) {
		LOG_WRN("FT_ASSOC: Rejecting PT 0x%04X. Policy requires security, but no PSK is provisioned.",
			pt_tx_short_rd_id);
		resp_fields.ack_nack = false;
		resp_fields.reject_cause = ASSOC_REJECT_CAUSE_NON_SECURED_NOT_ACCEPTED;
		resp_fields.reject_timer_code = 5; /* 30s as per ETSI Table 6.4.2.5-2 */
	} else {
		resp_fields.ack_nack = true;
		if (ctx->config.ft_policy_secure_on_assoc) {
			int kdf_err = security_derive_session_keys(ctx->master_psk, ctx->own_long_rd_id, pt_tx_long_rd_id,
								   ctx->role_ctx.ft.peer_integrity_keys[peer_slot_idx],
								   ctx->role_ctx.ft.peer_cipher_keys[peer_slot_idx]);
			if (kdf_err == 0) {
				ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx] = true;
				pt_peer_ctx->is_secure = true;
				pt_peer_ctx->hpc = 1;
			} else {
				resp_fields.ack_nack = false;
				resp_fields.reject_cause = ASSOC_REJECT_CAUSE_OTHER;
				pt_peer_ctx->is_valid = false;
			}
		}

		if (resp_fields.ack_nack) {
			/* --- HARQ Parameter Negotiation --- */
			if (req_fields.harq_params_present) {
				resp_fields.harq_mod_present = true;
				resp_fields.harq_processes_tx_val_ft = MIN(CONFIG_DECT_MAC_FT_HARQ_TX_PROC_CODE, req_fields.harq_processes_rx_val);
				resp_fields.max_harq_re_tx_delay_code_ft = MIN(CONFIG_DECT_MAC_FT_HARQ_RETX_DELAY_CODE, req_fields.max_harq_re_rx_delay_code);
				resp_fields.harq_processes_rx_val_ft = MIN(CONFIG_DECT_MAC_FT_HARQ_RX_PROC_CODE, req_fields.harq_processes_tx_val);
				resp_fields.max_harq_re_rx_delay_code_ft = MIN(CONFIG_DECT_MAC_FT_HARQ_RERX_DELAY_CODE, req_fields.max_harq_re_tx_delay_code);
			} else {
				resp_fields.harq_mod_present = false;
			}

			/* --- Flow ID Negotiation --- */
			if (req_fields.number_of_flows_val == 7) {
				resp_fields.number_of_flows_accepted = 7;
			} else if (req_fields.number_of_flows_val > 0 && req_fields.number_of_flows_val <= MAX_FLOW_IDS_IN_ASSOC_REQ) {
				resp_fields.number_of_flows_accepted = req_fields.number_of_flows_val;
				memcpy(resp_fields.accepted_flow_ids, req_fields.flow_ids, req_fields.number_of_flows_val);
			} else {
				resp_fields.number_of_flows_accepted = 0;
			}

			/* --- Group Assignment Decision --- */
			if (IS_ENABLED(CONFIG_DECT_MAC_FT_SUPPORTS_GROUP_ASSIGNMENT)) {
				resp_fields.group_assignment_active = true;
				resp_fields.group_id_val = 1;
				resp_fields.resource_tag_val = peer_slot_idx;
				pt_peer_ctx->group_id = resp_fields.group_id_val;
				pt_peer_ctx->resource_tag = resp_fields.resource_tag_val;
			} else {
				resp_fields.group_assignment_active = false;
			}
		}
	}

	if (resp_fields.ack_nack) {
		dect_mac_rd_capability_ie_t ft_cap_fields_to_send = {0};
		dect_mac_resource_alloc_ie_fields_t res_alloc_fields_to_send = {0};

		/* Populate FT's RD Capability IE from common Kconfig values */
		ft_cap_fields_to_send.release_version = CONFIG_DECT_MAC_CAP_RELEASE_VERSION;
		ft_cap_fields_to_send.num_phy_capabilities = 1;
		ft_cap_fields_to_send.supports_group_assignment = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_GROUP_ASSIGNMENT);
		ft_cap_fields_to_send.supports_paging = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_PAGING);
		ft_cap_fields_to_send.operating_modes_code = DECT_MAC_OP_MODE_FT_ONLY;
		ft_cap_fields_to_send.supports_mesh = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_MESH);
		ft_cap_fields_to_send.supports_sched_data = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_SCHED_DATA);
		ft_cap_fields_to_send.mac_security_modes_code = IS_ENABLED(CONFIG_DECT_MAC_SECURITY_ENABLE) ? DECT_MAC_SECURITY_SUPPORT_MODE1 : DECT_MAC_SECURITY_SUPPORT_NONE;

		dect_mac_phy_capability_set_t *ft_phy_set0 = &ft_cap_fields_to_send.phy_variants[0];
		ft_phy_set0->dlc_service_type_support_code = CONFIG_DECT_MAC_CAP_DLC_SERVICE_SUPPORT_CODE;
		ft_phy_set0->rx_for_tx_diversity_code = CONFIG_DECT_MAC_CAP_RX_TX_DIVERSITY_CODE;
		ft_phy_set0->mu_value = ctx->own_phy_params.mu;
		ft_phy_set0->beta_value = ctx->own_phy_params.beta;
		ft_phy_set0->max_nss_for_rx_code = CONFIG_DECT_MAC_CAP_MAX_NSS_RX_CODE;
		ft_phy_set0->max_mcs_code = CONFIG_DECT_MAC_CAP_MAX_MCS_CODE;
		ft_phy_set0->harq_soft_buffer_size_code = CONFIG_DECT_MAC_CAP_HARQ_BUFFER_CODE;
		ft_phy_set0->num_harq_processes_code = CONFIG_DECT_MAC_CAP_NUM_HARQ_PROC_CODE;
		ft_phy_set0->harq_feedback_delay_code = CONFIG_DECT_MAC_CAP_HARQ_FEEDBACK_DELAY_CODE;
		ft_phy_set0->supports_dect_delay = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_DECT_DELAY);
		ft_phy_set0->supports_half_duplex = IS_ENABLED(CONFIG_DECT_MAC_CAP_SUPPORTS_HALF_DUPLEX);

		/* Populate Resource Allocation IE */
		res_alloc_fields_to_send.alloc_type_val = RES_ALLOC_TYPE_BIDIR;
		res_alloc_fields_to_send.id_present = true;
		res_alloc_fields_to_send.short_rd_id_val = pt_tx_short_rd_id;
		res_alloc_fields_to_send.repeat_val = RES_ALLOC_REPEAT_FRAMES;
		res_alloc_fields_to_send.sfn_present = true;
		res_alloc_fields_to_send.res1_is_9bit_subslot = (pt_peer_ctx->peer_mu > 2);
		res_alloc_fields_to_send.res2_is_9bit_subslot = (pt_peer_ctx->peer_mu > 2);
		res_alloc_fields_to_send.start_subslot_val_res1 = (peer_slot_idx * 4 + 10) % MAX_SUBSLOTS_IN_FRAME_NOMINAL;
		res_alloc_fields_to_send.length_val_res1 = 2 - 1;
		res_alloc_fields_to_send.start_subslot_val_res2 = (peer_slot_idx * 4 + 12) % MAX_SUBSLOTS_IN_FRAME_NOMINAL;
		res_alloc_fields_to_send.length_val_res2 = 2 - 1;
		res_alloc_fields_to_send.repetition_value = CONFIG_DECT_MAC_FT_DEFAULT_SCHEDULE_REPEAT_FRAMES;
		res_alloc_fields_to_send.validity_value = CONFIG_DECT_MAC_FT_DEFAULT_SCHEDULE_VALIDITY_FRAMES;
		res_alloc_fields_to_send.sfn_val = (ctx->role_ctx.ft.sfn + CONFIG_DECT_MAC_FT_SCHEDULE_START_SFN_OFFSET) & 0xFF;

		ft_send_association_response_action(peer_slot_idx, &resp_fields, &ft_cap_fields_to_send, &res_alloc_fields_to_send);
	} else {
		ft_send_association_response_action(peer_slot_idx, &resp_fields, NULL, NULL);
	}
}




// Brief Overview: This is the complete ft_send_association_response_action function.
// It ensures all fields of dect_mac_assoc_resp_ie_t are populated correctly,
// including conditional HARQ parameters (if harq_mod_present is true),
// accepted Flow IDs (if number_of_flows_accepted is 1-6), and Group ID/Tag
// (if group_assignment_active is true), before serialization.

static void ft_send_association_response_action(
	int peer_slot_idx,
	const dect_mac_assoc_resp_ie_t *resp_fields,
	const dect_mac_rd_capability_ie_t *ft_cap_fields,
	const dect_mac_resource_alloc_ie_fields_t *res_alloc_fields)
{
	dect_mac_context_t *ctx = get_mac_context();

    // Add a guard to prevent crashes
    if (peer_slot_idx < 0 || peer_slot_idx >= MAX_PEERS_PER_FT) {
        LOG_ERR("ASSOC_RESP_SEND: Invalid peer_slot_idx %d. Aborting.", peer_slot_idx);
        return;
    }

	dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];
	uint8_t sdu_area_buf[256];
	int sdu_area_len_built_bytes = 0;
	size_t len_of_muxed_sec_ie_for_crypto_calc = 0;
	int ret;

	bool secure_this_response = (resp_fields->ack_nack && pt_peer_ctx->is_secure &&
				     ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx]);

	if (secure_this_response) {
		uint8_t ft_key_index_for_pt = pt_peer_ctx->current_key_index_for_peer;
		uint8_t sec_iv_type_for_assoc_resp = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
		int ie_len_sec_info = build_mac_security_info_ie_muxed(
			sdu_area_buf, sizeof(sdu_area_buf), 0, ft_key_index_for_pt,
			sec_iv_type_for_assoc_resp, ctx->hpc);
		if (ie_len_sec_info < 0) {
			LOG_ERR("FT_ASSOC_RESP: Build MAC Sec Info IE failed: %d", ie_len_sec_info);
			return;
		}
		sdu_area_len_built_bytes += ie_len_sec_info;
		len_of_muxed_sec_ie_for_crypto_calc = ie_len_sec_info;
	}

	int resp_ies_len = build_assoc_resp_sdu_area_content(
		sdu_area_buf, sizeof(sdu_area_buf), sdu_area_len_built_bytes, resp_fields,
		ft_cap_fields, res_alloc_fields);

	if (resp_ies_len < 0) {
		LOG_ERR("FT_ASSOC_RESP: Failed to build SDU area content: %d", resp_ies_len);
		return;
	}
	sdu_area_len_built_bytes += resp_ies_len;

	dect_mac_header_type_octet_t hdr_type_octet;
	hdr_type_octet.version = 0;
	hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST;
	hdr_type_octet.mac_security = secure_this_response ? MAC_SECURITY_USED_WITH_IE : MAC_SECURITY_NONE;

	dect_mac_unicast_header_t common_hdr;
	increment_psn_and_hpc(ctx);
	common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
	common_hdr.sequence_num_low = ctx->psn & 0xFF;
	common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
	common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(pt_peer_ctx->long_rd_id);

    mac_sdu_t *pdu_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
    if (!pdu_sdu) {
        LOG_ERR("FT_REJECT_SEND: Failed to alloc PDU buffer.");
        return;
    }

	uint16_t pdu_len;
	ret = dect_mac_phy_ctrl_assemble_final_pdu(pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE, &hdr_type_octet,
						   &common_hdr, sizeof(common_hdr), sdu_area_buf,
						   (size_t)sdu_area_len_built_bytes, &pdu_len);
	if (ret != 0) {
		dect_mac_api_buffer_free(pdu_sdu);
		return;
	}

	uint16_t final_tx_pdu_len = pdu_len;

	if (secure_this_response) {
		uint8_t iv[16];
		security_build_iv(iv, ctx->own_long_rd_id, pt_peer_ctx->long_rd_id, ctx->hpc, ctx->psn);

        uint8_t *pdu_data_ptr = pdu_sdu->data;
		uint8_t *mic_calc_start = pdu_data_ptr + sizeof(dect_mac_header_type_octet_t);
		size_t mic_calc_len = sizeof(common_hdr) + sdu_area_len_built_bytes;
		uint8_t *mic_loc = pdu_data_ptr + pdu_len;

		if (pdu_len + 5 > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
			dect_mac_api_buffer_free(pdu_sdu);
			return;
		}
		ret = security_calculate_mic(mic_calc_start, mic_calc_len, ctx->role_ctx.ft.peer_integrity_keys[peer_slot_idx], mic_loc);
		if (ret != 0) {
			dect_mac_api_buffer_free(pdu_sdu);
			return;
		}
		final_tx_pdu_len = pdu_len + 5;
		uint8_t *enc_start = mic_calc_start + sizeof(common_hdr) + len_of_muxed_sec_ie_for_crypto_calc;
		size_t enc_len = (sdu_area_len_built_bytes - len_of_muxed_sec_ie_for_crypto_calc) + 5;
		ret = security_crypt_payload(enc_start, enc_len, ctx->role_ctx.ft.peer_cipher_keys[peer_slot_idx], iv, true);
		if (ret != 0) {
			dect_mac_api_buffer_free(pdu_sdu);
			return;
		}
	}

		// uint32_t phy_op_handle = sys_rand32_get();
	uint32_t phy_op_handle;
	sys_rand_get(&phy_op_handle, sizeof(uint32_t));
	ctx->role_ctx.ft.last_assoc_resp_pt_short_id = pt_peer_ctx->short_rd_id;

	ret = dect_mac_phy_ctrl_start_tx_assembled(
		ctx->role_ctx.ft.operating_carrier, pdu_sdu->data, final_tx_pdu_len,
		pt_peer_ctx->short_rd_id, false, phy_op_handle, PENDING_OP_FT_ASSOC_RESP, true, 0,
		ctx->own_phy_params.mu, NULL);

    // 4. Free the buffer without needing a cast
    dect_mac_api_buffer_free(pdu_sdu);

	if (ret == 0) {
		LOG_INF("FT_SM: Association Response (%s) TX scheduled to PT 0x%04X (Hdl %u). Secure: %s",
			resp_fields->ack_nack ? "ACCEPT" : "REJECT", pt_peer_ctx->short_rd_id, phy_op_handle,
			secure_this_response ? "Yes" : "No");
		if (resp_fields->ack_nack) {
			pt_peer_ctx->is_fully_identified = true;
		}
	} else {
		LOG_ERR("FT_SM: Failed to schedule TX to PT 0x%04X: %d", pt_peer_ctx->short_rd_id, ret);
		if (resp_fields->ack_nack) {
			pt_peer_ctx->is_valid = false;
			ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx] = false;
		}
	}
}


uint16_t dect_mac_core_get_short_id_for_long_id(uint32_t long_rd_id)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->role == MAC_ROLE_FT) {
		for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
			if (ctx->role_ctx.ft.connected_pts[i].is_valid &&
			    ctx->role_ctx.ft.connected_pts[i].long_rd_id == long_rd_id) {
				return ctx->role_ctx.ft.connected_pts[i].short_rd_id;
			}
		}
	} else { /* PT Role */
		if (ctx->role_ctx.pt.associated_ft.is_valid &&
		    ctx->role_ctx.pt.associated_ft.long_rd_id == long_rd_id) {
			return ctx->role_ctx.pt.associated_ft.short_rd_id;
		}
	}

	return 0; /* Not found */
}


/**
 * @brief Core FT scheduler. Finds the next imminent scheduled event (TX or RX) and executes it.
 */
void ft_service_schedules(void)
{
	dect_mac_context_t *ctx = get_mac_context();
	uint64_t next_op_time = UINT64_MAX;
	int next_op_peer_idx = -1;
	bool next_op_is_tx = false;
	bool next_op_is_group_ul = false;

	if (ctx->pending_op_type != PENDING_OP_NONE) {
		return; /* An operation is already in flight */
	}

	/* Find the most imminent scheduled event across all connected PTs */
	for (int i = 0; i < MAX_PEERS_PER_FT; i++) {
		if (!ctx->role_ctx.ft.connected_pts[i].is_valid) {
			continue;
		}
		dect_mac_peer_info_t *pt_peer = &ctx->role_ctx.ft.connected_pts[i];
		dect_mac_schedule_t *unicast_sched = &ctx->role_ctx.ft.peer_schedules[i];

		/* Check for pending unicast DL data */
		if (unicast_sched->is_active && unicast_sched->dl_duration_subslots > 0) {
			update_next_occurrence(ctx, unicast_sched, ctx->last_known_modem_time);
			if (unicast_sched->is_active && unicast_sched->next_occurrence_modem_time < next_op_time) {
				next_op_time = unicast_sched->next_occurrence_modem_time;
				next_op_peer_idx = i;
				next_op_is_tx = true;
				next_op_is_group_ul = false;
			}
		}

		/* Check for expected unicast UL data */
		if (unicast_sched->is_active && unicast_sched->ul_duration_subslots > 0) {
			update_next_occurrence(ctx, unicast_sched, ctx->last_known_modem_time);
			if (unicast_sched->is_active && unicast_sched->next_occurrence_modem_time < next_op_time) {
				next_op_time = unicast_sched->next_occurrence_modem_time;
				next_op_peer_idx = i;
				next_op_is_tx = false;
				next_op_is_group_ul = false;
			}
		}

		/* Check for expected group UL data */
		// dect_mac_schedule_t *group_sched_template = &ctx->role_ctx.ft.group_schedule_fields;
		dect_mac_schedule_t *group_sched_template = &ctx->role_ctx.ft.group_schedule;
		if (group_sched_template->is_active && pt_peer->group_id != 0) {
			uint64_t repetition_ticks = 0;
			uint32_t frame_ticks = (uint32_t)FRAME_DURATION_MS_NOMINAL * (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
			if (group_sched_template->repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP) {
				repetition_ticks = (uint64_t)group_sched_template->repetition_value * frame_ticks;
			} else {
				/* Use the specific peer's mu for subslot-based group repetition timing */
				repetition_ticks = (uint64_t)group_sched_template->repetition_value * get_subslot_duration_ticks_for_mu(pt_peer->phy_variants[0].mu_value);
			}

			uint64_t pt_group_slot_time = group_sched_template->next_occurrence_modem_time + (pt_peer->resource_tag * repetition_ticks);
			
			if (pt_group_slot_time < next_op_time) {
				next_op_time = pt_group_slot_time;
				next_op_peer_idx = i;
				next_op_is_tx = false;
				next_op_is_group_ul = true;
			}
		}
	}

	if (next_op_peer_idx == -1) {
		return; /* No scheduled events */
	}

	uint32_t min_prep_time = modem_us_to_ticks(
		ctx->phy_latency.idle_to_active_tx_us +
			ctx->phy_latency.scheduled_operation_startup_us,
		NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

	if (next_op_time < (ctx->last_known_modem_time + min_prep_time)) {
		return; /* Too soon to schedule */
	}

	if (next_op_is_tx) {
		dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[next_op_peer_idx];
		dect_mac_schedule_t *sched = &ctx->role_ctx.ft.peer_schedules[next_op_peer_idx];
		dect_mac_peer_tx_dlist_set_t *dlists = &ctx->role_ctx.ft.peer_tx_data_dlists[next_op_peer_idx];
		sys_dlist_t *dlist_array[] = {&dlists->high_priority_dlist, &dlists->reliable_data_dlist, &dlists->best_effort_dlist};

		union nrf_modem_dect_phy_feedback feedback_to_send = {0};
		bool feedback_is_pending = false;

		if (pt_peer_ctx->num_pending_feedback_items > 0) {
			feedback_is_pending = true;
			feedback_to_send.format1.format = NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1;
			feedback_to_send.format1.harq_process_number0 = pt_peer_ctx->pending_feedback_to_send[0].harq_process_num_for_peer;
			feedback_to_send.format1.transmission_feedback0 = pt_peer_ctx->pending_feedback_to_send[0].is_ack;
		}

		/* Try to find a new SDU to send from the queues */
		for (int j = 0; j < MAC_FLOW_COUNT; j++) {
			sys_dnode_t *node = sys_dlist_get(dlist_array[j]);
			if (node) {
				mac_sdu_t *sdu = CONTAINER_OF(node, mac_sdu_t, node);
				if (does_sdu_fit_schedule(ctx, sdu, sched, next_op_peer_idx)) {
					/* Found an SDU that fits. Call the public data path API to send it. */
					int ret = dect_mac_data_path_send_new_sdu(
						sdu, (mac_flow_id_t)j, next_op_peer_idx,
						sched->channel, next_op_time,
						feedback_is_pending ? &feedback_to_send : NULL);

					if (ret == -EBUSY) {
						/* Data path had no free HARQ process. Re-queue and try later. */
						LOG_WRN("FT_SCHED: No free HARQ process, re-queueing SDU.");
						sys_dlist_prepend(dlist_array[j], &sdu->node);
					} else if (ret == 0 && feedback_is_pending) {
						/* Successfully sent and piggybacked feedback */
						memset(pt_peer_ctx->pending_feedback_to_send, 0, sizeof(pt_peer_ctx->pending_feedback_to_send));
						pt_peer_ctx->num_pending_feedback_items = 0;
					}
					return; /* Service one SDU per call */
				} else {
					LOG_WRN("FT_SCHED_TX: SDU (len %u) for PT %d does not fit schedule. Re-queueing.",
						sdu->len, next_op_peer_idx);
					sys_dlist_prepend(dlist_array[j], &sdu->node);
				}
			}
		}

		/* If we have pending feedback but found no data SDU to send, send a feedback-only PDU */
		if (feedback_is_pending) {
			LOG_INF("FT_SCHED_TX: Sending standalone feedback to PT %d.", next_op_peer_idx);
			/* Call the sender with a NULL SDU to indicate feedback-only */
			int ret = dect_mac_data_path_send_new_sdu(
				NULL, MAC_FLOW_HIGH_PRIORITY, next_op_peer_idx,
				sched->channel, next_op_time, &feedback_to_send);

			if (ret == 0) {
				memset(pt_peer_ctx->pending_feedback_to_send, 0, sizeof(pt_peer_ctx->pending_feedback_to_send));
				pt_peer_ctx->num_pending_feedback_items = 0;
			}
		}
	} else {
		/* Schedule an RX operation for the expected UL data */
		// Both expressions now result in a `dect_mac_schedule_t *`
		dect_mac_schedule_t *sched = next_op_is_group_ul ? 
			&ctx->role_ctx.ft.group_schedule : 
			&ctx->role_ctx.ft.peer_schedules[next_op_peer_idx];

		uint32_t rx_duration_ticks =
			sched->ul_duration_subslots *
			get_subslot_duration_ticks_for_mu(
				ctx->role_ctx.ft.connected_pts[next_op_peer_idx].phy_variants[0].mu_value);

		LOG_INF("FT_SCHED_RX: Scheduling UL listen for PT %d (0x%04X) at %llu, dur %u ticks. Group: %d",
			next_op_peer_idx, ctx->role_ctx.ft.connected_pts[next_op_peer_idx].short_rd_id,
			next_op_time, rx_duration_ticks, next_op_is_group_ul);

		dect_mac_phy_ctrl_start_rx(
			sched->channel, rx_duration_ticks, NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT,
			sys_rand32_get(), ctx->own_short_rd_id, next_op_is_group_ul ? PENDING_OP_FT_GROUP_RX : PENDING_OP_FT_DATA_RX);
	}
}		
// 		dect_mac_peer_info_t *pt_peer_ctx = &ctx->role_ctx.ft.connected_pts[next_op_peer_idx];
// 		union nrf_modem_dect_phy_feedback feedback_to_send = {0};
// 		bool feedback_is_pending = false;

// 		if (pt_peer_ctx->num_pending_feedback_items > 0) {
// 			feedback_is_pending = true;
// 			feedback_to_send.format1.format = NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1;
// 			feedback_to_send.format1.harq_process_number0 = pt_peer_ctx->pending_feedback_to_send[0].harq_process_num_for_peer;
// 			feedback_to_send.format1.transmission_feedback0 = pt_peer_ctx->pending_feedback_to_send[0].is_ack;
// 		}

// 		/* Prioritize HARQ retransmissions */
// 		for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
// 			dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
// 			if (harq_p->is_active && harq_p->needs_retransmission && harq_p->peer_short_id_for_ft_dl == pt_peer_ctx->short_rd_id) {
// 				LOG_INF("FT_SCHED_TX: Re-transmitting HARQ proc %d to PT %d.", i, next_op_peer_idx);
// 				int ret = send_data_mac_sdu_via_phy_internal(ctx, harq_p->sdu, i, true, harq_p->scheduled_carrier, next_op_peer_idx, next_op_time, harq_p->flow_id, feedback_is_pending ? &feedback_to_send : NULL);
// 				if (ret == 0 && feedback_is_pending) {
// 					memset(pt_peer_ctx->pending_feedback_to_send, 0, sizeof(pt_peer_ctx->pending_feedback_to_send));
// 					pt_peer_ctx->num_pending_feedback_items = 0;
// 				}
// 				return;
// 			}
// 		}

// 		/* Service new SDUs */
// 		int free_harq_idx = find_free_harq_tx_process(ctx);
// 		if (free_harq_idx != -1) {
// 			dect_mac_schedule_t *sched = &ctx->role_ctx.ft.peer_schedules[next_op_peer_idx];
// 			// REPLACE THIS
// 			// dect_mac_peer_tx_fifo_set_t *fifos = &ctx->role_ctx.ft.peer_tx_data_fifos[next_op_peer_idx];
// 			// struct k_fifo *fifo_array[] = {&fifos->high_priority_fifo, &fifos->reliable_data_fifo, &fifos->best_effort_fifo};
// 			// --- USE THE CORRECT DLIST MEMBER NAME ---
// 			dect_mac_peer_tx_dlist_set_t *dlists = &ctx->role_ctx.ft.peer_tx_data_dlists[next_op_peer_idx];
// 			sys_dlist_t *dlist_array[] = {&dlists->high_priority_dlist, &dlists->reliable_data_dlist, &dlists->best_effort_dlist};

// 			for (int j = 0; j < MAC_FLOW_COUNT; j++) {
// 				// mac_sdu_t *sdu = k_fifo_get(fifo_array[j], K_NO_WAIT);
// 				// if (sdu) {
// 				// --- USE DLIST GET ---
// 				sys_dnode_t *node = sys_dlist_get(dlist_array[j]);
// 				if (node) {
// 					mac_sdu_t *sdu = CONTAINER_OF(node, mac_sdu_t, node);
// 					if (does_sdu_fit_schedule(ctx, sdu, sched, next_op_peer_idx)) {
// 						/* Found an SDU that fits. Call the public data path API to send it. */
// 						int ret = dect_mac_data_path_send_new_sdu(
// 							sdu, (mac_flow_id_t)j, next_op_peer_idx,
// 							sched->channel, next_op_time,
// 							feedback_is_pending ? &feedback_to_send : NULL);

// 						if (ret == -EBUSY) {
// 							/* Data path had no free HARQ process. Re-queue and try later. */
// 							LOG_WRN("FT_SCHED: No free HARQ process, re-queueing SDU.");
// 							sys_dlist_prepend(dlist_array[j], &sdu->node);
// 						} else if (ret == 0 && feedback_is_pending) {
// 							/* Successfully sent and piggybacked feedback */
// 							memset(pt_peer_ctx->pending_feedback_to_send, 0, sizeof(pt_peer_ctx->pending_feedback_to_send));
// 							pt_peer_ctx->num_pending_feedback_items = 0;
// 						}
// 						return; /* Service one SDU per call */
// 					} else {
// 						LOG_WRN("FT_SCHED_TX: SDU (len %u) for PT %d does not fit schedule. Re-queueing.",
// 							sdu->len, next_op_peer_idx);
// 						sys_dlist_prepend(dlist_array[j], &sdu->node);
// 					}
// 				}
// 			}
// 		}
// 		/* If we have pending feedback but found no data SDU to send, send a feedback-only PDU */
// 		if (feedback_is_pending) {
// 			LOG_INF("FT_SCHED_TX: Sending standalone feedback to PT %d.", next_op_peer_idx);
// 			/* Call the sender with a NULL SDU to indicate feedback-only */
// 			int ret = dect_mac_data_path_send_new_sdu(
// 				NULL, MAC_FLOW_HIGH_PRIORITY, next_op_peer_idx,
// 				sched->channel, next_op_time, &feedback_to_send);

// 			if (ret == 0) {
// 				memset(pt_peer_ctx->pending_feedback_to_send, 0, sizeof(pt_peer_ctx->pending_feedback_to_send));
// 				pt_peer_ctx->num_pending_feedback_items = 0;
// 			}
// 		}
// 	} else {
// 		/* Schedule an RX operation for the expected UL data */
// 		dect_mac_schedule_t *sched = next_op_is_group_ul ? &ctx->role_ctx.ft.group_schedule_fields : &ctx->role_ctx.ft.peer_schedules[next_op_peer_idx];
// 		uint32_t rx_duration_ticks =
// 			sched->ul_duration_subslots *
// 			get_subslot_duration_ticks_for_mu(
// 				ctx->role_ctx.ft.connected_pts[next_op_peer_idx].phy_variants[0].mu_value);

// 		LOG_INF("FT_SCHED_RX: Scheduling UL listen for PT %d (0x%04X) at %llu, dur %u ticks. Group: %d",
// 			next_op_peer_idx, ctx->role_ctx.ft.connected_pts[next_op_peer_idx].short_rd_id,
// 			next_op_time, rx_duration_ticks, next_op_is_group_ul);

// 		// dect_mac_phy_ctrl_start_rx(
// 		// 	sched->channel, rx_duration_ticks, NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT,
// 		// 	sys_rand32_get(), ctx->own_short_rd_id, next_op_is_group_ul ? PENDING_OP_FT_GROUP_RX : PENDING_OP_FT_DATA_RX);
// 		uint32_t random_handle;
// 		sys_rand_get(&random_handle, sizeof(random_handle));

// 		dect_mac_phy_ctrl_start_rx(
// 			sched->channel, rx_duration_ticks, NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT,
// 			random_handle, ctx->own_short_rd_id, next_op_is_group_ul ? PENDING_OP_FT_GROUP_RX : PENDING_OP_FT_DATA_RX);
// 	}
// }


static void ft_send_reject_response_action(uint32_t pt_long_id, uint16_t pt_short_id,
                                           const dect_mac_assoc_resp_ie_t *reject_fields)
{
    dect_mac_context_t *ctx = get_mac_context();
    uint8_t sdu_area_buf[32];

    int sdu_area_len = build_assoc_resp_sdu_area_content(
        sdu_area_buf, sizeof(sdu_area_buf), 0, reject_fields, NULL, NULL);

    if (sdu_area_len < 0) {
        LOG_ERR("FT_REJECT_SEND: Failed to build reject SDU area: %d", sdu_area_len);
        return;
    }

    dect_mac_header_type_octet_t hdr_type = {.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST};
    dect_mac_unicast_header_t common_hdr;
    increment_psn_and_hpc(ctx);
    common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((ctx->psn >> 8) & 0x0F, 1);
    common_hdr.sequence_num_low = ctx->psn & 0xFF;
    common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
    common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(pt_long_id);

    /* --- CORRECTED BUFFER HANDLING --- */
    // 1. Declare the variable with the correct type: mac_sdu_t*
    mac_sdu_t *pdu_sdu = dect_mac_api_buffer_alloc(K_NO_WAIT);
    if (!pdu_sdu) {
        LOG_ERR("FT_REJECT_SEND: Failed to alloc PDU buffer.");
        return;
    }

    uint16_t pdu_len;
    // 2. Pass the correct member (.data) to the assembly function
    int ret = dect_mac_phy_ctrl_assemble_final_pdu(
        pdu_sdu->data, CONFIG_DECT_MAC_PDU_MAX_SIZE, &hdr_type, &common_hdr,
        sizeof(common_hdr), sdu_area_buf, (size_t)sdu_area_len, &pdu_len);

    if (ret == 0) {
        uint32_t phy_op_handle;
        sys_rand_get(&phy_op_handle, sizeof(uint32_t));
        
        // 3. Pass the correct member (.data) to the TX function
        dect_mac_phy_ctrl_start_tx_assembled(
            ctx->role_ctx.ft.operating_carrier, pdu_sdu->data, pdu_len, pt_short_id,
            false, phy_op_handle, PENDING_OP_FT_ASSOC_RESP, true, 0,
            ctx->own_phy_params.mu, NULL);
    }

    // 4. Free the buffer without needing a cast
    dect_mac_api_buffer_free(pdu_sdu);
}
/**/
