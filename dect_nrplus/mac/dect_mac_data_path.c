/* dect_mac/dect_mac_data_path.c */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/random/random.h>

#include <nrf_modem_dect_phy.h>

#include <mac/dect_mac_data_path.h>
#include <mac/dect_mac_main.h>
#include <mac/dect_mac_core.h>      // For get_mac_context(), increment_psn_and_hpc()
#include <mac/dect_mac_context.h>   // For dect_mac_context_t access and constants
#include <mac/dect_mac_phy_ctrl.h>  // For dect_mac_phy_ctrl_start_tx_assembled, _assemble_final_pdu, calculate_pcc_params
#include <mac/dect_mac_pdu.h>       // For IE_TYPE_USER_DATA_FLOW_1, MAC Common Headers, MAC Hdr Type, parse_mac_mux_header
#include <mac/dect_mac_main_dispatcher.h> // For string utils for logging, mac_event_msgq
#include <mac/dect_mac_api.h>       // For dect_mac_api_buffer_free, mac_sdu_t, mac_tx_fifos (generic for PT), g_mac_sdu_slab
#include <mac/dect_mac_security.h>  // For security_build_iv, _calculate_mic, _crypt_payload
#include <mac/dect_mac_phy_tbs_tables.h> // For TBS lookup tables
#include <mac/dect_mac_timeline_utils.h>

LOG_MODULE_REGISTER(dect_mac_data_path, CONFIG_DECT_MAC_DATA_PATH_LOG_LEVEL);


// Declare the pointer as external. The linker will resolve this to the
// variable defined in dect_mac_api.c.
extern sys_dlist_t *g_dlc_rx_sdu_dlist_ptr;

// Function pointer to the DLC's callback for reporting final TX status.
static dlc_tx_status_cb_t g_dlc_status_callback = NULL;

// Define a static function pointer to hold the scheduler hook.
static void (*g_role_specific_scheduler_hook)(void) = NULL;

/* --- The registration function implementation --- */
void dect_mac_data_path_register_scheduler_hook(void (*hook)(void))
{
    g_role_specific_scheduler_hook = hook;
}



/**
 * @brief Registers the DLC's callback for TX status reports.
 *
 * The DLC layer calls this during its initialization to provide the MAC data
 * path with a function to call when a reportable SDU transmission is complete.
 *
 * @param cb The DLC's callback handler function.
 */
void dect_mac_data_path_register_dlc_callback(dlc_tx_status_cb_t cb)
{
	g_dlc_status_callback = cb;
	if (cb) {
		LOG_INF("DLC TX status callback registered with MAC Data Path.");
	} else {
		LOG_WRN("DLC TX status callback unregistered (set to NULL).");
	}
}


uint32_t get_tbs_for_schedule(uint8_t num_subslots, uint8_t mcs_code, uint8_t mu_code,
				     uint8_t beta_code)
{
	if (num_subslots == 0 || num_subslots > TBS_MAX_SUB_SLOTS_J) {
		LOG_WRN("TBS_GET: Invalid num_subslots %u", num_subslots);
		return 0;
	}
	if (mcs_code > TBS_MAX_MCS_INDEX) {
		LOG_WRN("TBS_GET: Invalid mcs_code %u", mcs_code);
		return 0;
	}

	const uint16_t (*selected_tbs_table)[TBS_MAX_SUB_SLOTS_J];

	if (beta_code != 0) {
		LOG_WRN("TBS_GET: TBS lookup for beta_code %u not yet implemented. Using beta=1 table.",
			beta_code);
	}

	switch (mu_code) {
	case 0: /* mu = 1 */
		selected_tbs_table = tbs_single_slot_mu1_beta1;
		break;
	case 1: /* mu = 2 */
		selected_tbs_table = tbs_single_slot_mu2_beta1;
		break;
	case 2: /* mu = 4 */
		selected_tbs_table = tbs_single_slot_mu4_beta1;
		break;
	default:
		LOG_ERR("TBS_GET: Unsupported mu_code=%u. Using default mu=1 table.", mu_code);
		selected_tbs_table = tbs_single_slot_mu1_beta1;
		break;
	}

	uint32_t tbs_bits = selected_tbs_table[mcs_code][num_subslots - 1];

	return tbs_bits / 8;
}

/**
 * @brief Checks if a given SDU will fit into a scheduled allocation.
 *
 * @param ctx Pointer to the MAC context.
 * @param sdu Pointer to the SDU to check.
 * @param schedule Pointer to the schedule for the allocation.
 * @param peer_slot_idx For FT role, the index of the target peer. -1 for PT role.
 * @return True if the SDU fits, false otherwise.
 */
bool does_sdu_fit_schedule(dect_mac_context_t *ctx, mac_sdu_t *sdu,
				  dect_mac_schedule_t *schedule, int peer_slot_idx)
{
	size_t overhead = 0;

	overhead += sizeof(dect_mac_header_type_octet_t);
	overhead += sizeof(dect_mac_unicast_header_t);
	overhead += 3; /* Max MUX header size for user data IE */

	bool is_secure = false;
	dect_mac_peer_info_t *peer_ctx = NULL;

	if (ctx->role == MAC_ROLE_PT) {
		peer_ctx = &ctx->role_ctx.pt.associated_ft;
		is_secure = peer_ctx->is_secure && ctx->keys_provisioned;
	} else if (peer_slot_idx != -1) {
		peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_slot_idx];
		is_secure = peer_ctx->is_secure &&
			    ctx->role_ctx.ft.keys_provisioned_for_peer[peer_slot_idx];
	}

	if (is_secure) {
		overhead += 5; /* MIC */
        /* Simplified: Assume MAC Sec Info IE is not sent for this check. */
	}

	size_t total_pdu_size = sdu->len + overhead;
    
    // 2. Get available TBS from schedule
	uint8_t num_subslots = (schedule->alloc_type == RES_ALLOC_TYPE_UPLINK ||
				(ctx->role == MAC_ROLE_PT && schedule->alloc_type == RES_ALLOC_TYPE_BIDIR))
				       ? schedule->ul_duration_subslots
				       : schedule->dl_duration_subslots;

	uint8_t mcs_code = ctx->config.default_data_mcs_code;
	uint8_t mu_code = 0;
	uint8_t beta_code = 0;

	if (peer_ctx && peer_ctx->peer_phy_params_known) {
		mu_code = peer_ctx->peer_mu;
		beta_code = peer_ctx->peer_beta;
	} else {
		LOG_WRN("FIT_CHECK: Peer PHY params unknown. Using own params as fallback.");
		mu_code = ctx->own_phy_params.mu;
		beta_code = ctx->own_phy_params.beta;
	}

	uint32_t available_bytes = get_tbs_for_schedule(num_subslots, mcs_code, mu_code, beta_code);

	if (total_pdu_size > available_bytes) {
		LOG_WRN("FIT_CHECK: SDU (len %u, total ~%zu) does NOT fit in schedule (slots %u, mcs %u, tbs %u bytes)",
			sdu->len, total_pdu_size, num_subslots, mcs_code, available_bytes);
		return false;
	}

	LOG_DBG("FIT_CHECK: SDU (len %u, total ~%zu) fits in schedule (slots %u, mcs %u, tbs %u bytes)",
		sdu->len, total_pdu_size, num_subslots, mcs_code, available_bytes);
	return true;
}




// External FIFOs and slab (defined in dect_mac_api.c)
extern struct k_fifo * const mac_tx_fifos[]; // Generic TX FIFOs (used by PT for UL)
extern struct k_fifo *g_dlc_rx_sdu_fifo_ptr; // Pointer to DLC's RX FIFO
extern struct k_mem_slab g_mac_sdu_slab;     // For SDU buffers used by MAC API and internal PDU construction
extern struct k_msgq mac_event_msgq;         // For HARQ timer expiry events


/* --- Helper Functions --- */

uint32_t get_subslot_duration_ticks(dect_mac_context_t *ctx) {
    // ETSI TS 103 636-3, section 4: 1 subslot = 5 OFDM symbols.
    // NRF_MODEM_DECT_SYMBOL_DURATION is duration of 1 symbol in modem ticks (defined in nrf_modem_dect_phy.h)
    // This should be mu-independent as subslot is defined in terms of OFDM symbols,
    // and NRF_MODEM_DECT_SYMBOL_DURATION should be for the base numerology symbol.
    // If NRF_MODEM_DECT_SYMBOL_DURATION changes with mu, this needs ctx->phy_caps.mu.
    // For now, assuming NRF_MODEM_DECT_SYMBOL_DURATION is fixed for the base.
    ARG_UNUSED(ctx); // ctx might be needed if mu affects symbol duration reporting by PHY lib.
    return NRF_MODEM_DECT_SYMBOL_DURATION * 5;
}


// --- Initialization and HARQ Management Functions ---
void dect_mac_data_path_init(void) {
    dect_mac_context_t* ctx = get_mac_context();
    if (!ctx) {
        LOG_ERR("DATA_PATH_INIT: MAC Context is NULL. Cannot initialize HARQ.");
        return;
    }
    for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
        k_timer_init(&ctx->harq_tx_processes[i].retransmission_timer,
                     dect_mac_data_path_harq_timer_expired, NULL); // Expiry function
        // Store HARQ process index in timer's user_data for identification in callback
        ctx->harq_tx_processes[i].retransmission_timer.user_data = (void*)((uintptr_t)i);
        ctx->harq_tx_processes[i].is_active = false;
        ctx->harq_tx_processes[i].needs_retransmission = false;
        ctx->harq_tx_processes[i].sdu = NULL;
        ctx->harq_tx_processes[i].tx_attempts = 0;
        ctx->harq_tx_processes[i].redundancy_version = 0;
        ctx->harq_tx_processes[i].original_hpc = 0;
        ctx->harq_tx_processes[i].original_psn = 0;
        ctx->harq_tx_processes[i].peer_short_id_for_ft_dl = 0;
        ctx->harq_tx_processes[i].scheduled_carrier = 0;
        ctx->harq_tx_processes[i].scheduled_tx_start_time = 0;

    }
    LOG_INF("MAC Data Path Initialized (HARQ Timers and Processes set up).");
}

static int find_free_harq_tx_process(dect_mac_context_t* ctx) {
    if (!ctx) return -1;
    for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
        if (!ctx->harq_tx_processes[i].is_active) {
            return i;
        }
    }
    LOG_DBG("HARQ_ALLOC: No free HARQ TX processes available.");
    return -1; // No free process
}

void dect_mac_data_path_harq_timer_expired(struct k_timer *timer_id) {
    if (!timer_id) return;
    uintptr_t harq_idx_from_timer = (uintptr_t)timer_id->user_data;

    LOG_WRN("HARQ_TIMER: Timeout for HARQ process %u.", (unsigned int)harq_idx_from_timer);

    struct dect_mac_event_msg msg = {
        .type = MAC_EVENT_TIMER_EXPIRED_HARQ,
        // .modem_time_of_event = k_uptime_get(), // TODO: Use actual modem time if critical
        .data.timer_data.id = (int)harq_idx_from_timer
    };

    if (k_msgq_put(&mac_event_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("HARQ_TIMER: Failed to queue HARQ expiry for proc %u, msgq full.", (unsigned int)harq_idx_from_timer);
        // Critical: If event queue is full, the NACK action might be missed.
        // As a fallback, directly call the NACK handler. This breaks the strict event flow
        // but prevents losing the timeout information. This should be rare.
        dect_mac_context_t* ctx = get_mac_context(); // Get context again for safety
        if (ctx && harq_idx_from_timer < MAX_HARQ_PROCESSES &&
            ctx->harq_tx_processes[harq_idx_from_timer].is_active) {
            LOG_WRN("HARQ_TIMER: Directly handling NACK for proc %u due to full event queue.", (unsigned int)harq_idx_from_timer);
            dect_mac_data_path_handle_harq_nack_action((int)harq_idx_from_timer);
        }
    }
}

void dect_mac_data_path_handle_harq_ack_action(int harq_process_idx) {
    dect_mac_context_t* ctx = get_mac_context();
    if (!ctx || harq_process_idx < 0 || harq_process_idx >= MAX_HARQ_PROCESSES) {
        LOG_ERR("HARQ_ACK: Invalid context or process_idx %d", harq_process_idx);
        return;
    }
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_process_idx];

    if (harq_p->is_active) {
        LOG_INF("HARQ_ACK: ACK received for HARQ process %d (PSN: %u, Attempts: %u).",
                harq_process_idx, harq_p->original_psn, harq_p->tx_attempts);
        k_timer_stop(&harq_p->retransmission_timer);
        if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
            LOG_DBG("HARQ_ACK: Reporting success to DLC for SN %u.", harq_p->sdu->dlc_sn_for_status);
            g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, true);
        }        
        if (harq_p->sdu) {
            dect_mac_api_buffer_free(harq_p->sdu); // Free the SDU buffer
            harq_p->sdu = NULL;
        }
        // Reset process for reuse
        harq_p->is_active = false;
        harq_p->needs_retransmission = false;
        harq_p->tx_attempts = 0;
        harq_p->redundancy_version = 0;
        harq_p->original_hpc = 0;
        harq_p->original_psn = 0;
        harq_p->peer_short_id_for_ft_dl = 0;
        harq_p->scheduled_carrier = 0;
        harq_p->scheduled_tx_start_time = 0;
    } else {
        LOG_WRN("HARQ_ACK: Received for already inactive HARQ process %d.", harq_process_idx);
    }
}

void dect_mac_data_path_handle_harq_nack_action(int harq_process_idx) {
    dect_mac_context_t* ctx = get_mac_context();
     if (!ctx || harq_process_idx < 0 || harq_process_idx >= MAX_HARQ_PROCESSES) {
        LOG_ERR("HARQ_NACK: Invalid context or process_idx %d", harq_process_idx);
        return;
    }
    dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_process_idx];

    if (harq_p->is_active) {
        k_timer_stop(&harq_p->retransmission_timer); // Stop current ACK timeout timer

        /* For best-effort, any NACK/timeout is a permanent failure. No retransmissions. */
        if (harq_p->flow_id == MAC_FLOW_BEST_EFFORT) {
            LOG_INF("HARQ_NACK: Best-effort SDU (HARQ %d, PSN %u) failed. Discarding.",
                    harq_process_idx, harq_p->original_psn);
            if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
                g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, false);
            }
            if (harq_p->sdu) {
                dect_mac_api_buffer_free(harq_p->sdu);
                harq_p->sdu = NULL;
            }
            harq_p->is_active = false;
            return; /* Exit before retransmission logic */
        }        

        if (harq_p->tx_attempts >= MAX_HARQ_RETRIES) {
            LOG_ERR("HARQ_NACK: Max retries (%u) reached for HARQ process %d (PSN: %u). Discarding SDU.",
                    MAX_HARQ_RETRIES, harq_process_idx, harq_p->original_psn);
            if (harq_p->sdu && harq_p->sdu->dlc_status_report_required && g_dlc_status_callback) {
                LOG_DBG("HARQ_NACK: Reporting permanent failure to DLC for SN %u.", harq_p->sdu->dlc_sn_for_status);
                g_dlc_status_callback(harq_p->sdu->dlc_sn_for_status, false);
            }                    
            if (harq_p->sdu) {
                dect_mac_api_buffer_free(harq_p->sdu);
                harq_p->sdu = NULL;
            }
            // Reset process for reuse
            harq_p->is_active = false;
            harq_p->needs_retransmission = false;
            harq_p->tx_attempts = 0;
            harq_p->redundancy_version = 0;
        } else {
            // tx_attempts is incremented in send_data_mac_sdu_via_phy_internal before the actual reTX
            LOG_WRN("HARQ_NACK: NACK or Timeout for HARQ process %d (PSN: %u, Current Attempts: %u). Scheduling re-TX.",
                    harq_process_idx, harq_p->original_psn, harq_p->tx_attempts);
            harq_p->needs_retransmission = true;

            // ETSI TS 103 636-4, Section 5.5.1: RV sequence {0, 2, 3, 1, 0, ...}
            // Current harq_p->redundancy_version holds the RV of the *last failed attempt*.
            // We set the RV for the *next* attempt here.
            switch (harq_p->redundancy_version) {
                case 0: harq_p->redundancy_version = 2; break;
                case 2: harq_p->redundancy_version = 3; break;
                case 3: harq_p->redundancy_version = 1; break;
                case 1: harq_p->redundancy_version = 0; // Cycle back to 0, or could be to 2 for shorter cycle if preferred
                        // If cycling back to 0 after RV1, it implies SDU is effectively "new" again for combiner
                        // Or, some implementations might stop after one full cycle {0,2,3,1}.
                        // For now, simple cycle.
                        break;
                default: // Should not happen if initialized to 0
                    LOG_ERR("HARQ_NACK: Invalid current RV %u for proc %d. Resetting to RV0 for next attempt.",
                            harq_p->redundancy_version, harq_process_idx);
                    harq_p->redundancy_version = 0;
                    break;
            }
            LOG_DBG("HARQ_NACK: Next RV for HARQ %d will be %u (after %u attempts).",
                    harq_process_idx, harq_p->redundancy_version, harq_p->tx_attempts);
            // The dect_mac_data_path_service_tx function will see needs_retransmission=true and pick it up.
        }
    } else {
        LOG_WRN("HARQ_NACK: Received for already inactive HARQ process %d.", harq_process_idx);
    }
}

void dect_mac_data_path_process_harq_feedback(const union nrf_modem_dect_phy_feedback *feedback,
                                              uint16_t peer_short_rd_id) {
    if (!feedback) {
        LOG_ERR("HARQ_FB_PROC: NULL feedback pointer.");
        return;
    }

    // The format code is in the same position for format1, format2, format3, format5, format6
    uint8_t format_code = (feedback->format1.format & 0x0F);

    LOG_DBG("HARQ_FB_PROC: Rcvd from Peer 0x%04X, PHY Feedback Format Code: %u", peer_short_rd_id, format_code);

    switch (format_code) {
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1: // Single HARQ process feedback
        {
            int harq_idx = feedback->format1.harq_process_number0;
            if (harq_idx >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT1: Invalid HARQ idx %d", harq_idx); return; }
            if (feedback->format1.transmission_feedback0 == 1) { // ACK
                dect_mac_data_path_handle_harq_ack_action(harq_idx);
            } else { // NACK
                dect_mac_data_path_handle_harq_nack_action(harq_idx);
            }
            break;
        }
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_3: // Dual HARQ process feedback
        {
            int harq_idx0 = feedback->format3.harq_process_number0;
            if (harq_idx0 >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT3: Invalid HARQ idx0 %d", harq_idx0); /* continue to idx1? */ }
            else {
                if (feedback->format3.transmission_feedback0 == 1) {
                    dect_mac_data_path_handle_harq_ack_action(harq_idx0);
                } else {
                    dect_mac_data_path_handle_harq_nack_action(harq_idx0);
                }
            }

            int harq_idx1 = feedback->format3.harq_process_number1;
            if (harq_idx1 >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT3: Invalid HARQ idx1 %d", harq_idx1); return; }
            else {
                if (feedback->format3.transmission_feedback1 == 1) {
                    dect_mac_data_path_handle_harq_ack_action(harq_idx1);
                } else {
                    dect_mac_data_path_handle_harq_nack_action(harq_idx1);
                }
            }
            break;
        }
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_6: // DF Redundancy Version reset requested
        {
            int harq_idx = feedback->format6.harq_process_number;
            if (harq_idx >= MAX_HARQ_PROCESSES) { LOG_ERR("HARQ_FB_FMT6: Invalid HARQ idx %d", harq_idx); return; }

            LOG_INF("HARQ_FB_FMT6: Peer 0x%04X requested RV0 reset for OUR HARQ process %d.", peer_short_rd_id, harq_idx);
            dect_mac_context_t* ctx = get_mac_context();
            if (ctx && ctx->harq_tx_processes[harq_idx].is_active) {
                ctx->harq_tx_processes[harq_idx].redundancy_version = 0;
                // Should this also increment tx_attempts? If peer couldn't decode even previous RVs.
                // For now, just set RV=0 and mark for reTX if not already.
                // If it was already NACKed, needs_retransmission is true. If ACKed, this is unusual.
                if (!ctx->harq_tx_processes[harq_idx].needs_retransmission) {
                    // This implies we thought it was ACKed, or it's a new SDU, but peer asks for RV0.
                    // This is more like a NACK if it was for an ongoing transmission.
                     LOG_WRN("HARQ_FB_FMT6: RV0 reset for HARQ proc %d which was not pending reTX. Marking for reTX.", harq_idx);
                    ctx->harq_tx_processes[harq_idx].needs_retransmission = true;
                }
                 // Restart timer, as peer effectively NACKed current state by asking for RV0.
                k_timer_stop(&ctx->harq_tx_processes[harq_idx].retransmission_timer);
                k_timer_start(&ctx->harq_tx_processes[harq_idx].retransmission_timer, K_MSEC(HARQ_ACK_TIMEOUT_MS), K_NO_WAIT);

            } else {
                 LOG_WRN("HARQ_FB_FMT6: RV0 reset for inactive HARQ proc %d.", harq_idx);
            }
            break;
        }
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_NONE: // Explicitly no feedback
            LOG_DBG("HARQ_FB_PROC: Received 'No Feedback' (Format 0).");
            break;
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_2: // MIMO / Codebook
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_4: // HARQ Bitmap
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_5: // MIMO / Codebook extended
        case NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_7: // CQI select
            LOG_WRN("HARQ_FB_PROC: Feedback format %u currently unhandled/not expected for basic HARQ ACK/NACK.", format_code);
            break;
        default: // Reserved or unknown formats
            LOG_ERR("HARQ_FB_PROC: Unknown feedback format code %u from 0x%04X", format_code, peer_short_rd_id);
            break;
    }
}

/****** */
// Overview: This is the complete, corrected implementation of the core data transmission function. It fixes several critical bugs:
// 1. **User Data Omission:** The function now correctly calls `build_user_data_ie_muxed` to include the actual DLC PDU payload in the MAC SDU Area.
// 2. **Encryption Boundary:** The `len_of_muxed_sec_ie_for_crypto_calc` variable is now correctly assigned the length of the Security IE when it's built, ensuring the encryption starts after this cleartext header.
// 3. **HARQ Feedback:** The function signature is updated to accept a `feedback` parameter, which is then copied into the PCC header, enabling the transmission of ACKs/NACKs.
// 4. **QoS Awareness:** The function now accepts a `flow_id` and stores it in the HARQ process context, enabling the NACK handler to make QoS-based retransmission decisions.
// --- REPLACE ENTIRE FUNCTION: [send_data_mac_sdu_via_phy_internal] ---
static int send_data_mac_sdu_via_phy_internal(dect_mac_context_t* ctx,
                                     mac_sdu_t *mac_sdu_dlc_pdu, /* Contains DLC PDU */
                                     int harq_proc_idx, bool is_retransmission,
                                     uint16_t tx_carrier_from_schedule,
                                     int ft_target_peer_slot_idx,
                                     uint64_t phy_op_target_start_time,
                                     mac_flow_id_t flow_id,
                                     const union nrf_modem_dect_phy_feedback *feedback)
{
    uint8_t *full_mac_pdu_phy_buf_slab_alloc;
    int ret = k_mem_slab_alloc(&g_mac_sdu_slab, (void**)&full_mac_pdu_phy_buf_slab_alloc, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("DATA_TX_INT: Failed to alloc full MAC PDU TX buffer: %d", ret);
        return -ENOMEM;
    }
    uint8_t * const full_mac_pdu_for_phy = full_mac_pdu_phy_buf_slab_alloc;


    dect_mac_header_type_octet_t mac_hdr_type_octet;
    mac_hdr_type_octet.version = 0;
    mac_hdr_type_octet.mac_header_type = MAC_COMMON_HEADER_TYPE_UNICAST;

    bool security_active_for_this_pdu = false;
    bool include_mac_sec_info_ie = false;
    const uint8_t *session_integrity_key = NULL;
    const uint8_t *session_cipher_key = NULL;

    uint32_t receiver_long_id = 0;
    uint16_t receiver_short_id = 0xFFFF;

    if (ctx->role == MAC_ROLE_PT) {
        if (ctx->role_ctx.pt.associated_ft.is_valid) {
            receiver_long_id = ctx->role_ctx.pt.associated_ft.long_rd_id;
            receiver_short_id = ctx->role_ctx.pt.associated_ft.short_rd_id;
            if (ctx->role_ctx.pt.associated_ft.is_secure && ctx->keys_provisioned) {
                security_active_for_this_pdu = true;
                session_integrity_key = ctx->integrity_key;
                session_cipher_key = ctx->cipher_key;
            }
        } else {
            LOG_ERR("DATA_TX_INT: PT not associated, cannot determine target for data TX.");
            ret = -ENOTCONN;
            goto free_slab_and_return_error_tx_sec_path;
        }
    } else { // MAC_ROLE_FT
        if (ft_target_peer_slot_idx < 0 || ft_target_peer_slot_idx >= MAX_PEERS_PER_FT ||
            !ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_valid) {
            LOG_ERR("DATA_TX_INT: FT role, but invalid or inactive peer_slot_idx %d for TX.", ft_target_peer_slot_idx);
            ret = -EINVAL;
            goto free_slab_and_return_error_tx_sec_path;
        }
        receiver_long_id = ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].long_rd_id;
        receiver_short_id = ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].short_rd_id;
        if (ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_secure &&
            ctx->role_ctx.ft.keys_provisioned_for_peer[ft_target_peer_slot_idx]) {
            security_active_for_this_pdu = true;
            session_integrity_key = ctx->role_ctx.ft.peer_integrity_keys[ft_target_peer_slot_idx];
            session_cipher_key = ctx->role_ctx.ft.peer_cipher_keys[ft_target_peer_slot_idx];
        }
    }

    if (receiver_long_id == 0 || receiver_short_id == 0xFFFF) {
        LOG_ERR("DATA_TX_INT: Invalid receiver ID for data TX (L:0x%08X, S:0x%04X).", receiver_long_id, receiver_short_id);
        ret = -EINVAL;
        goto free_slab_and_return_error_tx_sec_path;
    }

    uint16_t psn_for_this_pdu;
    uint32_t hpc_for_tx_iv_build;

    if (!is_retransmission) {
        increment_psn_and_hpc(ctx);
        psn_for_this_pdu = ctx->psn;
        hpc_for_tx_iv_build = ctx->hpc;
    } else {
        if (harq_proc_idx < 0 || harq_proc_idx >= MAX_HARQ_PROCESSES || !ctx->harq_tx_processes[harq_proc_idx].is_active) {
            LOG_ERR("DATA_TX_INT: Invalid HARQ process %d for retransmission.", harq_proc_idx);
            ret = -EINVAL; goto free_slab_and_return_error_tx_sec_path;
        }
        psn_for_this_pdu = ctx->harq_tx_processes[harq_proc_idx].original_psn;
        hpc_for_tx_iv_build = ctx->harq_tx_processes[harq_proc_idx].original_hpc;
    }

    uint8_t sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
    bool send_hpc_resync_initiate_to_peer = false;
    bool send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = false;

    if (security_active_for_this_pdu && !is_retransmission) {
        dect_mac_peer_info_t *peer_context_for_tx_flags_check = NULL;
        if (ctx->role == MAC_ROLE_PT) {
            if (ctx->role_ctx.pt.associated_ft.is_valid && ctx->role_ctx.pt.associated_ft.long_rd_id == receiver_long_id) {
                peer_context_for_tx_flags_check = &ctx->role_ctx.pt.associated_ft;
            }
        } else if (ft_target_peer_slot_idx != -1) {
             if (ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].is_valid &&
                 ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx].long_rd_id == receiver_long_id) {
                peer_context_for_tx_flags_check = &ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx];
            }
        }

        if (peer_context_for_tx_flags_check) {
            if (peer_context_for_tx_flags_check->self_needs_to_request_hpc_from_peer) {
                send_hpc_resync_initiate_to_peer = true;
            } else if (peer_context_for_tx_flags_check->peer_requested_hpc_resync) {
                send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = true;
            }
        }
        if (ctx->send_mac_sec_info_ie_on_next_tx && !send_hpc_resync_initiate_to_peer && !send_own_hpc_as_provided_due_to_peer_req_or_self_wrap) {
            send_own_hpc_as_provided_due_to_peer_req_or_self_wrap = true;
        }

        if (send_hpc_resync_initiate_to_peer) {
            include_mac_sec_info_ie = true;
            sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_RESYNC_INITIATE;
        } else if (send_own_hpc_as_provided_due_to_peer_req_or_self_wrap) {
            include_mac_sec_info_ie = true;
            sec_iv_type_for_current_tx_ie = SEC_IV_TYPE_MODE1_HPC_PROVIDED;
        }
    }
    if (is_retransmission) {
        include_mac_sec_info_ie = false;
    }

    if (security_active_for_this_pdu) {
        mac_hdr_type_octet.mac_security = include_mac_sec_info_ie ? MAC_SECURITY_USED_WITH_IE : MAC_SECURITY_USED_NO_IE;
    } else {
        mac_hdr_type_octet.mac_security = MAC_SECURITY_NONE;
    }

    dect_mac_unicast_header_t common_hdr;
    common_hdr.sequence_num_high_reset_rsv = SET_SEQ_NUM_HIGH_RESET_RSV((psn_for_this_pdu >> 8) & 0x0F, !is_retransmission);
    common_hdr.sequence_num_low = psn_for_this_pdu & 0xFF;
    common_hdr.transmitter_long_rd_id_be = sys_cpu_to_be32(ctx->own_long_rd_id);
    common_hdr.receiver_long_rd_id_be = sys_cpu_to_be32(receiver_long_id);

    uint8_t sdu_area_buf[CONFIG_DECT_MAC_SDU_MAX_SIZE + 10];
    size_t current_sdu_area_len = 0;
    size_t len_of_muxed_sec_ie_for_crypto_calc = 0;
    int ie_len;

    if (security_active_for_this_pdu && include_mac_sec_info_ie) {
        ie_len = build_mac_security_info_ie_muxed(
            sdu_area_buf, sizeof(sdu_area_buf),
            0, ctx->current_key_index,
            sec_iv_type_for_current_tx_ie,
            ctx->hpc);
        if (ie_len < 0) { ret = ie_len; LOG_ERR("DATA_TX_INT: Build SecInfoIE failed: %d", ret); goto free_slab_and_return_error_tx_sec_path; }
        current_sdu_area_len += ie_len;
        len_of_muxed_sec_ie_for_crypto_calc = ie_len;
    }

    ie_len = build_user_data_ie_muxed(sdu_area_buf + current_sdu_area_len,
                                      sizeof(sdu_area_buf) - current_sdu_area_len,
                                      mac_sdu_dlc_pdu->data, mac_sdu_dlc_pdu->len,
                                      IE_TYPE_USER_DATA_FLOW_1);
    if (ie_len < 0) {
        ret = ie_len;
        LOG_ERR("DATA_TX_INT: Build User Data IE failed: %d", ret);
        goto free_slab_and_return_error_tx_sec_path;
    }
    current_sdu_area_len += ie_len;

    uint16_t assembled_pdu_len_pre_mic;
    ret = dect_mac_phy_ctrl_assemble_final_pdu(
              full_mac_pdu_for_phy, CONFIG_DECT_MAC_PDU_MAX_SIZE,
              &mac_hdr_type_octet,
              &common_hdr, sizeof(common_hdr),
              sdu_area_buf, current_sdu_area_len,
              &assembled_pdu_len_pre_mic);
    if (ret != 0) { LOG_ERR("DATA_TX_INT: Assemble final PDU failed: %d", ret); goto free_slab_and_return_error_tx_sec_path; }

    uint16_t final_tx_pdu_len_for_phy_ctrl = assembled_pdu_len_pre_mic;

    if (security_active_for_this_pdu) {
        uint8_t iv[16];
        security_build_iv(iv, ctx->own_long_rd_id, receiver_long_id, hpc_for_tx_iv_build, psn_for_this_pdu);

        uint8_t *mic_calculation_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t);
        size_t mic_calculation_length = sizeof(common_hdr) + current_sdu_area_len;

        if ((assembled_pdu_len_pre_mic + 5) > CONFIG_DECT_MAC_PDU_MAX_SIZE) {
            LOG_ERR("DATA_TX_INT: Not enough space in PDU buffer for MIC.");
            ret = -ENOMEM;
            goto free_slab_and_return_error_tx_sec_path;
        }
        uint8_t *mic_location_ptr = full_mac_pdu_for_phy + assembled_pdu_len_pre_mic;
        ret = security_calculate_mic(mic_calculation_start_ptr, mic_calculation_length,
                                   session_integrity_key, mic_location_ptr);
        if (ret != 0) {
            LOG_ERR("DATA_TX_INT: MIC calculation failed: %d", ret);
            goto free_slab_and_return_error_tx_sec_path;
        }
        final_tx_pdu_len_for_phy_ctrl = assembled_pdu_len_pre_mic + 5;

        uint8_t *encryption_start_ptr;
        size_t encryption_length;

        if (mac_hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) {
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) +
                                   sizeof(common_hdr) + len_of_muxed_sec_ie_for_crypto_calc;
            encryption_length = (current_sdu_area_len - len_of_muxed_sec_ie_for_crypto_calc) + 5;
        } else {
            encryption_start_ptr = full_mac_pdu_for_phy + sizeof(dect_mac_header_type_octet_t) + sizeof(common_hdr);
            encryption_length = current_sdu_area_len + 5;
        }

        if (encryption_length > 0) {
             uint8_t* pdu_buffer_end_with_mic = full_mac_pdu_for_phy + final_tx_pdu_len_for_phy_ctrl;
             if (encryption_start_ptr < full_mac_pdu_for_phy || (encryption_start_ptr + encryption_length) > pdu_buffer_end_with_mic ) {
                //  LOG_ERR("DATA_TX_INT: Encryption range error. Start %p + Len %zu > PDU End %p",
                //          encryption_start_ptr, encryption_length, full_mac_pdu_for_phy, pdu_buffer_end_with_mic);
                 LOG_ERR("DATA_TX_INT: Encryption range error. Start %p + Len %zu, PDU %p > End %p",
                 encryption_start_ptr, encryption_length, full_mac_pdu_for_phy, pdu_buffer_end_with_mic);                
                 ret = -EINVAL; goto free_slab_and_return_error_tx_sec_path;
             }
             ret = security_crypt_payload(encryption_start_ptr, encryption_length, session_cipher_key, iv, true);
             if (ret != 0) {
                 LOG_ERR("DATA_TX_INT: Encryption failed: %d", ret);
                 goto free_slab_and_return_error_tx_sec_path;
             }
        }
        LOG_DBG("DATA_TX_INT: Secured PDU. Final len %u. Mode: %s, MUXSecIELen: %zu",
                final_tx_pdu_len_for_phy_ctrl,
                (mac_hdr_type_octet.mac_security == MAC_SECURITY_USED_WITH_IE) ? "WITH_SEC_IE" : "NO_SEC_IE",
                len_of_muxed_sec_ie_for_crypto_calc);
    }

    pending_op_type_t op_type_for_phy = (ctx->role == MAC_ROLE_PT) ? PENDING_OP_PT_DATA_TX_HARQ0 : PENDING_OP_FT_DATA_TX_HARQ0;
    op_type_for_phy = (pending_op_type_t)((int)op_type_for_phy + harq_proc_idx);
    // uint32_t phy_op_handle = sys_rand32_get();
    uint32_t phy_op_handle;
    sys_rand_get(&phy_op_handle, sizeof(phy_op_handle));

    ret = dect_mac_phy_ctrl_start_tx_assembled(
        tx_carrier_from_schedule,
        full_mac_pdu_for_phy,
        final_tx_pdu_len_for_phy_ctrl,
        receiver_short_id,
        false,
        phy_op_handle,
        op_type_for_phy,
        true,
        phy_op_target_start_time,
        ctx->own_phy_params.mu,
        feedback);

free_slab_and_return_error_tx_sec_path:
    k_mem_slab_free(&g_mac_sdu_slab, (void**)&full_mac_pdu_phy_buf_slab_alloc);

    if (ret == 0) {
        dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[harq_proc_idx];
        if (!is_retransmission) {
            harq_p->sdu = mac_sdu_dlc_pdu;
            harq_p->is_active = true;
            harq_p->flow_id = flow_id;
            harq_p->original_psn = psn_for_this_pdu;
            harq_p->original_hpc = hpc_for_tx_iv_build;
            harq_p->tx_attempts = 1;
            harq_p->redundancy_version = 0;
            harq_p->scheduled_carrier = tx_carrier_from_schedule;
            harq_p->scheduled_tx_start_time = phy_op_target_start_time;
            harq_p->peer_short_id_for_ft_dl = receiver_short_id;
        } else {
            harq_p->tx_attempts++;
        }
        harq_p->needs_retransmission = false;
        k_timer_start(&harq_p->retransmission_timer, K_MSEC(HARQ_ACK_TIMEOUT_MS), K_NO_WAIT);

		if (include_mac_sec_info_ie && security_active_for_this_pdu && !is_retransmission) {
			dect_mac_peer_info_t *peer_ctx = NULL;

			if (ctx->role == MAC_ROLE_PT) {
				peer_ctx = &ctx->role_ctx.pt.associated_ft;
			} else if (ft_target_peer_slot_idx != -1) {
				peer_ctx =
					&ctx->role_ctx.ft.connected_pts[ft_target_peer_slot_idx];
			}

			if (peer_ctx) {
				if (send_hpc_resync_initiate_to_peer) {
					LOG_DBG("DATA_TX_INT: Cleared self_needs_to_request_hpc_from_peer for 0x%04X.",
						peer_ctx->short_rd_id);
					peer_ctx->self_needs_to_request_hpc_from_peer = false;
				}
				if (send_own_hpc_as_provided_due_to_peer_req_or_self_wrap) {
					if (peer_ctx->peer_requested_hpc_resync) {
						LOG_DBG("DATA_TX_INT: Cleared peer_requested_hpc_resync for 0x%04X after sending PROVIDED.",
							peer_ctx->short_rd_id);
						peer_ctx->peer_requested_hpc_resync = false;
					}
				}
			}
			if (ctx->send_mac_sec_info_ie_on_next_tx) {
				LOG_DBG("DATA_TX_INT: Cleared global send_mac_sec_info_ie_on_next_tx flag.");
				ctx->send_mac_sec_info_ie_on_next_tx = false;
			}
		}
    } else {
        LOG_ERR("DATA_TX_INT: PHY TX schedule failed for HARQ %d (err %d).", harq_proc_idx, ret);
        if (!is_retransmission) {
            LOG_ERR("DATA_TX_INT: Freeing new SDU (len %u) due to PHY TX schedule failure for HARQ %d.",
                    mac_sdu_dlc_pdu->len, harq_proc_idx);
            dect_mac_api_buffer_free(mac_sdu_dlc_pdu);
        } else {
            ctx->harq_tx_processes[harq_proc_idx].needs_retransmission = true;
        }
    }
    return ret;
}

// void dect_mac_data_path_service_tx(void)
// {
// 	dect_mac_context_t *ctx = get_mac_context();

// 	if (ctx->state < MAC_STATE_ASSOCIATED) {
// 		return;
// 	}

// 	if (ctx->pending_op_type != PENDING_OP_NONE) {
// 		return;
// 	}

// 	/* --- 1. Prioritize HARQ Retransmissions --- */
// 	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
// 		dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
// 		if (harq_p->is_active && harq_p->needs_retransmission) {
// 			/* Simplified re-TX: attempt immediately if possible */
// 			LOG_INF("DATA_PATH_SVC_TX: Attempting re-TX for HARQ proc %d", i);
// 			union nrf_modem_dect_phy_feedback feedback_to_send = {0};
// 			bool feedback_is_pending = false;
// 			dect_mac_peer_info_t *peer_ctx = NULL;
// 			int peer_idx = -1; /* Not applicable for PT role here */
// 			if (ctx->role == MAC_ROLE_FT) {
// 				peer_idx = ft_get_peer_slot_idx(ctx, harq_p->peer_short_id_for_ft_dl);
// 				if (peer_idx != -1) {
// 					peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_idx];
// 				}
// 			} else { /* PT Role */
// 				peer_ctx = &ctx->role_ctx.pt.associated_ft;
// 			}

// 			if (peer_ctx && peer_ctx->num_pending_feedback_items > 0) {
// 				feedback_is_pending = true;
// 				feedback_to_send.format1.format = NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1;
// 				feedback_to_send.format1.harq_process_number0 = peer_ctx->pending_feedback_to_send[0].harq_process_num_for_peer;
// 				feedback_to_send.format1.transmission_feedback0 = peer_ctx->pending_feedback_to_send[0].is_ack;
// 			}

// 			int ret = send_data_mac_sdu_via_phy_internal(
// 				ctx, harq_p->sdu, i, true, harq_p->scheduled_carrier, peer_idx, 0,
// 				harq_p->flow_id, feedback_is_pending ? &feedback_to_send : NULL);

// 			if (ret == 0 && feedback_is_pending && peer_ctx) {
// 				memset(peer_ctx->pending_feedback_to_send, 0, sizeof(peer_ctx->pending_feedback_to_send));
// 				peer_ctx->num_pending_feedback_items = 0;
// 			}
// 			return; /* Service one re-TX per call */
// 		}
// 	}

// 	/* --- 2. Service New SDUs based on Role and Schedules --- */
// 	if (ctx->role == MAC_ROLE_FT) {
// 		ft_service_schedules();
// 	} else { /* PT Role */
// 		dect_mac_schedule_t *ul_schedule = &ctx->role_ctx.pt.ul_schedule;
// 		if (!ul_schedule->is_active) {
// 			ul_schedule = &ctx->role_ctx.pt.group_schedule;
// 		}

// 		if (ul_schedule->is_active) {
// 			update_next_occurrence(ctx, ul_schedule, ctx->last_known_modem_time);
// 			uint64_t target_start_time = ul_schedule->next_occurrence_modem_time;
// 			uint32_t prep_latency_ticks = modem_us_to_ticks(
// 				ctx->phy_latency.idle_to_active_tx_us +
// 				ctx->phy_latency.scheduled_operation_startup_us,
// 				NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);

// 			if (target_start_time >= (ctx->last_known_modem_time + prep_latency_ticks)) {
// 				int free_harq_idx = find_free_harq_tx_process(ctx);
// 				if (free_harq_idx != -1) {
// 					for (int flow_idx = 0; flow_idx < MAC_FLOW_COUNT; flow_idx++) {
// 						// mac_sdu_t *sdu = k_fifo_get(mac_tx_fifos[flow_idx], K_NO_WAIT);
// 						// if (sdu) {
//                         sys_dnode_t *node = sys_dlist_get(mac_tx_dlists[flow_idx]);
//                         if (node) {
//                             mac_sdu_t *sdu = CONTAINER_OF(node, mac_sdu_t, node);
// 							if (does_sdu_fit_schedule(ctx, sdu, ul_schedule, -1)) {
// 								union nrf_modem_dect_phy_feedback feedback_to_send = {0};
// 								bool feedback_is_pending = false;
// 								dect_mac_peer_info_t *ft_peer_ctx = &ctx->role_ctx.pt.associated_ft;

// 								if (ft_peer_ctx->num_pending_feedback_items > 0) {
// 									feedback_is_pending = true;
// 									feedback_to_send.format1.format = NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1;
// 									feedback_to_send.format1.harq_process_number0 = ft_peer_ctx->pending_feedback_to_send[0].harq_process_num_for_peer;
// 									feedback_to_send.format1.transmission_feedback0 = ft_peer_ctx->pending_feedback_to_send[0].is_ack;
// 								}

// 								int ret = send_data_mac_sdu_via_phy_internal(
// 									ctx, sdu, free_harq_idx, false,
// 									ul_schedule->channel, -1,
// 									target_start_time,
// 									(mac_flow_id_t)flow_idx,
// 									feedback_is_pending ? &feedback_to_send : NULL);

// 								if (ret == 0 && feedback_is_pending) {
// 									memset(ft_peer_ctx->pending_feedback_to_send, 0, sizeof(ft_peer_ctx->pending_feedback_to_send));
// 									ft_peer_ctx->num_pending_feedback_items = 0;
// 								}
// 								return; /* Service one SDU per call */
// 							} else {
// 								LOG_WRN("PT_SCHED_TX: SDU (len %u) does not fit UL schedule. Re-queueing.", sdu->len);
// 								// k_fifo_prepend(mac_tx_fifos[flow_idx], sdu);
//                                 // Put the SDU back at the FRONT of the list.
//                                 sys_dlist_prepend(mac_tx_dlists[flow_idx], &sdu->node);
// 								/* SDU is too big for this opportunity, but maybe a smaller one from a lower priority queue will fit */
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// }


void dect_mac_data_path_service_tx(void)
{
	dect_mac_context_t *ctx = get_mac_context();

	if (ctx->state < MAC_STATE_ASSOCIATED || ctx->pending_op_type != PENDING_OP_NONE) {
		return;
	}

	/* --- 1. Prioritize HARQ Retransmissions --- */
	for (int i = 0; i < MAX_HARQ_PROCESSES; i++) {
		dect_harq_tx_process_t *harq_p = &ctx->harq_tx_processes[i];
		if (harq_p->is_active && harq_p->needs_retransmission) {
			LOG_INF("DATA_PATH_SVC_TX: Attempting re-TX for HARQ proc %d", i);
			
            dect_mac_peer_info_t *peer_ctx = NULL;
            int peer_idx = -1;

            if (ctx->role == MAC_ROLE_FT) {
                // --- THIS IS THE FIX for the 'ft_get_peer_slot_idx' error ---
                // Replace the direct call to the static function...
                // peer_idx = ft_get_peer_slot_idx(ctx, harq_p->peer_short_id_for_ft_dl);
                
                // ...with a call to the new public helper function.
                peer_idx = dect_mac_core_get_peer_slot_idx(harq_p->peer_short_id_for_ft_dl);

                if (peer_idx != -1) {
                    peer_ctx = &ctx->role_ctx.ft.connected_pts[peer_idx];
                }
            } else { /* PT Role */
                peer_ctx = &ctx->role_ctx.pt.associated_ft;
            }
			
			return; // Service one re-TX per call
		}
	}

	/* --- 2. Service New SDUs by calling the registered scheduler hook --- */
    //
    // THIS IS THE FIX for the 'ft_service_schedules' error.
    // The direct, role-specific `if/else` block is replaced by a single call to the hook.
    //
	if (g_role_specific_scheduler_hook) {
		g_role_specific_scheduler_hook();
	}
}

int dect_mac_data_path_send_new_sdu(mac_sdu_t *sdu, mac_flow_id_t flow_id, int peer_slot_idx,
                                    uint16_t carrier, uint64_t start_time,
                                    const union nrf_modem_dect_phy_feedback *feedback)
{
    dect_mac_context_t *ctx = get_mac_context();
    int free_harq_idx = find_free_harq_tx_process(ctx);

    if (free_harq_idx == -1) {
        return -EBUSY; // No free HARQ process
    }

    return send_data_mac_sdu_via_phy_internal(ctx, sdu, free_harq_idx, false,
                                              carrier, peer_slot_idx, start_time,
                                              flow_id, feedback);
}

void dect_mac_data_path_handle_rx_sdu(const uint8_t *mac_sdu_area_data,
				      size_t mac_sdu_area_len,
				      uint32_t transmitter_long_rd_id)
{
	if (mac_sdu_area_data == NULL || mac_sdu_area_len == 0) {
		LOG_DBG("RX_SDU_HANDLER: Received empty or NULL MAC SDU Area.");
		return;
	}

	LOG_DBG("RX_SDU_HANDLER: Processing MAC SDU Area from 0x%08X, len %zu",
		transmitter_long_rd_id, mac_sdu_area_len);

	const uint8_t *current_ie_ptr = mac_sdu_area_data;
	size_t remaining_sdu_area_len = mac_sdu_area_len;

	while (remaining_sdu_area_len > 0) {
		uint8_t ie_type_from_mux;
		uint16_t dlc_pdu_len_from_mux;
		const uint8_t *dlc_pdu_ptr_from_mux;
		int parsed_mux_header_len;

		/* The parse_mac_mux_header function now correctly handles all length calculations internally. */
		parsed_mux_header_len =
			parse_mac_mux_header(current_ie_ptr, remaining_sdu_area_len,
					     &ie_type_from_mux, &dlc_pdu_len_from_mux,
					     &dlc_pdu_ptr_from_mux);

		if (parsed_mux_header_len < 0) {
			LOG_ERR("RX_SDU_HANDLER: Failed to parse MAC MUX header in SDU Area: %d. Dropping rest of SDU Area.",
				parsed_mux_header_len);
			break;
		}

		// if (dlc_pdu_len_from_mux == 0 && ((current_ie_ptr[0] >> 6) & 0x03) == 0b00) {
		// 	/* MAC_Ext=00 means length is defined by type. */
		// 	dlc_pdu_len_from_mux = get_fixed_ie_payload_len(ie_type_from_mux);
		// 	if (dlc_pdu_len_from_mux == 0 && ie_type_from_mux != IE_TYPE_SHORT_PADDING &&
		// 	    ie_type_from_mux != IE_TYPE_SHORT_KEEP_ALIVE) {
		// 		/* If it's not a known zero-length IE, assume it's a legacy data type that fills the rest */
		// 		if (remaining_sdu_area_len < (size_t)parsed_mux_header_len) {
		// 			break;
		// 		}
		// 		dlc_pdu_len_from_mux = remaining_sdu_area_len - parsed_mux_header_len;
		// 	}
		// }

        /* This check is still valid and important */
		if (remaining_sdu_area_len < (size_t)parsed_mux_header_len + dlc_pdu_len_from_mux) {
			LOG_ERR("RX_SDU_HANDLER: MUX IE (type 0x%X) declared payload len %u exceeds actual remaining SDU area %zu. Corrupted PDU?",
				ie_type_from_mux, dlc_pdu_len_from_mux,
				remaining_sdu_area_len - parsed_mux_header_len);
			break;
		}

		if (ie_type_from_mux >= IE_TYPE_USER_DATA_FLOW_1 &&
		    ie_type_from_mux <= IE_TYPE_USER_DATA_FLOW_4) {
			
            if (g_dlc_rx_sdu_dlist_ptr != NULL) {
                mac_sdu_t *sdu_for_dlc = dect_mac_api_buffer_alloc(K_NO_WAIT);

				if (sdu_for_dlc) {
					if (dlc_pdu_len_from_mux <= CONFIG_DECT_MAC_SDU_MAX_SIZE) {
						memcpy(sdu_for_dlc->data, dlc_pdu_ptr_from_mux,
						       dlc_pdu_len_from_mux);
						sdu_for_dlc->len = dlc_pdu_len_from_mux;
						// k_fifo_put(g_dlc_rx_sdu_fifo_ptr, sdu_for_dlc);
                        sys_dlist_append(g_dlc_rx_sdu_dlist_ptr, &sdu_for_dlc->node);
					} else {
						LOG_ERR("RX_SDU_HANDLER: Extracted DLC PDU too large (%u > %d). Dropped.",
							dlc_pdu_len_from_mux,
							CONFIG_DECT_MAC_SDU_MAX_SIZE);
						dect_mac_api_buffer_free(sdu_for_dlc);
					}
				} else {
					LOG_ERR("RX_SDU_HANDLER: Failed to alloc SDU buffer for DLC RX. DLC PDU (len %u) dropped.",
						dlc_pdu_len_from_mux);
				}
			} else {
				LOG_ERR("RX_SDU_HANDLER: DLC RX queue is NULL! DLC PDU dropped.");
			}
		} else {
			LOG_DBG("RX_SDU_HANDLER: Skipping non-UserData MUX IE type 0x%X.",
				ie_type_from_mux);
		}

        // Declare and calculate the total number of bytes consumed by the current IE.
        size_t consumed = (size_t)parsed_mux_header_len + dlc_pdu_len_from_mux;

        // Check for a zero-length IE to prevent an infinite loop.
        // This can happen with padding IEs or if the logic above results in zero.
        if (consumed == 0) {
            LOG_WRN("RX_SDU_HANDLER: Consumed 0 bytes for IE type 0x%X. Breaking loop to prevent stall.", ie_type_from_mux);
            break;
        }

		current_ie_ptr += consumed;
		remaining_sdu_area_len -= consumed;
	}
}

