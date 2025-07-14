/* dect_mac/dect_mac_timeline_utils.c */
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <nrf_modem_dect_phy.h>

#include <mac/dect_mac_timeline_utils.h>

LOG_MODULE_REGISTER(dect_mac_timeline, LOG_LEVEL_INF);


/* FUDGED RANDOM NUMBER GENERATOR
* TODO: Need to rebuild using CRYPTO 
*  mbedTLS constant-time
* PSA Crypto API
*/
int constant_time_memcmp(const void *a, const void *b, size_t len)
{
    const unsigned char *pa = (const unsigned char *)a;
    const unsigned char *pb = (const unsigned char *)b;
    unsigned char result = 0;
    
    for (size_t i = 0; i < len; i++) {
        result |= pa[i] ^ pb[i];
    }
    
    return result;
}



/**
 * @brief Converts a duration in microseconds to modem time ticks.
 *
 * @param us Duration in microseconds.
 * @param tick_rate_khz The modem's tick rate in kHz.
 * @return The equivalent duration in modem time ticks.
 */
uint32_t modem_us_to_ticks(uint32_t us, uint32_t tick_rate_khz)
{
    if (tick_rate_khz == 0) {
        return 0;
    }
    // Use 64-bit arithmetic to prevent intermediate overflow for large us values
    return (uint32_t)(((uint64_t)us * tick_rate_khz) / 1000U);
}

/**
 * @brief Calculates the duration of one subslot in modem time ticks for a given mu_code.
 *
 * A subslot is 5 OFDM symbols. Symbol duration depends on mu.
 * NRF_MODEM_DECT_SYMBOL_DURATION is for mu=1 (code 0).
 * mu_actual = 2^mu_code. Symbol_duration_mu = Symbol_duration_mu1 / (2^(mu_code)).
 *
 * @param mu_code The mu code (0-7).
 * @return Subslot duration in modem ticks, or 0 on error.
 */
uint32_t get_subslot_duration_ticks_for_mu(uint8_t mu_code)
{
    if (mu_code > 7) {
        LOG_ERR("PHY_TIMING: Invalid mu_code %u for subslot duration.", mu_code);
        return 0;
    }

    uint32_t base_symbol_duration_ticks = NRF_MODEM_DECT_SYMBOL_DURATION;
    uint32_t actual_symbol_duration_ticks = base_symbol_duration_ticks;

    if (mu_code > 0) {
        actual_symbol_duration_ticks = base_symbol_duration_ticks / (1U << mu_code);
    }
    
    if (actual_symbol_duration_ticks == 0 && base_symbol_duration_ticks != 0) {
        LOG_ERR("PHY_TIMING: Calculated symbol duration is 0 for mu_code %u.", mu_code);
        return 0;
    }
    return actual_symbol_duration_ticks * 5;
}


/**
 * @brief Gets the number of subslots per ETSI slot for a given mu_code.
 *
 * @param mu_code The mu code (0-3).
 * @return Number of subslots per ETSI slot.
 */
uint8_t get_subslots_per_etsi_slot_for_mu(uint8_t mu_code)
{
    if (mu_code > 3) {
        LOG_WRN("PHY_TIMING: mu_code %u > 3, N_slot_subslot may not be standard. Defaulting for mu=8.", mu_code);
        return 16;
    }
    return 2 * (1U << mu_code);
}


uint64_t calculate_target_modem_time(dect_mac_context_t *ctx, uint64_t sfn_zero_anchor_time,
                                            uint8_t sfn_of_anchor_relevance, uint8_t target_sfn_val,
                                            uint16_t target_subslot_idx,
                                            uint8_t link_mu_code, uint8_t link_beta_code)
{
    ARG_UNUSED(link_beta_code); // Beta currently not used in subslot/frame duration calculations here

    if (!ctx) {
        LOG_ERR("CALC_TIME: NULL MAC context provided!");
        return UINT64_MAX; 
    }

    if (sfn_zero_anchor_time == 0 && ctx->last_known_modem_time == 0) {
        LOG_WRN("CALC_TIME: SFN Zero Anchor and last_known_modem_time are both 0. Cannot calculate. Returning large future estimate.");
        return modem_us_to_ticks(500000, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
    }
    if (sfn_zero_anchor_time == 0) {
        LOG_WRN("CALC_TIME: SFN Zero Anchor is 0. Using last_known_modem_time + fallback delay.");
        uint32_t fallback_delay_ticks = modem_us_to_ticks(FRAME_DURATION_MS_NOMINAL * 1000 * 2, NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ);
        return ctx->last_known_modem_time + fallback_delay_ticks;
    }

    uint32_t frame_duration_ticks_val = 0;
    if (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ > 0) {
        frame_duration_ticks_val = (uint32_t)FRAME_DURATION_MS_NOMINAL * (NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ / 1000U);
    }
    if (frame_duration_ticks_val == 0) {
        LOG_ERR("CALC_TIME: Calculated frame_duration_ticks is 0! Modem tick rate likely 0.");
        return UINT64_MAX;
    }

    // Use the provided link_mu_code for subslot duration calculation
    uint8_t mu_code_for_calc = link_mu_code;
    if (mu_code_for_calc > 7) { // Max mu_code for 2^7=128, typical DECT NR+ 0-3
        LOG_WRN("CALC_TIME: Invalid link_mu_code %u provided, defaulting to 0 (mu=1).", link_mu_code);
        mu_code_for_calc = 0;
    }

    uint32_t subslot_duration_ticks_val = get_subslot_duration_ticks_for_mu(mu_code_for_calc);
    if (subslot_duration_ticks_val == 0) {
        LOG_ERR("CALC_TIME: Calculated subslot_duration_ticks is 0 for mu_code %u!", mu_code_for_calc);
        return UINT64_MAX;
    }

    uint64_t anchor_relevance_frame_start_time = sfn_zero_anchor_time +
                                                 ((uint64_t)sfn_of_anchor_relevance * frame_duration_ticks_val);

    int16_t sfn_diff = (int16_t)target_sfn_val - (int16_t)sfn_of_anchor_relevance;
    if (sfn_diff > 128) { sfn_diff -= 256; }
    else if (sfn_diff < -128) { sfn_diff += 256; }

    uint64_t target_frame_start_time = anchor_relevance_frame_start_time + ((int64_t)sfn_diff * frame_duration_ticks_val);
    uint64_t target_subslot_offset_in_frame_ticks = (uint64_t)target_subslot_idx * subslot_duration_ticks_val;
    uint64_t final_target_time = target_frame_start_time + target_subslot_offset_in_frame_ticks;

    LOG_DBG("CALC_TIME: AnchorSFN %u (rel. to SFN0@%llu), TargetSFN %u, TargetSS %u (using link_mu_code %u) => FinalTime %llu",
            sfn_of_anchor_relevance, sfn_zero_anchor_time,
            target_sfn_val, target_subslot_idx, mu_code_for_calc, final_target_time);

    return final_target_time;
}




void update_next_occurrence(dect_mac_context_t *ctx, dect_mac_schedule_t *schedule, uint64_t current_modem_time) {
    if (!schedule->is_active) {
        return;
    }

    uint32_t subslot_duration_ticks = get_subslot_duration_ticks_for_mu(ctx->own_phy_params.mu);
    uint32_t frame_duration_ticks = (uint32_t)MAX_SUBSLOTS_IN_FRAME_NOMINAL * subslot_duration_ticks;
    uint64_t repetition_period_ticks = 0;
    uint32_t scheduled_duration_subslots = 0;
    uint16_t current_schedule_start_subslot = 0;


    if (schedule->alloc_type == RES_ALLOC_TYPE_DOWNLINK ||
        (schedule->alloc_type == RES_ALLOC_TYPE_BIDIR && ctx->role == MAC_ROLE_FT)) {
        scheduled_duration_subslots = schedule->dl_duration_subslots;
        current_schedule_start_subslot = schedule->dl_start_subslot;
    } else if (schedule->alloc_type == RES_ALLOC_TYPE_UPLINK ||
               (schedule->alloc_type == RES_ALLOC_TYPE_BIDIR && ctx->role == MAC_ROLE_PT)) {
        scheduled_duration_subslots = schedule->ul_duration_subslots;
        current_schedule_start_subslot = schedule->ul_start_subslot;
    } else {
        LOG_ERR("SCHED_UPD: Invalid alloc_type %d in schedule for Ch %u, SS %u. Deactivating.",
                schedule->alloc_type, schedule->channel, current_schedule_start_subslot);
        schedule->is_active = false;
        return;
    }
    if (scheduled_duration_subslots == 0 && schedule->repeat_type != RES_ALLOC_REPEAT_SINGLE) {
        LOG_ERR("SCHED_UPD: Zero duration for repeating schedule (Ch %u, SS %u). Deactivating.",
                schedule->channel, current_schedule_start_subslot);
        schedule->is_active = false;
        return;
    }


    if (schedule->repeat_type == RES_ALLOC_REPEAT_SINGLE) {
        uint64_t single_occurrence_end_time = schedule->next_occurrence_modem_time + ((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);
        if (single_occurrence_end_time < current_modem_time && schedule->next_occurrence_modem_time != 0) { // Check if already passed
            LOG_DBG("SCHED_UPD: Single occurrence schedule (Ch %u, SS %u @ %llu) has passed. Deactivating.",
                    schedule->channel, current_schedule_start_subslot, schedule->next_occurrence_modem_time);
            schedule->is_active = false;
        }
        return;
    }

    if (schedule->repetition_value == 0) {
        LOG_ERR("SCHED_UPD: Repetition value 0 is undefined for schedule (Ch %u, SS %u). Deactivating.",
                schedule->channel, current_schedule_start_subslot);
        schedule->is_active = false;
        return;
    }

    if (schedule->repeat_type == RES_ALLOC_REPEAT_FRAMES || schedule->repeat_type == RES_ALLOC_REPEAT_FRAMES_GROUP) {
        repetition_period_ticks = (uint64_t)schedule->repetition_value * frame_duration_ticks;
    } else if (schedule->repeat_type == RES_ALLOC_REPEAT_SUBSLOTS || schedule->repeat_type == RES_ALLOC_REPEAT_SUBSLOTS_GROUP) {
        repetition_period_ticks = (uint64_t)schedule->repetition_value * subslot_duration_ticks;
    } else {
        LOG_ERR("SCHED_UPD: Unknown repeat type %d for schedule (Ch %u, SS %u). Deactivating.",
                schedule->repeat_type, schedule->channel, current_schedule_start_subslot);
        schedule->is_active = false;
        return;
    }

    if (repetition_period_ticks == 0) { // Should be caught by repetition_value == 0
        schedule->is_active = false;
        return;
    }

    uint64_t current_slot_end_time_for_update = schedule->next_occurrence_modem_time +
                                                ((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);

    if (schedule->next_occurrence_modem_time == 0 && schedule->schedule_init_modem_time != 0) {
         // This can happen if initial calculation was deferred or failed. Re-calculate based on init time.
         // This path needs robust initial calculation logic, for now, just log.
         LOG_WRN("SCHED_UPD: next_occurrence_modem_time is 0 for active repeating schedule. Init needed.");
         // For now, just advance from current_modem_time as a fallback.
         schedule->next_occurrence_modem_time = current_modem_time;
         current_slot_end_time_for_update = current_modem_time; // Recalculate based on this assumption
    }


    while (schedule->next_occurrence_modem_time < current_modem_time || current_slot_end_time_for_update <= current_modem_time) {
        schedule->next_occurrence_modem_time += repetition_period_ticks;
        current_slot_end_time_for_update = schedule->next_occurrence_modem_time +
                                           ((uint64_t)scheduled_duration_subslots * subslot_duration_ticks);
    }

    if (schedule->validity_value != 0xFF && schedule->sfn_of_initial_occurrence != 0xFF && ctx->ft_sfn_zero_modem_time_anchor != 0) {
        uint64_t ticks_from_sfn0_anchor = schedule->next_occurrence_modem_time - ctx->ft_sfn_zero_modem_time_anchor;
        if (schedule->next_occurrence_modem_time < ctx->ft_sfn_zero_modem_time_anchor) { // next_occurrence is before anchor (e.g. SFN wrap)
             ticks_from_sfn0_anchor = (UINT64_MAX - ctx->ft_sfn_zero_modem_time_anchor) + schedule->next_occurrence_modem_time + 1;
        }
        uint64_t frames_from_sfn0_anchor = ticks_from_sfn0_anchor / frame_duration_ticks;
        uint8_t sfn_of_next_occurrence = (uint8_t)(frames_from_sfn0_anchor % 256);

        int16_t frames_elapsed_since_initial = (int16_t)sfn_of_next_occurrence - (int16_t)schedule->sfn_of_initial_occurrence;
        if (frames_elapsed_since_initial < 0) {
            frames_elapsed_since_initial += 256;
        }

        if ((uint8_t)frames_elapsed_since_initial >= schedule->validity_value) {
            LOG_INF("SCHED_UPD: Schedule (Ch %u, SS %u) deactivated: validity expired. Init SFN %u, Next Occ SFN %u, Validity %u frames, Elapsed ~%d",
                    schedule->channel, current_schedule_start_subslot,
                    schedule->sfn_of_initial_occurrence, sfn_of_next_occurrence,
                    schedule->validity_value, frames_elapsed_since_initial);
            schedule->is_active = false;
            return;
        }
    }
    LOG_DBG("SCHED_UPD: Updated schedule (Ch %u, SS %u): next occurrence at %llu",
            schedule->channel, current_schedule_start_subslot, schedule->next_occurrence_modem_time);
}

