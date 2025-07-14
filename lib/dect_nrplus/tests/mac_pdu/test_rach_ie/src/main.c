/* tests/mac_pdu/test_rach_ie/src/main.c */
// Overview: This file is converted from a standalone test program to a formal Ztest suite. `printk` based checks are replaced with `zassert` macros for automated verification.
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include "dect_mac_pdu.h"
#include "dect_mac_context.h"

LOG_MODULE_REGISTER(test_rach_ie, LOG_LEVEL_DBG);

static void verify_rach_fields(const dect_mac_rach_info_ie_fields_t *original, const dect_mac_rach_info_ie_fields_t *parsed)
{
	zassert_equal(original->repeat_type_is_subslots, parsed->repeat_type_is_subslots, "Mismatch: repeat_type_is_subslots");
	zassert_equal(original->sfn_validity_present, parsed->sfn_validity_present, "Mismatch: sfn_validity_present");
	zassert_equal(original->channel_field_present, parsed->channel_field_present, "Mismatch: channel_field_present");
	zassert_equal(original->channel2_field_present, parsed->channel2_field_present, "Mismatch: channel2_field_present");
	zassert_equal(original->max_len_type_is_slots, parsed->max_len_type_is_slots, "Mismatch: max_len_type_is_slots");
	zassert_equal(original->dect_delay_for_response, parsed->dect_delay_for_response, "Mismatch: dect_delay_for_response");
	zassert_equal(original->start_subslot_index, parsed->start_subslot_index, "Mismatch: start_subslot_index");
	zassert_equal(original->length_type_is_slots, parsed->length_type_is_slots, "Mismatch: length_type_is_slots");
	zassert_equal(original->num_subslots_or_slots, parsed->num_subslots_or_slots, "Mismatch: num_subslots_or_slots");
	zassert_equal(original->max_rach_pdu_len_units, parsed->max_rach_pdu_len_units, "Mismatch: max_rach_pdu_len_units");
	zassert_equal(original->cwmin_sig_code, parsed->cwmin_sig_code, "Mismatch: cwmin_sig_code");
	zassert_equal(original->cwmax_sig_code, parsed->cwmax_sig_code, "Mismatch: cwmax_sig_code");
	zassert_equal(original->repetition_code, parsed->repetition_code, "Mismatch: repetition_code");
	zassert_equal(original->response_window_subslots_val_minus_1, parsed->response_window_subslots_val_minus_1, "Mismatch: response_window_subslots_val_minus_1");

	if (original->sfn_validity_present) {
		zassert_equal(original->sfn_value, parsed->sfn_value, "Mismatch: sfn_value");
		zassert_equal(original->validity_frames, parsed->validity_frames, "Mismatch: validity_frames");
	}
	if (original->channel_field_present) {
		zassert_equal(original->channel_abs_freq_num, parsed->channel_abs_freq_num, "Mismatch: channel_abs_freq_num");
	}
	if (original->channel2_field_present) {
		zassert_equal(original->channel2_abs_freq_num, parsed->channel2_abs_freq_num, "Mismatch: channel2_abs_freq_num");
	}
	zassert_equal(original->mu_value_for_ft_beacon, parsed->mu_value_for_ft_beacon, "Mismatch: mu_value_for_ft_beacon");
}

static void run_and_verify_rach_ie_test(const dect_mac_rach_info_ie_fields_t *original_fields)
{
	uint8_t serialized_buf[64];
	dect_mac_rach_info_ie_fields_t parsed_fields;
	int serialized_len;
	int parse_ret;

	serialized_len = serialize_rach_info_ie_payload(serialized_buf, sizeof(serialized_buf), original_fields);
	zassert_true(serialized_len > 0, "Serialization failed with error %d", serialized_len);

	parse_ret = parse_rach_info_ie_payload(serialized_buf, (uint16_t)serialized_len,
					       original_fields->mu_value_for_ft_beacon, &parsed_fields);
	zassert_ok(parse_ret, "Parsing failed with error %d", parse_ret);

	verify_rach_fields(original_fields, &parsed_fields);
}

ZTEST(rach_ie_tests, test_rach_ie_basic_mu1)
{
	dect_mac_rach_info_ie_fields_t tc = {
		.mu_value_for_ft_beacon = 1,
		.repeat_type_is_subslots = false, .sfn_validity_present = false,
		.channel_field_present = false, .channel2_field_present = false,
		.max_len_type_is_slots = false, .dect_delay_for_response = false,
		.start_subslot_index = 10, .length_type_is_slots = false, .num_subslots_or_slots = 4,
		.max_rach_pdu_len_units = 20,
		.cwmin_sig_code = 2, .cwmax_sig_code = 5, .repetition_code = 0,
		.response_window_subslots_val_minus_1 = 47,
	};
	run_and_verify_rach_ie_test(&tc);
}

ZTEST(rach_ie_tests, test_rach_ie_adv_mu8)
{
	dect_mac_rach_info_ie_fields_t tc = {
		.mu_value_for_ft_beacon = 3, /* mu=8 */
		.repeat_type_is_subslots = true, .sfn_validity_present = true,
		.channel_field_present = true, .channel2_field_present = true,
		.max_len_type_is_slots = true, .dect_delay_for_response = true,
		.start_subslot_index = 300,
		.length_type_is_slots = true, .num_subslots_or_slots = 2,
		.max_rach_pdu_len_units = 10,
		.cwmin_sig_code = 1, .cwmax_sig_code = 4, .repetition_code = 1,
		.response_window_subslots_val_minus_1 = 23,
		.sfn_value = 100, .validity_frames = 200,
		.channel_abs_freq_num = 1234, .channel2_abs_freq_num = 1235,
	};
	run_and_verify_rach_ie_test(&tc);
}

ZTEST(rach_ie_tests, test_rach_ie_edge_ss8)
{
	dect_mac_rach_info_ie_fields_t tc = {
		.mu_value_for_ft_beacon = 1,
		.start_subslot_index = 255,
	};
	run_and_verify_rach_ie_test(&tc);
}

ZTEST(rach_ie_tests, test_rach_ie_edge_ss9)
{
	dect_mac_rach_info_ie_fields_t tc = {
		.mu_value_for_ft_beacon = 3, /* mu=8 */
		.start_subslot_index = 511,
	};
	run_and_verify_rach_ie_test(&tc);
}

ZTEST_SUITE(rach_ie_tests, NULL, NULL, NULL, NULL, NULL);