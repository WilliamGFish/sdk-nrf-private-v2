/* tests/mac_pdu/test_assoc_ies/src/main.c */
// Overview: This file is converted from a standalone test program to a formal Ztest suite. `printk` based checks are replaced with `zassert` macros for automated verification. The test for `ft_mode_capable` fields is also included.
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include "dect_mac_pdu.h"
#include "dect_mac_context.h"

LOG_MODULE_REGISTER(test_assoc_ies, LOG_LEVEL_DBG);

static void run_and_verify_assoc_req_test(const dect_mac_assoc_req_ie_t *original)
{
	uint8_t buf[128];
	dect_mac_assoc_req_ie_t parsed;
	int len;

	len = serialize_assoc_req_ie_payload(buf, sizeof(buf), original);
	zassert_true(len > 0, "serialize_assoc_req_ie_payload failed with %d", len);

	int ret = parse_assoc_req_ie_payload(buf, len, &parsed);
	zassert_ok(ret, "parse_assoc_req_ie_payload failed with %d", ret);

	zassert_equal(original->setup_cause_val, parsed.setup_cause_val, "setup_cause_val mismatch");
	zassert_equal(original->number_of_flows_val, parsed.number_of_flows_val, "number_of_flows_val mismatch");
	zassert_equal(original->ft_mode_capable, parsed.ft_mode_capable, "ft_mode_capable mismatch");
	zassert_equal(original->power_const_active, parsed.power_const_active, "power_const_active mismatch");

	if (original->harq_params_present) {
		zassert_true(parsed.harq_params_present, "harq_params_present flag mismatch");
		zassert_equal(original->harq_processes_tx_val, parsed.harq_processes_tx_val, "harq_processes_tx_val mismatch");
		zassert_equal(original->max_harq_re_tx_delay_code, parsed.max_harq_re_tx_delay_code, "max_harq_re_tx_delay_code mismatch");
		zassert_equal(original->harq_processes_rx_val, parsed.harq_processes_rx_val, "harq_processes_rx_val mismatch");
		zassert_equal(original->max_harq_re_rx_delay_code, parsed.max_harq_re_rx_delay_code, "max_harq_re_rx_delay_code mismatch");
	}

	if (original->ft_mode_capable) {
		zassert_true(parsed.ft_mode_capable, "ft_mode_capable flag should be set");
		zassert_equal(original->ft_beacon_periods_octet_present,
			      parsed.ft_beacon_periods_octet_present,
			      "ft_beacon_periods_octet_present mismatch");
		if (original->ft_beacon_periods_octet_present) {
			zassert_equal(original->ft_network_beacon_period_code,
				      parsed.ft_network_beacon_period_code,
				      "ft_network_beacon_period_code mismatch");
			zassert_equal(original->ft_cluster_beacon_period_code,
				      parsed.ft_cluster_beacon_period_code,
				      "ft_cluster_beacon_period_code mismatch");
		}

		zassert_equal(original->ft_param_flags_octet_present,
			      parsed.ft_param_flags_octet_present,
			      "ft_param_flags_octet_present mismatch");
		if (original->ft_param_flags_octet_present) {
			zassert_equal(original->ft_next_channel_present,
				      parsed.ft_next_channel_present,
				      "ft_next_channel_present mismatch");
			zassert_equal(original->ft_time_to_next_present,
				      parsed.ft_time_to_next_present,
				      "ft_time_to_next_present mismatch");
			zassert_equal(original->ft_current_channel_present,
				      parsed.ft_current_channel_present,
				      "ft_current_channel_present mismatch");
		}

		if (original->ft_next_channel_present) {
			zassert_equal(original->ft_next_cluster_channel_val,
				      parsed.ft_next_cluster_channel_val,
				      "ft_next_cluster_channel_val mismatch");
		}
		if (original->ft_time_to_next_present) {
			zassert_equal(original->ft_time_to_next_us_val,
				      parsed.ft_time_to_next_us_val,
				      "ft_time_to_next_us_val mismatch");
		}
		if (original->ft_current_channel_present) {
			zassert_equal(original->ft_current_cluster_channel_val,
				      parsed.ft_current_cluster_channel_val,
				      "ft_current_cluster_channel_val mismatch");
		}
	}
}

static void run_and_verify_assoc_resp_test(const dect_mac_assoc_resp_ie_t *original)
{
	uint8_t buf[128];
	dect_mac_assoc_resp_ie_t parsed;
	int len;

	len = serialize_assoc_resp_ie_payload(buf, sizeof(buf), original);
	zassert_true(len > 0, "serialize_assoc_resp_ie_payload failed with %d", len);

	int ret = parse_assoc_resp_ie_payload(buf, len, &parsed);
	zassert_ok(ret, "parse_assoc_resp_ie_payload failed with %d", ret);

	zassert_equal(original->ack_nack, parsed.ack_nack, "ack_nack mismatch");

	if (!original->ack_nack) {
		zassert_equal(original->reject_cause, parsed.reject_cause, "reject_cause mismatch");
		zassert_equal(original->reject_timer_code, parsed.reject_timer_code, "reject_timer_code mismatch");
	} else {
		zassert_equal(original->harq_mod_present, parsed.harq_mod_present, "harq_mod_present mismatch");
		zassert_equal(original->number_of_flows_accepted, parsed.number_of_flows_accepted, "number_of_flows_accepted mismatch");
		zassert_equal(original->group_assignment_active, parsed.group_assignment_active, "group_assignment_active mismatch");
	}
}

ZTEST(assoc_ie_tests, test_assoc_req_basic)
{
	dect_mac_assoc_req_ie_t req = {
		.setup_cause_val = ASSOC_CAUSE_INITIAL_ASSOCIATION,
		.number_of_flows_val = 0,
		.ft_mode_capable = false,
		.power_const_active = false,
		.harq_params_present = false,
	};
	run_and_verify_assoc_req_test(&req);
}

ZTEST(assoc_ie_tests, test_assoc_req_full)
{
	dect_mac_assoc_req_ie_t req = {
		.setup_cause_val = ASSOC_CAUSE_MOBILITY,
		.number_of_flows_val = 2,
		.flow_ids = { 1, 5 },
		.ft_mode_capable = false,
		.power_const_active = true,
		.harq_params_present = true,
		.harq_processes_tx_val = 3,
		.max_harq_re_tx_delay_code = 15,
		.harq_processes_rx_val = 2,
		.max_harq_re_rx_delay_code = 10,
	};
	run_and_verify_assoc_req_test(&req);
}

ZTEST(assoc_ie_tests, test_assoc_req_ft_capable)
{
	dect_mac_assoc_req_ie_t req = {
		.setup_cause_val = ASSOC_CAUSE_INITIAL_ASSOCIATION,
		.number_of_flows_val = 0,
		.ft_mode_capable = true,
		.power_const_active = false,
		.harq_params_present = true,
		.harq_processes_tx_val = 1,
		.max_harq_re_tx_delay_code = 5,
		.harq_processes_rx_val = 1,
		.max_harq_re_rx_delay_code = 5,
		.ft_beacon_periods_octet_present = true,
		.ft_network_beacon_period_code = 3,
		.ft_cluster_beacon_period_code = 2,
		.ft_param_flags_octet_present = true,
		.ft_next_channel_present = true,
		.ft_time_to_next_present = true,
		.ft_current_channel_present = false,
		.ft_next_cluster_channel_val = 1886976,
		.ft_time_to_next_us_val = 5000000,
	};
	run_and_verify_assoc_req_test(&req);
}

ZTEST(assoc_ie_tests, test_assoc_resp_nack)
{
	dect_mac_assoc_resp_ie_t resp = {
		.ack_nack = false,
		.reject_cause = ASSOC_REJECT_CAUSE_NO_HW_CAP,
		.reject_timer_code = 5,
	};
	run_and_verify_assoc_resp_test(&resp);
}

ZTEST(assoc_ie_tests, test_assoc_resp_ack_basic)
{
	dect_mac_assoc_resp_ie_t resp = {
		.ack_nack = true,
		.harq_mod_present = false,
		.number_of_flows_accepted = 7, /* All flows accepted */
		.group_assignment_active = false,
	};
	run_and_verify_assoc_resp_test(&resp);
}

ZTEST(assoc_ie_tests, test_assoc_resp_ack_full)
{
	dect_mac_assoc_resp_ie_t resp = {
		.ack_nack = true,
		.harq_mod_present = true,
		.harq_processes_tx_val_ft = 1,
		.max_harq_re_tx_delay_code_ft = 20,
		.harq_processes_rx_val_ft = 1,
		.max_harq_re_rx_delay_code_ft = 18,
		.number_of_flows_accepted = 1,
		.accepted_flow_ids = { 3 },
		.group_assignment_active = true,
		.group_id_val = 10,
		.resource_tag_val = 5,
	};
	run_and_verify_assoc_resp_test(&resp);
}

ZTEST_SUITE(assoc_ie_tests, NULL, NULL, NULL, NULL, NULL);