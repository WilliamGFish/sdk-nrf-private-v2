#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include "dect_mac_pdu.h"
#include "dect_mac_context.h"

LOG_MODULE_REGISTER(test_mac_ies, LOG_LEVEL_DBG);

/* --- Test Cases --- */

ZTEST(mac_ie_tests, test_resource_allocation_ie)
{
	uint8_t buf[32];
	dect_mac_resource_alloc_ie_fields_t original, parsed;

	memset(&original, 0, sizeof(original));
	original.alloc_type_val = RES_ALLOC_TYPE_BIDIR;
	original.id_present = true;
	original.short_rd_id_val = 0xABCD;
	original.repeat_val = RES_ALLOC_REPEAT_FRAMES;
	original.repetition_value = 10;
	original.validity_value = 100;
	original.sfn_present = true;
	original.sfn_val = 200;
	original.res1_is_9bit_subslot = true;
	original.start_subslot_val_res1 = 300;
	original.length_val_res1 = 5;
	original.res2_is_9bit_subslot = true;
	original.start_subslot_val_res2 = 400;
	original.length_val_res2 = 3;

	int len = serialize_resource_alloc_ie_payload(buf, sizeof(buf), &original);
	zassert_true(len > 0, "serialize_resource_alloc_ie_payload failed");

	int ret = parse_resource_alloc_ie_payload(buf, len, 8, &parsed);
	zassert_ok(ret, "parse_resource_alloc_ie_payload failed");

	zassert_equal(original.alloc_type_val, parsed.alloc_type_val);
	zassert_equal(original.id_present, parsed.id_present);
	zassert_equal(original.short_rd_id_val, parsed.short_rd_id_val);
	zassert_equal(original.start_subslot_val_res1, parsed.start_subslot_val_res1);
}

ZTEST(mac_ie_tests, test_rd_capability_ie)
{
	uint8_t buf[64];
	dect_mac_rd_capability_ie_t original, parsed;

	memset(&original, 0, sizeof(original));
	original.num_phy_capabilities = 1;
	original.release_version = 1;
	original.supports_paging = true;
	original.operating_modes_code = DECT_MAC_OP_MODE_BOTH;
	original.phy_variants[0].mu_value = 1;
	original.phy_variants[0].beta_value = 0;
	original.phy_variants[0].max_mcs_code = 11;

	int len = serialize_rd_capability_ie_payload(buf, sizeof(buf), &original);
	zassert_true(len > 0, "serialize_rd_capability_ie_payload failed");

	int ret = parse_rd_capability_ie_payload(buf, len, &parsed);
	zassert_ok(ret, "parse_rd_capability_ie_payload failed");

	zassert_equal(original.num_phy_capabilities, parsed.num_phy_capabilities);
	zassert_equal(original.operating_modes_code, parsed.operating_modes_code);
	zassert_equal(original.phy_variants[0].max_mcs_code, parsed.phy_variants[0].max_mcs_code);
}

ZTEST(mac_ie_tests, test_group_assignment_ie)
{
	uint8_t buf[16];
	uint8_t tags[] = {10, 20, 30};
	int len = build_group_assignment_ie_muxed(buf, sizeof(buf), false, false, 5, tags, 3);
	zassert_true(len > 0, "build_group_assignment_ie_muxed failed");
	/* Parsing logic for this IE is not implemented in the SMs yet, so no parse test */
}

ZTEST(mac_ie_tests, test_auth_ies)
{
	uint8_t buf[64];
	int len;

	/* Test Auth Initiate */
	len = build_auth_initiate_ie_muxed(buf, sizeof(buf), 0x12345678);
	zassert_true(len > 0, "build_auth_initiate_ie_muxed failed");

	/* Test Auth Challenge */
	len = build_auth_challenge_ie_muxed(buf, sizeof(buf), 0x12345678, 0x87654321);
	zassert_true(len > 0, "build_auth_challenge_ie_muxed failed");

	/* Test Auth Response */
	uint8_t test_mac[DECT_MAC_AUTH_MAC_SIZE] = {1,2,3,4,5,6,7,8};
	len = build_auth_response_ie_muxed(buf, sizeof(buf), test_mac);
	zassert_true(len > 0, "build_auth_response_ie_muxed failed");

	/* Test Auth Success */
	len = build_auth_success_ie_muxed(buf, sizeof(buf), test_mac);
	zassert_true(len > 0, "build_auth_success_ie_muxed failed");
}

ZTEST_SUITE(mac_ie_tests, NULL, NULL, NULL, NULL, NULL);