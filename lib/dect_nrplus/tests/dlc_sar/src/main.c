/* This is a new file. It contains a Ztest suite for verifying the DLC layer's Segmentation and Reassembly (SAR) functionality, ensuring large packets are handled correctly. */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

#include "dect_dlc.h"
#include "dect_mac_api.h"
#include "dect_mac_context.h"

LOG_MODULE_REGISTER(test_dlc_sar, LOG_LEVEL_DBG);

/* Mock the MAC API send function to capture segmented PDUs */
static struct k_fifo g_mock_mac_tx_fifo;
static int mock_dect_mac_api_send(mac_sdu_t *sdu, mac_flow_id_t flow)
{
	k_fifo_put(&g_mock_mac_tx_fifo, sdu);
	return 0;
}

/* Mock the DLC receive function to capture reassembled SDUs */
static struct k_fifo g_mock_cvg_rx_fifo;
int dect_cvg_receive(uint8_t *app_sdu_buf, size_t *len_inout, k_timeout_t timeout)
{
	mac_sdu_t *sdu_buf = k_fifo_get(&g_mock_cvg_rx_fifo, timeout);
	if (!sdu_buf) {
		return -EAGAIN;
	}
	*len_inout = sdu_buf->len;
	memcpy(app_sdu_buf, sdu_buf->data, sdu_buf->len);
	dect_mac_api_buffer_free(sdu_buf);
	return 0;
}

/* Override the real MAC send function with our mock */
#define dect_mac_api_send mock_dect_mac_api_send

/* Test Fixture */
static void *setup(void)
{
	k_fifo_init(&g_mock_mac_tx_fifo);
	k_fifo_init(&g_mock_cvg_rx_fifo);
	/* We need to init the real modules to use their buffer pools and threads */
	dect_mac_api_init(&g_dlc_internal_mac_rx_fifo);
	dect_dlc_init();
	return NULL;
}

static void teardown(void *data)
{
	ARG_UNUSED(data);
	mac_sdu_t *sdu;
	while ((sdu = k_fifo_get(&g_mock_mac_tx_fifo, K_NO_WAIT)) != NULL) {
		dect_mac_api_buffer_free(sdu);
	}
	while ((sdu = k_fifo_get(&g_mock_cvg_rx_fifo, K_NO_WAIT)) != NULL) {
		dect_mac_api_buffer_free(sdu);
	}
}

ZTEST(dlc_sar_tests, test_dlc_segmentation_and_reassembly)
{
	uint8_t large_payload[1000];
	for (int i = 0; i < sizeof(large_payload); i++) {
		large_payload[i] = (uint8_t)i;
	}

	/* Send a large payload that requires segmentation */
	int ret = dlc_send_data(DLC_SERVICE_TYPE_1_SEGMENTATION, 0x11223344,
				large_payload, sizeof(large_payload));
	zassert_ok(ret, "dlc_send_data failed");

	/* Collect all the segmented MAC SDUs */
	mac_sdu_t *segment;
	int segment_count = 0;
	while ((segment = k_fifo_get(&g_mock_mac_tx_fifo, K_MSEC(100))) != NULL) {
		segment_count++;
		LOG_DBG("Captured segment %d, len %u", segment_count, segment->len);
		/* Feed the segment back into the DLC's RX path */
		k_fifo_put(&g_dlc_internal_mac_rx_fifo, segment);
	}
	zassert_true(segment_count > 1, "Payload was not segmented");

	/* Receive the reassembled payload */
	uint8_t reassembled_payload[sizeof(large_payload)];
	size_t reassembled_len = sizeof(reassembled_payload);
	dlc_service_type_t received_service;

	ret = dlc_receive_data(&received_service, reassembled_payload, &reassembled_len, K_SECONDS(1));
	zassert_ok(ret, "dlc_receive_data failed with %d", ret);

	/* Verify correctness */
	zassert_equal(reassembled_len, sizeof(large_payload), "Reassembled length mismatch");
	zassert_mem_equal(reassembled_payload, large_payload, sizeof(large_payload), "Reassembled payload mismatch");
}

ZTEST_SUITE(dlc_sar_tests, NULL, setup, NULL, NULL, teardown);