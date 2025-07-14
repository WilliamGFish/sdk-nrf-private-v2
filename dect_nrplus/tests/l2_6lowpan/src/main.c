/* lib/dect_nrplus/tests/l2_6lowpan/src/main.c */
/* This is a new file. It contains a Ztest suite for verifying the L2 driver's integration with the 6LoWPAN stack, including TX compression and RX decompression paths. */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/udp.h>
#include <zephyr/net/sixlowpan.h>

#include "dect_mac_core.h"
#include "dect_mac_context.h"
#include "dect_cvg.h"

LOG_MODULE_REGISTER(test_l2_6lowpan, LOG_LEVEL_DBG);

/* --- Mocks --- */
static struct {
	uint16_t endpoint_id;
	uint32_t dest_long_id;
	uint8_t payload[256];
	size_t len;
	int call_count;
} g_mock_cvg_send;

static struct k_fifo g_mock_cvg_rx_fifo;

int dect_cvg_send(uint16_t endpoint_id, uint32_t dest_long_id, const uint8_t *app_sdu,
		  size_t app_sdu_len)
{
	g_mock_cvg_send.call_count++;
	g_mock_cvg_send.endpoint_id = endpoint_id;
	g_mock_cvg_send.dest_long_id = dest_long_id;
	g_mock_cvg_send.len = MIN(app_sdu_len, sizeof(g_mock_cvg_send.payload));
	memcpy(g_mock_cvg_send.payload, app_sdu, g_mock_cvg_send.len);
	return 0;
}

int dect_cvg_receive(uint8_t *app_sdu_buf, size_t *len_inout, k_timeout_t timeout)
{
	mac_sdu_t *sdu_buf = k_fifo_get(&g_mock_cvg_rx_fifo, timeout);
	if (!sdu_buf) {
		return -EAGAIN;
	}
	*len_inout = sdu_buf->len;
	memcpy(app_sdu_buf, sdu_buf->data, sdu_buf->len);
	k_free(sdu_buf);
	return 0;
}

/* --- Test Fixture --- */
struct l2_test_fixture {
	struct net_if *iface;
	struct sockaddr_in6 my_addr;
	struct sockaddr_in6 peer_addr;
	struct k_sem rx_sem;
	uint8_t received_data[128];
	size_t received_len;
};

static void udp_recv_cb(struct net_context *context, struct net_pkt *pkt,
			union net_ip_header *ip_hdr, union net_udp_header *udp_hdr,
			int status, void *user_data)
{
	struct l2_test_fixture *fixture = user_data;
	if (!pkt) {
		return;
	}

	fixture->received_len = net_pkt_get_len(pkt);
	net_pkt_read(pkt, fixture->received_data, fixture->received_len);
	net_pkt_unref(pkt);
	k_sem_give(&fixture->rx_sem);
}

static void *setup(void)
{
	static struct l2_test_fixture fixture;
	memset(&fixture, 0, sizeof(fixture));
	k_sem_init(&fixture.rx_sem, 0, 1);

	fixture.iface = net_if_get_default();
	zassert_not_null(fixture.iface, "Default network interface not found");

	/* Set a fake IID to make the interface seem ready */
	uint8_t iid[8] = {0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB, 0xCC, 0xDD};
	net_if_set_link_addr(fixture.iface, iid, sizeof(iid), NET_LINK_IEEE802154);

	/* Set up addresses */
	zassert_ok(net_addr_pton(AF_INET6, "2001:db8::1", &fixture.my_addr.sin6_addr));
	fixture.my_addr.sin6_family = AF_INET6;
	fixture.my_addr.sin6_port = sys_cpu_to_be16(12345);

	zassert_ok(net_addr_pton(AF_INET6, "2001:db8::2", &fixture.peer_addr.sin6_addr));
	fixture.peer_addr.sin6_family = AF_INET6;
	fixture.peer_addr.sin6_port = sys_cpu_to_be16(54321);

	/* Set 6lo context */
	struct in6_addr prefix;
	zassert_ok(net_addr_pton(AF_INET6, "2001:db8::", &prefix));
	zassert_ok(sixlowpan_set_context(1, &prefix), "Failed to set 6lo context");

	/* Mock the MAC context to be associated */
	dect_mac_context_t *mac_ctx = get_mac_context();
	mac_ctx->state = MAC_STATE_ASSOCIATED;
	mac_ctx->role = MAC_ROLE_PT;
	mac_ctx->own_long_rd_id = 0xAABBCCDD;
	mac_ctx->role_ctx.pt.associated_ft.long_rd_id = 0x11223344;

	return &fixture;
}

/* --- Test Cases --- */

ZTEST_F(l2_test_fixture, test_l2_send_path)
{
	struct net_context *udp_ctx;
	const char *test_payload = "Hello 6LoWPAN";

	zassert_ok(net_context_get(AF_INET6, SOCK_DGRAM, IPPROTO_UDP, &udp_ctx), "Cannot get context");
	zassert_ok(net_context_bind(udp_ctx, (struct sockaddr *)&fixture->my_addr, sizeof(fixture->my_addr)), "Cannot bind");

	/* Send a UDP packet */
	zassert_ok(net_context_sendto(udp_ctx, test_payload, strlen(test_payload),
				      (struct sockaddr *)&fixture->peer_addr, sizeof(fixture->peer_addr),
				      NULL, K_NO_WAIT, NULL), "Cannot send UDP");

	/* Check what was passed to CVG */
	zassert_equal(g_mock_cvg_send.call_count, 1, "dect_cvg_send was not called once");
	zassert_equal(g_mock_cvg_send.endpoint_id, CVG_EP_IPV6_PROFILE, "Incorrect CVG endpoint");
	zassert_true(g_mock_cvg_send.len > 0, "Compressed payload is empty");
	zassert_true(g_mock_cvg_send.len < strlen(test_payload) + 40, "Payload was not compressed");

	/* Check IPHC header: stateless compression of context-based address */
	/* 0x7A = 01111010 -> DAM=10 (IID), SAM=10 (IID), Contexts, NH=0 */
	zassert_equal(g_mock_cvg_send.payload[0], 0x7A, "Invalid IPHC header");

	net_context_put(udp_ctx);
}

ZTEST_F(l2_test_fixture, test_l2_receive_path)
{
	struct net_context *udp_ctx;
	const char *test_payload = "Hello From Mock";

	/* Set up UDP receiver */
	zassert_ok(net_context_get(AF_INET6, SOCK_DGRAM, IPPROTO_UDP, &udp_ctx), "Cannot get context");
	zassert_ok(net_context_bind(udp_ctx, (struct sockaddr *)&fixture->my_addr, sizeof(fixture->my_addr)), "Cannot bind");
	zassert_ok(net_context_recv(udp_ctx, udp_recv_cb, K_NO_WAIT, fixture), "Cannot set recv cb");

	/* Prepare a compressed 6LoWPAN packet to be "received" from CVG */
	uint8_t compressed_pkt[] = {
		0x7A, /* IPHC: DAM=IID, SAM=IID, Contexts */
		0x11, /* Context IDs: SCI=1, DCI=1 */
		0x11, /* NHC: UDP */
		0xD5, 0x39, /* UDP Ports: 54321 -> 12345 */
		0x00, 0x00, /* UDP Checksum placeholder */
		'H', 'e', 'l', 'l', 'o', ' ', 'F', 'r', 'o', 'm', ' ', 'M', 'o', 'c', 'k'
	};

	/* Simulate CVG delivering this packet to the L2 RX thread */
	mac_sdu_t *rx_sdu = k_malloc(sizeof(mac_sdu_t));
	zassert_not_null(rx_sdu, "Failed to alloc mock SDU");
	memcpy(rx_sdu->data, compressed_pkt, sizeof(compressed_pkt));
	rx_sdu->len = sizeof(compressed_pkt);
	k_fifo_put(&g_mock_cvg_rx_fifo, rx_sdu);

	/* Wait for the RX thread to process and deliver the packet */
	int ret = k_sem_take(&fixture->rx_sem, K_SECONDS(1));
	zassert_ok(ret, "Timed out waiting for UDP packet");

	/* Verify the received data */
	zassert_equal(fixture->received_len, strlen(test_payload), "Received length mismatch");
	zassert_mem_equal(fixture->received_data, test_payload, strlen(test_payload), "Received payload mismatch");

	net_context_put(udp_ctx);
}


ZTEST_F(l2_test_fixture, test_l2_fragmentation)
{
	struct net_context *udp_ctx;
	/* Create a payload larger than a typical single MAC frame to force fragmentation */
	uint8_t large_payload[500];
	for (int i = 0; i < sizeof(large_payload); i++) {
		large_payload[i] = i % 256;
	}

	zassert_ok(net_context_get(AF_INET6, SOCK_DGRAM, IPPROTO_UDP, &udp_ctx), "Cannot get context");

	/* Send a large UDP packet */
	zassert_ok(net_context_sendto(udp_ctx, large_payload, sizeof(large_payload),
				      (struct sockaddr *)&fixture->peer_addr, sizeof(fixture->peer_addr),
				      NULL, K_NO_WAIT, NULL), "Cannot send large UDP");

	/* Verify that dect_cvg_send was called multiple times */
	zassert_true(g_mock_cvg_send.call_count > 1, "dect_cvg_send was not called multiple times for a large packet (fragmentation failed)");
	TC_PRINT("Fragmentation test passed: %d fragments sent to CVG.\n", g_mock_cvg_send.call_count);

	net_context_put(udp_ctx);
}

ZTEST_F(l2_test_fixture, test_l2_fragmentation)
{
	struct net_context *udp_ctx;
	/* Create a payload larger than a typical single MAC frame to force fragmentation */
	uint8_t large_payload[500];
	for (int i = 0; i < sizeof(large_payload); i++) {
		large_payload[i] = i % 256;
	}

	zassert_ok(net_context_get(AF_INET6, SOCK_DGRAM, IPPROTO_UDP, &udp_ctx), "Cannot get context");

	/* Send a large UDP packet */
	zassert_ok(net_context_sendto(udp_ctx, large_payload, sizeof(large_payload),
				      (struct sockaddr *)&fixture->peer_addr, sizeof(fixture->peer_addr),
				      NULL, K_NO_WAIT, NULL), "Cannot send large UDP");

	/* Verify that dect_cvg_send was called multiple times */
	zassert_true(g_mock_cvg_send.call_count > 1, "dect_cvg_send was not called multiple times for a large packet (fragmentation failed)");
	TC_PRINT("Fragmentation test passed: %d fragments sent to CVG.\n", g_mock_cvg_send.call_count);

	net_context_put(udp_ctx);
}


ZTEST_SUITE(l2_6lowpan_tests, NULL, setup, NULL, NULL, teardown);