#include <zephyr/ztest.h>
#include <zephyr/kernel.h> // For k_fifo, k_timeout_t
#include <stdint.h>        // For uintxx_t types
#include <stddef.h>        // For size_t

#include <mocks/mock_nrf_modem_dect_phy.h>
#include <mac/dect_mac_api.h>

/* --- Test Harness State & Mocks --- */
struct mac_flow_fixture {
	dect_mac_context_t ft_ctx;
	dect_mac_context_t pt_ctx;
	dect_mac_context_t *current_ctx;
};

static struct mac_flow_fixture g_harness;

K_MSGQ_DEFINE(dect_mac_event_msgq, sizeof(struct dect_mac_event_msg), 32, 4);

dect_mac_context_t *dect_get_mac_context(void)
{
	return g_harness.current_ctx;
}

// Event handler for mock PHY events
static void dect_mac_phy_if_event_handler(const struct nrf_modem_dect_phy_event *event) {
    zassert_not_null(event, "Event is NULL");
    printf("[Test] Received event: type=%d\n", event->id);
}

/* --- Test Fixture --- */
static void *setup(void)
{
	mock_phy_reset();
	// The event handler is now public via dect_mac_phy_if.h
	nrf_modem_dect_phy_event_handler_set(dect_mac_phy_if_event_handler);
	return &g_harness;
}

static void before(void *data)
{
	// struct mac_flow_fixture *fixture = data;
	// struct dect_mac_event_msg msg;
	// while (k_msgq_get(&dect_mac_event_msgq, &msg, K_NO_WAIT) == 0) { /* drain */
	// }

	// g_harness.current_ctx = &fixture->ft_ctx;
	// dect_mac_core_init(MAC_ROLE_FT, 0x11223344);
	// g_harness.current_ctx = &fixture->pt_ctx;
	// dect_mac_core_init(MAC_ROLE_PT, 0xAABBCCDD);

	// // Register scheduler hooks for the test
	// dect_mac_data_path_register_scheduler_hook(dect_mac_data_path_service_tx);
}

static void teardown(void *data)
{
	ARG_UNUSED(data);
	/* This function is required by ZTEST_SUITE but can be empty if no cleanup is needed. */
}




ZTEST_SUITE(mac_flow_tests, NULL, setup, before, teardown, NULL);
// ZTEST_SUITE(mac_flow_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(mac_flow_tests, test_phy_init) {
    mock_phy_reset();
    zassert_equal(nrf_modem_dect_phy_init(), 0, "Failed to initialize mock PHY");
}

ZTEST(mac_flow_tests, test_mac_api_init) {
    sys_dlist_t rx_dlist;
    sys_dlist_init(&rx_dlist);
    zassert_equal(dect_mac_api_init(&rx_dlist), 0, "Failed to initialize MAC API");
}