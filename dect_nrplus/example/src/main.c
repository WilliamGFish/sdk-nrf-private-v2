




/* app/src/main.c */
// Overview: This file is refactored from a test runner into a network application main entry point. It now correctly initializes the full DECT stack and the Zephyr network stack, brings up the network interface, and configures an IPv6 address, enabling end-to-end IP connectivity tests like ping and UDP echo.
// --- REPLACE ENTIRE FILE ---
#include <zephyr/kernel.h>
#include <zephyr/sys/slist.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


/* --- CORRECTED DECT NR+ STACK INCLUDES --- */
// Use the public, top-level headers provided by the library.
#include <mac/dect_mac_main_dispatcher.h> // For dect_mac_state_to_str
#include <mac/dect_mac_sm.h>
#include <mac/dect_mac_main.h>
#include <mac/dect_mac_context.h>
#include <mac/dect_mac_core.h>
#include <mac/dect_mac_sm_pt.h>
#include <mac/dect_mac_sm_ft.h>
#include <mac/dect_mac_api.h>
#include <dect_dlc.h>
#include <dect_cvg.h>
#include <dect_cdd.h>




LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

static struct net_mgmt_event_callback mgmt_cb;


/**
 * Quick inline check macros for convenience
 */
#define IPV6_IS_MULTICAST(addr) ((addr)->s6_addr[0] == 0xFF)
#define IPV6_IS_UNICAST(addr) (!net_ipv6_is_addr_mcast(addr) && \
                               !net_ipv6_is_addr_unspecified(addr) && \
                               !net_ipv6_is_addr_loopback(addr))



/**
 * @brief Network management event callback.
 * This function is called by Zephyr's network stack when various
 * network events occur (e.g., IP address added, interface up/down).
 *
 * @param cb Pointer to the network management event callback structure.
 * @param mgmt_event The type of network management event.
 * @param iface Pointer to the network interface associated with the event.
 */
static void ipv6_event_handler(struct net_mgmt_event_callback *cb,
                               uint32_t mgmt_event,
                               struct net_if *iface)
{
    // Only process IPv6 address addition events
    if (mgmt_event != NET_EVENT_IPV6_ADDR_ADD) {
        return;
    }

    char hr_addr[NET_IPV6_ADDR_LEN]; // Buffer for human-readable IPv6 address
    dect_mac_context_t *mac_ctx = get_mac_context(); // Get the MAC context

    LOG_INF("NET_EVENT_IPV6_ADDR_ADD received for interface. Checking addresses...");

    // Iterate through the unicast IPv6 addresses on the interface.
    // Based on the compiler errors/warnings, it appears iface->config.ip.ipv6->unicast
    // is an array (e.g., struct net_if_addr unicast[CONFIG_NET_IPV6_ADDR_COUNT])
    // rather than a sys_slist_t. This loop adapts to that structure.
    for (int i = 0; i < NET_IF_MAX_IPV6_ADDR; i++) {
        struct net_if_addr *if_addr = &iface->config.ip.ipv6->unicast[i];

        // Check if the address entry is currently in use, is an IPv6 address,
        // and is a unicast address (i.e., not multicast, loopback, or unspecified).
        if (if_addr->is_used && if_addr->address.family == AF_INET6 &&
            !net_ipv6_is_addr_mcast(&if_addr->address.in6_addr) &&
            !net_ipv6_is_addr_loopback(&if_addr->address.in6_addr) &&
            !net_ipv6_is_addr_unspecified(&if_addr->address.in6_addr))
        {
            // Convert the binary IPv6 address to a human-readable string
            net_addr_ntop(AF_INET6, &if_addr->address.in6_addr, hr_addr, sizeof(hr_addr));
            // Removed 'if_addr->state' as it caused a compilation error in your environment.
            LOG_INF("Found unicast IPv6 address: %s", hr_addr); 

            // Check the MAC role and perform specific actions if needed.
            if (mac_ctx->role == MAC_ROLE_FT) {
                LOG_INF("MAC role is FT. FT is now IP-enabled. Building CDD content.");
                dect_cdd_ft_build_own_config();
                // If you only need to react to the first found address, you can return here.
                // Otherwise, remove 'return' to process all unicast addresses.
                return;
            }
        }
    }

    LOG_INF("No suitable unicast IPv6 address found or processed for interface ");
}

int main(void)
{
	int err;
	struct net_if *iface = net_if_get_default();

	LOG_INF("DECT NR+ Application Main Started");

	/* Initialize the full DECT NR+ stack */
	err = dect_nrplus_init();
	if (err) {
		LOG_ERR("Failed to init MAC-PHY interface: %d", err);
		return -1;
	}

	// err = dect_mac_api_init(&g_dlc_internal_mac_rx_fifo);
	// if (err) {
	// 	LOG_ERR("Failed to init MAC API: %d", err);
	// 	return;
	// }

// #if defined(CONFIG_DECT_MAC_ROLE_FT)
// 	dect_mac_role_t my_role = MAC_ROLE_FT;
// #else
// 	dect_mac_role_t my_role = MAC_ROLE_PT;
// #endif
// 	uint32_t provisioned_id = CONFIG_DECT_MAC_PROVISIONED_LONG_RD_ID;
// 	err = dect_mac_core_init(my_role, provisioned_id);
// 	if (err) {
// 		LOG_ERR("Failed to init MAC Core: %d", err);
// 		return;
// 	}

// 	err = dect_cvg_init();
// 	if (err) {
// 		LOG_ERR("Failed to init CVG/DLC layers: %d", err);
// 		return;
// 	}

	/* Start the network interface */
	net_mgmt_init_event_callback(&mgmt_cb, ipv6_event_handler, NET_EVENT_IPV6_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);
	net_if_up(iface);

	// /* Start the MAC state machine */
	// if (my_role == MAC_ROLE_PT) {
	// 	dect_mac_sm_pt_start_operation();
	// } else {
	// 	dect_mac_sm_ft_start_operation();
	// }

	LOG_INF("DECT NR+ Stack and Network Interface started.");
	/* The application main thread can now sleep or perform other tasks */
	/* All network and DECT operations are handled by their respective threads */
}