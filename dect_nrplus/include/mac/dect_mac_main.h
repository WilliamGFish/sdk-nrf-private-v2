/* dect_mac/dect_mac_core.h */
#ifndef DECT_MAC_MAIN_H__
#define DECT_MAC_MAIN_H__


enum nrf_modem_dect_phy_feedback_format {
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_NONE = 0,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_1 = 1,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_2 = 2,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_3 = 3,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_4 = 4,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_5 = 5,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_6 = 6,
	NRF_MODEM_DECT_PHY_FEEDBACK_FORMAT_7 = 7,
};


/**
 * The main application entry point.
 */
int dect_nrplus_init(void);

/* Called by the FT to build its initial configuration data. */
void dect_cdd_ft_build_own_config(void);

/**
 * @brief Initializes the entire DECT NR+ protocol stack.
 *
 * This function is the single entry point for an application to initialize
 * the MAC, DLC, and CVG layers in the correct order.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int dect_nrplus_stack_init(void);

/**
 * @brief Starts the DECT NR+ stack's operation.
 *
 * This function starts the MAC state machine (scanning for a PT, or
 * beaconing for an FT). It should be called after a successful init.
 */
void dect_nrplus_stack_start(void);

#endif /* DECT_MAC_MAIN_H__ */