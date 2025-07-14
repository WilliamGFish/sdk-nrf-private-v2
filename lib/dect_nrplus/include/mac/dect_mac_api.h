/* dect_mac/dect_mac_api.h */
#ifndef DECT_MAC_API_H_
#define DECT_MAC_API_H_

#include <zephyr/kernel.h> // For k_fifo, k_timeout_t
#include <stdint.h>        // For uintxx_t types
#include <stddef.h>        // For size_t
#include <zephyr/sys/dlist.h>

#include <mac/dect_mac_core.h>
#include <mac/dect_mac_context.h>   // For dect_mac_context_t structure and sub-types


// Define the number of SDU buffers available in the slab.
// This should be sized based on expected throughput, number of HARQ processes,
// and desired depth of TX queues.
#ifndef MAX_MAC_SDU_BUFFERS_CONFIG 
// Example: 8 HARQ processes + 8 for TX queues
#define MAX_MAC_SDU_BUFFERS_CONFIG 16 
#endif

// This Kconfig should be defined in prj.conf or a central Kconfig.deci
// It defines the max size of the data[] array within mac_sdu_t,
// which holds a DLC PDU (the MAC's SDU). Max ETSI MAC SDU payload is 1636 octets.
// Note: The full MAC PDU can be larger due to MAC headers (1 octet type + common_header + MUX_header(s) + MIC).
// CONFIG_DECT_MAC_SDU_MAX_SIZE refers to the capacity of mac_sdu_t.data[] for the DLC PDU.
#ifndef CONFIG_DECT_MAC_SDU_MAX_SIZE
#define CONFIG_DECT_MAC_SDU_MAX_SIZE 1636
#endif

// Declare the memory slab for mac_sdu_t buffers shared between DLC and MAC.
extern struct k_mem_slab g_mac_sdu_slab;

/* Dlist declares*/
extern sys_dlist_t g_mac_tx_dlist_high_priority;
extern sys_dlist_t g_mac_tx_dlist_reliable_data;
extern sys_dlist_t g_mac_tx_dlist_best_effort;
extern sys_dlist_t * const mac_tx_dlists[MAC_FLOW_COUNT];

/**
 * @brief Initialize the MAC data plane API.
 *
 * This function must be called by the DLC layer once at startup. It provides
 * the MAC layer with the DLC's dlist for receiving MAC SDUs (DLC PDUs).
 *
 * @param rx_dlist_from_dlc A pointer to a sys_dlist_t initialized by the DLC.
 * @return 0 on success, negative error code on failure.
 */
int dect_mac_api_init(sys_dlist_t *rx_dlist_from_dlc);

/**
 * @brief Allocate a buffer for a new MAC SDU (which will contain a DLC PDU).
 *
 * This function gets a free buffer from the MAC's internal memory slab. The DLC layer
 * should fill this buffer with a DLC PDU and then pass it to the appropriate send function.
 *
 * @param timeout The time to wait for a free buffer. Use K_NO_WAIT for non-blocking,
 *                K_FOREVER to wait indefinitely.
 * @return Pointer to a mac_sdu_t buffer, or NULL if no buffer is available within the timeout.
 */
mac_sdu_t* dect_mac_api_buffer_alloc(k_timeout_t timeout);

/**
 * @brief Free a MAC SDU buffer.
 *
 * This should be called by the DLC layer after it has finished processing a received
 * MAC SDU (DLC PDU) that it retrieved from its rx_fifo. This returns the buffer to the MAC's pool.
 * Also used by DLC to free buffers allocated for TX if sending fails before queueing to MAC,
 * or if an SDU is allocated but not used.
 *
 * @param sdu Pointer to the SDU buffer to free. Must have been allocated by `dect_mac_api_buffer_alloc`.
 */
void dect_mac_api_buffer_free(mac_sdu_t *sdu);

/**
 * @brief (PT ROLE or generic MAC internal) Send a prepared MAC SDU (containing a DLC PDU)
 *        to the MAC layer for transmission with a specific QoS flow.
 *
 * For a PT, the target is implicitly its associated FT.
 * The MAC layer takes ownership of the buffer. The caller (DLC/MAC internal) must not access it after
 * this call. The MAC will free the buffer back to the slab once transmission is complete
 * (or HARQ processing finishes/fails).
 *
 * @param sdu Pointer to the SDU buffer (obtained from dect_mac_api_buffer_alloc),
 *            where sdu->data contains the DLC PDU and sdu->len is its length.
 *            sdu->target_peer_short_rd_id can be 0 or the associated FT's ID for PT.
 * @param flow The Quality of Service flow to use for this SDU.
 * @return 0 on success.
 * @retval -ENOMEM If the specified MAC TX queue is full.
 * @retval -EINVAL For invalid parameters (NULL SDU, invalid flow, invalid SDU length).
 */
int dect_mac_api_send(mac_sdu_t *sdu, mac_flow_id_t flow);

/**
 * @brief (FT ROLE ONLY) Send a prepared MAC SDU (containing a DLC PDU) to a specific
 *        Portable Termination (PT) that is connected to this FT.
 *
 * The MAC layer takes ownership of the buffer.
 *
 * @param sdu Pointer to the SDU buffer (obtained from dect_mac_api_buffer_alloc).
 *            sdu->data contains the DLC PDU, sdu->len is its length.
 * @param flow The Quality of Service flow to use for this SDU.
 * @param target_pt_short_rd_id The 16-bit Short RD ID of the target PT.
 * @return 0 on success.
 * @retval -EPERM If called when MAC role is not FT.
 * @retval -EINVAL Invalid parameters (NULL SDU, invalid flow, invalid SDU length,
 *                   unknown or invalid target_pt_short_rd_id).
 * @retval -ENOMEM If the specified MAC TX queue for that PT/flow is full.
 * @retval -ENOTCONN If the target PT is not currently connected/known to the FT.
 */
int dect_mac_api_ft_send_to_pt(mac_sdu_t *sdu, mac_flow_id_t flow, uint16_t target_pt_short_rd_id);


/**
 * @brief Request the MAC to have the PT enter a low-power paging cycle.
 *
 * This is only applicable for a device in the PT role that is already associated.
 * The device will turn off its radio for long intervals, waking only briefly
 * to check for pages from the FT. This is key for battery-powered applications.
 * The actual transition is handled by the PT state machine upon receiving an
 * internal command event triggered by this API call.
 *
 * @return 0 on success (command queued to MAC).
 * @retval -EIO If the command could not be queued to the MAC thread.
 * @retval -EPERM If called when MAC role is not PT or PT is not in a state where paging can be entered.
 *                (Initial check is for role; SM handles state validity).
 */
int dect_mac_api_enter_paging_mode(void);

/**
 * @brief (FT ROLE ONLY) Request to page a connected Portable Termination.
 *
 * This function flags a specific PT to be paged. The MAC layer will then
 * include a Broadcast Indication IE in its next available beacon to wake
 * up the PT.
 *
 * @param target_pt_long_rd_id The Long RD ID of the PT to be paged.
 * @return 0 on success (PT flagged for paging).
 * @retval -EPERM If called when MAC role is not FT.
 * @retval -ENOTCONN If the specified PT is not currently associated.
 */
int dect_mac_api_ft_page_pt(uint32_t target_pt_long_rd_id);


#endif /* DECT_MAC_API_H__ */