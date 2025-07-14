/* dect_mac/dect_mac_nvs.c */
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/device.h>
#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/zms.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include <mac/dect_mac_nvs.h>
LOG_MODULE_REGISTER(dect_mac_nvs, CONFIG_DECT_MAC_NVS_LOG_LEVEL);

/* Define a unique key ID for the HPC value within the ZMS namespace. */
#define ZMS_KEY_ID_HPC 1

/* ZMS filesystem instance. */
static struct zms_fs fs;

/* In-memory cache of the HPC value. */
static uint32_t g_mac_hpc;
// static char g_mac_hpc[32];

int dect_mac_nvs_init(void)
{
	int rc;
	struct flash_pages_info info;

	/* Define the ZMS storage partition from the device tree.
	 * This requires a "storage_partition" with a compatible="zephyr,zms-storage"
	 * in your board's .dts file.
	 */
	fs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s is not ready", fs.flash_device->name);
		return -ENODEV;
	}

	// fs.mount_point = "/zms"; // Mount point is required by the API
	fs.offset = FIXED_PARTITION_OFFSET(storage_partition);

	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		printk("Unable to get page info, rc=%d\n", rc);
		return 0;
	}
	fs.sector_size = info.size;
	fs.sector_count = 3U; //TODO: WHAT SIZE SHOULD IT BE

// fs.sector_size = CONFIG_DECT_MAC_NVS_ZMS_BLOCK_SIZE; // Must be configured
	
	rc = zms_mount(&fs);;
	if (rc) {
		LOG_ERR("ZMS init failed: %d", rc);
		return rc;
	}

	/* Try to read the saved HPC value. */
	ssize_t bytes_read = zms_read(&fs, ZMS_KEY_ID_HPC, &g_mac_hpc, sizeof(g_mac_hpc));
	if (bytes_read == sizeof(g_mac_hpc)) {
		LOG_INF("Loaded HPC from ZMS: %u", g_mac_hpc);
	} else {
		/* If not found or error, initialize to a safe default. */
		LOG_WRN("HPC not found in ZMS (err: %d). Initializing to 1.", (int)bytes_read);
		g_mac_hpc = 1;
	}

	return 0;
}

uint32_t dect_mac_nvs_get_hpc(void)
{
	return g_mac_hpc;
}

int dect_mac_nvs_save_hpc(uint32_t hpc)
{
	g_mac_hpc = hpc;

	int rc = zms_write(&fs, ZMS_KEY_ID_HPC, &g_mac_hpc, sizeof(g_mac_hpc));
	if (rc) {
		LOG_ERR("Failed to write HPC to ZMS (key=%u): %d", ZMS_KEY_ID_HPC, rc);
		return rc;
	}

	LOG_DBG("HPC value %u saved to ZMS.", hpc);
	return 0;
}