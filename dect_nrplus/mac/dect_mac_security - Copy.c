/* dect_mac/dect_mac_security.c */
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto.h> // For Zephyr Crypto API
// #include <zephyr/crypto/mac.h>	  // MAC (Message Authentication Code) API
#include <zephyr/sys/byteorder.h> // For sys_put_be32, sys_cpu_to_be16, sys_be16_to_cpu etc.
#include <string.h>               // For memcpy, memset

#include <mac/dect_mac_security.h>

LOG_MODULE_REGISTER(dect_mac_security, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);

// Global device handle for the Zephyr crypto driver.
// Ensure a PSA-based crypto driver is enabled, e.g., CONFIG_PSA_CRYPTO_DRIVER_CC3XX for nRF91.
// If using legacy crypto: DEVICE_DT_GET(DT_CHOSEN(zephyr_crypto_aes_ctr_cc3xx)) etc.
// For PSA, it's usually simpler as PSA API abstracts specific HW.
// However, direct Zephyr crypto API (cipher_begin_session) is used here, not PSA client API.
#if defined(CONFIG_CRYPTO_NRF_CC3XX_PSA) || defined(CONFIG_CRYPTO_MBEDTLS_PSA)
// If PSA is the backend for the legacy Zephyr crypto API
static const struct device *const g_crypto_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crypto));
#else
// Fallback or specific non-PSA driver if needed
static const struct device *const g_crypto_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crypto));
#endif


void security_build_iv(uint8_t *iv_out,
                       uint32_t transmitter_long_rd_id,
                       uint32_t receiver_long_rd_id,
                       uint32_t hpc,
                       uint16_t psn) // PSN is 12-bit
{
    if (!iv_out) {
        LOG_ERR("IV Build: Output buffer is NULL.");
        return;
    }
    // As per ETSI TS 103 636-4, Table 5.9.1.3-1
    // All fields are Big Endian in the IV.

    // Octets 0-3: Long RD ID of the transmitter
    sys_put_be32(transmitter_long_rd_id, &iv_out[0]);

    // Octets 4-7: Long RD ID of the receiver
    sys_put_be32(receiver_long_rd_id, &iv_out[4]);

    // Octets 8-11: Hyper Packet Counter (HPC)
    sys_put_be32(hpc, &iv_out[8]);

    // Octets 12-13: Packet Sequence Number (PSN) (12 MSBs) + Initial Counter Block (4 LSBs = 0)
    // PSN is 12 bits. It occupies bits 15-4 of this 16-bit field (0-indexed from MSB of the 16-bit field).
    // The 4 LSBs (bits 3-0) are part of the CTR mode's initial counter block, set to 0.
    uint16_t psn_field_value = (psn & 0x0FFF) << 4; // Shift PSN left by 4, LSBs become 0
    sys_put_be16(psn_field_value, &iv_out[12]);

    // Octets 14-15: Ciphering engine internal byte counter (for CTR mode).
    // Initialized to 0 for the first 16-byte block of the PDU.
    iv_out[14] = 0;
    iv_out[15] = 0;
}

int security_calculate_mic(const uint8_t *pdu_data_for_mic,
                           size_t pdu_data_len,
                           const uint8_t *integrity_key, /* 16 bytes */
                           uint8_t *mic_out_5_bytes)
{
    if (!pdu_data_for_mic || pdu_data_len == 0 || !integrity_key || !mic_out_5_bytes) {
        LOG_ERR("MIC Calc: Invalid parameters (NULL ptrs or zero len).");
        return -EINVAL;
    }
    if (!device_is_ready(g_crypto_dev)) {
        LOG_ERR("MIC Calc: Crypto device (%s) is not ready.", g_crypto_dev->name);
        return -ENODEV;
    }

    uint8_t full_mic_tag[16]; // AES-CMAC produces a 16-byte tag
    struct mac_ctx cmac_session_ctx; // Use the correct context struct for MAC
    int err;

    // Initialize context for CMAC using the MAC API
    err = mac_begin_session(g_crypto_dev, &cmac_session_ctx, CRYPTO_MAC_ALGO_CMAC,
                            (uint8_t *)integrity_key, 16, NULL);
    if (err != 0) {
        LOG_ERR("MIC Calc: Failed to begin AES-CMAC session, err: %d", err);
        return err;
    }

    // The mac_pkt struct is used to pass data and receive the result
    struct mac_pkt seq;
    uint8_t *p_data_buf[1];
    size_t p_data_len[1];

    p_data_buf[0] = (uint8_t *)pdu_data_for_mic;
    p_data_len[0] = pdu_data_len;
    seq.in_buf = p_data_buf;
    seq.in_num = 1;
    seq.out_buf = full_mic_tag; // Buffer to receive the full 16-byte tag

    // Perform the CMAC operation.
    err = mac_compute(&cmac_session_ctx, &seq);
    if (err != 0) {
        LOG_ERR("MIC Calc: Failed to perform AES-CMAC compute, err: %d", err);
        mac_free_session(g_crypto_dev, &cmac_session_ctx);
        return err;
    }

    mac_free_session(g_crypto_dev, &cmac_session_ctx);

    // Truncate the full 16-byte MIC to the 5 bytes required by ETSI spec.
    memcpy(mic_out_5_bytes, full_mic_tag, 5);

    return 0;
}

int security_crypt_payload(uint8_t *payload_in_out,
                           size_t len,
                           const uint8_t *cipher_key, /* 16 bytes */
                           uint8_t *iv,               /* 16 bytes, will be modified by CTR op */
                           bool encrypt)
{
    if (!payload_in_out || (len > 0 && !iv) || !cipher_key) {
        LOG_ERR("Crypt: Invalid parameters (NULL ptrs).");
        return -EINVAL;
    }
    if (len == 0) {
        return 0; // Nothing to encrypt/decrypt
    }
    if (!device_is_ready(g_crypto_dev)) {
        LOG_ERR("Crypt: Crypto device (%s) is not ready.", g_crypto_dev->name);
        return -ENODEV;
    }

    struct cipher_ctx crypto_session_ctx;
    int err;
    enum cipher_operator op_mode = encrypt ? CRYPTO_CIPHER_OP_ENCRYPT : CRYPTO_CIPHER_OP_DECRYPT;

    crypto_session_ctx.keylen = 16; // AES-128
    crypto_session_ctx.key.bit_stream = (uint8_t *)cipher_key;
    crypto_session_ctx.flags = CAP_RAW_KEY | CAP_INPLACE_OPS; // Enable in-place operation

    err = cipher_begin_session(g_crypto_dev, &crypto_session_ctx, CRYPTO_CIPHER_ALGO_AES,
                               CRYPTO_CIPHER_MODE_CTR, op_mode);
    if (err != 0) {
        LOG_ERR("Crypt: Failed to begin AES-CTR session (op: %s), err: %d",
                encrypt ? "encrypt" : "decrypt", err);
        return err;
    }

    // Perform the CTR operation. The IV is passed and will be updated by the driver.
    // The `cipher_ctr_op` expects `in_buf` and `out_buf`. For in-place, they are the same.
    err = cipher_pkt_op(&crypto_session_ctx, &(struct cipher_pkt){
                                                .in_buf = payload_in_out,
                                                .in_len = len,
                                                .out_buf = payload_in_out, // In-place
                                                .out_buf_max = len
                                            },
                        iv); // Pass IV here for CTR mode

    if (err != 0) {
        LOG_ERR("Crypt: Failed to perform AES-CTR operation (op: %s), err: %d",
                encrypt ? "encrypt" : "decrypt", err);
        // No session to free explicitly if cipher_pkt_op was used, as it's stateless after begin_session
        // However, if begin_session was stateful, free it.
        // The Zephyr API implies begin_session creates context, so free is needed.
        cipher_free_session(g_crypto_dev, &crypto_session_ctx);
        return err;
    }

    // No explicit cipher_update_iv or cipher_final for pkt_op with CTR.
    // The IV is updated by the driver.
    cipher_free_session(g_crypto_dev, &crypto_session_ctx);

    return 0;
}


int security_derive_auth_key(const uint8_t *master_key, uint8_t *out_auth_key)
{
	if (!master_key || !out_auth_key) {
		return -EINVAL;
	}

	const char *label = "DECT-NR-AuthKey";
	struct mac_ctx cmac_ctx;
	int err;

	if (!device_is_ready(g_crypto_dev)) {
		return -ENODEV;
	}

	err = mac_begin_session(g_crypto_dev, &cmac_ctx, CRYPTO_MAC_ALGO_CMAC,
                            (uint8_t *)master_key, 16, NULL);
	if (err != 0) {
		return err;
	}

	struct mac_pkt seq;
	uint8_t *p_data_buf[1];
    size_t p_data_len[1];

    p_data_buf[0] = (uint8_t *)label;
    p_data_len[0] = strlen(label);
    seq.in_buf = p_data_buf;
    seq.in_num = 1;
    seq.out_buf = out_auth_key; // Write the full 16-byte tag directly

	err = mac_compute(&cmac_ctx, &seq);
	mac_free_session(g_crypto_dev, &cmac_ctx);

	return err;
}



int security_generate_auth_mac(const uint8_t *auth_key, uint32_t pt_nonce, uint32_t ft_nonce,
			       uint32_t pt_long_id, uint32_t ft_long_id,
			       uint8_t *out_mac_8_bytes)
{
	if (!auth_key || !out_mac_8_bytes) {
		return -EINVAL;
	}
	if (!device_is_ready(g_crypto_dev)) {
		return -ENODEV;
	}

	uint8_t data_to_mac[16];
	sys_put_be32(pt_nonce, &data_to_mac[0]);
	sys_put_be32(ft_nonce, &data_to_mac[4]);
	sys_put_be32(pt_long_id, &data_to_mac[8]);
	sys_put_be32(ft_long_id, &data_to_mac[12]);

	uint8_t full_mic_tag[16];
	struct mac_ctx cmac_ctx;
	int err;

	err = mac_begin_session(g_crypto_dev, &cmac_ctx, CRYPTO_MAC_ALGO_CMAC,
                            (uint8_t *)auth_key, 16, NULL);
	if (err != 0) {
		return err;
	}

	struct mac_pkt seq;
	uint8_t *p_data_buf[1];
    size_t p_data_len[1];

    p_data_buf[0] = data_to_mac;
    p_data_len[0] = sizeof(data_to_mac);
    seq.in_buf = p_data_buf;
    seq.in_num = 1;
    seq.out_buf = full_mic_tag;

	err = mac_compute(&cmac_ctx, &seq);
	mac_free_session(g_crypto_dev, &cmac_ctx);

	if (err != 0) {
		return err;
	}

	memcpy(out_mac_8_bytes, full_mic_tag, DECT_MAC_AUTH_MAC_SIZE);
	return 0;
}




static int security_cmac_kdf(const uint8_t *key, const char *label, uint32_t id_local,
			     uint32_t id_peer, uint8_t *out_key);

int security_derive_session_keys(const uint8_t *auth_key, uint32_t id_local, uint32_t id_peer,
				 uint8_t *out_session_integrity_key,
				 uint8_t *out_session_cipher_key)
{
	if (!auth_key || !out_session_integrity_key || !out_session_cipher_key) {
		return -EINVAL;
	}

	int err;

	/* Derive Integrity Key using CMAC-based KDF */
	err = security_cmac_kdf(auth_key, "IntegrityKey", id_local, id_peer,
				out_session_integrity_key);
	if (err != 0) {
		LOG_ERR("Failed to derive integrity key: %d", err);
		return err;
	}

	/* Derive Cipher Key using CMAC-based KDF */
	err = security_cmac_kdf(auth_key, "CipheringKey", id_local, id_peer,
				out_session_cipher_key);
	if (err != 0) {
		LOG_ERR("Failed to derive cipher key: %d", err);
	}

	return err;
}

/**
 * @brief Implements a KDF based on NIST SP 800-108 (CMAC in counter mode).
 *
 * KDF(KI, Label, Context) = CMAC(KI, i || Label || 0x00 || Context || L)
 * For our use, Context = id_local || id_peer.
 */
static int security_cmac_kdf(const uint8_t *key, const char *label, uint32_t id_local,
			     uint32_t id_peer, uint8_t *out_key)
{
	uint8_t data_to_mac[64];
	size_t current_len = 0;
	struct cipher_ctx cmac_ctx;
	int err;

	if (!device_is_ready(g_crypto_dev)) {
		return -ENODEV;
	}

	/* Construct the input data for CMAC */
	data_to_mac[current_len++] = 0x01; /* Counter 'i' */

	size_t label_len = strlen(label);
	memcpy(data_to_mac + current_len, label, label_len);
	current_len += label_len;

	data_to_mac[current_len++] = 0x00; /* Separator */

	sys_put_be32(id_local, data_to_mac + current_len);
	current_len += sizeof(uint32_t);
	sys_put_be32(id_peer, data_to_mac + current_len);
	current_len += sizeof(uint32_t);

	sys_put_be16(128, data_to_mac + current_len); /* Output length 'L' in bits (128 for AES-128) */
	current_len += sizeof(uint16_t);

	/* Perform CMAC operation */
	cmac_ctx.keylen = 16;
	cmac_ctx.key.bit_stream = (uint8_t *)key;
	cmac_ctx.flags = CAP_RAW_KEY;

	err = cipher_begin_session(g_crypto_dev, &cmac_ctx, CRYPTO_CIPHER_ALGO_AES,
				   CRYPTO_CIPHER_MODE_CMAC, CRYPTO_CIPHER_OP_MAC);
	if (err != 0) {
		return err;
	}

	struct mac_params cmac_params;
	cmac_params.tag = out_key;
	cmac_params.tag_len = 16;

	err = cipher_mac_update(&cmac_ctx, data_to_mac, current_len);
	if (err == 0) {
		err = cipher_mac_final(&cmac_ctx, &cmac_params);
	}

	cipher_free_session(g_crypto_dev, &cmac_ctx);

	return err;
}