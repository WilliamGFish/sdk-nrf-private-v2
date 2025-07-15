/*
 * Copyright (c) 2024 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * DECT NR+ MAC Layer Security Primitives using PSA Crypto API.
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

/* Use the modern and secure Platform Security Architecture (PSA) Crypto API */
#include <psa/crypto.h>

#include <mac/dect_mac_security.h>

LOG_MODULE_REGISTER(dect_mac_security, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);

/**
 * @brief Helper function to import a raw key into the PSA keystore for an operation.
 *
 * This function creates a transient key from raw key material for a single
 * cryptographic operation and returns its handle. The key must be destroyed
 * by the caller using psa_destroy_key().
 *
 * @param key_material Pointer to the raw key bytes.
 * @param key_len Length of the key material.
 * @param usage The permitted usage for the key (e.g., PSA_KEY_USAGE_SIGN_MESSAGE).
 * @param alg The permitted algorithm for the key (e.g., PSA_ALG_CMAC).
 * @param out_key_id Pointer to store the resulting key handle.
 * @return psa_status_t PSA_SUCCESS on success, or a PSA error code.
 */
static psa_status_t import_raw_key(const uint8_t *key_material, size_t key_len,
				   psa_key_usage_t usage, psa_algorithm_t alg,
				   psa_key_id_t *out_key_id)
{
	psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_status_t status;

	psa_set_key_type(&key_attr, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&key_attr, 128);
	psa_set_key_usage_flags(&key_attr, usage);
	psa_set_key_algorithm(&key_attr, alg);

	status = psa_import_key(&key_attr, key_material, key_len, out_key_id);

	psa_reset_key_attributes(&key_attr);

	return status;
}

void security_build_iv(uint8_t *iv_out,
                       uint32_t transmitter_long_rd_id,
                       uint32_t receiver_long_rd_id,
                       uint32_t hpc,
                       uint16_t psn)
{
	if (!iv_out) {
		LOG_ERR("IV Build: Output buffer is NULL.");
		return;
	}
	/* Per ETSI TS 103 636-4, Table 5.9.1.3-1 */
	sys_put_be32(transmitter_long_rd_id, &iv_out[0]);
	sys_put_be32(receiver_long_rd_id, &iv_out[4]);
	sys_put_be32(hpc, &iv_out[8]);
	uint16_t psn_field_value = (psn & 0x0FFF) << 4;
	sys_put_be16(psn_field_value, &iv_out[12]);
	iv_out[14] = 0;
	iv_out[15] = 0;
}

int security_calculate_mic(const uint8_t *pdu_data_for_mic,
                           size_t pdu_data_len,
                           const uint8_t *integrity_key,
                           uint8_t *mic_out_5_bytes)
{
	if (!pdu_data_for_mic || pdu_data_len == 0 || !integrity_key || !mic_out_5_bytes) {
		return -EINVAL;
	}

	psa_status_t status;
	psa_key_id_t key_id = 0;
	psa_mac_operation_t op = PSA_MAC_OPERATION_INIT;
	uint8_t full_mic_tag[16];
	size_t actual_mic_len;

	status = import_raw_key(integrity_key, 16, PSA_KEY_USAGE_SIGN_MESSAGE,
				PSA_ALG_CMAC, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: Failed to import integrity key: %d", (int)status);
		return -EIO;
	}

	status = psa_mac_sign_setup(&op, key_id, PSA_ALG_CMAC);
	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: MAC sign setup failed: %d", (int)status);
		goto cleanup;
	}

	status = psa_mac_update(&op, pdu_data_for_mic, pdu_data_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: MAC update failed: %d", (int)status);
		goto cleanup;
	}

	status = psa_mac_sign_finish(&op, full_mic_tag, sizeof(full_mic_tag), &actual_mic_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("MIC Calc: MAC sign finish failed: %d", (int)status);
	}

cleanup:
	psa_destroy_key(key_id);
	/* psa_mac_sign_finish also cleans up, but abort is safe for cleanup paths */
	psa_mac_abort(&op);

	if (status == PSA_SUCCESS) {
		memcpy(mic_out_5_bytes, full_mic_tag, 5);
		return 0;
	}

	return -EIO;
}

int security_crypt_payload(uint8_t *payload_in_out,
                           size_t len,
                           const uint8_t *cipher_key,
                           uint8_t *iv, /* This is the pre-constructed IV */
                           bool encrypt)
{
	if (!payload_in_out || (len > 0 && !iv) || !cipher_key) {
		return -EINVAL;
	}
	if (len == 0) {
		return 0; // Nothing to encrypt/decrypt
	}

	psa_status_t status;
	psa_key_id_t key_id = 0;
	size_t output_len = 0;
	size_t total_output_len = 0;

	/* 1. Declare the operation handle */
	psa_cipher_operation_t op_handle = PSA_CIPHER_OPERATION_INIT;

	psa_algorithm_t alg = PSA_ALG_CTR;
	psa_key_usage_t usage = encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT;

	/* 2. Import the raw key material into a transient key handle */
	status = import_raw_key(cipher_key, 16, usage, alg, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Crypt: Failed to import cipher key: %d", (int)status);
		return -EIO;
	}

	/* 3. Set up the cipher operation */
	if (encrypt) {
		status = psa_cipher_encrypt_setup(&op_handle, key_id, alg);
	} else {
		status = psa_cipher_decrypt_setup(&op_handle, key_id, alg);
	}

	if (status != PSA_SUCCESS) {
		LOG_ERR("Crypt: Cipher setup failed: %d", (int)status);
		goto crypt_cleanup;
	}

	/* 4. Set the pre-constructed Initialization Vector (IV) */
	status = psa_cipher_set_iv(&op_handle, iv, 16);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Crypt: Failed to set IV: %d", (int)status);
		goto crypt_cleanup;
	}

	/* 5. Process the payload. For CTR, this can be done in one shot. */
	status = psa_cipher_update(&op_handle, payload_in_out, len,
				   payload_in_out, len, &output_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Crypt: Cipher update failed: %d", (int)status);
		goto crypt_cleanup;
	}
	total_output_len += output_len;

	/* 6. Finalize the operation to get any remaining buffered data (usually none for CTR) */
	status = psa_cipher_finish(&op_handle, payload_in_out + total_output_len,
				   len - total_output_len, &output_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Crypt: Cipher finish failed: %d", (int)status);
	}

crypt_cleanup:
	/* psa_cipher_finish or an error path should be followed by abort */
	psa_cipher_abort(&op_handle);
	psa_destroy_key(key_id);

	return (status == PSA_SUCCESS) ? 0 : -EIO;
}


/**
 * @brief Implements a KDF based on NIST SP 800-108 (CMAC in counter mode).
 */
static int security_cmac_kdf(const uint8_t *key, const char *label, uint32_t id_local,
			     uint32_t id_peer, uint8_t *out_key)
{
	uint8_t data_to_mac[64];
	size_t current_len = 0;

	/* Construct the input data for CMAC: i || Label || 0x00 || Context || L */
	data_to_mac[current_len++] = 0x01; /* Counter 'i' */

	size_t label_len = strlen(label);
	memcpy(data_to_mac + current_len, label, label_len);
	current_len += label_len;

	data_to_mac[current_len++] = 0x00; /* Separator */

	sys_put_be32(id_local, data_to_mac + current_len);
	current_len += sizeof(uint32_t);
	sys_put_be32(id_peer, data_to_mac + current_len);
	current_len += sizeof(uint32_t);

	sys_put_be16(128, data_to_mac + current_len); /* Output length 'L' in bits */
	current_len += sizeof(uint16_t);

	/* Perform CMAC operation using PSA API */
	psa_status_t status;
	psa_key_id_t key_id = 0;
	size_t mac_len;

	status = import_raw_key(key, 16, PSA_KEY_USAGE_SIGN_MESSAGE, PSA_ALG_CMAC, &key_id);
	if (status != PSA_SUCCESS) {
		LOG_ERR("KDF: Failed to import base key: %d", (int)status);
		return -EIO;
	}

	status = psa_mac_compute(key_id, PSA_ALG_CMAC, data_to_mac, current_len,
				 out_key, 16, &mac_len);

	if (status != PSA_SUCCESS) {
		LOG_ERR("KDF: psa_mac_compute failed: %d", (int)status);
	}

	psa_destroy_key(key_id);

	return (status == PSA_SUCCESS) ? 0 : -EIO;
}

int security_derive_auth_key(const uint8_t *master_key, uint8_t *out_auth_key)
{
	if (!master_key || !out_auth_key) {
		return -EINVAL;
	}
	return security_cmac_kdf(master_key, "DECT-NR-AuthKey", 0, 0, out_auth_key);
}

int security_derive_session_keys(const uint8_t *auth_key, uint32_t id_local, uint32_t id_peer,
				 uint8_t *out_session_integrity_key,
				 uint8_t *out_session_cipher_key)
{
	if (!auth_key || !out_session_integrity_key || !out_session_cipher_key) {
		return -EINVAL;
	}

	int err;

	err = security_cmac_kdf(auth_key, "IntegrityKey", id_local, id_peer,
				out_session_integrity_key);
	if (err != 0) {
		LOG_ERR("Failed to derive integrity key: %d", err);
		return err;
	}

	err = security_cmac_kdf(auth_key, "CipheringKey", id_local, id_peer,
				out_session_cipher_key);
	if (err != 0) {
		LOG_ERR("Failed to derive cipher key: %d", err);
	}

	return err;
}

int security_generate_auth_mac(const uint8_t *auth_key, uint32_t pt_nonce, uint32_t ft_nonce,
			       uint32_t pt_long_id, uint32_t ft_long_id,
			       uint8_t *out_mac_8_bytes)
{
	if (!auth_key || !out_mac_8_bytes) {
		return -EINVAL;
	}

	/* Data to MAC = PT_Nonce || FT_Nonce || PT_Long_ID || FT_Long_ID */
	uint8_t data_to_mac[16];
	sys_put_be32(pt_nonce, &data_to_mac[0]);
	sys_put_be32(ft_nonce, &data_to_mac[4]);
	sys_put_be32(pt_long_id, &data_to_mac[8]);
	sys_put_be32(ft_long_id, &data_to_mac[12]);

	uint8_t full_mic_tag[16];
	int ret = security_calculate_mic(data_to_mac, sizeof(data_to_mac), auth_key, full_mic_tag);

	if (ret != 0) {
		LOG_ERR("Auth MAC: CMAC calculation failed: %d", ret);
		return ret;
	}

	/* Truncate the full 16-byte MIC to the 8 bytes required. */
	memcpy(out_mac_8_bytes, full_mic_tag, DECT_MAC_AUTH_MAC_SIZE);

	return 0;
}