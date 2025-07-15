/* dect_mac/dect_mac_security.c */
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <psa/crypto.h>
#include <mac/dect_mac_security.h>

LOG_MODULE_REGISTER(dect_mac_security, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);

/* Forward declarations for internal functions */
static int security_cmac_compute(const uint8_t *key, size_t key_len,
                                const uint8_t *data, size_t data_len,
                                uint8_t *tag, size_t tag_len);
static int security_cmac_kdf(const uint8_t *key, const char *label, 
                            uint32_t id_local, uint32_t id_peer, 
                            uint8_t *out_key);

void security_build_iv(uint8_t *iv_out,
                       uint32_t transmitter_long_rd_id,
                       uint32_t receiver_long_rd_id,
                       uint32_t hpc,
                       uint16_t psn)
{
    if (!iv_out) {
        LOG_ERR("IV Build: Output buffer is NULL");
        return;
    }

    /* As per ETSI TS 103 636-4, Table 5.9.1.3-1 */
    /* All fields are Big Endian in the IV */
    sys_put_be32(transmitter_long_rd_id, &iv_out[0]); /* Octets 0-3 */
    sys_put_be32(receiver_long_rd_id, &iv_out[4]);   /* Octets 4-7 */
    sys_put_be32(hpc, &iv_out[8]);                  /* Octets 8-11 */
    uint16_t psn_field_value = (psn & 0x0FFF) << 4; /* Octets 12-13: PSN (12 MSBs) + 4 LSBs = 0 */
    sys_put_be16(psn_field_value, &iv_out[12]);
    iv_out[14] = 0; /* Octets 14-15: Ciphering engine internal byte counter */
    iv_out[15] = 0;

    LOG_DBG("IV Build: Successfully constructed IV");
}

int security_calculate_mic(const uint8_t *pdu_data_for_mic,
                           size_t pdu_data_len,
                           const uint8_t *integrity_key,
                           uint8_t *mic_out_5_bytes)
{
    if (!pdu_data_for_mic || pdu_data_len == 0 || !integrity_key || !mic_out_5_bytes) {
        LOG_ERR("MIC Calc: Invalid parameters");
        return -EINVAL;
    }

    uint8_t full_mic_tag[16];
    int err;

    /* Calculate full AES-CMAC with 16-byte key */
    err = security_cmac_compute(integrity_key, 16, pdu_data_for_mic, pdu_data_len,
                               full_mic_tag, sizeof(full_mic_tag));
    if (err != 0) {
        LOG_ERR("MIC Calc: Failed to compute AES-CMAC, err: %d", err);
        return err;
    }

    /* Truncate to 5 bytes as required by ETSI spec */
    memcpy(mic_out_5_bytes, full_mic_tag, 5);
    LOG_DBG("MIC Calc: Successfully computed 5-byte MIC");

    return 0;
}

int security_crypt_payload(uint8_t *payload_in_out,
                           size_t len,
                           const uint8_t *cipher_key,
                           uint8_t *iv,
                           bool encrypt)
{
    if (!payload_in_out || (len > 0 && !iv) || !cipher_key) {
        LOG_ERR("Crypt: Invalid parameters");
        return -EINVAL;
    }

    if (len == 0) {
        LOG_DBG("Crypt: Empty payload, nothing to process");
        return 0; /* Nothing to encrypt/decrypt */
    }

    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    size_t output_length;

    /* Set up key attributes for AES-128 */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);

    /* Import the key */
    status = psa_import_key(&attributes, cipher_key, 16, &key_id);
    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to import key, status: %d", status);
        psa_reset_key_attributes(&attributes);
        return -EINVAL;
    }

    /* Perform AES-CTR encryption/decryption */
    if (encrypt) {
        status = psa_cipher_encrypt(key_id, PSA_ALG_CTR, iv, 16,
                                    payload_in_out, len, payload_in_out, len, &output_length);
    } else {
        status = psa_cipher_decrypt(key_id, PSA_ALG_CTR, iv, 16,
                                    payload_in_out, len, payload_in_out, len, &output_length);
    }

    /* Clean up */
    psa_destroy_key(key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to perform AES-CTR %s, status: %d",
                encrypt ? "encrypt" : "decrypt", status);
        return -EINVAL;
    }

    if (output_length != len) {
        LOG_ERR("Crypt: Output length mismatch (%zu != %zu)", output_length, len);
        return -EINVAL;
    }

    LOG_DBG("Crypt: Successfully %s %zu bytes", encrypt ? "encrypted" : "decrypted", len);
    return 0;
}

int security_derive_auth_key(const uint8_t *master_key, uint8_t *out_auth_key)
{
    if (!master_key || !out_auth_key) {
        LOG_ERR("Auth Key: Invalid parameters");
        return -EINVAL;
    }

    const char *label = "DECT-NR-AuthKey";
    size_t label_len = strlen(label);

    if (label_len > 32) {
        LOG_ERR("Auth Key: Label too long (%zu > 32)", label_len);
        return -EINVAL;
    }

    int err = security_cmac_compute(master_key, 16, (const uint8_t *)label, 
                                   label_len, out_auth_key, 16);
    if (err == 0) {
        LOG_DBG("Auth Key: Successfully derived authentication key");
    }
    return err;
}

int security_generate_auth_mac(const uint8_t *auth_key, uint32_t pt_nonce, uint32_t ft_nonce,
                              uint32_t pt_long_id, uint32_t ft_long_id,
                              uint8_t *out_mac_8_bytes)
{
    if (!auth_key || !out_mac_8_bytes) {
        LOG_ERR("Auth MAC: Invalid parameters");
        return -EINVAL;
    }

    uint8_t data_to_mac[16];
    sys_put