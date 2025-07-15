/* dect_mac/dect_mac_security.c */
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/crypto/crypto.h>
#include <string.h>

#if defined(CONFIG_PSA_CRYPTO_DRIVER_CC3XX) || defined(CONFIG_MBEDTLS_PSA_CRYPTO_C)
#include <psa/crypto.h>
#else
#include <zephyr/crypto/crypto.h>
#endif

#include <mac/dect_mac_security.h>

LOG_MODULE_REGISTER(dect_mac_security, CONFIG_DECT_MAC_SECURITY_LOG_LEVEL);



/* Global device handle for the Zephyr crypto driver */
#if defined(CONFIG_PSA_CRYPTO_DRIVER_CC3XX) || defined(CONFIG_MBEDTLS_PSA_CRYPTO_C)
/* PSA is the backend, but we still need device handle for some operations */
static const struct device *const g_crypto_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crypto));
#else
/* Fallback or specific non-PSA driver if needed */
// static const struct device *const g_crypto_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_crypto));
// static const struct device *g_crypto_dev = device_get_binding(CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME);
// static struct device *g_crypto_dev = NULL;
struct device *g_crypto_dev *g_crypto_dev = device_get_binding(CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME);
#endif

/* Forward declarations for internal functions */
int security_cmac_compute(const uint8_t *key, size_t key_len,
                                const uint8_t *data, size_t data_len,
                                uint8_t *tag, size_t tag_len);
int security_cmac_kdf(const uint8_t *key, const char *label, 
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

    /* Octets 0-3: Long RD ID of the transmitter */
    sys_put_be32(transmitter_long_rd_id, &iv_out[0]);

    /* Octets 4-7: Long RD ID of the receiver */
    sys_put_be32(receiver_long_rd_id, &iv_out[4]);

    /* Octets 8-11: Hyper Packet Counter (HPC) */
    sys_put_be32(hpc, &iv_out[8]);

    /* Octets 12-13: Packet Sequence Number (PSN) (12 MSBs) + Initial Counter Block (4 LSBs = 0) */
    uint16_t psn_field_value = (psn & 0x0FFF) << 4;
    sys_put_be16(psn_field_value, &iv_out[12]);

    /* Octets 14-15: Ciphering engine internal byte counter */
    iv_out[14] = 0;
    iv_out[15] = 0;
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

    /* Calculate full AES-CMAC */
    err = security_cmac_compute(integrity_key, 16, pdu_data_for_mic, pdu_data_len,
                               full_mic_tag, sizeof(full_mic_tag));
    if (err != 0) {
        LOG_ERR("MIC Calc: Failed to compute AES-CMAC, err: %d", err);
        return err;
    }

    /* Truncate to 5 bytes as required by ETSI spec */
    memcpy(mic_out_5_bytes, full_mic_tag, 5);

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
        return 0; /* Nothing to encrypt/decrypt */
    }

#if defined(CONFIG_PSA_CRYPTO_DRIVER_CC3XX) || defined(CONFIG_MBEDTLS_PSA_CRYPTO_C)
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    size_t output_length;

    /* Set up key attributes */
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

    // /* Perform AES-CTR encryption/decryption */
    // if (encrypt) {
    //     status = psa_cipher_encrypt(key_id, PSA_ALG_CTR, iv, 16,
    //                                payload_in_out, len, payload_in_out, len, &output_length);
    // } else {
    //     status = psa_cipher_decrypt(key_id, PSA_ALG_CTR, iv, 16,
    //                                payload_in_out, len, payload_in_out, len, &output_length);
    // }


    /* Perform AES-CTR encryption/decryption */
    if (encrypt) {
        status = psa_cipher_encrypt(key_id, PSA_ALG_CTR, iv, 16,
                                   payload_in_out, len, &output_length);
    } else {
        status = psa_cipher_decrypt(key_id, PSA_ALG_CTR, iv, 16,
                                   payload_in_out, len, &output_length);
    }

    /* Clean up */
    psa_destroy_key(key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS) {
        LOG_ERR("Crypt: Failed to perform AES-CTR operation, status: %d", status);
        return -EINVAL;
    }

    return 0;

#else
    /* Fallback to legacy Zephyr crypto API */
    if (!device_is_ready(g_crypto_dev)) {
        LOG_ERR("Crypt: Crypto device (%s) is not ready", g_crypto_dev->name);
        return -ENODEV;
    }

    struct cipher_ctx crypto_session_ctx;
    int err;
    enum cipher_op op_mode = encrypt ? CRYPTO_CIPHER_OP_ENCRYPT : CRYPTO_CIPHER_OP_DECRYPT;

    crypto_session_ctx.keylen = 16; /* AES-128 */
    crypto_session_ctx.key.bit_stream = (uint8_t *)cipher_key;
    crypto_session_ctx.flags = CAP_RAW_KEY | CAP_INPLACE_OPS;

    err = cipher_begin_session(g_crypto_dev, &crypto_session_ctx, CRYPTO_CIPHER_ALGO_AES,
                               CRYPTO_CIPHER_MODE_CTR, op_mode);
    if (err != 0) {
        LOG_ERR("Crypt: Failed to begin AES-CTR session (op: %s), err: %d",
                encrypt ? "encrypt" : "decrypt", err);
        return err;
    }

    /* Perform the CTR operation */
    struct cipher_pkt pkt = {
        .in_buf = payload_in_out,
        .in_len = len,
        .out_buf = payload_in_out,
        .out_buf_max = len
    };

    err = cipher_pkt_op(&crypto_session_ctx, &pkt, iv);
    if (err != 0) {
        LOG_ERR("Crypt: Failed to perform AES-CTR operation (op: %s), err: %d",
                encrypt ? "encrypt" : "decrypt", err);
    }

    cipher_free_session(g_crypto_dev, &crypto_session_ctx);
    return err;
#endif
}

int security_derive_auth_key(const uint8_t *master_key, uint8_t *out_auth_key)
{
    if (!master_key || !out_auth_key) {
        return -EINVAL;
    }

    const char *label = "DECT-NR-AuthKey";
    
    return security_cmac_compute(master_key, 16, (const uint8_t *)label, 
                                strlen(label), out_auth_key, 16);
}

int security_generate_auth_mac(const uint8_t *auth_key, uint32_t pt_nonce, uint32_t ft_nonce,
                               uint32_t pt_long_id, uint32_t ft_long_id,
                               uint8_t *out_mac_8_bytes)
{
    if (!auth_key || !out_mac_8_bytes) {
        return -EINVAL;
    }

    uint8_t data_to_mac[16];
    sys_put_be32(pt_nonce, &data_to_mac[0]);
    sys_put_be32(ft_nonce, &data_to_mac[4]);
    sys_put_be32(pt_long_id, &data_to_mac[8]);
    sys_put_be32(ft_long_id, &data_to_mac[12]);

    uint8_t full_mic_tag[16];
    int err;

    err = security_cmac_compute(auth_key, 16, data_to_mac, sizeof(data_to_mac),
                               full_mic_tag, sizeof(full_mic_tag));
    if (err != 0) {
        return err;
    }

    memcpy(out_mac_8_bytes, full_mic_tag, DECT_MAC_AUTH_MAC_SIZE);
    return 0;
}

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
 * @brief Helper function to compute AES-CMAC
 *
 * @param key       Input key for CMAC
 * @param key_len   Key length in bytes (should be 16 for AES-128)
 * @param data      Input data to authenticate
 * @param data_len  Length of input data
 * @param tag       Output buffer for the MAC tag
 * @param tag_len   Length of output tag buffer
 *
 * @return 0 on success, negative error code on failure
 */
static int security_cmac_compute(const uint8_t *key, size_t key_len,
                                const uint8_t *data, size_t data_len,
                                uint8_t *tag, size_t tag_len)
{
    if (!key || !data || !tag || key_len == 0 || data_len == 0 || tag_len == 0) {
        return -EINVAL;
    }

#if defined(CONFIG_PSA_CRYPTO_DRIVER_CC3XX) || defined(CONFIG_MBEDTLS_PSA_CRYPTO_C)
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    size_t mac_length;

    /* Set up key attributes for CMAC */
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, key_len * 8);

    /* Import the key */
    status = psa_import_key(&attributes, key, key_len, &key_id);
    if (status != PSA_SUCCESS) {
        LOG_ERR("CMAC: Failed to import key, status: %d", status);
        psa_reset_key_attributes(&attributes);
        return -EINVAL;
    }

    /* Compute CMAC */
    status = psa_mac_compute(key_id, PSA_ALG_CMAC, data, data_len, tag, tag_len, &mac_length);

    /* Clean up */
    psa_destroy_key(key_id);
    psa_reset_key_attributes(&attributes);

    if (status != PSA_SUCCESS) {
        LOG_ERR("CMAC: Failed to compute, status: %d", status);
        return -EINVAL;
    }

    if (mac_length != tag_len) {
        LOG_ERR("CMAC: Unexpected MAC length: %zu, expected: %zu", mac_length, tag_len);
        return -EINVAL;
    }

    return 0;

#else
    if (!device_is_ready(g_crypto_dev)) {
        LOG_ERR("CMAC: Crypto device (%s) is not ready", g_crypto_dev->name);
        return -ENODEV;
    }
    struct cipher_ctx ctx = {
        .keylen = key_len,
        .key.bit_stream = (uint8_t *)key,
        .flags = CAP_RAW_KEY,
    };
    int err = cipher_begin_session(g_crypto_dev, &ctx, CRYPTO_CIPHER_ALGO_AES,
                                CRYPTO_CIPHER_MODE_CMAC, CRYPTO_CIPHER_OP_MAC);
    if (err) {
        LOG_ERR("CMAC: Failed to begin session, err: %d", err);
        return err;
    }
    struct cipher_pkt pkt = {
        .in_buf = (uint8_t *)data,
        .in_len = data_len,
        .out_buf = tag,
        .out_buf_max = tag_len,
    };
    err = cipher_cmac_op(&ctx, &pkt);
    cipher_free_session(g_crypto_dev, &ctx);
    return err;
#endif
}

/**
 * @brief Implements a KDF based on NIST SP 800-108 (CMAC in counter mode)
 *
 * KDF(KI, Label, Context) = CMAC(KI, i || Label || 0x00 || Context || L)
 * For our use, Context = id_local || id_peer
 *
 * @param key       Input key material
 * @param label     Label string for key derivation
 * @param id_local  Local identity
 * @param id_peer   Peer identity
 * @param out_key   Output derived key (16 bytes)
 *
 * @return 0 on success, negative error code on failure
 */
int security_cmac_kdf(const uint8_t *key, const char *label, 
                      uint32_t id_local, uint32_t id_peer, 
                      uint8_t *out_key)
{
    if (!key || !label || !out_key) {
        return -EINVAL;
    }

    uint8_t kdf_input[64];
    size_t input_len = 0;
    size_t label_len = strlen(label);

    /* Ensure we don't overflow the buffer */
    // if (label_len > 32) {
    //     LOG_ERR("KDF: Label too long");
    //     return -EINVAL;
    // }
    if (label_len > 32 || (label_len + 11) > sizeof(kdf_input)) {
        LOG_ERR("KDF: Input size too large (%zu > %zu)", label_len + 11, sizeof(kdf_input));
        return -EINVAL;
    }

    /* Construct KDF input: i || Label || 0x00 || Context || L */
    kdf_input[input_len++] = 0x01; /* Counter 'i' = 1 */

    memcpy(&kdf_input[input_len], label, label_len);
    input_len += label_len;

    kdf_input[input_len++] = 0x00; /* Separator */

    sys_put_be32(id_local, &kdf_input[input_len]);
    input_len += sizeof(uint32_t);

    sys_put_be32(id_peer, &kdf_input[input_len]);
    input_len += sizeof(uint32_t);

    sys_put_be16(128, &kdf_input[input_len]); /* Output length 'L' in bits */
    input_len += sizeof(uint16_t);

    /* Compute CMAC over the constructed input */
    return security_cmac_compute(key, 16, kdf_input, input_len, out_key, 16);
}