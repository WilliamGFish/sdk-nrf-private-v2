```c
#include <ztest.h>
#include <zephyr/sys/byteorder.h>
#include <psa/crypto.h>
#include <mac/dect_mac_security.h>

/* Mock or real PSA Crypto initialization */
static void test_psa_crypto_init(void)
{
    psa_status_t status = psa_crypto_init();
    zassert_equal(status, PSA_SUCCESS, "PSA Crypto initialization failed: %d", status);
}

/* Test security_build_iv */
static void test_security_build_iv(void)
{
    uint8_t iv[16];
    uint32_t transmitter_id = 0x12345678;
    uint32_t receiver_id = 0x87654321;
    uint32_t hpc = 0x00000001;
    uint16_t psn = 0x0ABC;

    /* Test valid input */
    security_build_iv(iv, transmitter_id, receiver_id, hpc, psn);
    uint8_t expected_iv[16] = {
        0x12, 0x34, 0x56, 0x78, /* transmitter_id */
        0x87, 0x65, 0x43, 0x21, /* receiver_id */
        0x00, 0x00, 0x00, 0x01, /* hpc */
        0x0A, 0xBC, 0x00, 0x00  /* psn (12 MSBs) + 0s */
    };
    zassert_mem_equal(iv, expected_iv, 16, "IV construction failed");

    /* Test NULL output */
    security_build_iv(NULL, transmitter_id, receiver_id, hpc, psn);
    /* No crash expected, function should log error and return */
}

/* Test security_calculate_mic */
static void test_security_calculate_mic(void)
{
    uint8_t integrity_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint8_t pdu_data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t mic[5];
    int err;

    /* Test valid input */
    err = security_calculate_mic(pdu_data, sizeof(pdu_data), integrity_key, mic);
    zassert_equal(err, 0, "MIC calculation failed: %d", err);
    /* Note: Actual MIC value depends on PSA Crypto implementation. 
     * For deterministic testing, mock psa_mac_compute or compare against known output. */

    /* Test invalid inputs */
    err = security_calculate_mic(NULL, sizeof(pdu_data), integrity_key, mic);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL pdu_data");
    err = security_calculate_mic(pdu_data, 0, integrity_key, mic);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for zero pdu_data_len");
    err = security_calculate_mic(pdu_data, sizeof(pdu_data), NULL, mic);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL integrity_key");
    err = security_calculate_mic(pdu_data, sizeof(pdu_data), integrity_key, NULL);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL mic_out");
}

/* Test security_crypt_payload */
static void test_security_crypt_payload(void)
{
    uint8_t cipher_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint8_t iv[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    uint8_t payload[16] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t payload_copy[16];
    int err;

    /* Test encryption and decryption round-trip */
    memcpy(payload_copy, payload, sizeof(payload));
    err = security_crypt_payload(payload, sizeof(payload), cipher_key, iv, true);
    zassert_equal(err, 0, "Encryption failed: %d", err);
    err = security_crypt_payload(payload, sizeof(payload), cipher_key, iv, false);
    zassert_equal(err, 0, "Decryption failed: %d", err);
    zassert_mem_equal(payload, payload_copy, sizeof(payload), "Round-trip encryption/decryption failed");

    /* Test empty payload */
    err = security_crypt_payload(payload, 0, cipher_key, iv, true);
    zassert_equal(err, 0, "Empty payload should succeed: %d", err);

    /* Test invalid inputs */
    err = security_crypt_payload(NULL, sizeof(payload), cipher_key, iv, true);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL payload");
    err = security_crypt_payload(payload, sizeof(payload), NULL, iv, true);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL cipher_key");
    err = security_crypt_payload(payload, sizeof(payload), cipher_key, NULL, true);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL iv with non-zero length");
}

/* Test security_derive_auth_key */
static void test_security_derive_auth_key(void)
{
    uint8_t master_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint8_t out_auth_key[16];
    int err;

    /* Test valid input */
    err = security_derive_auth_key(master_key, out_auth_key);
    zassert_equal(err, 0, "Auth key derivation failed: %d", err);

    /* Test invalid inputs */
    err = security_derive_auth_key(NULL, out_auth_key);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL master_key");
    err = security_derive_auth_key(master_key, NULL);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL out_auth_key");
}

/* Test security_generate_auth_mac */
static void test_security_generate_auth_mac(void)
{
    uint8_t auth_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint32_t pt_nonce = 0x12345678;
    uint32_t ft_nonce = 0x87654321;
    uint32_t pt_long_id = 0xAABBCCDD;
    uint32_t ft_long_id = 0xDDCCBBAA;
    uint8_t out_mac[DECT_MAC_AUTH_MAC_SIZE];
    int err;

    /* Test valid input */
    err = security_generate_auth_mac(auth_key, pt_nonce, ft_nonce, pt_long_id, ft_long_id, out_mac);
    zassert_equal(err, 0, "Auth MAC generation failed: %d", err);

    /* Test invalid inputs */
    err = security_generate_auth_mac(NULL, pt_nonce, ft_nonce, pt_long_id, ft_long_id, out_mac);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL auth_key");
    err = security_generate_auth_mac(auth_key, pt_nonce, ft_nonce, pt_long_id, ft_long_id, NULL);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL out_mac");
}

/* Test security_derive_session_keys */
static void test_security_derive_session_keys(void)
{
    uint8_t auth_key[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                            0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint32_t id_local = 0x12345678;
    uint32_t id_peer = 0x87654321;
    uint8_t integrity_key[16];
    uint8_t cipher_key[16];
    int err;

    /* Test valid input */
    err = security_derive_session_keys(auth_key, id_local, id_peer, integrity_key, cipher_key);
    zassert_equal(err, 0, "Session key derivation failed: %d", err);

    /* Test invalid inputs */
    err = security_derive_session_keys(NULL, id_local, id_peer, integrity_key, cipher_key);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL auth_key");
    err = security_derive_session_keys(auth_key, id_local, id_peer, NULL, cipher_key);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL integrity_key");
    err = security_derive_session_keys(auth_key, id_local, id_peer, integrity_key, NULL);
    zassert_equal(err, -EINVAL, "Expected -EINVAL for NULL cipher_key");
}

/* Setup and teardown for each test case */
static void *security_setup(void)
{
    test_psa_crypto_init();
    return NULL;
}

static void security_teardown(void *fixture)
{
    /* Clean up PSA Crypto if needed */
}

/* Define test suite */
ZTEST_SUITE(dect_mac_security_tests, NULL, security_setup, NULL, NULL, security_teardown);

/* Register test cases */
ZTEST(dect_mac_security_tests, test_security_build_iv)
{
    test_security_build_iv();
}

ZTEST(dect_mac_security_tests, test_security_calculate_mic)
{
    test_security_calculate_mic();
}

ZTEST(dect_mac_security_tests, test_security_crypt_payload)
{
    test_security_crypt_payload();
}

ZTEST(dect_mac_security_tests, test_security_derive_auth_key)
{
    test_security_derive_auth_key();
}

ZTEST(dect_mac_security_tests, test_security_generate_auth_mac)
{
    test_security_generate_auth_mac();
}

ZTEST(dect_mac_security_tests, test_security_derive_session_keys)
{
    test_security_derive_session_keys();
}
```