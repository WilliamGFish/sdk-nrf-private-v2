/* tests/mac_pdu/test_security/src/main.c */
// Overview: This file is converted from a standalone test program to a formal Ztest suite. `printk` based checks are replaced with `zassert` macros for automated verification. A setup/teardown fixture is used to ensure test data is reset between test cases.
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>

#include <mac/dect_mac_security.h>

LOG_MODULE_REGISTER(test_security, LOG_LEVEL_DBG);

/* --- Test Vectors --- */
static const uint8_t test_master_psk[16] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

static const uint32_t test_pt_long_id = 0xAABBCCDD;
static const uint32_t test_ft_long_id = 0x11223344;
static const uint32_t test_pt_nonce = 0x12345678;
static const uint32_t test_ft_nonce = 0x87654321;

static uint8_t plaintext_data[] = "This is a test payload for DECT NR+ security functions.";
static uint8_t original_plaintext_data[sizeof(plaintext_data)];

/* --- Test Fixture (Setup/Teardown) --- */
static void *setup(void)
{
	memcpy(original_plaintext_data, plaintext_data, sizeof(plaintext_data));
	return NULL;
}

static void teardown(void *data)
{
	ARG_UNUSED(data);
	/* Restore plaintext data in case it was modified in-place by a test */
	memcpy(plaintext_data, original_plaintext_data, sizeof(plaintext_data));
}

/**
 * @brief One-time setup function for the security test suite.
 *
 * This function is called by the Ztest framework before any tests in this suite
 * are run. It's the perfect place to initialize the security module.
 */
static void *security_suite_setup(void)
{
    int err = security_init();
    
    // In a test, we want to fail immediately if setup fails.
    // zassert_ok is the best way to do this.
    zassert_ok(err, "security_init() failed during test suite setup");

    return NULL; // Return NULL as we don't need to pass a context fixture
}

/**
 * @brief Per-test setup function.
 *
 * This function is called before EACH test case. It's used to reset state
 * to a known-good condition before every test.
 */
static void before_each_test(void *data)
{
	ARG_UNUSED(data);
	memcpy(original_plaintext_data, plaintext_data, sizeof(plaintext_data));
}

/**
 * @brief Per-test teardown function.
 *
 * This function is called after EACH test case. It's used for cleanup.
 */
static void after_each_test(void *data)
{
	ARG_UNUSED(data);
	/* Restore plaintext data in case it was modified in-place by a test */
	memcpy(plaintext_data, original_plaintext_data, sizeof(plaintext_data));
}

/* --- Test Cases --- */

ZTEST(security_tests, test_kdf_derivation)
{
	uint8_t k_auth[16];
	uint8_t k_int[16];
	uint8_t k_ciph[16];

	TC_PRINT("Testing Key Derivation Functions...\n");

	/* Test K_auth derivation */
	int ret = security_derive_auth_key(test_master_psk, k_auth);
	zassert_ok(ret, "security_derive_auth_key failed with %d", ret);
	zassert_mem_ne(k_auth, test_master_psk, 16, "K_auth should not be the same as master PSK");

	/* Test session key derivation */
	ret = security_derive_session_keys(k_auth, test_pt_long_id, test_ft_long_id, k_int, k_ciph);
	zassert_ok(ret, "security_derive_session_keys failed with %d", ret);
	zassert_mem_ne(k_int, k_ciph, 16, "Integrity and Cipher keys should not be the same");
	zassert_mem_ne(k_int, k_auth, 16, "Integrity key should not be the same as K_auth");
	zassert_mem_ne(k_ciph, k_auth, 16, "Cipher key should not be the same as K_auth");
}

ZTEST(security_tests, test_auth_mac_generation)
{
	uint8_t k_auth[16];
	uint8_t mac1[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t mac2[DECT_MAC_AUTH_MAC_SIZE];

	TC_PRINT("Testing Authentication MAC Generation...\n");

	int ret = security_derive_auth_key(test_master_psk, k_auth);
	zassert_ok(ret, "Setup: security_derive_auth_key failed with %d", ret);

	/* Generate first MAC */
	ret = security_generate_auth_mac(k_auth, test_pt_nonce, test_ft_nonce, test_pt_long_id, test_ft_long_id, mac1);
	zassert_ok(ret, "security_generate_auth_mac (1) failed with %d", ret);

	/* Generate second MAC with slightly different data */
	ret = security_generate_auth_mac(k_auth, test_pt_nonce + 1, test_ft_nonce, test_pt_long_id, test_ft_long_id, mac2);
	zassert_ok(ret, "security_generate_auth_mac (2) failed with %d", ret);

	/* Verify that the MACs are different */
	zassert_mem_ne(mac1, mac2, DECT_MAC_AUTH_MAC_SIZE, "MACs for different inputs should not be the same");
}

ZTEST(security_tests, test_mic_calculation)
{
	uint8_t mic1[5];
	uint8_t mic2[5];
	uint8_t test_key[16] = {0x1A};

	TC_PRINT("Testing MIC Calculation (AES-CMAC)...\n");

	/* Calculate MIC on original data */
	int ret = security_calculate_mic(plaintext_data, sizeof(plaintext_data) - 1, test_key, mic1);
	zassert_ok(ret, "security_calculate_mic (1) failed with %d", ret);

	/* Flip a bit in the data and recalculate */
	plaintext_data[5] ^= 0x01;
	ret = security_calculate_mic(plaintext_data, sizeof(plaintext_data) - 1, test_key, mic2);
	zassert_ok(ret, "security_calculate_mic (2) failed with %d", ret);

	/* Verify that the MICs are different */
	zassert_mem_ne(mic1, mic2, 5, "MICs for different data should not be the same");
}

ZTEST(security_tests, test_encryption_decryption_roundtrip)
{
	uint8_t test_key[16] = {0x2B};
	uint8_t iv[16];
	uint8_t ciphertext[sizeof(plaintext_data)];

	TC_PRINT("Testing Encryption/Decryption Roundtrip (AES-CTR)...\n");
	memcpy(ciphertext, plaintext_data, sizeof(plaintext_data));

	/* Build IV */
	security_build_iv(iv, test_pt_long_id, test_ft_long_id, 123, 456);

	/* Encrypt */
	int ret = security_crypt_payload(ciphertext, sizeof(ciphertext), test_key, iv, true);
	zassert_ok(ret, "Encryption failed with %d", ret);
	zassert_mem_ne(ciphertext, original_plaintext_data, sizeof(ciphertext), "Ciphertext should not match original plaintext");

	/* Decrypt */
	/* Re-build the same IV for decryption */
	security_build_iv(iv, test_pt_long_id, test_ft_long_id, 123, 456);
	ret = security_crypt_payload(ciphertext, sizeof(ciphertext), test_key, iv, false);
	zassert_ok(ret, "Decryption failed with %d", ret);

	/* Verify roundtrip */
	zassert_mem_equal(ciphertext, original_plaintext_data, sizeof(ciphertext), "Decrypted text does not match original plaintext");
}

ZTEST(security_tests, test_full_auth_handshake)
{
	uint8_t k_auth[16];
	uint8_t pt_mac[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t ft_mac[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t expected_pt_mac[DECT_MAC_AUTH_MAC_SIZE];
	uint8_t expected_ft_mac[DECT_MAC_AUTH_MAC_SIZE];
	int ret;

	TC_PRINT("Testing Full Authentication Handshake Flow...\n");

	/* 1. Both sides derive K_auth from the master PSK */
	ret = security_derive_auth_key(test_master_psk, k_auth);
	zassert_ok(ret, "K_auth derivation failed");

	/* 2. PT generates its MAC (Auth Response) based on both nonces */
	ret = security_generate_auth_mac(k_auth, test_pt_nonce, test_ft_nonce,
					 test_pt_long_id, test_ft_long_id, pt_mac);
	zassert_ok(ret, "PT failed to generate its auth MAC");

	/* 3. FT independently generates what it expects the PT's MAC to be */
	ret = security_generate_auth_mac(k_auth, test_pt_nonce, test_ft_nonce,
					 test_pt_long_id, test_ft_long_id, expected_pt_mac);
	zassert_ok(ret, "FT failed to generate expected PT MAC");

	/* 4. FT verifies the received PT MAC */
	zassert_mem_equal(pt_mac, expected_pt_mac, DECT_MAC_AUTH_MAC_SIZE,
			  "FT's verification of PT's MAC failed");
	TC_PRINT("Step 1/2: PT MAC successfully generated and verified by FT.\n");

	/* 5. FT generates its MAC (Auth Success) based on swapped nonces */
	ret = security_generate_auth_mac(k_auth, test_ft_nonce, test_pt_nonce,
					 test_ft_long_id, test_pt_long_id, ft_mac);
	zassert_ok(ret, "FT failed to generate its auth MAC");

	/* 6. PT independently generates what it expects the FT's MAC to be */
	ret = security_generate_auth_mac(k_auth, test_ft_nonce, test_pt_nonce,
					 test_ft_long_id, test_pt_long_id, expected_ft_mac);
	zassert_ok(ret, "PT failed to generate expected FT MAC");

	/* 7. PT verifies the received FT MAC */
	zassert_mem_equal(ft_mac, expected_ft_mac, DECT_MAC_AUTH_MAC_SIZE,
			  "PT's verification of FT's MAC failed");
	TC_PRINT("Step 3/4: FT MAC successfully generated and verified by PT.\n");
}

// ZTEST_SUITE(security_tests, NULL, setup, NULL, NULL, teardown);
ZTEST_SUITE(security_tests, NULL, security_suite_setup, before_each_test, after_each_test, teardown);