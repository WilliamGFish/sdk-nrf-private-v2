# lib/dect_nrplus/tests/CMakeLists.txt
#
# Copyright (c) 2025 Manulytica Ltd
#
#

# This file adds all the subdirectories containing Ztest suites for the DECT NR+ stack.

cmake_minimum_required(VERSION 3.20)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_nrplus_ztest)


target_include_directories(ztester)
target_include_directories(mocks)


# zephyr_add_test_subdirectory(mac_pdu/test_rach_ie)
# zephyr_add_test_subdirectory(mac_pdu/test_assoc_ies)
# zephyr_add_test_subdirectory(mac_pdu/test_security)
# zephyr_add_test_subdirectory(mac_pdu/test_mac_ies)
# zephyr_add_test_subdirectory(l2_6lowpan)
# zephyr_add_test_subdirectory(mac_flow)
# zephyr_add_test_subdirectory(cdd_service)
# zephyr_add_test_subdirectory(test_advanced_mac)
# zephyr_add_test_subdirectory(test_dlc_advanced)
# zephyr_add_test_subdirectory(test_cvg_advanced)




# zephyr_add_test_subdirectory(dlc_sar)
# zephyr_add_test_subdirectory(mac_error_handling)

# zephyr_add_test_subdirectory(mac_pdu)
# zephyr_add_test_subdirectory(security)
# zephyr_add_test_subdirectory(dlc_sar)
# zephyr_add_test_subdirectory(l2_6lowpan)
# zephyr_add_test_subdirectory(cdd_service)
# zephyr_add_test_subdirectory(mac_integration)


# Add a new test target for the MAC core logic
target_sources(
    #include <mac/dect_mac_api.h>
#    ../../mac/dect_mac_api.c
   # Add the mock PHY as a source file for THIS TEST ONLY
   mocks/mock_nrf_modem_dect_phy.c
   )