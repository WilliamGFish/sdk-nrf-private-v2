#
# Copyright (c) 2025 Manulytica Ltd
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.21)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dect_nrplus_mac_flow)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE src/main.c)

# #################################################################
# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(mac_flow)

# # Enable Ztest
# set(ENABLE_ZTEST TRUE)

# # Add test source files
# target_sources(app PRIVATE
#     src/test_mac_flow.c
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/tests/mocks/mock_nrf_modem_dect_phy.c
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/src/dect_mac_api.c
#     # Add other source files as needed
# )

# # Include directories
# target_include_directories(app PRIVATE
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/tests/mocks
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include
#     C:/ncs/v3-zettalink/nrf/lib/dect_nrplus/include/mac
#     ${ZEPHYR_BASE}/include
# )

# # Link Ztest library
# target_link_libraries(app PRIVATE ztest)
