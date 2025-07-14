#/* lib/dect_nrplus/mac/CMakeLists.txt */
#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20)

zephyr_library()

zephyr_library_sources(
    dect_mac_api.c
    dect_mac_core.c
    dect_mac_data_path.c
    dect_mac_main.c
    dect_mac_main_dispatcher.c
    dect_mac_mgmt.c
    dect_mac_nvs.c
    dect_mac_pdu.c
    dect_mac_phy_ctrl.c
    dect_mac_phy_if.c
    dect_mac_phy_tbs_tables.c
    dect_mac_security.c
    dect_mac_shell.c
    dect_mac_sm_ft.c
    dect_mac_sm_pt.c
    dect_mac_timeline_utils.c

    mock_nrf_modem_dect_phy.c
)

# zephyr_library_include_directories(
#     include
# )
