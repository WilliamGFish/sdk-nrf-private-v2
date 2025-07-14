#
# Copyright (c) 2025 Manulytica Ltd
#
#

# This file adds all the subdirectories containing Ztest suites for the DECT NR+ stack.
cmake_minimum_required(VERSION 3.20.0)
project(dect_mac_security_test)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

target_sources(app PRIVATE
    ${CMAKE_SOURCE_DIR}/src/dect_mac_security.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_dect_mac_security.c
)

target_include_directories(app PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)