# lib/dect_nrplus/CMakeLists.txt
# Copyright (c) 2024 Manulytica Ltd
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dect_nrplus_driver)

# Create a regular Zephyr library
zephyr_library()

# Add source files to the library
zephyr_library_sources(
    dect_cdd.c
    dect_cvg.c
    dect_dlc.c
)

# Add subdirectory
add_subdirectory(mac)

# Make public headers available using zephyr_library_include_directories
zephyr_library_include_directories(include)
zephyr_library_include_directories(include/mac)

# Add the driver header directory
zephyr_library_include_directories(../../drivers/net/dect_nrplus)

# Also make them available to external consumers
zephyr_include_directories(include)
zephyr_include_directories(include/mac)