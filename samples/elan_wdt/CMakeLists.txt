# Copyright (c) 2024 ELAN Microelectronics Corp.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Add elan-zephyr as extra module
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(elan_wdt_sample)

target_sources(app PRIVATE src/main.c)