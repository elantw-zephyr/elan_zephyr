# SPDX-License-Identifier: Apache-2.0

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

message(STATUS "Include compat_overlays.cmake!")
include(${ZEPHYR_EXTRA_MODULES}/cmake/compat_overlays.cmake)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(testusb)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
