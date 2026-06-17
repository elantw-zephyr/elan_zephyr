# SPDX-License-Identifier: Apache-2.0
#
# EM32F967 All-Port-B PWM Sample — runs PWMA..PWMF simultaneously on PB10..PB15.

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

message(STATUS "Include compat_overlays.cmake!")
include(${ZEPHYR_EXTRA_MODULES}/cmake/compat_overlays.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(elan_pb_pwm_sample)

target_sources(app PRIVATE src/main.c)
