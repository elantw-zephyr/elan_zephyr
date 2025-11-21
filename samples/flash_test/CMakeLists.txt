# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

#message(STATUS "Sample App CMake loaded!")

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

#message(STATUS "ZEPHYR_EXTRA_MODULES: ${ZEPHYR_EXTRA_MODULES}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flash_test)

target_sources(app PRIVATE src/main.c)
