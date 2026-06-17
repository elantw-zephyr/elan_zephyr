##
# EM32F967 PDSW4 Power Management Sample
#
# This sample lives in the elan-zephyr module, so we explicitly add the
# module as an extra Zephyr module and include the compatibility overlay
# helper, following the pattern used by samples/elan_wdt.
##

cmake_minimum_required(VERSION 3.20.0)

# Add elan-zephyr as extra module
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

message(STATUS "Include compat_overlays.cmake!")
include(${ZEPHYR_EXTRA_MODULES}/cmake/compat_overlays.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(elan_pdsw4_sample)

target_sources(app PRIVATE src/main.c)
