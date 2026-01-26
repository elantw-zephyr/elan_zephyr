cmake_minimum_required(VERSION 3.13.1)

# Ensure this module is visible to Zephyr's build system
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(elan_pa14)

target_sources(app PRIVATE src/main.c)
