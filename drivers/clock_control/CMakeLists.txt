# Copyright (c) 2025 Elan Microelectronics Corp.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_EM32_AHB            clock_control_em32_ahb.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_EM32_APB            clock_control_em32_apb.c)
