# Copyright (c) 2024 ELAN Microelectronics Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PWM_EM32 pwm_em32.c)

