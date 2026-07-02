# SPDX-FileCopyrightText: 2026 ELAN Microelectronics Corp.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_FLASH_EM32 flash_em32.c)
