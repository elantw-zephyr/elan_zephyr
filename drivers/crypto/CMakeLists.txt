# Copyright (c) 2024 Elan Microelectronics Corp.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_CRYPTO_EM32_SHA crypto_em32_sha.c)