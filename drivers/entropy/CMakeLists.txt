# EM32F967 TRNG entropy driver build file
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ENTROPY_EM32_TRNG entropy_em32_trng.c)
