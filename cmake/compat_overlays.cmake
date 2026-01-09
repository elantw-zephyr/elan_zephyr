message(STATUS "ZEPHYR_EXTRA_MODULES: ${ZEPHYR_EXTRA_MODULES}")
message(STATUS "ZEPHYR_BASE: ${ZEPHYR_BASE}")
message(STATUS "CMAKE Set module board defconf after upstream PR merged!")


# 若使用者未指定 OVERLAY_CONFIG，就自動附加
if(NOT DEFINED OVERLAY_CONFIG)
  set(OVERLAY_CONFIG
    "${ZEPHYR_EXTRA_MODULES}/boards/elan/32f967_dv/32f967_dv.overlay.conf"
    CACHE STRING "App Kconfig overlays" FORCE)
  message(STATUS "自動載入 module Kconfig overlays: ${OVERLAY_CONFIG}")
endif()
