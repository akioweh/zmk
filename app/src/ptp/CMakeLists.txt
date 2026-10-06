# Copyright (c) 2026 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources(app PRIVATE frame.c)
target_sources_ifdef(CONFIG_ZMK_SPLIT app PRIVATE split.c)
if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(app PRIVATE ptp.c)
  target_sources_ifdef(CONFIG_ZMK_USB app PRIVATE usb.c)
  target_sources_ifdef(CONFIG_ZMK_BLE app PRIVATE hog.c)
endif()
