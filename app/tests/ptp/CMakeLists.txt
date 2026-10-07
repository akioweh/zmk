# Copyright (c) 2026 The ZMK Contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13.1)
# Core tests must not load keyboard-specific West modules/drivers.
set(ZEPHYR_MODULES "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND DTS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ptp_core)
target_include_directories(app PRIVATE ../../include ../../src/ptp)
target_sources(app PRIVATE src/queue.c ../../src/ptp/frame.c ../../src/ptp/split.c)
target_compile_definitions(app PRIVATE CONFIG_ZMK_LOG_LEVEL=0)
if(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(app PRIVATE src/main.c ../../src/ptp/ptp.c ../../src/split/wired/wired.c)
  target_sources_ifdef(CONFIG_ZMK_PTP_TOUCH app PRIVATE ../../src/ptp/touch.c)
else()
  target_sources(app PRIVATE src/peripheral.c)
endif()
# Mock three BLE profiles without starting a Bluetooth controller.
target_compile_definitions(app PRIVATE CONFIG_BT_MAX_PAIRED=3)
