#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project("Zigbee generic custom cluster with lcd")

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/zb_zcl_custom_cluster.c
  src/lcd1602a.c
  src/bmi160.c
)

target_include_directories(app PRIVATE include)
# NORDIC SDK APP END
