# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD v_mimxrt1060_evk)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

target_sources(app PRIVATE src/main.c)
