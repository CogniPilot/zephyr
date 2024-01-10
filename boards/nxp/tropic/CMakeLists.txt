#
# Copyright (c) 2020, Bernhard Kraemer
# Copyright (c) 2024, Peter van der Perk
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
if(CONFIG_NXP_IMXRT_BOOT_HEADER)
  zephyr_compile_definitions(XIP_BOOT_HEADER_ENABLE=1)
  zephyr_compile_definitions(XIP_BOOT_HEADER_DCD_ENABLE=1)
  zephyr_compile_definitions(BOARD_FLASH_SIZE=CONFIG_FLASH_SIZE*1024)
  zephyr_library_sources(flexspi_nor_config.c)
endif()
