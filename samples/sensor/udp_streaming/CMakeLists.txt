# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(udp_streaming)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE include)

zephyr_code_relocate(LIBRARY app LOCATION ITCM_TEXT_RODATA)
zephyr_code_relocate(LIBRARY subsys__rtio LOCATION ITCM_TEXT_RODATA)
zephyr_code_relocate(LIBRARY drivers__spi LOCATION ITCM_TEXT_RODATA)
zephyr_code_relocate(LIBRARY drivers__sensor__tdk__icm4268x LOCATION ITCM_TEXT_RODATA)

zephyr_code_relocate(LIBRARY subsys__net LOCATION ITCM_TEXT_RODATA)
zephyr_code_relocate(LIBRARY drivers__ethernet LOCATION ITCM_TEXT_RODATA)

zephyr_code_relocate(LIBRARY app LOCATION DTCM_DATA_BSS)
zephyr_code_relocate(LIBRARY subsys__rtio LOCATION DTCM_DATA_BSS)
zephyr_code_relocate(LIBRARY drivers__spi LOCATION DTCM_DATA_BSS)
zephyr_code_relocate(LIBRARY drivers__sensor__tdk__icm4268x LOCATION DTCM_DATA_BSS)
zephyr_code_relocate(LIBRARY kernel LOCATION DTCM_DATA_BSS)

zephyr_code_relocate(LIBRARY subsys__net LOCATION DTCM_DATA_BSS)
zephyr_code_relocate(LIBRARY drivers__ethernet LOCATION DTCM_DATA_BSS)
