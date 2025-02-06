# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SOC_MIMX9596_M7)
  board_runner_args(jlink "--device=cortex-m7")
  board_runner_args(pyocd "--target=mimx95_cm7")
endif()

if (CONFIG_SOF AND CONFIG_BOARD_MR_NAVQ95A_MIMX9596_M7_DDR)
  board_set_rimage_target(imx95)
endif()

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)