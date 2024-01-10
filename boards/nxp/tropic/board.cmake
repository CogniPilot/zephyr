#SPDX-License-Identifier: Apache-2.0

board_set_flasher_ifnset(teensy)

board_runner_args(teensy "--mcu=TEENSY41")

include(${ZEPHYR_BASE}/boards/common/teensy.board.cmake)