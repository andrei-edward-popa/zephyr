# Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

board_runner_args(stm8flash "--device=stm8s207k8")
include(${ZEPHYR_BASE}/boards/common/stm8flash.board.cmake)

board_runner_args(openocd "--cmd-reset-halt=reset halt")
# SWIM reset halts in the ROM debug module before fetching the reset vector.
board_runner_args(openocd "--gdb-pre-debug=monitor reset halt"
  "--gdb-pre-debug=set $pc = 0x8000")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
