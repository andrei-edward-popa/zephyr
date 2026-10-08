# Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

board_runner_args(stm8flash "--device=stm8s207k8")
include(${ZEPHYR_BASE}/boards/common/stm8flash.board.cmake)
