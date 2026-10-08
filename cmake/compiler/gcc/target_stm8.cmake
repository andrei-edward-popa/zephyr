# SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

list(APPEND TOOLCHAIN_C_FLAGS -mmodel=large -ffunction-sections -fdata-sections)
list(APPEND TOOLCHAIN_LD_FLAGS -mmodel=large)

# Flash block erase executes from writable RAM on this architecture.
# Its RAM load segment must therefore permit both writes and execution.
list(APPEND TOOLCHAIN_LD_FLAGS -Wl,--no-warn-rwx-segments)
