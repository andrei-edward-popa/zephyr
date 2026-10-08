# SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_REBOOT reboot.c)
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT "${ZEPHYR_BASE}/arch/stm8/core/linker.ld" CACHE INTERNAL "")
