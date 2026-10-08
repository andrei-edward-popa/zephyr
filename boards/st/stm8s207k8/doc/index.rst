.. SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
.. SPDX-License-Identifier: Apache-2.0

STM8S207K8 development board
###########################

This board uses the SDCC-compatible GCC large code model, a 16 MHz HSI clock,
64 KiB Flash, 6 KiB RAM and 1 KiB data EEPROM. The port supports cooperative
and preemptive threads, static interrupts and a tickless TIM4 system clock.
The C ABI uses 16-bit integers and data pointers, 32-bit long integers and
24-bit code pointers.

UART3 console
*************

UART3 supports polling and the interrupt-driven UART API. On NUCLEO-8S207K8,
UART3 TX (PD5, MCU pin 30) and RX (PD6, MCU pin 31) connect to the onboard
ST-Link virtual COM port through the default closed SB3/SB4 solder bridges.
Use the USB connector and the enumerated serial device, usually
``/dev/ttyACM0`` on Linux. The Arduino D1/D0 header pins are NC by default:
SB7/SB9 would need to be closed and SB3/SB4 opened to use those header pins
with an external 3.3 V USB-UART adapter. No solder changes are needed for VCP.
UART1 PA4/PA5 pins are absent from the LQFP32 package and there is no alternate
mapping to the available pins (DS5839, Figure 7 and Table 6; UM2391, section 7).
UART3 pin configuration is supplied through pinctrl. Runtime line configuration is available with
:kconfig:option:`CONFIG_UART_USE_RUNTIME_CONFIGURE`. The default line format is
115200 baud, 8N1. The asynchronous UART API is unsupported.

UART TX and RX use the ordinary architecture IRQ dispatcher and the standard
interrupt-driven UART API. All interrupt locks mask every maskable interrupt.
At 115200 baud a character takes about 87 microseconds, so long critical sections
or handlers can cause hardware overrun. Interactive typing works; unpaced paste
is not guaranteed. The driver reports overruns through ``uart_err_check()``.
ISR entry handles the DIV/DIVW and priority errata in ES036 sections 2.1.4 and 2.1.5.
UART1 is present in devicetree but disabled on this package.

GPIO, pinctrl, clocks and reset
*****************************

GPIO ports A-F describe the pins bonded in the LQFP32 package. PD1 is reserved
for SWIM. PB4/PB5 are true open-drain pins and cannot supply push-pull outputs
or internal pull-ups. The GPIO API supports port operations, input pull-ups,
push-pull/open-drain outputs, configuration queries and callbacks.

EXTI supports rising, falling and both edges, and low-level interrupts on the
available port A-E interrupt pins. Sensitivity is shared per port and there are
no per-pin pending flags. The driver permits one interrupt pin per port vector
to identify the callback source without guessing. PD7's unmaskable TLI is
unsupported because ordinary kernel handlers cannot nest safely.

Pinctrl configures direction, drive type, initial output level, input pull-up
and output slew. Each pin configuration state contains groups of pins sharing
the same electrical properties, following the common Zephyr pinctrl layout.
Alternate-function remaps are option bytes; pinctrl validates
their required value and never programs them. The SoC early initialization hook
configures HSI and CPU dividers. The clock driver supplies peripheral gate and
rate APIs. UART and timer drivers obtain their rates through this controller.
The driver supports HSI, HSE and LSI sources. Runtime switching and divider
changes are available with :kconfig:option:`CONFIG_CLOCK_CONTROL_STM8_RUNTIME`;
active transfers prevent a clock transition. The system timer and idle
peripherals update their configuration when the clock changes.

``sys_reboot()`` resets the system through WWDG. The SoC has no individual
peripheral reset lines and therefore no reset-controller device. Hwinfo exposes
the 96-bit factory UID and the reset causes recorded by RST_SR; power-on and pin reset causes
cannot be distinguished. During debug, OpenOCD sets SWIM_CSR.SAFE_MASK, which
masks watchdog resets; clear it when testing watchdog reset.

Timers
******

TIM4 supports both tickless and periodic system-clock configurations. Tickless
operation is the default; set ``CONFIG_TICKLESS_KERNEL=n`` for periodic ticks.
The driver programs the next deadline within the 8-bit counter's range and
retains the counter phase across timeout updates. At 16 MHz the longest TIM4
period is 2.048 ms, so long sleeps still require overflow maintenance interrupts.
The CPU enters Wait mode with ``WFI`` while idle, atomically enabling interrupts.
Peripheral clocks remain active; this does not use Halt mode or AWU.

TIM1, TIM2 and TIM3 default to disabled and can be enabled in an overlay. The
common counter driver supports start, stop, counter read, programmable top and
overflow callbacks. TIM1-TIM3 also support compare alarms, input capture and
PWM through separate counter and PWM child nodes. A timer instance can serve
one API at a time; PWM channels on that instance share the period. TIM4
is reserved for the system clock. To use it as a counter, disable
``CONFIG_SYS_CLOCK_EXISTS`` and delete ``st,system-timer`` in an overlay; the
separate system-clock driver is then omitted from the build.


SPI and I2C
***********

SPI uses the common Zephyr ``spi_context`` helpers and IRQ 10. The controller
supports 8-bit full-duplex transfers, all four clock modes, both bit orders,
scatter/gather buffers, GPIO chip selects, synchronous and asynchronous calls,
and a transfer timeout. Peripheral mode is available with
:kconfig:option:`CONFIG_SPI_PERIPHERAL` and the peripheral pinctrl state.
Half duplex and hardware CRC are unsupported. Pins are PC5 (SCK), PC6 (MOSI)
and PC7 (MISO).

I2C uses IRQ 19, supports 7-bit and 10-bit controller transfers, contiguous message fragments,
repeated START, and the RM0016 method 2 receive sequences. Standard mode runs at
no more than 88 kHz to satisfy the repeated START erratum; Fast mode runs at no
more than 400 kHz. Errors and timeouts reset the peripheral state. On this package,
SCL/SDA require the AFR6 option-byte remap to PB4/PB5 and external pull-ups.
Pinctrl checks this option byte and never programs it. Target mode is available
with :kconfig:option:`CONFIG_I2C_TARGET` and supports address, receive, transmit
and STOP callbacks. Both buses default to disabled and can be enabled in an
application overlay.

Bus recovery is available through ``i2c_recover_bus()`` when GPIO support is
enabled and both ``scl-gpios`` and ``sda-gpios`` are supplied. The board defines
these pins on PB4/PB5. Recovery releases the open-drain lines, honors clock
stretching, generates up to nine SCL pulses and a STOP, then restores pinctrl
and the controller configuration. External pull-ups are required. Recovery is
rejected while a target is registered.

Build and flash
***************

The STM8 cross-compiler is not part of the official Zephyr SDK. Set
``CROSS_COMPILE`` to the installed GCC STM8 toolchain prefix. From a west
workspace, for example::

   export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
   export CROSS_COMPILE=$HOME/toolchains/stm8/bin/stm8-unknown-elf-
   west build -b stm8s207k8 zephyr/samples/hello_world -d build/hello_world
   west flash -d build/hello_world

The board defaults to the stm8flash runner and the stlinkv21 programmer. USB
permissions must permit ST-Link access. RESET can be used to restart the
application. The default console is 115200 baud, 8N1. The expected UART3
output is ``Hello World! stm8s207k8/stm8s207k8``.

Debugging
*********

The board uses the OpenOCD runner for ``west debug``, ``west attach`` and
``west debugserver``. OpenOCD must support the ST-Link SWIM transport and the
toolchain must provide ``stm8-unknown-elf-gdb``. For example::

   west debug -d build/hello_world
   west attach -d build/hello_world

Both commands connect under reset, halting the CPU. This is required for
reliable SWIM entry while the application is in Wait mode. ``debug`` loads
the build's ELF image; ``attach`` leaves Flash unchanged. Consequently,
``attach`` restarts the application rather than preserving its running state.
Use the build directory corresponding to the programmed image. In GDB,
``break main`` followed by ``continue`` runs from the reset vector to ``main``.
The configuration accounts for STM8's initial stall in the ROM debug module
by setting PC to ``0x8000``.

OpenOCD leaves interrupt masking under program control during instruction
stepping. Changing the mask in the debugger would alter the state saved by
``irq_lock()`` and overwrite changes made by instructions such as ``SIM`` and
``WFI``. A step can therefore enter an interrupt handler.

Exit GDB with ``quit``. OpenOCD resets the MCU on shutdown with SWIM_CSR.RST
set, releasing the persistent debug mode and its reset-vector stall. Firmware
then runs normally and the RESET button works without unplugging USB. This
shutdown reset is necessary because a pin reset alone does not clear SWIM_DM
(UM0470 sections 3.10.1 and 4.3.1).

For a debug session whose state must survive server shutdown, use::

   west attach -d build/hello_world --cmd-pre-init "set STM8_KEEP_DEBUG_STATE 1"

This suppresses only the shutdown reset; connecting still resets the CPU.
While SWIM remains active, a subsequent pin reset can stall in the debug module.
Run a normal debug or attach session and quit to release it. Close other
OpenOCD instances before starting a session; only one process can own ST-Link.

Shell sample
************

The board configuration for ``samples/subsys/shell/shell_module`` follows
``prj_minimal.conf`` and keeps help, demo commands, dynamic commands, readline,
bypass and the boot banner enabled. Logging, history, tab completion and the
larger diagnostic command modules, thread names/monitoring and timeslicing are
disabled to fit the available memory. The sample uses 115200 baud, 8N1 with
the normal interrupt-driven UART API.

With the toolchain environment above::

   west build -b stm8s207k8 zephyr/samples/subsys/shell/shell_module \
     -d build/shell_module
   west flash -d build/shell_module
   python -m serial.tools.miniterm /dev/ttyACM0 115200 --raw --eol CR

To use 9600 baud, provide an overlay setting ``&uart3 { current-speed = <9600>; };``
through ``EXTRA_DTC_OVERLAY_FILE`` and use the same speed in the terminal.

Close other applications using the serial port before opening the terminal.
Detach OpenOCD before using the shell to avoid SWIM resets during UART traffic
with the debugger connected.
After ``*** Booting Zephyr OS build ... ***``, the prompt is ``uart:~$``.
Try ``help``, ``demo ping``, ``demo board``, ``dynamic add test``,
``dynamic execute test`` and ``demo readline``. Exit miniterm with Ctrl-].

The STM8 linker places static kernel/thread stacks in a dedicated section below
``0x1400`` and rejects configurations exceeding that boundary. STM8S207's
hardware stack roll-over would otherwise reset SP to ``0x17ff`` when a push
crosses the boundary. Ordinary data may still use all 6 KiB of RAM. This follows
the customized stack model in RM0016 section 3.1.2. Dynamically allocated stacks
also need to respect this hardware constraint.

FLASH and EEPROM
****************

The Flash API provides byte programming and 128-byte block erase across all
64 KiB. Byte programming performs an implicit erase, while explicit block
erase leaves zero-filled storage. The architecture copies the block erase
sequence into RAM before execution. Application storage must avoid pages
containing the running firmware. EEPROM is available through the EEPROM API
and shares the controller lock with Flash.

Other peripherals
*****************

ADC2 uses the ADC API with interrupt-driven conversions. TIM1-TIM3 PWM outputs,
ADC, IWDG, WWDG and BEEP default to disabled. Enable the corresponding nodes
and configure pins in an application overlay. The watchdogs cannot be stopped
after setup. BEEP uses the buzzer API for tone, duration and mute control;
its hardware has no variable amplitude control.
