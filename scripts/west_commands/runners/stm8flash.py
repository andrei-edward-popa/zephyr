# SPDX-FileCopyrightText: Copyright (c) 2026 Andrei-Edward Popa
# SPDX-License-Identifier: Apache-2.0

"""Runner for programming STM8 devices over SWIM using stm8flash."""

from runners.core import RunnerCaps, ZephyrBinaryRunner


class Stm8flashBinaryRunner(ZephyrBinaryRunner):
    """Flash an Intel HEX image with an ST-Link programmer."""

    def __init__(self, cfg, device, programmer="stlinkv21"):
        super().__init__(cfg)
        self.device = device
        self.programmer = programmer

    @classmethod
    def name(cls):
        return "stm8flash"

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={"flash"})

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument("--device", required=True, help="stm8flash MCU name")
        parser.add_argument(
            "--programmer",
            default="stlinkv21",
            help="stm8flash programmer name (default: stlinkv21)",
        )

    @classmethod
    def do_create(cls, cfg, args):
        return cls(cfg, device=args.device, programmer=args.programmer)

    def do_run(self, command, **kwargs):
        self.require("stm8flash")
        self.ensure_output("hex")
        self.check_call(
            ["stm8flash", "-c", self.programmer, "-p", self.device, "-w", self.cfg.hex_file]
        )
