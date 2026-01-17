#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Command modules for MQTT Client CLI.

This package contains domain-specific command modules that are registered
with the main CLI group.
"""

import asyncclick as click

from . import (
    system,
    ble,
    files,
    port,
    power,
    device,
    display,
    wifi,
    telemetry,
    scheduler,
    diagnostics,
)


def register_all(main_group: click.Group) -> None:
    """Register all command modules with the main CLI group.

    Args:
        main_group: The Click group to register commands with.
    """
    system.register(main_group)
    ble.register(main_group)
    files.register(main_group)
    port.register(main_group)
    power.register(main_group)
    device.register(main_group)
    display.register(main_group)
    wifi.register(main_group)
    telemetry.register(main_group)
    scheduler.register(main_group)
    diagnostics.register(main_group)
