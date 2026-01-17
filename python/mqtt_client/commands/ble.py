"""BLE and syslog commands for MQTT Client CLI."""

import asyncclick as click

import admin_pb2
import command_pb2

from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive
from ..decorators import ensure_connected, cast_state_to_int


def register(main_group):
    """Register BLE and syslog commands with main CLI group."""

    @main_group.command()
    @click.argument(
        "state",
        type=click.Choice(["on", "off"], case_sensitive=False),
        callback=cast_state_to_int,
    )
    @ensure_connected
    def set_ble_state(ctx, state: int):
        """Enable or disable BLE advertising."""
        req = command_pb2.CommandRequest()
        req.set_ble_state.enable = bool(state)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_BLE_STATE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Set BLE state to {state}", fg="green")

    @main_group.command()
    @click.argument(
        "state",
        type=click.Choice(["on", "off"], case_sensitive=False),
        callback=cast_state_to_int,
    )
    @ensure_connected
    def set_syslog_state(ctx, state: int):
        """Enable or disable syslog output."""
        req = admin_pb2.AdminCommandRequest()
        req.set_syslog_state.enable = bool(state)
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.SET_SYSLOG_STATE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Set syslog state to {state}", fg="green")
