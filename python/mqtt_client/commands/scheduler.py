"""Task scheduling commands for MQTT Client CLI."""

import json
import pathlib
import time
import typing

import asyncclick as click

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.port import PORTS
from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive
from ..decorators import ensure_connected
from ..utils.validators import validate_port

# Strategy name to value mapping
CHARGING_STRATEGY_MAP = {"fast": 0, "slow": 1, "usba": 6}


def _make_display_config_request(is_slow: bool) -> command_pb2.CommandRequest:
    """Create a display config request with preset values."""
    req = command_pb2.CommandRequest()
    if is_slow:
        req.set_display_config.config.intensity = 0
        req.set_display_config.config.mode = data_types_pb2.DisplayMode.OFF
    else:
        req.set_display_config.config.intensity = 128
        req.set_display_config.config.mode = data_types_pb2.DisplayMode.POWER_METER
    req.set_display_config.config.flip = data_types_pb2.DisplayFlip.NORMAL
    return req


def _do_schedule_task(
    ctx: click.Context,
    *commands: typing.Tuple[ServiceCommand, command_pb2.CommandRequest],
) -> None:
    """Internal helper to schedule multiple commands as a task."""
    req = admin_pb2.AdminCommandRequest()
    req.schedule_task.task.id = ctx.obj["task_id"]
    req.schedule_task.task.run_at = int(time.time()) + ctx.obj["task_delay"]
    for service, command_req in commands:
        cmd: data_types_pb2.Task.Command = req.schedule_task.task.commands.add()
        cmd.service = service.value
        cmd.data = command_req.SerializeToString()
    response = send_and_receive(
        ctx.obj["psn"],
        AdminCommand.SCHEDULE_TASK,
        req,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho("Schedule task", fg="green")


def register(main_group):
    """Register task scheduling commands with main CLI group."""

    @main_group.group()
    @click.option("--task-id", type=int, default=0)
    @click.option("--delay", type=int, default=0)
    @click.pass_context
    def schedule_task(ctx, task_id: int, delay: int):
        """Schedule commands to run at a future time."""
        ctx.obj["task_id"] = task_id
        ctx.obj["task_delay"] = delay

    @schedule_task.command(name="turn-on-port")
    @click.argument("port", type=str, nargs=-1, callback=validate_port)
    @ensure_connected
    def turn_on_port_task(ctx, port: typing.Tuple[str, ...]):
        """Schedule turning on port(s)."""
        req = command_pb2.CommandRequest()
        req.turn_on_port.ports.extend(PORTS.index(p) for p in port)
        _do_schedule_task(ctx, (ServiceCommand.TURN_ON_PORT, req))

    @schedule_task.command(name="turn-off-port")
    @click.argument("port", type=str, nargs=-1, callback=validate_port)
    @ensure_connected
    def turn_off_port_task(ctx, port: typing.Tuple[str, ...]):
        """Schedule turning off port(s)."""
        req = command_pb2.CommandRequest()
        req.turn_off_port.ports.extend(PORTS.index(p) for p in port)
        _do_schedule_task(ctx, (ServiceCommand.TURN_OFF_PORT, req))

    @schedule_task.command("set-charging-strategy")
    @click.argument(
        "strategy", type=click.Choice(["fast", "slow", "usba"], case_sensitive=False)
    )
    @ensure_connected
    def set_charging_strategy_task(ctx, strategy: str):
        """Schedule setting charging strategy."""
        req = command_pb2.CommandRequest()
        req.set_charging_strategy.charging_strategy = CHARGING_STRATEGY_MAP[strategy]
        display_req = _make_display_config_request(is_slow=(strategy == "slow"))
        _do_schedule_task(
            ctx,
            (ServiceCommand.SET_CHARGING_STRATEGY, req),
            (ServiceCommand.SET_DISPLAY_CONFIG, display_req),
        )

    @schedule_task.command("set-static-allocator")
    @click.argument("identifier", type=click.IntRange(0, 0xFFFF))
    @click.argument("version", type=click.IntRange(0, 0xFF))
    @click.argument("file", type=click.Path(exists=True, readable=True), nargs=1)
    @ensure_connected
    def set_static_allocator_task(ctx, identifier: int, version: int, file: str):
        """Schedule setting static power allocator."""
        req = command_pb2.CommandRequest()
        req.set_static_allocator.identifier = identifier
        req.set_static_allocator.table.version = version
        power_allocation_chart = json.loads(pathlib.Path(file).read_text())
        for allocation in power_allocation_chart:
            if len(allocation) != 5:
                raise ValueError("invalid allocation")
            entry = req.set_static_allocator.table.entries.add()
            entry.power_allocation.extend(allocation)
        display_req = _make_display_config_request(is_slow=False)
        _do_schedule_task(
            ctx,
            (ServiceCommand.SET_STATIC_ALLOCATOR, req),
            (ServiceCommand.SET_DISPLAY_CONFIG, display_req),
        )

    @main_group.command()
    @click.argument("task_id", type=int)
    @ensure_connected
    def get_task_status(ctx, task_id: int):
        """Get status of a scheduled task."""
        req = admin_pb2.AdminCommandRequest()
        req.get_task_status.task_id = task_id
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.GET_TASK_STATUS,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            status_mapping = {0: "pending", 1: "succeeded", 2: "failed"}
            status = response.admin_pb().get_task_status.status
            click.secho(
                f"Get task status: {status_mapping[status]}",
                fg="green",
            )
