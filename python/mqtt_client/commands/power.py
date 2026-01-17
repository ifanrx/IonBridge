"""Power management commands for MQTT Client CLI."""

import json
import pathlib
import typing

import asyncclick as click
from google.protobuf.json_format import MessageToJson

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.port import FastChargingProtocol
from data_types.strategy import ChargingStrategy
from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive
from ..decorators import ensure_connected, cast_state_to_int


def register(main_group):
    """Register power management commands with main CLI group."""

    @main_group.command()
    @click.argument(
        "enabled",
        type=click.Choice(["on", "off"], case_sensitive=False),
        callback=cast_state_to_int,
    )
    @ensure_connected
    def set_power_allocator_enabled(ctx, enabled: int):
        """Enable or disable the power allocator."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_feature_toggle.action = data_types_pb2.ManageAction.UPDATE
        req.manage_feature_toggle.feature = data_types_pb2.ManageFeature.POWER_ALLOCATOR
        req.manage_feature_toggle.enabled = bool(enabled)
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FEATURE_TOGGLE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Set power allocator enabled to {bool(enabled)}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_power_allocator_enabled(ctx):
        """Get power allocator enabled status."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_feature_toggle.action = data_types_pb2.ManageAction.READ
        req.manage_feature_toggle.feature = data_types_pb2.ManageFeature.POWER_ALLOCATOR
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FEATURE_TOGGLE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Get power allocator enabled: {response.admin_pb().manage_feature_toggle.enabled}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def reset_power_allocator_enabled(ctx):
        """Reset power allocator enabled to default."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_feature_toggle.action = data_types_pb2.ManageAction.DELETE
        req.manage_feature_toggle.feature = data_types_pb2.ManageFeature.POWER_ALLOCATOR
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FEATURE_TOGGLE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Reset power allocator enabled", fg="green")

    @main_group.command()
    @click.argument("adc_threshold_low", type=click.IntRange(0, 255))
    @click.argument("adc_threshold_high", type=click.IntRange(0, 255))
    @click.argument("action_deadzone", type=click.IntRange(0, 255))
    @click.option("--level", type=click.IntRange(0, 3), default=0, help="FPGA config level")
    @ensure_connected
    def set_fpga_config(
        ctx,
        adc_threshold_low: int,
        adc_threshold_high: int,
        action_deadzone: int,
        level: int = 0,
    ):
        """Set FPGA configuration."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_fpga_config.action = data_types_pb2.ManageAction.UPDATE
        req.manage_fpga_config.config.adc_threshold_low = adc_threshold_low
        req.manage_fpga_config.config.adc_threshold_high = adc_threshold_high
        req.manage_fpga_config.config.action_deadzone = action_deadzone
        req.manage_fpga_config.level = level
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FPGA_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(
                f"Set FPGA config: adc_threshold_low: {adc_threshold_low}, "
                f"adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}",
                fg="green",
            )

    @main_group.command()
    @click.option("--level", type=click.IntRange(0, 3), default=0, help="FPGA config level")
    @ensure_connected
    def get_fpga_config(ctx, level: int = 0):
        """Get FPGA configuration."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_fpga_config.action = data_types_pb2.ManageAction.READ
        req.manage_fpga_config.level = level
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FPGA_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            pb_message = response.admin_pb()
            adc_threshold_low = pb_message.manage_fpga_config.config.adc_threshold_low
            adc_threshold_high = pb_message.manage_fpga_config.config.adc_threshold_high
            action_deadzone = pb_message.manage_fpga_config.config.action_deadzone
            click.secho(
                f"Get FPGA config: adc_threshold_low: {adc_threshold_low}, "
                f"adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}",
                fg="green",
            )

    @main_group.command()
    @click.option("--level", type=click.IntRange(0, 3), default=0, help="FPGA config level")
    @ensure_connected
    def reset_fpga_config(ctx, level: int = 0):
        """Reset FPGA configuration to default."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_fpga_config.action = data_types_pb2.ManageAction.DELETE
        req.manage_fpga_config.level = level
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FPGA_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Reset FPGA config", fg="green")

    @main_group.command()
    @ensure_connected
    def get_power_config(ctx):
        """Get power configuration."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_power_config.action = data_types_pb2.ManageAction.READ
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_POWER_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(
                f"Get power config: {response.to_json()}",
                fg="green",
            )

    @main_group.command()
    @click.argument("items", type=str, nargs=-1)
    @ensure_connected
    def set_power_config(ctx, items: typing.Tuple[str, ...]):
        """Set power config.

        ITEMS is a list of key=value pairs, e.g. power_budget=100
        """
        req = admin_pb2.AdminCommandRequest()
        req.manage_power_config.action = data_types_pb2.ManageAction.READ
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_POWER_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response:
            click.secho("Failed to get power config", fg="red")
            return

        req = admin_pb2.AdminCommandRequest()
        req.manage_power_config.action = data_types_pb2.ManageAction.UPDATE
        req.manage_power_config.config.CopyFrom(
            response.admin_pb().manage_power_config.config
        )
        field_names = [field.name for field in data_types_pb2.PowerConfig.DESCRIPTOR.fields]
        for item in items:
            key, value = item.split("=")
            if key not in field_names:
                raise ValueError(f"Invalid key {key}")
            setattr(req.manage_power_config.config, key, int(value))
        config_json = MessageToJson(
            req.manage_power_config.config,
            preserving_proto_field_name=True,
            always_print_fields_with_no_presence=True,
        )
        click.secho(f"Setting power config: {config_json}", fg="cyan")
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_POWER_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Set power config", fg="green")

    @main_group.command()
    @ensure_connected
    def reset_power_config(ctx):
        """Reset power configuration to default."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_power_config.action = data_types_pb2.ManageAction.DELETE
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_POWER_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Reset power config", fg="green")

    @main_group.command()
    @ensure_connected
    def get_all_power_statistics(ctx):
        """Get power statistics for all ports."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_ALL_POWER_STATISTICS,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response:
            return
        pb_message = response.command_pb()
        for port, stat in enumerate(pb_message.get_all_power_statistics.stats):
            protocol = FastChargingProtocol(stat.charging_protocol_version).name
            power = stat.current * stat.voltage / 1e6
            click.secho(
                f"[{port}] Protocol: {protocol.ljust(12, ' ')}, Power: {power:.2f}W, "
                f"Current: {stat.current / 1000:.2f}A, Voltage: {stat.voltage / 1000:.2f}V, "
                f"Temperature: {stat.temperature}C, Last full cap: {stat.battery_last_full_charge_capacity * 100}mWh, "
                f"Present cap: {stat.battery_present_capacity * 100}mWh",
                fg="green",
            )

    @main_group.command()
    @click.argument(
        "strategy",
        type=click.Choice(ChargingStrategy.get_choices(), case_sensitive=False),
        callback=ChargingStrategy.click_callback,
    )
    @ensure_connected
    def set_charging_strategy(ctx, strategy: int):
        """Set charging strategy."""
        req = command_pb2.CommandRequest()
        req.set_charging_strategy.charging_strategy = strategy  # type: ignore
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_CHARGING_STRATEGY,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Set charging strategy to {ChargingStrategy.to_string(strategy)}",
                fg="green",
            )

    @main_group.command()
    @click.argument("identifier", type=click.IntRange(0, 0xFFFF))
    @click.argument("version", type=click.IntRange(0, 0xFF))
    @click.argument("file", type=click.Path(exists=True, readable=True), nargs=1)
    @ensure_connected
    def set_static_allocator(ctx, identifier: int, version: int, file: str):
        """Set static power allocator from JSON file."""
        req = command_pb2.CommandRequest()
        req.set_static_allocator.identifier = identifier
        req.set_static_allocator.table.version = version
        power_allocation_chart = json.loads(pathlib.Path(file).read_text())
        for allocation in power_allocation_chart:
            if len(allocation) != 5:
                raise ValueError("invalid allocation")
            entry = req.set_static_allocator.table.entries.add()
            entry.power_allocation.extend(allocation)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_STATIC_ALLOCATOR,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Set static allocator", fg="green")

    @main_group.command()
    @ensure_connected
    def get_static_allocator(ctx):
        """Get static power allocator configuration."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_STATIC_ALLOCATOR,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Get static allocator: {response.to_json()}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    @click.argument("power_allocation", type=click.IntRange(0, 256), nargs=5)
    def set_temporary_allocator(
        ctx, power_allocation: typing.Tuple[int, int, int, int, int]
    ):
        """Set temporary power allocation (5 values for ports A, C1-C4)."""
        req = command_pb2.CommandRequest()
        req.set_temporary_allocator.power_allocation.extend(power_allocation)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_TEMPORARY_ALLOCATOR,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Set temporary allocator", fg="green")
