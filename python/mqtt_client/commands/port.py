"""Port management commands for MQTT Client CLI."""

import json
import typing

import asyncclick as click

import command_pb2

from data_types.port import (
    CableCompensation,
    FixedVoltageOffset,
    PORTS,
)
from data_types.service import ServiceCommand
from ..core import send_and_receive
from ..decorators import ensure_connected
from ..utils.validators import validate_port, validate_custom_pdo_voltage
from ..utils.helpers import handle_port_config_resp


def register(main_group):
    """Register port management commands with main CLI group."""

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.argument("enable", type=bool)
    @click.argument("resistance", type=click.IntRange(0, 1))
    @click.argument("voltage_offset", type=click.IntRange(0, 3))
    @ensure_connected
    def set_cable_compensation(
        ctx, port: str, enable: bool, resistance: int, voltage_offset: int
    ):
        """Set cable compensation: PORT ENABLE RESISTANCE VOLTAGE_OFFSET

        RESISTANCE: 0=65mOhm, 1=100mOhm
        VOLTAGE_OFFSET: 0=0mV, 1=100mV, 2=150mV, 3=200mV

        Example: set-cable-compensation C1 true 1 2
        """
        req = command_pb2.CommandRequest()
        req.set_cable_compensation.port_mask = 1 << PORTS.index(port)
        req.set_cable_compensation.disable = not enable
        req.set_cable_compensation.resistance = resistance
        req.set_cable_compensation.voltage_offset = voltage_offset

        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_CABLE_COMPENSATION,
            req,
            timeout=ctx.obj["timeout"],
        )

        if response and response.status == 0:
            click.secho(
                f"Set {port}: enable={enable}, R={resistance}, V={voltage_offset}",
                fg="green",
            )
        else:
            click.secho("Failed", fg="red")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.option("--json", "output_json", is_flag=True, help="Output in JSON format")
    @ensure_connected
    def get_cable_compensation(ctx, port: str, output_json: bool):
        """Get cable compensation settings.

        PORT: Port name to check. Available ports: A,C1,C2,C3,C4 (use --product-family to filter)
        """
        req = command_pb2.CommandRequest()
        req.get_cable_compensation.port = PORTS.index(port)

        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_CABLE_COMPENSATION,
            req,
            timeout=ctx.obj["timeout"],
        )

        if not response or response.status != 0:
            click.secho(f"Failed to get cable compensation for port {port}", fg="red")
            return

        compensation_data = response.command_pb().get_cable_compensation

        if output_json:
            result = {
                "port": port,
                "enable": compensation_data.enable,
                "resistance": {
                    "value": compensation_data.resistance,
                    "description": CableCompensation.to_string(
                        compensation_data.resistance
                    ),
                },
                "voltage_offset": {
                    "value": compensation_data.voltage_offset,
                    "description": FixedVoltageOffset.to_string(
                        compensation_data.voltage_offset
                    ),
                },
            }
            click.echo(json.dumps(result, indent=2))
        else:
            resistance_str = CableCompensation.to_string(compensation_data.resistance)
            voltage_str = FixedVoltageOffset.to_string(compensation_data.voltage_offset)
            enable_str = "enabled" if compensation_data.enable else "disabled"

            click.secho(f"Port {port} cable compensation:", fg="green")
            click.secho(f"  Status: {enable_str}", fg="cyan")
            click.secho(f"  Resistance: {resistance_str}", fg="cyan")
            click.secho(f"  Voltage offset: {voltage_str}", fg="cyan")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @ensure_connected
    def get_port_pd_status(ctx, port: str):
        """Get port PD status.

        PORT: Port name to check. Available ports: A,C1,C2,C3,C4 (use --product-family to filter)
        """
        req = command_pb2.CommandRequest()
        req.get_port_pd_status.port = PORTS.index(port)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_PORT_PD_STATUS,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(response.to_json(), fg="green")

    @main_group.command()
    @click.argument("port", type=str, nargs=-1, callback=validate_port)
    @ensure_connected
    def turn_on_port(ctx, port: typing.Tuple[str, ...]):
        """Turn on charging port.

        PORT: Port name(s) to turn on. Available ports: A,C1,C2,C3,C4 (use --product-family to filter)
        """
        req = command_pb2.CommandRequest()
        req.turn_on_port.ports.extend(PORTS.index(p) for p in port)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.TURN_ON_PORT,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Turn on port {port}", fg="green")

    @main_group.command()
    @click.argument("port", type=str, nargs=-1, callback=validate_port)
    @ensure_connected
    def turn_off_port(ctx, port: typing.Tuple[str, ...]):
        """Turn off charging port.

        PORT: Port name(s) to turn off. Available ports: A,C1,C2,C3,C4 (use --product-family to filter)
        """
        req = command_pb2.CommandRequest()
        req.turn_off_port.ports.extend(PORTS.index(p) for p in port)
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.TURN_OFF_PORT,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Turn off port {port}", fg="green")

    @main_group.command()
    @click.argument("ports", type=str, nargs=-1, required=False, callback=validate_port)
    @ensure_connected
    def get_port_config(ctx, ports: typing.Tuple[str, ...]):
        """Get port configuration. Optionally filter by specific ports.

        Examples:
            get-port-config           # Show all ports
            get-port-config C1        # Show only port C1
            get-port-config C1 C2 C3  # Show ports C1, C2, and C3
        """
        req = command_pb2.CommandRequest()
        req.get_port_config.version = 1
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_PORT_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        handle_port_config_resp(response, ports if ports else None)

    @main_group.command()
    @click.argument("ports", type=str, nargs=-1, callback=validate_port)
    @click.option("--tfcp", is_flag=True, default=False, help="Enable TFCP")
    @click.option("--pe", is_flag=True, default=False, help="Enable PE")
    @click.option("--qc2p0", is_flag=True, default=False, help="Enable QC 2.0")
    @click.option("--qc3p0", is_flag=True, default=False, help="Enable QC 3.0")
    @click.option("--qc3plus", is_flag=True, default=False, help="Enable QC 3+")
    @click.option("--afc", is_flag=True, default=False, help="Enable AFC")
    @click.option("--fcp", is_flag=True, default=False, help="Enable FCP")
    @click.option("--hvscp", is_flag=True, default=False, help="Enable HV SCP")
    @click.option("--lvscp", is_flag=True, default=False, help="Enable LV SCP")
    @click.option("--sfcp", is_flag=True, default=False, help="Enable SFCP")
    @click.option("--apple", is_flag=True, default=False, help="Enable Apple protocol")
    @click.option("--samsung", is_flag=True, default=False, help="Enable Samsung protocol")
    @click.option("--ufcs", is_flag=True, default=False, help="Enable UFCS")
    @click.option("--pd", is_flag=True, default=False, help="Enable PD")
    @click.option(
        "--pd_compat", is_flag=True, default=False, help="Enable PD compatibility mode"
    )
    @click.option("--pd_lvpps", is_flag=True, default=False, help="Enable PD LV PPS")
    @click.option("--pd_epr", is_flag=True, default=False, help="Enable PD EPR")
    @click.option("--pd_rpi", is_flag=True, default=False, help="Enable PD RPI (5V5A)")
    @click.option("--pd_hvpps", is_flag=True, default=False, help="Enable PD HV PPS")
    @click.option(
        "--pd_hard_reset_on_refusal",
        is_flag=True,
        default=False,
        help="Enable PD hard reset on refusal",
    )
    @click.option(
        "--pd_spr_avs",
        is_flag=True,
        default=False,
        help="Enable PD SPR AVS",
    )
    @click.option(
        "--reset", is_flag=True, default=False, help="Reset port config after setting"
    )
    @ensure_connected
    def set_port_config(
        ctx,
        ports: typing.Tuple[str, ...],
        tfcp: bool,
        pe: bool,
        qc2p0: bool,
        qc3p0: bool,
        qc3plus: bool,
        afc: bool,
        fcp: bool,
        hvscp: bool,
        lvscp: bool,
        sfcp: bool,
        apple: bool,
        samsung: bool,
        ufcs: bool,
        pd: bool,
        pd_compat: bool,
        pd_lvpps: bool,
        pd_epr: bool,
        pd_rpi: bool,
        pd_hvpps: bool,
        pd_hard_reset_on_refusal: bool,
        pd_spr_avs: bool,
        reset: bool,
    ):
        """Control port power features. Features version is 2."""
        req = command_pb2.CommandRequest()
        version = 2
        set_ports = 0
        for i, port in enumerate(PORTS):
            config = req.set_port_config.configs.add()
            config.version = version
            if port in ports:
                set_ports |= 1 << i
            config.features.enable_tfcp = tfcp if not reset else False
            config.features.enable_pe = pe if not reset else True
            config.features.enable_qc2p0 = qc2p0 if not reset else True
            config.features.enable_qc3p0 = qc3p0 if not reset else True
            config.features.enable_qc3plus = qc3plus if not reset else port != "A"
            config.features.enable_afc = afc if not reset else True
            config.features.enable_fcp = fcp if not reset else True
            config.features.enable_hv_scp = hvscp if not reset else True
            config.features.enable_lv_scp = lvscp if not reset else True
            config.features.enable_sfcp = sfcp if not reset else True
            config.features.enable_apple = apple if not reset else True
            config.features.enable_samsung = samsung if not reset else True
            config.features.enable_ufcs = ufcs if not reset else False
            config.features.enable_pd = pd if not reset else port != "A"
            config.features.enable_pd_compat_mode = (
                (pd and pd_compat) if not reset else False
            )
            config.features.enable_pd_lvpps = (
                (pd and pd_lvpps) if not reset else port != "A"
            )
            config.features.enable_pd_epr = (pd and pd_epr) if not reset else port != "A"
            config.features.enable_pd_rpi = (pd and pd_rpi) if not reset else False
            config.features.enable_pd_hvpps = (
                (pd and pd_hvpps) if not reset else port != "A"
            )
            config.features.enable_pd_hard_reset_on_refusal = (
                pd and pd_hard_reset_on_refusal
            ) if not reset else False
            config.features.enable_pd_spr_avs = (
                (pd and pd_spr_avs) if not reset else False
            )
        req.set_port_config.ports = set_ports

        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_PORT_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Set port config", fg="green")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.argument("voltage_mv", type=int, callback=validate_custom_pdo_voltage)
    @ensure_connected
    def set_custom_pdo_voltage(ctx, port: str, voltage_mv: int):
        """Set custom PDO voltage for a port.

        PORT: Port name to configure
        VOLTAGE_MV: Voltage in millivolts (5000-20000, must not be standard 5/9/15/20V)
        """
        req = command_pb2.CommandRequest()
        req.set_custom_pdo_voltage.ports.append(PORTS.index(port))
        req.set_custom_pdo_voltage.voltage_mv = voltage_mv
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_CUSTOM_PDO_VOLTAGE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Set custom PDO voltage for port {port} to {voltage_mv}mV",
                fg="green",
            )
