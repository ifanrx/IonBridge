"""Diagnostic and debug commands for MQTT Client CLI."""

import asyncio
import dataclasses
import pathlib
import time
import typing

import asyncclick as click

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.port import (
    CableCompensation,
    FastChargingProtocol,
    FixedVoltageOffset,
    GetPortDetailsResponse,
    PortCommand,
    PORTS,
    SetMaxPowerBudgetRequest,
    SetMaxPowerBudgetResponse,
    SystemFlags,
)
from data_types.service import ServiceCommand, AdminCommand, TelemetryCommand
from usb_pd import pcap
from ..core import send_and_receive, wait_for_response, unsubscribe
from ..decorators import ensure_connected, cast_to_bool
from ..utils.validators import validate_port, validate_port_with_index, validate_port_command
from ..utils.helpers import do_forward_port, decode_forward_port_resp

# Power budget limits for port diagnostic (in Watts)
MAX_POWER_BUDGET_W = 140
MIN_POWER_BUDGET_W = 15


async def _asleep(secs):
    """Async sleep helper."""
    for _ in range(secs * 10):
        await asyncio.sleep(0.1)


@dataclasses.dataclass()
class DiagnosticResult:
    """Result of a port diagnostic test."""
    budget: int  # W
    voltage: int = 0  # mV
    current: int = 0  # mA
    vin: int = 0  # mV
    protocol: FastChargingProtocol = FastChargingProtocol.NOT_CHARGING
    finished: bool = False
    failed_reason: str = ""


async def _do_port_diagnostic(
    psn: str,
    port_id: int,
    timeout: int,
    step: int,
    delay_secs: int,
):
    """Execute port diagnostic test sequence."""
    diagnostic_results = []
    budget_range = range(MAX_POWER_BUDGET_W, MIN_POWER_BUDGET_W - 1, -step)

    with click.progressbar(budget_range, label="Port diagnostic") as bar:
        for budget in bar:
            result = DiagnosticResult(budget)
            diagnostic_results.append(result)

            # Shutdown port
            resp = do_forward_port(
                psn, port_id, PortCommand.SET_PORT_STATE, bytes([1]), timeout
            )
            if not resp or resp.status != 0:
                result.failed_reason = "Failed to turn off port"
                break

            # Set power budget
            payload = SetMaxPowerBudgetRequest(
                max_power_budget=budget,
                watermark=budget,
                force_rebroadcast=1,
                unused=0,
            )
            resp = do_forward_port(
                psn, port_id, PortCommand.SET_MAX_POWER_BUDGET, bytes(payload), timeout
            )
            if not resp or resp.status != 0:
                result.failed_reason = "Failed to set power budget"
                continue

            response = SetMaxPowerBudgetResponse.from_buffer_copy(
                resp.admin_pb().forward_port.data
            )
            if response.budget != budget:
                result.failed_reason = f"Response budget mismatch"
                continue

            # Turn on port
            resp = do_forward_port(
                psn, port_id, PortCommand.SET_PORT_STATE, bytes([0]), timeout
            )
            if not resp or resp.status != 0:
                result.failed_reason = "Failed to turn on port"
                break

            await _asleep(delay_secs)

            # Get port details
            resp = do_forward_port(psn, port_id, PortCommand.GET_PORT_DETAILS, timeout=timeout)
            if not resp or resp.status != 0:
                result.failed_reason = "Failed to get port details"
                continue

            cmd = resp.admin_pb().forward_port.command
            if cmd != PortCommand.GET_PORT_DETAILS:
                result.failed_reason = "Failed to get port details"
                continue

            cmd_resp = GetPortDetailsResponse.from_buffer_copy(
                resp.admin_pb().forward_port.data
            )
            details = cmd_resp.details
            result.finished = True
            result.voltage = details.vout_value
            result.current = details.iout_value
            result.vin = details.vin_value
            result.protocol = FastChargingProtocol(details.protocol)
            if details.vout_value < 3000 or details.iout_value < 100:
                result.failed_reason = "Invalid voltage or current"

    if diagnostic_results:
        click.secho(
            "Budget(V)\tFinished\tVoltage(mV)\tCurrent(mA)\tVin(mV)\t\tProtocol\tReason",
            fg="green",
        )
        for res in diagnostic_results:
            click.secho(
                f"{res.budget}\t\t{res.finished}\t\t{res.voltage}\t\t{res.current}\t\t"
                f"{res.vin}\t\t{res.protocol.name}\t{res.failed_reason}",
                fg="green" if not res.failed_reason else "red",
            )


async def _back_to_bootloader(psn: str, port_id: int, timeout: int) -> bool:
    """Send system reset to enter bootloader mode."""
    response = do_forward_port(psn, port_id, PortCommand.SYSTEM_RESET, timeout=timeout)
    if not response or response.status != 0:
        click.secho(f"Failed to reset port {PORTS[port_id]}", fg="red", err=True)
        return False

    await _asleep(3)

    response = do_forward_port(psn, port_id, PortCommand.KEEP_ALIVE, timeout=timeout)
    if not response or response.status != 0:
        click.secho(f"Failed to send keep alive to port {PORTS[port_id]}", fg="red", err=True)
        return False

    return True


async def _erase_flash(psn: str, port_id: int, timeout: int) -> bool:
    """Erase flash on port."""
    response = do_forward_port(
        psn, port_id, PortCommand.ERASE_FLASH, bytes([0, 0, 0, 0]), timeout=timeout
    )
    if not response or response.status != 0:
        click.secho(f"Failed to erase flash on port {PORTS[port_id]}", fg="red", err=True)
        return False
    return True


async def _upgrade_3566(psn: str, port_id: int, firmware: str, timeout: int) -> bool:
    """Upload firmware to port."""
    firmware_path = pathlib.Path(firmware)
    if not firmware_path.exists():
        click.secho(f"Firmware file not found: {firmware}", fg="red", err=True)
        return False

    firmware_data = firmware_path.read_bytes()
    chunk_size = 256
    for offset in range(0, len(firmware_data), chunk_size):
        chunk = firmware_data[offset:offset + chunk_size]
        payload = bytearray()
        payload.extend(offset.to_bytes(4, "little"))
        payload.extend(chunk)
        response = do_forward_port(
            psn, port_id, PortCommand.WRITE_FLASH, bytes(payload), timeout=timeout
        )
        if not response or response.status != 0:
            click.secho(f"Failed to write flash at offset {offset}", fg="red", err=True)
            return False

    # Reset after upgrade
    response = do_forward_port(psn, port_id, PortCommand.SYSTEM_RESET, timeout=timeout)
    return True


def register(main_group):
    """Register diagnostic commands with main CLI group."""

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.argument("command", type=str, callback=validate_port_command, metavar="COMMAND_NAME")
    @click.argument("data", type=str, default="")
    @click.option("-a", "--address", type=str, default=None)
    @ensure_connected
    def forward_port(
        ctx, port: str, command: PortCommand, data: str, address: str
    ):
        """Forward a command to a specific port.

        PORT: Target port name
        COMMAND_NAME: Port command name to execute
        DATA: Optional hex-encoded data payload
        """
        response = do_forward_port(
            ctx.obj["psn"],
            PORTS.index(port),
            command.value,
            bytes.fromhex(data),
            timeout=ctx.obj["timeout"],
        )
        if not (response and response.status == 0):
            click.secho("Failed to forward port command", fg="red", err=True)
            return

        decode_forward_port_resp(response.admin_pb().forward_port, port, command)

    @main_group.command()
    @ensure_connected
    def read_adc_value(ctx):
        """Read ADC value from device."""
        req = admin_pb2.AdminCommandRequest()
        resp = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.READ_ADC,
            req,
            timeout=ctx.obj["timeout"],
        )
        if resp and resp.status == 0:
            click.secho(f"ADC value: {resp.admin_pb().read_adc.adc_value}", fg="green")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.option("-c", "--commit", is_flag=True, default=False)
    @click.option("-f", "--disable_flash_cache", is_flag=True, default=False)
    @click.option("-a", "--type_a_port", is_flag=True, default=False)
    @click.option("-d", "--disable_cable_compensation", is_flag=True, default=False)
    @click.option(
        "-s", "--cable_compensation",
        type=click.Choice(CableCompensation.get_choices(), case_sensitive=False),
        callback=CableCompensation.click_callback,
        default="65mOhm",
    )
    @click.option(
        "-v", "--fixed_voltage_offset",
        type=click.Choice(FixedVoltageOffset.get_choices(), case_sensitive=False),
        callback=FixedVoltageOffset.click_callback,
        default="0mV",
    )
    @click.option("-V", "--verbose", is_flag=True, default=False)
    @ensure_connected
    def set_system_flags(
        ctx,
        port: str,
        commit: bool,
        disable_flash_cache: bool,
        type_a_port: bool,
        disable_cable_compensation: bool,
        cable_compensation: int,
        fixed_voltage_offset: int,
        verbose: bool,
    ):
        """Set system flags for a port."""
        flags = SystemFlags()
        flags.commit = 0x55 if commit else 0
        flags.enable_flash_cache = not disable_flash_cache
        flags.port_type = 0 if type_a_port else 1
        flags.enable_cable_compensation = not disable_cable_compensation
        flags.cable_compensation = cable_compensation
        flags.fixed_voltage_offset = fixed_voltage_offset
        if verbose and commit:
            click.secho(f"Set system flags: {flags}", fg="green")

        req = admin_pb2.AdminCommandRequest()
        req.forward_port.port = PORTS.index(port) & 0xFF
        req.forward_port.command = PortCommand.SYSTEM_FLAGS
        req.forward_port.data = bytes(flags)
        if verbose:
            click.secho(f"Serialized payload: {req.forward_port.data.hex()}", fg="green")
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.FORWARD_PORT,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            pb_msg = response.admin_pb().forward_port
            payload = pb_msg.data
            if verbose:
                click.secho(f"Response data: {payload.hex()}", fg="green")
            cmd = PortCommand.from_value(pb_msg.command)
            if cmd != PortCommand.SYSTEM_FLAGS:
                return
            flags = SystemFlags.from_buffer_copy(payload)
            click.secho(f"System flags for port {port}:\n{flags}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_messages(ctx):
        """Listen for all incoming messages."""
        from ..core import transport
        transport.ignore_push_msg = False
        while True:
            message = wait_for_response(return_all=True)
            if message:
                click.secho(f"Message: {message.to_json()}", fg="green")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @click.argument("subscribe", type=str, callback=cast_to_bool)
    @click.option("-f", "--filename", type=str, default=None, help="Save pcap data to file")
    @click.option("-r", "--raw", is_flag=True, default=False, help="Save raw data")
    @ensure_connected
    def subscribe_pd_pcap_streaming(ctx, port: str, subscribe: bool, filename: str, raw: bool):
        """Subscribe to PD PCAP streaming from a port."""
        port_id = PORTS.index(port) & 0xFF
        req = command_pb2.CommandRequest()
        req.subscribe_pdpcap_streaming.port = port_id
        req.subscribe_pdpcap_streaming.subscribe = subscribe
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SUBSCRIBE_PD_PCAP_STREAMING,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response or response.status != 0:
            click.secho(
                f"Failed to {'un' if not subscribe else ''}subscribe to PD pcap streaming",
                fg="red", err=True,
            )
            return

        click.secho(f"Subscribe PD pcap streaming from port {port}: {subscribe}", fg="green")
        if not (subscribe and filename):
            return

        # Save pcap data to csv file
        with pcap.PDPcapParser(filename) as parser:
            click.secho("Receiving and parsing pcap data, press Ctrl+C to exit", fg="green")
            while True:
                try:
                    message = wait_for_response(
                        cmd=TelemetryCommand.PD_PCAP_DATA, timeout=ctx.obj["timeout"]
                    )
                except KeyboardInterrupt:
                    click.secho(f"Saving pcap data to file: {filename}", fg="yellow")
                    break

                if not message or message.status != 0 or len(message.payload) < 1:
                    continue
                pb_message = message.telemetry_pb().pdp_cap_data
                if (pb_message.port & 0xFF) != port_id:
                    continue

                pcap_data = pb_message.data
                if raw:
                    raw_file = pathlib.Path(filename).with_suffix(".bin")
                    with raw_file.open("ab") as f:
                        f.write(pcap_data)
                try:
                    parser.parse_pcap(pcap_data)
                except Exception as e:
                    click.secho(f"Failed to parse pcap data: {e}", fg="red")
                    continue

        # Unsubscribe when done
        req.subscribe_pdpcap_streaming.subscribe = False
        send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SUBSCRIBE_PD_PCAP_STREAMING,
            req,
            timeout=ctx.obj["timeout"],
        )

    @main_group.command()
    @click.argument("state", type=click.Choice(["enable", "disable"], case_sensitive=False))
    @ensure_connected
    def toggle_temperature_protect(ctx, state: str):
        """Enable or disable temperature protection."""
        req = admin_pb2.AdminCommandRequest()
        req.toggle_temperature_protect.enable = state == "enable"
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.TOGGLE_TEMPERATURE_PROTECT,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Toggle temperature protect: {state}", fg="green")

    @main_group.command()
    @click.argument("port", type=str, callback=validate_port)
    @ensure_connected
    def toggle_mcu_gpio(ctx, port: str):
        """Toggle SW3566 GPIO2 for specified port."""
        req = admin_pb2.AdminCommandRequest()
        req.toggle_mcu_gpio.port = PORTS.index(port)
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.TOGGLE_MCU_GPIO,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Toggle MCU GPIO2 port {port}", fg="green")

    @main_group.command()
    @click.argument(
        "port", type=str, callback=validate_port_with_index,
    )
    @click.option("--step", type=int, default=5, help="Power budget step, in W")
    @click.option("--delay", type=int, default=10, help="Delay in seconds")
    @click.option("--disable_pps", is_flag=True, default=False)
    @click.option("--disable_epr", is_flag=True, default=False)
    @ensure_connected
    async def port_diagnostic(
        ctx,
        port: typing.Tuple[str, int],
        step: int,
        delay: int,
        disable_pps: bool,
        disable_epr: bool,
    ):
        """Run port diagnostic test."""
        click.clear()
        port_name, port_id = port
        req = admin_pb2.AdminCommandRequest()
        req.port_diagnostic.port = port_id
        req.port_diagnostic.diagnostic = True
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.PORT_DIAGNOSTIC,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response or response.status != 0:
            click.secho(f"Failed to start diagnostic for port {port_name}", fg="red")
            return

        await _do_port_diagnostic(
            ctx.obj["psn"], port_id, ctx.obj["timeout"], step, delay
        )

    @main_group.command()
    @click.argument("firmware", type=click.Path(exists=True))
    @click.argument("ports", type=str, nargs=-1, callback=validate_port)
    @click.option("-y", "--yes", is_flag=True, default=False)
    @ensure_connected
    async def upgrade_bird_3566(ctx, firmware: str, ports: typing.Tuple[str, ...], yes: bool):
        """Upgrade SW3566 firmware on Bird board."""
        # Temporarily unsubscribe from telemetry
        if "telemetry" in ctx.obj["topic"]:
            unsubscribe(ctx.obj["topic"]["telemetry"])
            click.secho("Temporarily unsubscribed from telemetry", fg="yellow")

        async def _upgrade_port(port: str) -> bool:
            click.secho(f"Upgrading port: {port}", fg="green")
            port_id = PORTS.index(port) & 0xFF

            click.secho("\tBack to bootloader", fg="blue")
            if not await _back_to_bootloader(ctx.obj["psn"], port_id, ctx.obj["timeout"]):
                return False

            click.secho("\tErase flash", fg="blue")
            if not await _erase_flash(ctx.obj["psn"], port_id, ctx.obj["timeout"]):
                return False

            click.secho("\tUpgrade firmware", fg="blue")
            if not await _upgrade_3566(ctx.obj["psn"], port_id, firmware, ctx.obj["timeout"]):
                return False

            click.secho(f"Upgrade port {port} done", fg="green")
            return True

        ports = ports or PORTS
        for idx, port in enumerate(ports):
            click.clear()
            await _upgrade_port(port)
            if idx == len(ports) - 1:
                click.secho("All ports upgraded", fg="green")
                return

            if yes:
                continue

            try:
                input(f"Press Enter to continue to upgrade port {ports[idx + 1]} or Ctrl+C to exit")
            except KeyboardInterrupt:
                click.secho("Upgrade cancelled", fg="red")
                break

    @main_group.command()
    @ensure_connected
    def push_license(ctx):
        """Push license data to device."""
        req = admin_pb2.AdminCommandRequest()
        set_license_req = admin_pb2.SetLicenseReq()
        license_data = data_types_pb2.LicenseData()
        license_data.f.extend([10000, 100, 20101, 20102])
        license_data.b = 1743391253
        license_data.a = int(time.time()) + 864000
        set_license_req.data.append(license_data)
        req.set_license.CopyFrom(set_license_req)

        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.PUSH_LICENSE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Set license", fg="green")
