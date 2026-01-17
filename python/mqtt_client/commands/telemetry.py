"""Telemetry commands for MQTT Client CLI, including subscribe functionality."""

import ipaddress
import random
import signal
import sys
import typing

import asyncclick as click
import paho.mqtt.client as mqtt_client
import paho.mqtt.enums
from google.protobuf.json_format import MessageToJson

import admin_pb2
import command_pb2
import telemetry_pb2

from data_types.port import PortType, PORTS
from data_types.service import ServiceCommand, TelemetryCommand, AdminCommand, Topic
from ..core import send_and_receive, ResponseMessage
from ..decorators import ensure_connected, cast_to_bool
from ..utils.formatters import info

# Stream commands (subset of ServiceCommand)
_stream_commands = [
    ServiceCommand.STREAM_PORT_STATUS,
    ServiceCommand.STREAM_DEVICE_STATUS,
    ServiceCommand.STREAM_PORT_PD_STATUS,
    ServiceCommand.STREAM_PROGRAMMER_STATUS,
]

# Service choices for subscribe command filtering
_service_choices = [
    *[cmd.name for cmd in _stream_commands],
    *[cmd.name for cmd in TelemetryCommand],
    *[cmd.name for cmd in AdminCommand],
]


def _build_topic(psn: str, cmd) -> str:
    """Map any command to its MQTT topic."""
    if isinstance(cmd, TelemetryCommand):
        return f"device/{psn}/telemetry/{cmd.value}"
    if isinstance(cmd, ServiceCommand):
        return f"device/{psn}/enduser/response/{cmd.value}"
    if isinstance(cmd, AdminCommand):
        return f"device/{psn}/admin/response/{cmd.value}"
    raise ValueError(f"Unknown command type: {type(cmd)}")


def _parse_commands(services: typing.List[str], only: str = None) -> typing.List:
    """Parse service names into command objects based on filter mode."""
    if only == "telemetry":
        if not services:
            return list(TelemetryCommand)
        return [TelemetryCommand[s.upper()] for s in services
                if s.upper() in TelemetryCommand.__members__]
    if only == "stream":
        if not services:
            return list(_stream_commands)
        return [ServiceCommand[s.upper()] for s in services
                if s.upper() in ServiceCommand.__members__
                and ServiceCommand[s.upper()] in _stream_commands]
    if only == "admin":
        if not services:
            return list(AdminCommand)
        return [AdminCommand[s.upper()] for s in services
                if s.upper() in AdminCommand.__members__]

    # No filter - accept all command types
    commands = []
    for s in services:
        upper = s.upper()
        if upper in TelemetryCommand.__members__:
            commands.append(TelemetryCommand[upper])
        elif upper in ServiceCommand.__members__:
            commands.append(ServiceCommand[upper])
        elif upper in AdminCommand.__members__:
            commands.append(AdminCommand[upper])
    return commands


def _is_remote_broker(host: str) -> bool:
    """Check if host requires TLS (not localhost or IP)."""
    if host in ("localhost", "127.0.0.1"):
        return False
    try:
        ipaddress.ip_address(host)
        return False
    except ValueError:
        return True


def _show_stream_port_status(message: command_pb2.CommandResponse):
    """Display StreamPortStatus message with colored output."""
    port_status = message.stream_port_status
    click.secho("StreamPortStatus:")
    click.secho(f"  Status: {message.status}")
    click.secho(f"  Port Status Map: {bin(port_status.port_status_map)}")
    for i in range(len(PORTS)):
        port_name = PORTS[i]
        port = port_status.ports[i]
        open_state = ((port_status.port_status_map >> i) & 0x1) == 0x1
        fg = None
        if open_state:
            fg = "green"
        if port.details.connected:
            fg = "magenta"
        click.secho(f"  Port {port_name}({i}):", fg=fg)
        click.secho(
            f"    Features: {MessageToJson(port.features, indent=None, always_print_fields_with_no_presence=True)}",
        )
        click.secho(f"    Charging Minutes: {port.charging_minutes}")
        click.secho(
            f"    Details: {MessageToJson(port.details, indent=None, always_print_fields_with_no_presence=True)}",
        )
        click.secho(f"    Port Type: {PortType(port.port_type).name}")
        click.secho(f"    Custom PDO voltage: {port.custom_pdo_voltage_mv}mV")
        click.secho(f"    Cable Compensation Enable: {port.cable_compensation_enable}")
        click.secho(f"    Cable Resistance: {port.cable_resistance}")
        click.secho(f"    Cable Voltage Offset: {port.cable_voltage_offset}")
        click.secho("\n")


def _sanitize_filename(name: str) -> str:
    """Sanitize a string for use in filenames by removing path separators."""
    # Remove path separators and other dangerous characters
    return name.replace("/", "_").replace("\\", "_").replace("..", "_")


def _save_coredump_info(psn: str, message: telemetry_pb2.TelemetryData):
    """Save coredump slice data to file.

    Raises:
        IOError: If file write fails, propagated to caller for handling.
    """
    coredump_slice = message.coredump_slice
    # Sanitize PSN to prevent path traversal attacks
    safe_psn = _sanitize_filename(psn)
    filename = f"{safe_psn}-{coredump_slice.timestamp}.elf"

    try:
        with open(filename, "r+b" if coredump_slice.offset > 0 else "wb") as f:
            if coredump_slice.offset == 0:
                click.secho(f"Received coredump slice, store to file {filename}", fg="green")
            f.seek(coredump_slice.offset)
            f.write(bytes(coredump_slice.data))
    except IOError as e:
        click.secho(f"Failed to write coredump slice to file: {e}", fg="red")
        raise  # Propagate error to caller instead of silent return

    if coredump_slice.last:
        click.secho("All coredump slice received, file saved.", fg="green")


def _handle_config_report(message, topic: str):
    """Handle CONFIG_REPORT telemetry message."""
    click.secho("ConfigReport received:", fg="green")
    pb_message = message.telemetry_pb()
    if pb_message.HasField("config_report"):
        config_report = pb_message.config_report
        click.secho(f"  Timestamp: {config_report.timestamp}", fg="cyan")
        if config_report.HasField("config"):
            cfg = config_report.config
            click.secho(f"  Config timestamp: {cfg.timestamp}", fg="cyan")
            click.secho(f"  Ports: {len(cfg.ports)}", fg="cyan")
    click.secho(message.to_json())


def _handle_coredump(message, topic: str):
    """Handle COREDUMP_INFO telemetry message."""
    topic_obj = Topic.parse(topic)
    try:
        _save_coredump_info(topic_obj.psn, message.telemetry_pb())
    except IOError:
        # Error already logged in _save_coredump_info, continue receiving messages
        pass


def _handle_stream_port_status(message, topic: str):
    """Handle STREAM_PORT_STATUS service message."""
    _show_stream_port_status(message.command_pb())


def _handle_default(message, topic: str):
    """Default handler - just print JSON."""
    click.secho(message.to_json())


# Message handler dispatch table (nested by command type to avoid IntEnum hash collisions)
# IntEnums with same value (e.g., CONFIG_REPORT=0x80, STREAM_PORT_STATUS=0x80) compare equal,
# so we must key by (type, value) to distinguish them.
_message_handlers = {
    TelemetryCommand: {
        TelemetryCommand.CONFIG_REPORT: _handle_config_report,
        TelemetryCommand.COREDUMP_INFO: _handle_coredump,
    },
    ServiceCommand: {
        ServiceCommand.STREAM_PORT_STATUS: _handle_stream_port_status,
    },
}


def register(main_group):
    """Register telemetry commands with main CLI group."""

    @main_group.command()
    @ensure_connected
    def start_telemetry_stream(ctx):
        """Start telemetry streaming from device."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.START_TELEMETRY_STREAM,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Start telemetry stream", fg="green")

    @main_group.command()
    @ensure_connected
    def stop_telemetry_stream(ctx):
        """Stop telemetry streaming from device."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.STOP_TELEMETRY_STREAM,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Stop telemetry stream", fg="green")

    @main_group.command()
    @click.argument("debug", type=str, callback=cast_to_bool)
    @ensure_connected
    def set_telemetry_debug(ctx, debug: bool):
        """Enable or disable telemetry debug mode."""
        req = admin_pb2.AdminCommandRequest()
        req.set_telemetry_debug.enable = debug
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.SET_TELEMETRY_DEBUG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Set telemetry debug: {debug}", fg="green")

    @main_group.command()
    @click.option(
        "--count", type=int, default=0,
        help="Number of messages to receive before disconnecting (0=unlimited)",
    )
    @click.option("-s", "--start", is_flag=True, help="Start telemetry stream before subscribing")
    @click.option(
        "--only", type=click.Choice(["telemetry", "stream", "admin"]),
        help="Filter by command type",
    )
    @click.option("--list-commands", is_flag=True, help="List all available commands and exit")
    @click.argument(
        "services",
        type=click.Choice(_service_choices, case_sensitive=False),
        nargs=-1,
    )
    @click.pass_context
    def subscribe(ctx, count: int, start: bool, only: str, list_commands: bool, services: typing.Tuple[str, ...]):
        """Subscribe to MQTT topics and display messages.

        Examples:
            subscribe                          # All messages
            subscribe CONFIG_REPORT            # Specific command
            subscribe --only telemetry         # All telemetry
            subscribe --only stream            # All stream
            subscribe --only admin             # All admin
        """
        if list_commands:
            _list_commands()
            return

        psn = ctx.obj.get("psn")
        if not psn:
            raise click.UsageError("--psn is required")

        # Parse commands and build topics
        commands = _parse_commands(list(services), only)
        if commands:
            topics = [(_build_topic(psn, cmd), 0) for cmd in commands]
        else:
            topics = [
                (f"device/{psn}/telemetry/+", 0),
                (f"device/{psn}/enduser/response/+", 0),
                (f"device/{psn}/admin/response/+", 0),
            ]

        # Run the subscriber
        _run_subscriber(ctx, psn, topics, commands, count, start)


def _list_commands():
    """Display available commands."""
    click.secho("\nTelemetry Commands:", fg="green", bold=True)
    for cmd in TelemetryCommand:
        click.secho(f"  {cmd.name:<30} (0x{cmd.value:02X})", fg="cyan")

    click.secho("\nStream Commands:", fg="green", bold=True)
    for cmd in _stream_commands:
        click.secho(f"  {cmd.name:<30} (0x{cmd.value:02X})", fg="yellow")

    click.secho("\nAdmin Commands:", fg="green", bold=True)
    for cmd in AdminCommand:
        click.secho(f"  {cmd.name:<30} (0x{cmd.value:02X})", fg="magenta")


def _run_subscriber(ctx, psn: str, topics: list, commands: list, count: int, start: bool):
    """Run the MQTT subscriber loop."""
    broker = ctx.obj.get("broker", "localhost")
    port = ctx.obj.get("port", 1883)
    ca_certs = ctx.obj.get("ca_certs", "certs/ca.crt")
    certfile = ctx.obj.get("certfile", "certs/client.pem")
    keyfile = ctx.obj.get("keyfile", "certs/client.key")

    receive_count = 0

    def on_connect(client, userdata, flags, rc, properties):
        if rc == 0:
            click.echo("Connected to MQTT Broker!")
            for topic, qos in topics:
                click.echo(f"Subscribing to topic: {topic}")
                client.subscribe(topic, qos)
        else:
            click.echo(f"Failed to connect, return code {rc}")

    def on_message(client, userdata, msg):
        nonlocal receive_count
        receive_count += 1

        try:
            message = ResponseMessage.deserialize(msg.topic, msg.payload)
        except Exception as e:
            click.secho(f"ERROR: Failed to parse: {e}", fg="red")
            return

        if commands and message.command not in commands:
            return

        # Dispatch to handler (nested lookup by command type to handle IntEnum collisions)
        handlers_for_type = _message_handlers.get(type(message.command), {})
        handler = handlers_for_type.get(message.command, _handle_default)
        handler(message, msg.topic)

        if count > 0 and receive_count >= count:
            client.disconnect()

    # Create client
    client = mqtt_client.Client(
        paho.mqtt.enums.CallbackAPIVersion.VERSION2, protocol=mqtt_client.MQTTv5
    )
    if _is_remote_broker(broker):
        client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)

    if start:
        req = command_pb2.CommandRequest()
        req.id = random.randrange(1, 65535)
        client.publish(
            f"device/{psn}/enduser/request/{ServiceCommand.START_TELEMETRY_STREAM.value}",
            req.SerializeToString(),
        )

    def signal_handler(signum, frame):
        info("\nShutting down...")
        client.disconnect()
        client.loop_stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        client.disconnect()
        client.loop_stop()
