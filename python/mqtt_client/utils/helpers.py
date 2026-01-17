"""Shared helper functions for MQTT Client CLI."""

import logging
import typing

import asyncclick as click
from google.protobuf.json_format import MessageToJson

import admin_pb2
import data_types.port as port_types
from data_types.port import (
    GetPortDetailsResponse,
    KeepAliveResponse,
    PortCommand,
    PORTS,
)
from data_types.service import AdminCommand

from ..core import send_and_receive, ResponseMessage

logger = logging.getLogger(__name__)


def get_port_docstring(action, multiple=False):
    """Generate port argument docstring."""
    ports_list = ",".join(PORTS)
    port_desc = "Port name(s)" if multiple else "Port name"
    return f"""{action.title()} one or more ports.

    PORT: {port_desc} to {action}. Available: {ports_list} (use --product-family to filter)
    """


def do_forward_port(
    psn: str,
    port_id: int,
    command: int | PortCommand,
    payload: typing.Optional[bytes] = None,
    timeout: int = 5,
) -> typing.Optional[ResponseMessage]:
    """Forward a command to a specific port."""
    req = admin_pb2.AdminCommandRequest()
    req.forward_port.port = port_id
    req.forward_port.command = command
    req.forward_port.data = payload if payload else bytes([0])
    return send_and_receive(
        psn,
        AdminCommand.FORWARD_PORT,
        req,
        timeout=timeout,
    )


def decode_forward_port_resp(
    response: admin_pb2.ForwardPortResp, port: str, command: PortCommand
) -> None:
    """Decode and display forward port response."""
    resp_command = PortCommand(response.command)
    if resp_command != command:
        click.secho(
            f"Unexpected command 0x{response.command:04X} in response",
            fg="red",
            err=True,
        )
        return

    match resp_command:
        case PortCommand.KEEP_ALIVE:
            resp = KeepAliveResponse.from_buffer_copy(response.data)
            click.secho(f"{port} Keep alive response received: {resp}", fg="green")
        case PortCommand.GET_PORT_DETAILS:
            resp = GetPortDetailsResponse.from_buffer_copy(response.data)
            if resp.status != 0:
                click.secho("Failed to get port details", fg="red", err=True)
                return
            click.secho(f"{port} Port details received: {resp.details}", fg="green")
        case PortCommand.REPORT_FW_INFO:
            resp = port_types.ReportFWInfoResponse.from_buffer_copy(response.data)
            if resp.status != 0:
                click.secho("Failed to get FW info", fg="red", err=True)
                return
            click.secho(f"{port} FW info: {resp}", fg="green")
        case _:
            click.secho(
                f"Forward port response data: {response.data.hex()}", fg="green"
            )


def handle_port_config_resp(
    response: typing.Optional[ResponseMessage],
    ports: typing.Optional[typing.Tuple[str, ...]] = None,
) -> None:
    """Handle and display port configuration response."""
    if not response or response.status != 0:
        return

    configs = response.command_pb().get_port_config.configs
    for i in range(len(PORTS)):
        port_name = PORTS[i]
        # If ports filter is provided, skip ports not in the filter
        if ports and port_name not in ports:
            continue
        config = configs[i]
        click.secho(
            f"port {port_name}: version {config.version} {MessageToJson(config.features, always_print_fields_with_no_presence=True)}",
            fg="green",
        )
        click.secho("")
