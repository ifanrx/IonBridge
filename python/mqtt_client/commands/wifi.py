"""WiFi management commands for MQTT Client CLI."""

import asyncclick as click

import command_pb2
import data_types_pb2

from data_types.wifi import WiFiAuthMode, WiFiStateType
from data_types.service import ServiceCommand
from ..core import send_and_receive
from ..decorators import ensure_connected


def register(main_group):
    """Register WiFi management commands with main CLI group."""

    @main_group.command()
    @ensure_connected
    def scan_wifi(ctx):
        """Scan for available WiFi networks."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SCAN_WIFI,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response or response.status != 0:
            click.secho("Failed to scan wifi", fg="red")
            return

        ssid_list = response.command_pb().scan_wifi.ssid_list
        for item in ssid_list:
            click.secho(
                f"SSID: {item.ssid} - RSSI: {item.rssi} - "
                f"Auth: {WiFiAuthMode.get_name_from_value(item.auth_mode)} - Stored: {item.saved}",
                fg="green",
            )

    @main_group.command()
    @click.argument("ssid", type=str)
    @click.argument("password", type=str, default="")
    @click.option("-n", "--no-passwd", is_flag=True, default=False)
    @ensure_connected
    def set_wifi_ssid_and_password(ctx, ssid: str, password: str, no_passwd: bool):
        """Set WiFi SSID and password to connect to."""
        ssid_bytes = ssid.encode("utf-8")
        if len(ssid_bytes) > 255:
            raise ValueError("SSID is too long")
        if len(password) > 63:
            raise ValueError("Password is too long")
        req = command_pb2.CommandRequest()
        req.set_wifi_ssid_and_password.ssid = ssid
        req.set_wifi_ssid_and_password.password = password
        req.set_wifi_ssid_and_password.auth_mode = (
            data_types_pb2.WiFiAuthMode.WIFI_AUTH_OPEN
            if no_passwd
            else data_types_pb2.WiFiAuthMode.WIFI_AUTH_WPA3_PSK
        )
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_WIFI_SSID_AND_PASSWORD,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Set wifi ssid and password", fg="green")

    @main_group.command()
    @click.argument(
        "action", type=click.Choice(["create", "update", "delete"], case_sensitive=False)
    )
    @click.argument("ssid", type=str)
    @click.argument("password", type=str, default="")
    @click.option("-n", "--no-passwd", is_flag=True, default=False)
    @ensure_connected
    def operate_wifi_record(ctx, action: str, ssid: str, password: str, no_passwd: bool):
        """Create, update, or delete a WiFi record."""
        ssid_bytes = ssid.encode("utf-8")
        if len(ssid_bytes) > 255:
            raise ValueError("SSID is too long")
        if len(password) > 63:
            raise ValueError("Password is too long")
        action_map = {
            "create": data_types_pb2.ManageAction.CREATE,
            "update": data_types_pb2.ManageAction.UPDATE,
            "delete": data_types_pb2.ManageAction.DELETE,
        }
        req = command_pb2.CommandRequest()
        req.operate_wifi_record.action = action_map[action]
        req.operate_wifi_record.ssid = ssid
        req.operate_wifi_record.password = password
        req.operate_wifi_record.auth_mode = (
            data_types_pb2.WiFiAuthMode.WIFI_AUTH_OPEN
            if no_passwd
            else data_types_pb2.WiFiAuthMode.WIFI_AUTH_WPA3_PSK
        )
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.OPERATE_WIFI_RECORD,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"{action} wifi record", fg="green")

    @main_group.command()
    @ensure_connected
    def reset_wifi(ctx):
        """Reset WiFi connection."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.RESET_WIFI,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Reset wifi", fg="green")

    @main_group.command()
    @ensure_connected
    def get_wifi_records(ctx):
        """Get stored WiFi records."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_WIFI_RECORDS,
            req,
            timeout=ctx.obj["timeout"],
        )
        if not response or response.status != 0:
            return

        click.secho(response.to_json(), fg="green")

    @main_group.command()
    @ensure_connected
    def get_wifi_state_machine(ctx):
        """Get WiFi state machine status."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_WIFI_STATE_MACHINE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            state = response.command_pb().get_wifi_state_machine.state
            click.secho(
                f"Get wifi state machine: {WiFiStateType(state).name}",
                fg="green",
            )
