"""Device configuration commands for MQTT Client CLI."""

import json
import pathlib
import time
import zlib

import asyncclick as click
from google.protobuf.json_format import MessageToDict

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive
from ..decorators import ensure_connected, cast_state_to_int
from ..models import ResetDeviceOptions

# PHY data file size in bytes (includes PHYINIT header)
PHY_DATA_SIZE = 144


def register(main_group):
    """Register device configuration commands with main CLI group."""

    @main_group.command()
    @click.argument(
        "entity",
        type=click.Choice(
            ["hardware_revision", "product_family", "device_model", "device_color", "device_name"],
            case_sensitive=False,
        ),
    )
    @click.argument(
        "action",
        type=click.Choice(["read", "update"], case_sensitive=False),
    )
    @click.argument("value", required=False)
    @ensure_connected
    def manage_device_config(ctx, entity: str, action: str, value: str = None):
        """Manage device config (hardware_revision, product_family, device_model, device_color, device_name)"""
        entity_map = {
            "hardware_revision": data_types_pb2.ENTITY_HARDWARE_REVISION,
            "product_family": data_types_pb2.ENTITY_PRODUCT_FAMILY,
            "device_model": data_types_pb2.ENTITY_DEVICE_MODEL,
            "device_color": data_types_pb2.ENTITY_DEVICE_COLOR,
            "device_name": data_types_pb2.ENTITY_DEVICE_NAME,
        }
        action_map = {
            "read": data_types_pb2.READ,
            "update": data_types_pb2.UPDATE,
        }

        req = admin_pb2.AdminCommandRequest()
        req.manage_device_config.entity = entity_map[entity.lower()]
        req.manage_device_config.action = action_map[action.lower()]

        if action.lower() == "update":
            if not value:
                click.secho("Value is required for UPDATE action", fg="red")
                return
            req.manage_device_config.value = value

        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_DEVICE_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            pb_message = response.admin_pb()
            result_value = pb_message.manage_device_config.value

            if action.lower() == "read":
                click.secho(f"{entity}: {result_value}", fg="green")
            else:
                click.secho(f"{entity} updated to: {result_value}", fg="green")

    @main_group.command()
    @click.option(
        "-o",
        "--output",
        type=click.Path(),
        default="phy_init.bin",
        help="Output file path (default: phy_init.bin)",
    )
    @ensure_connected
    def read_phy_data(ctx, output: str):
        """Read current PHY data from device and save to file."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_phy_data.action = data_types_pb2.READ
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_PHY_DATA,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            pb_message = response.admin_pb()
            phy_data = pb_message.manage_phy_data.data
            crc32 = pb_message.manage_phy_data.crc32

            # Save to file
            output_path = pathlib.Path(output)
            output_path.write_bytes(phy_data)

            click.secho(f"PHY data saved to: {output_path}", fg="green")
            click.secho(f"Size: {len(phy_data)} bytes", fg="green")
            click.secho(f"CRC32: 0x{crc32:08x}", fg="green")
            # Show magic header
            if len(phy_data) >= 8:
                magic = phy_data[:8]
                click.secho(f"Magic header: {magic!r}", fg="cyan")

    @main_group.command()
    @click.argument("file_path", type=click.Path(exists=True))
    @ensure_connected
    def write_phy_data(ctx, file_path: str):
        """Write PHY data to device from a local file.

        FILE_PATH: Path to the PHY data file (144 bytes, must have PHYINIT header)
        """
        # Read the file
        phy_file = pathlib.Path(file_path)
        phy_data = phy_file.read_bytes()

        # Validate size
        if len(phy_data) != PHY_DATA_SIZE:
            click.secho(f"Error: PHY data must be {PHY_DATA_SIZE} bytes, got {len(phy_data)}", fg="red")
            return

        # Validate magic header
        if not phy_data.startswith(b"PHYINIT\x00"):
            click.secho("Error: Invalid PHY magic header (expected 'PHYINIT')", fg="red")
            return

        # Calculate CRC32
        expected_crc32 = zlib.crc32(phy_data) & 0xFFFFFFFF

        click.secho(f"PHY data size: {len(phy_data)} bytes", fg="cyan")
        click.secho(f"Expected CRC32: 0x{expected_crc32:08x}", fg="cyan")

        req = admin_pb2.AdminCommandRequest()
        req.manage_phy_data.action = data_types_pb2.UPDATE
        req.manage_phy_data.data = phy_data
        req.manage_phy_data.expected_crc32 = expected_crc32

        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_PHY_DATA,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            pb_message = response.admin_pb()
            written_crc32 = pb_message.manage_phy_data.crc32
            click.secho(f"PHY data written successfully!", fg="green")
            click.secho(f"Written CRC32: 0x{written_crc32:08x}", fg="green")
        else:
            click.secho("Failed to write PHY data", fg="red")

    @main_group.command()
    @ensure_connected
    def get_device_password(ctx):
        """Get device password."""
        req = admin_pb2.AdminCommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.GET_DEVICE_PASSWORD,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            password = response.admin_pb().get_device_password.password
            click.secho(f"Get password: {''.join(str(d) for d in password[:4])}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_hardware_revision(ctx):
        """Get hardware revision."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_HARDWARE_REVISION,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Get hardware revision: {response.command_pb().get_hardware_revision.revision}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def get_device_info(ctx):
        """Get device information (versions, etc.)."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DEVICE_INFO,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            fmt_version = lambda v: f"{v.major}.{v.minor}.{v.patch}"  # noqa
            info = response.command_pb().get_device_info.info
            info_d = MessageToDict(info, always_print_fields_with_no_presence=True)
            info_d["apVersion"] = fmt_version(info.ap_version)
            info_d["bpVersion"] = fmt_version(info.bp_version)
            info_d["fpgaVersion"] = fmt_version(info.fpga_version)
            info_d["zrlibVersion"] = fmt_version(info.zrlib_version)
            click.secho(
                f"Get device info: {json.dumps(info_d, indent=4)}",
                fg="green",
            )

    @main_group.command()
    @click.option("--keep-wifi-nvs", is_flag=True, help="Keep WiFi NVS namespace after reset")
    @click.option("--keep-mqtt-nvs", is_flag=True, help="Keep MQTT NVS namespace after reset")
    @click.option("--keep-ble-nvs", is_flag=True, help="Keep BLE NVS namespace after reset")
    @click.option(
        "--keep-display-nvs", is_flag=True, help="Keep display NVS namespace after reset"
    )
    @click.option("--keep-power-nvs", is_flag=True, help="Keep power NVS namespace after reset")
    @click.option("--keep-system-nvs", is_flag=True, help="Keep system NVS namespace after reset")
    @click.option(
        "--keep-license-nvs", is_flag=True, help="Keep license NVS namespace after reset"
    )
    @click.option("--keep-syslog-nvs", is_flag=True, help="Keep syslog NVS namespace after reset")
    @click.option("--keep-fpga-nvs", is_flag=True, help="Keep FPGA NVS namespace after reset")
    @click.option("--keep-ota-nvs", is_flag=True, help="Keep OTA NVS namespace after reset")
    @click.option("--prevent-reboot", is_flag=True, help="Prevent device reboot after reset")
    @click.option("--reset-ports", is_flag=True, help="Reset ports configuration")
    @click.option(
        "--reset-power-strategy", is_flag=True, help="Reset power strategy configuration"
    )
    @click.option("--reset-wifi", is_flag=True, help="Reset current Wi-Fi connection")
    @click.option("--reset-display", is_flag=True, help="Reset current display settings")
    @click.option("--reset-fc-protocol", is_flag=True, help="Reset port protocol configuration")
    @click.option(
        "--reset-temperature-mode",
        is_flag=True,
        help="Reset temperature control mode to power priority",
    )
    @click.option(
        "--reset-cable-compensation",
        is_flag=True,
        help="Reset cable compensation to default",
    )
    @click.option("--keep-all-nvs", is_flag=True, help="Keep all NVS namespaces (shortcut)")
    @ensure_connected
    def reset_device(ctx, **kwargs):
        """Reset device with configurable options for NVS preservation and reset operations."""
        # Create options object from CLI arguments
        options = ResetDeviceOptions.from_kwargs(**kwargs)

        # Show what will be preserved/reset
        options.display_summary()

        # Confirm destructive operation
        if not click.confirm("Are you sure you want to reset the device?"):
            click.secho("Reset cancelled", fg="yellow")
            return

        req = command_pb2.CommandRequest()
        options.to_protobuf(req)

        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.RESET_DEVICE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Reset device: {response.status}", fg="green")

    @main_group.command()
    @click.argument(
        "feature",
        type=click.Choice(data_types_pb2.ManageFeature.keys(), case_sensitive=False),
    )
    @click.argument(
        "action",
        type=click.Choice(data_types_pb2.ManageAction.keys(), case_sensitive=False),
    )
    @click.argument(
        "enabled", type=click.Choice(["on", "off"], case_sensitive=False), default="on"
    )
    @ensure_connected
    def manage_feature_toggle(ctx, feature: str, action: str, enabled: str):
        """Manage feature toggles (read/update/delete)."""
        req = admin_pb2.AdminCommandRequest()
        req.manage_feature_toggle.action = data_types_pb2.ManageAction.Value(action.upper())
        req.manage_feature_toggle.feature = data_types_pb2.ManageFeature.Value(feature.upper())
        req.manage_feature_toggle.enabled = enabled == "on"
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.MANAGE_FEATURE_TOGGLE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            pb_message = response.admin_pb()
            click.secho(
                f"Manage feature toggle: {action} {feature} -> "
                f"{'on' if pb_message.manage_feature_toggle.enabled else 'off'}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def set_system_time(ctx):
        """Set device system time to current UTC time."""
        ts = int(time.time())
        req = admin_pb2.AdminCommandRequest()
        req.set_system_time.timestamp = ts
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.SET_SYSTEM_TIME,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Set system time to {ts}", fg="green")

    @main_group.command()
    @click.argument(
        "state",
        type=click.Choice(["on", "off"], case_sensitive=False),
        callback=cast_state_to_int,
    )
    @ensure_connected
    def set_device_power_state(ctx, state: int):
        """Set device power state (on/off)."""
        req = command_pb2.CommandRequest()
        req.set_power_state.state = state  # type: ignore
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DEVICE_POWER_STATE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Switch device: {state}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_charging_status(ctx):
        """Get charging status for all ports."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_CHARGING_STATUS,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Get charging status: {bin(response.command_pb().get_charging_status.status)}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def lock_swd(ctx):
        """Lock SWD debug interface."""
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.LOCK_SWD,
            command_pb2.CommandRequest(),
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Lock SWD", fg="green")

    @main_group.command()
    @ensure_connected
    def unlock_swd(ctx):
        """Unlock SWD debug interface."""
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.UNLOCK_SWD,
            command_pb2.CommandRequest(),
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Unlock SWD", fg="green")

    @main_group.command()
    @click.option(
        "--operation",
        type=click.Choice(["update", "request-upload"], case_sensitive=False),
        default="request-upload",
        help="Operation type: update device config or request device to upload config",
    )
    @click.option(
        "--config-file",
        type=click.Path(exists=True, readable=True),
        help="JSON file containing MachineConfig (required for update operation)",
    )
    @ensure_connected
    def set_machine_config(ctx, operation: str, config_file: str):
        """Set machine configuration or request device to upload its config."""
        req = admin_pb2.AdminCommandRequest()

        if operation == "update":
            if not config_file:
                click.secho("Config file is required for update operation", fg="red")
                return

            # Load config from JSON file
            with open(config_file, "r") as f:
                config_data = json.load(f)

            # Set operation
            req.set_machine_config.operation = data_types_pb2.ConfigOperation.CONFIG_UPDATE

            # Parse timestamp
            req.set_machine_config.config.timestamp = int(
                config_data.get("timestamp", time.time())
            )

            # Parse port configs
            if "ports" in config_data:
                for port_data in config_data["ports"]:
                    port_config = req.set_machine_config.config.ports.add()
                    port_config.port_id = port_data["port_id"]
                    port_config.config.enabled = port_data["config"]["enabled"]

                    # Parse features
                    features = port_data["config"].get("features", {})
                    if features:
                        port_config.config.features.enable_tfcp = features.get("tfcp", False)
                        port_config.config.features.enable_pe = features.get("pe", False)
                        port_config.config.features.enable_qc2p0 = features.get("qc2p0", False)
                        port_config.config.features.enable_qc3p0 = features.get("qc3p0", False)
                        port_config.config.features.enable_qc3plus = features.get("qc3plus", False)
                        port_config.config.features.enable_afc = features.get("afc", False)
                        port_config.config.features.enable_fcp = features.get("fcp", False)
                        port_config.config.features.enable_hv_scp = features.get("hv_scp", False)
                        port_config.config.features.enable_lv_scp = features.get("lv_scp", False)
                        port_config.config.features.enable_sfcp = features.get("sfcp", False)
                        port_config.config.features.enable_apple = features.get("apple", False)
                        port_config.config.features.enable_samsung = features.get("samsung", False)
                        port_config.config.features.enable_ufcs = features.get("ufcs", False)
                        port_config.config.features.enable_pd = features.get("pd", False)
        else:
            # Request upload operation
            req.set_machine_config.operation = data_types_pb2.ConfigOperation.REQUEST_UPLOAD

        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.SET_MACHINE_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Machine config {operation} successful", fg="green")
