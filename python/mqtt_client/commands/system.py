"""System and OTA commands for MQTT Client CLI."""

import asyncclick as click

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive
from ..decorators import ensure_connected


def register(main_group):
    """Register system commands with main CLI group."""

    @main_group.command()
    @ensure_connected
    def get_esp32_version(ctx):
        """Get ESP32 firmware version."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_AP_VERSION,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            version = response.command_pb().get_ap_version.version
            click.secho(
                f"Get ESP32 version: {version.major}.{version.minor}.{version.patch}",
                fg="green",
            )

    @main_group.command()
    @click.argument("esp32_new_version", type=str)
    @click.argument("sw3566_new_version", type=str)
    @click.argument("fpga_new_version", type=str)
    @click.option("--force", is_flag=True)
    @ensure_connected
    def start_ota(
        ctx,
        esp32_new_version: str,
        sw3566_new_version: str,
        fpga_new_version: str,
        force: bool,
    ):
        """Start OTA firmware update."""
        esp32_major, esp32_minor, esp32_patch = esp32_new_version.split(".")
        sw3566_major, sw3566_minor, sw3566_patch = sw3566_new_version.split(".")
        fpga_major, fpga_minor, fpga_patch = fpga_new_version.split(".")
        req = command_pb2.CommandRequest()
        req.start_ota.sw3566_version.CopyFrom(
            data_types_pb2.Version(
                major=int(sw3566_major), minor=int(sw3566_minor), patch=int(sw3566_patch)
            )
        )
        req.start_ota.fpga_version.CopyFrom(
            data_types_pb2.Version(
                major=int(fpga_major), minor=int(fpga_minor), patch=int(fpga_patch)
            )
        )
        req.start_ota.esp32_version.CopyFrom(
            data_types_pb2.Version(
                major=int(esp32_major), minor=int(esp32_minor), patch=int(esp32_patch)
            )
        )
        req.start_ota.force = force
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.START_OTA,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Start OTA", fg="green")

    @main_group.command()
    @click.argument("programmer_new_version", type=str)
    @click.argument("product_family", type=str, default="")
    @ensure_connected
    def start_programmer(
        ctx,
        programmer_new_version: str,
        product_family: str,
    ):
        """Start programmer mode for firmware flashing."""
        programmer_major, programmer_minor, programmer_patch = programmer_new_version.split(
            "."
        )
        req = command_pb2.CommandRequest()
        req.start_programmer.version.CopyFrom(
            data_types_pb2.Version(
                major=int(programmer_major),
                minor=int(programmer_minor),
                patch=int(programmer_patch),
            )
        )
        req.start_programmer.product_family = product_family
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.START_PROGRAMMER,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Start Programmer", fg="green")

    @main_group.command()
    @click.argument("sw3566_new_version", type=str)
    @click.argument("fpga_new_version", type=str)
    @ensure_connected
    def download_firmware(
        ctx,
        sw3566_new_version: str,
        fpga_new_version: str,
    ):
        """Download firmware files for OTA update."""
        sw3566_major, sw3566_minor, sw3566_patch = sw3566_new_version.split(".")
        fpga_major, fpga_minor, fpga_patch = fpga_new_version.split(".")
        req = admin_pb2.AdminCommandRequest()
        req.download_firmware.sw3566_version.major = int(sw3566_major)
        req.download_firmware.sw3566_version.minor = int(sw3566_minor)
        req.download_firmware.sw3566_version.patch = int(sw3566_patch)
        req.download_firmware.fpga_version.major = int(fpga_major)
        req.download_firmware.fpga_version.minor = int(fpga_minor)
        req.download_firmware.fpga_version.patch = int(fpga_patch)
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.DOWNLOAD_FIRMWARE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("Download firmware", fg="green")

    @main_group.command()
    @ensure_connected
    def reboot_device(ctx):
        """Reboot the device."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.REBOOT_DEVICE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("reboot device", fg="green")
