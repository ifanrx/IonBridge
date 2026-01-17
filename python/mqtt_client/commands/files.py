"""File operations commands for MQTT Client CLI."""

import pathlib

import asyncclick as click

import admin_pb2
import command_pb2
import data_types_pb2

from data_types.service import ServiceCommand, AdminCommand
from ..core import send_and_receive, unsubscribe
from ..decorators import ensure_connected


def register(main_group):
    """Register file operation commands with main CLI group."""

    @main_group.command()
    @click.argument("local_path", type=click.Path(exists=True))
    @click.argument("remote_path", default="")
    @ensure_connected
    def upload_resources(ctx, local_path: str, remote_path: str):
        """Upload a file or directory to device LittleFS.

        LOCAL_PATH: Local file or directory to upload
        REMOTE_PATH: Remote path (relative to /littlefs, default: same as local basename)

        Examples:
            upload-resources ./resources              # Upload to /littlefs/resources
            upload-resources ./boot.bin images/boot   # Upload to /littlefs/images/boot.bin
        """
        import os
        import time

        # Unsubscribe from telemetry to reduce noise during upload
        if "telemetry" in ctx.obj["topic"]:
            unsubscribe(ctx.obj["topic"]["telemetry"])
            click.secho("Unsubscribed from telemetry during upload", fg="yellow")

        # Turn off display to stop animation and prevent file access during upload
        click.echo("Turning off display... ", nl=False)
        req = command_pb2.CommandRequest()
        req.set_display_config.config.mode = data_types_pb2.DisplayMode.OFF
        req.set_display_config.config.intensity = 0
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("OK", fg="green")
        else:
            click.secho("FAILED (continuing anyway)", fg="yellow")

        # Turn off BLE to free up resources before uploading
        click.echo("Turning off BLE... ", nl=False)
        req = command_pb2.CommandRequest()
        req.set_ble_state.enable = False
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_BLE_STATE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("OK", fg="green")
        else:
            click.secho("FAILED (continuing anyway)", fg="yellow")

        local = pathlib.Path(local_path)
        base_remote = remote_path if remote_path else local.name

        # Auto-delete existing resource before upload
        click.echo(f"Removing existing /littlefs/{base_remote}... ", nl=False)
        req = admin_pb2.AdminCommandRequest()
        req.delete_file.path = base_remote
        req.delete_file.recursive = local.is_dir()  # Only recursive for directories
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.DELETE_FILE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.admin_pb().delete_file.success:
            click.secho("OK", fg="green")
        else:
            click.secho("SKIPPED (folder may not exist)", fg="yellow")

        chunk_size = 512  # Must match pb.WriteFileReq.data max_size in admin.options

        def write_file(remote_file_path: str, data: bytes) -> bool:
            """Write file in chunks"""
            for offset in range(0, len(data), chunk_size):
                chunk = data[offset : offset + chunk_size]
                req = admin_pb2.AdminCommandRequest()
                req.write_file.path = remote_file_path
                req.write_file.offset = offset
                req.write_file.data = chunk
                response = send_and_receive(
                    ctx.obj["psn"],
                    AdminCommand.WRITE_FILE,
                    req,
                    timeout=ctx.obj["timeout"],
                )
                if not response or not response.admin_pb().write_file.success:
                    return False
            return True

        files_uploaded = 0
        bytes_uploaded = 0

        if local.is_file():
            # Single file upload
            dest = remote_path if remote_path else local.name
            click.echo(f"Uploading {local} -> /littlefs/{dest}")
            data = local.read_bytes()
            if write_file(dest, data):
                files_uploaded += 1
                bytes_uploaded += len(data)
                click.secho(f"OK ({len(data)} bytes)", fg="green")
            else:
                click.secho("FAILED", fg="red")
                return
        else:
            # Directory upload
            for root, dirs, files in os.walk(local):
                rel_root = pathlib.Path(root).relative_to(local)
                for f in files:
                    local_file = pathlib.Path(root) / f
                    remote_file = (
                        f"{base_remote}/{rel_root}/{f}"
                        if str(rel_root) != "."
                        else f"{base_remote}/{f}"
                    )
                    click.echo(
                        f"Uploading {local_file} -> /littlefs/{remote_file} ... ",
                        nl=False,
                    )
                    data = local_file.read_bytes()
                    if write_file(remote_file, data):
                        files_uploaded += 1
                        bytes_uploaded += len(data)
                        click.secho(f"OK ({len(data)} bytes)", fg="green")
                    else:
                        click.secho("FAILED", fg="red")

        click.secho(
            f"\nUploaded {files_uploaded} files, {bytes_uploaded} bytes total", fg="cyan"
        )

        # Restore display intensity
        click.echo("Restoring display... ", nl=False)
        req = command_pb2.CommandRequest()
        req.set_display_intensity.intensity = 100
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_INTENSITY,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("OK", fg="green")
        else:
            click.secho("FAILED", fg="yellow")

        # Wait for cloud sync before reboot
        click.echo("Waiting 3s for cloud sync... ", nl=False)
        time.sleep(3)
        click.secho("OK", fg="green")

        # Reboot device to apply new resources
        click.echo("Rebooting device... ", nl=False)
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.REBOOT_DEVICE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho("OK", fg="green")
        else:
            click.secho("FAILED", fg="yellow")

    @main_group.command()
    @click.argument("remote_path")
    @ensure_connected
    def list_dir(ctx, remote_path: str):
        """List directory contents on device LittleFS.

        REMOTE_PATH: Remote path (relative to /littlefs)
        """
        req = admin_pb2.AdminCommandRequest()
        req.list_dir.path = remote_path
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.LIST_DIR,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            resp = response.admin_pb().list_dir
            if resp.success:
                click.echo(f"Contents of /littlefs/{remote_path}:")
                for entry in resp.entries:
                    click.echo(f"  {entry}")
            else:
                click.secho(f"Failed to list directory", fg="red")

    @main_group.command()
    @click.argument("remote_path")
    @click.option(
        "--recursive", "-r", is_flag=True, help="Recursively delete folder contents"
    )
    @ensure_connected
    def delete_file(ctx, remote_path: str, recursive: bool):
        """Delete a file or directory on device LittleFS.

        REMOTE_PATH: Remote path (relative to /littlefs)

        Use --recursive to delete non-empty directories.
        """
        req = admin_pb2.AdminCommandRequest()
        req.delete_file.path = remote_path
        req.delete_file.recursive = recursive
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.DELETE_FILE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            if response.admin_pb().delete_file.success:
                click.secho(f"Deleted /littlefs/{remote_path}", fg="green")
            else:
                click.secho(f"Failed to delete", fg="red")

    @main_group.command()
    @click.argument("remote_path")
    @ensure_connected
    def remove_folder(ctx, remote_path: str):
        """Remove a folder and all its contents on device LittleFS.

        REMOTE_PATH: Remote folder path (relative to /littlefs)

        This is equivalent to: delete-file --recursive REMOTE_PATH
        """
        req = admin_pb2.AdminCommandRequest()
        req.delete_file.path = remote_path
        req.delete_file.recursive = True
        response = send_and_receive(
            ctx.obj["psn"],
            AdminCommand.DELETE_FILE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            if response.admin_pb().delete_file.success:
                click.secho(f"Removed folder /littlefs/{remote_path}", fg="green")
            else:
                click.secho(f"Failed to remove folder", fg="red")
