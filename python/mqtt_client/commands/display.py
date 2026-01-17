"""Display management commands for MQTT Client CLI."""

import asyncclick as click

import command_pb2

from data_types.service import ServiceCommand
from ..core import send_and_receive
from ..decorators import ensure_connected

# Display mode mappings
MODE_NAME_TO_VALUE = {"off": 0, "manual": 1, "power_meter": 2}
MODE_VALUE_TO_NAME = {v: k for k, v in MODE_NAME_TO_VALUE.items()}

# Display flip mappings
FLIP_NAME_TO_VALUE = {"normal": 0, "flip": 1}
FLIP_VALUE_TO_NAME = {v: k for k, v in FLIP_NAME_TO_VALUE.items()}

# Display rotation mappings
ROTATION_NAME_TO_VALUE = {"0": 0, "90": 1, "180": 2, "270": 3}
ROTATION_VALUE_TO_NAME = {v: k for k, v in ROTATION_NAME_TO_VALUE.items()}

# Idle animation mappings
IDLE_NAME_TO_VALUE = {"none": 0, "meteor": 1, "gol": 2}
IDLE_VALUE_TO_NAME = {v: k for k, v in IDLE_NAME_TO_VALUE.items()}


def register(main_group):
    """Register display management commands with main CLI group."""

    @main_group.command()
    @click.argument("intensity", type=int, default=128)
    @ensure_connected
    def set_display_intensity(ctx, intensity: int):
        """Set display intensity (brightness)."""
        req = command_pb2.CommandRequest()
        req.set_display_intensity.intensity = intensity
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_INTENSITY,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response:
            click.secho(f"Set display intensity to {intensity}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_display_intensity(ctx):
        """Get current display intensity."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DISPLAY_INTENSITY,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Display intensity: {response.command_pb().get_display_intensity.intensity}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def get_display_mode(ctx):
        """Get current display mode."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DISPLAY_MODE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            mode = response.command_pb().get_display_mode.mode
            click.secho(f"Display mode: {mode}({MODE_VALUE_TO_NAME.get(mode)})", fg="green")

    @main_group.command()
    @click.argument(
        "mode",
        type=click.Choice(["off", "manual", "power_meter"], case_sensitive=False),
        default="power_meter",
    )
    @ensure_connected
    def set_display_mode(ctx, mode: str):
        """Set display mode (off/manual/power_meter)."""
        req = command_pb2.CommandRequest()
        req.set_display_mode.mode = MODE_NAME_TO_VALUE[mode]  # type: ignore
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_MODE,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Set display mode to {mode}", fg="green")

    @main_group.command()
    @click.argument(
        "flip",
        type=click.Choice(["normal", "flip"], case_sensitive=False),
        default="normal",
    )
    @ensure_connected
    def set_display_flip(ctx, flip: str):
        """Set display flip state (normal/flip)."""
        req = command_pb2.CommandRequest()
        req.set_display_flip.flip = FLIP_NAME_TO_VALUE[flip]  # type: ignore
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_FLIP,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Set display flip to {flip}", fg="green")

    @main_group.command()
    @ensure_connected
    def get_display_flip(ctx):
        """Get current display flip state."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DISPLAY_FLIP,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            flip = response.command_pb().get_display_flip.flip
            click.secho(
                f"Get display flip: {FLIP_VALUE_TO_NAME[flip]}",
                fg="green",
            )

    @main_group.command()
    @click.option(
        "--intensity", "-i", type=int, default=50, help="Display intensity (0-100)"
    )
    @click.option(
        "--mode",
        "-m",
        type=click.Choice(["off", "manual", "power_meter"], case_sensitive=False),
        default="power_meter",
        help="Display mode",
    )
    @click.option(
        "--flip",
        "-f",
        type=click.Choice(["normal", "flip"], case_sensitive=False),
        default="normal",
        help="Display flip state",
    )
    @ensure_connected
    def set_display_config(ctx, intensity: int, mode: str, flip: str):
        """Set display configuration (intensity, mode, flip) in one command."""
        req = command_pb2.CommandRequest()
        req.set_display_config.config.intensity = intensity
        req.set_display_config.config.mode = MODE_NAME_TO_VALUE[mode]
        req.set_display_config.config.flip = FLIP_NAME_TO_VALUE[flip]

        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_CONFIG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Set display config: intensity={intensity}, mode={mode}, flip={flip}",
                fg="green",
            )

    @main_group.command()
    @ensure_connected
    def get_display_rotation(ctx):
        """Get current display rotation."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DISPLAY_ROTATION,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            rotation = response.command_pb().get_display_rotation.rotation
            click.secho(
                f"Display rotation: {rotation}({ROTATION_VALUE_TO_NAME.get(rotation)})",
                fg="green",
            )

    @main_group.command()
    @click.argument(
        "rotation",
        type=click.Choice(["0", "90", "180", "270"], case_sensitive=False),
        default="0",
    )
    @ensure_connected
    def set_display_rotation(ctx, rotation: str):
        """Set display rotation (0/90/180/270 degrees)."""
        req = command_pb2.CommandRequest()
        req.set_display_rotation.rotation = ROTATION_NAME_TO_VALUE[rotation]  # type: ignore
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_ROTATION,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(f"Set display rotation to {rotation}", fg="green")

    @main_group.command()
    @ensure_connected
    def clear_factory_dm_flag(ctx):
        """Clear the factory Data Matrix flag (one-way factory to production transition)."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.CLEAR_FACTORY_DM_FLAG,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho("Factory DM flag cleared successfully", fg="green")
        else:
            click.secho("Failed to clear factory DM flag", fg="red")

    @main_group.command()
    @ensure_connected
    def get_idle_animation(ctx):
        """Get current idle animation setting."""
        req = command_pb2.CommandRequest()
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.GET_DISPLAY_SETUP,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            setup = response.command_pb().get_display_setup.setup
            click.secho(
                f"Idle Animation: {setup.idle_animation}({IDLE_VALUE_TO_NAME.get(setup.idle_animation)})",
                fg="green",
            )

    @main_group.command("set-display-setup")
    @click.option(
        "--intensity", "-i", type=int, required=True, help="Display intensity (0-100)"
    )
    @click.option(
        "--rotation",
        "-r",
        type=click.Choice(["0", "90", "180", "270"]),
        required=True,
    )
    @click.option(
        "--idle-animation",
        "-a",
        type=click.Choice(["none", "meteor", "gol"], case_sensitive=False),
        required=True,
        help="Idle animation type",
    )
    @ensure_connected
    def set_display_setup(
        ctx, intensity: int, rotation: str, idle_animation: str
    ):
        """Set complete display setup (intensity, rotation, idle animation)."""
        req = command_pb2.CommandRequest()
        req.set_display_setup.setup.intensity = intensity
        req.set_display_setup.setup.rotation = ROTATION_NAME_TO_VALUE[rotation]
        req.set_display_setup.setup.idle_animation = IDLE_NAME_TO_VALUE[idle_animation]
        response = send_and_receive(
            ctx.obj["psn"],
            ServiceCommand.SET_DISPLAY_SETUP,
            req,
            timeout=ctx.obj["timeout"],
        )
        if response and response.status == 0:
            click.secho(
                f"Set display setup: intensity={intensity}, rotation={rotation}, "
                f"idle_animation={idle_animation}",
                fg="green",
            )
