"""Output formatting utilities for MQTT Client CLI."""

import asyncclick as click


def info(msg: str) -> None:
    """Print info message in green."""
    click.secho(msg, fg="green")


def error(msg: str) -> None:
    """Print error message in red."""
    click.secho(msg, fg="red")


def warn(msg: str) -> None:
    """Print warning message in yellow."""
    click.secho(msg, fg="yellow")


def format_mac(raw: bytes) -> str:
    """Convert raw bytes to MAC address string format."""
    return ":".join(f"{b:02x}" for b in raw)
