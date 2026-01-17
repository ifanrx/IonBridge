#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""CLI decorators and utilities."""

import contextlib
import functools
import inspect
import re

import asyncclick as click
import utils

from .core import (
    init_mqtt,
    is_initialized,
    is_interactive_shell,
    disconnect,
)


def validate_psn(ctx, param, value):
    """Validate PSN format."""
    if value is None:
        return value
    if not re.match(r"^\d{16}$", value):
        raise click.BadParameter("PSN must be a 16-digit string.")
    return value


def cast_to_bool(ctx, param, value) -> bool:
    """Convert value to boolean."""
    try:
        return utils.eval_true(value)
    except (ValueError, TypeError):
        raise click.BadParameter("Must be a boolean")


def cast_state_to_int(ctx, param, value) -> int:
    """Convert on/off to 1/0."""
    return 1 if value.lower() == "on" else 0


@contextlib.contextmanager
def connect_mqtt(
    broker,
    port,
    ca_certs,
    certfile,
    keyfile,
    subscribe_topic,
    *args,
    **kwargs,
):
    """Context manager for MQTT connection."""
    if not is_initialized():
        init_mqtt(broker, port, ca_certs, certfile, keyfile, subscribe_topic)
    try:
        yield
    finally:
        if not is_interactive_shell():
            click.secho("Disconnecting...", fg="yellow")
            disconnect()


def ensure_connected(func):
    """Decorator to ensure MQTT connection before executing a command."""

    @click.pass_context
    @functools.wraps(func)
    async def wrapper(ctx, *args, **kwargs):
        # Check if PSN is provided
        if ctx.obj.get("psn") is None:
            raise click.ClickException(
                "PSN is required. Please provide it with --psn option."
            )

        subscribe_topic = [(topic, 0) for topic in ctx.obj["topic"].values()]

        with connect_mqtt(
            ctx.obj["broker"],
            ctx.obj["port"],
            ctx.obj["ca_certs"],
            ctx.obj["certfile"],
            ctx.obj["keyfile"],
            subscribe_topic,
        ):
            if inspect.iscoroutinefunction(func):
                return await func(ctx, *args, **kwargs)
            return func(ctx, *args, **kwargs)

    return wrapper
