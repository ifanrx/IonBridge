#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Interactive shell for MQTT client."""

import readline
import shlex
import sys
import typing

import anyio
import asyncclick as click

from .core import disconnect, set_interactive_shell


async def wrapper_run(
    func: typing.Callable[..., typing.Awaitable],
    args: typing.List[typing.Any],
    kwargs: typing.Dict[str, typing.Any],
):
    return await func(*args, **kwargs)


def completer(text, state):
    from .cli import main
    commands = main.list_commands(ctx=None)
    matches = [cmd for cmd in commands if cmd.startswith(text)]
    if state < len(matches):
        return matches[state]
    return None


def interactive_shell():
    """Run the interactive shell."""
    from .cli import main

    set_interactive_shell(True)
    readline.parse_and_bind("tab: complete")
    readline.set_completer(completer)
    delims = readline.get_completer_delims()
    readline.set_completer_delims(delims.replace("-", ""))

    click.echo("Enter Click commands (type 'exit' or Ctrl+C or Ctrl+D to quit)")
    sys_args = sys.argv[1:]
    while True:
        try:
            line = input("> ").strip()
            if line in ("exit", "quit"):
                break
            if line:
                cli_args = [*sys_args]
                cli_args.extend(shlex.split(line))
                anyio.run(
                    wrapper_run, main.main, [cli_args], {"standalone_mode": False}
                )
        except (KeyboardInterrupt, EOFError):
            click.echo("\nExiting.")
            break
        except Exception as e:
            import traceback

            traceback.print_exc()
            click.echo(f"Error: {e}", file=sys.stderr)
            break
    click.secho("Disconnecting...", fg="yellow")
    disconnect()
