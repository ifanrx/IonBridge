#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""MQTT Client CLI - Main command group entry point.

This module defines the main Click command group and registers all
domain-specific command modules.
"""

import logging

import asyncclick as click

from data_types.port import PORTS

from .decorators import validate_psn
from .commands import register_all

logger = logging.getLogger(__name__)


def setup_logger(verbose: bool):
    """Configure logging based on verbosity setting."""
    handler = logging.StreamHandler()
    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    if verbose:
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)


@click.group()
@click.option("--psn", type=str, callback=validate_psn)
@click.option("--broker", type=str, default="localhost")
@click.option("--port", type=int, default=1883)
@click.option(
    "--product-family",
    type=click.Choice(["CP02", "PA768S"]),
    default="CP02",
    help="Product family (CP02=5 ports, PA768S=3 ports)",
)
@click.option("--ca_certs", type=str, default="certs/ca.crt")
@click.option("--certfile", type=str, default="certs/client.pem")
@click.option("--keyfile", type=str, default="certs/client.key")
@click.option("--timeout", type=int, default=15)
@click.option(
    "-u", "--unsubscribe_telemetry", is_flag=True, help="Unsubscribe telemetry"
)
@click.option("-v", "--verbose", is_flag=True, help="Enable verbose logging")
@click.pass_context
async def main(
    ctx,
    psn: str,
    broker: str,
    port: int,
    product_family: str,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    timeout: int,
    unsubscribe_telemetry: bool,
    verbose: bool,
):
    """MQTT Client CLI for device communication and management."""
    setup_logger(verbose)

    # Set product family for dynamic PORTS configuration
    PORTS.set_family(product_family)

    ctx.ensure_object(dict)
    ctx.obj["broker"] = broker
    ctx.obj["port"] = port
    ctx.obj["product_family"] = product_family
    ctx.obj["ca_certs"] = ca_certs
    ctx.obj["certfile"] = certfile
    ctx.obj["keyfile"] = keyfile
    ctx.obj["psn"] = psn
    ctx.obj["timeout"] = timeout
    ctx.obj["verbose"] = verbose

    ctx.obj["topic"] = {
        "enduser": f"device/{psn}/enduser/response/+",
        "admin": f"device/{psn}/admin/response/+",
        "telemetry": f"device/{psn}/telemetry/+",
    }
    if unsubscribe_telemetry:
        ctx.obj["topic"].pop("telemetry", None)


# Register all command modules with the main group
register_all(main)
