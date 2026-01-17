#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Message transport layer for MQTT communication."""

import logging
import queue
import threading
import time
import typing

import asyncclick as click

import admin_pb2
import command_pb2
from data_types.service import AdminCommand, ServiceCommand, TelemetryCommand, Topic

from .messaging import Message, ResponseMessage

logger = logging.getLogger(__name__)

# Global queues
request_queue = queue.Queue()
response_queue = queue.Queue()
ignore_push_msg: bool = True


def handle_push_message(message: ResponseMessage) -> bool:
    """Handle push messages (telemetry)."""
    if isinstance(message.command, TelemetryCommand):
        match message.command:
            case TelemetryCommand.DEVICE_STATS_DATA:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.DEVICE_BOOT_INFO:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.PD_CHARGING_ALERT:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.POWER_HISTORICAL_DATA:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.TASK_EXECUTION_RESULT:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.CONFIG_REPORT:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.CONFIG_REQUEST:
                click.secho(message.to_json(), fg="green")
                return True
            case TelemetryCommand.PD_PCAP_DATA:
                return False
            case _:
                return ignore_push_msg
    return False


def send_message(topic: str, message: Message):
    """Queue a message for sending."""
    request_queue.put((topic, message))


def process_requests():
    """Process outgoing requests (runs in background thread)."""
    from .connection import client

    while True:
        try:
            topic, message = request_queue.get(timeout=0.1)
            logger.debug("Sending message, topic=%s message=%s", topic, message)
            client.publish(topic, payload=message.serialize())
            request_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error processing request: {e}")


def start_request_processor():
    """Start the request processor thread."""
    thread = threading.Thread(target=process_requests, daemon=True)
    thread.start()
    return thread


def wait_for_response(
    request: typing.Optional[Message] = None,
    cmd: typing.Optional[
        typing.Union[ServiceCommand, AdminCommand, TelemetryCommand]
    ] = None,
    timeout=5,
    return_all=False,
) -> typing.Optional[ResponseMessage]:
    """Wait for a specific response with timeout."""
    if return_all:
        try:
            return response_queue.get(block=True, timeout=0.1)
        except queue.Empty:
            return None

    if not (request or cmd):
        raise ValueError("Either request or cmd must be provided")
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            msg: ResponseMessage = response_queue.get(block=True, timeout=0.1)
            if cmd and type(msg.command) is type(cmd) and msg.command == cmd:
                return msg
            if (
                request
                and msg.msg_id == request.msg_id
                and type(msg.command) is type(request.command)
                and msg.command == request.command
            ):
                return msg
            # Put back messages for different commands
            response_queue.put(msg)
        except queue.Empty:
            continue
    return None


def send_and_receive(
    psn: str,
    cmd: typing.Union[ServiceCommand, AdminCommand],
    request: typing.Union[command_pb2.CommandRequest, admin_pb2.AdminCommandRequest],
    timeout=5,
) -> typing.Optional[ResponseMessage]:
    """Send a message and wait for response."""
    message = Message(cmd, pb_message=request)
    topic_obj = Topic(psn, cmd, is_request=True)
    send_message(topic_obj.to_topic(), message)
    response = wait_for_response(message, timeout=timeout)
    if not response:
        click.secho(
            f"Receive timeout for command: 0x{cmd:04X} after {timeout} seconds",
            fg="red",
        )
    return response
