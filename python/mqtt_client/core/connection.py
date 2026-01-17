#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""MQTT client connection management."""

import logging
import random
import time
import typing

import asyncclick as click
import paho.mqtt.client as mqtt
import paho.mqtt.enums

from .messaging import ResponseMessage
from .transport import response_queue, handle_push_message

logger = logging.getLogger(__name__)

# Global state
subscribe_topic: typing.Any = []
mqtt_initialized: bool = False
in_interactive_shell: bool = False

# Create MQTT client
client = mqtt.Client(
    paho.mqtt.enums.CallbackAPIVersion.VERSION2,
    f"python-mqtt-{random.randint(0, 1000)}",
    protocol=mqtt.MQTTv5,
)


def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected with result code " + str(rc))
    subscribe(subscribe_topic)


def on_subscribe(client, userdata, mid, reasonCodes, properties):
    print(f"Subscribed: {mid}")


def on_message(client, userdata, msg):
    try:
        message = ResponseMessage.deserialize(msg.topic, msg.payload)
        logger.debug(
            "Received message: topic=%s cmd=%s message=%s",
            msg.topic,
            message.command,
            message,
        )
        if handle_push_message(message):
            return
        response_queue.put(message)
    except Exception:
        import traceback

        traceback.print_exc()
        print(msg.payload)


# Set callbacks
client.on_connect = on_connect
client.on_subscribe = on_subscribe
client.on_message = on_message


def connect(host, port=1883):
    client.connect(host, port, 60)
    client.loop_start()
    while not client.is_connected():
        time.sleep(0.1)


def disconnect():
    client.loop_stop()
    client.disconnect()


def subscribe(topic):
    client.subscribe(topic)


def unsubscribe(topic):
    client.unsubscribe(topic)


def init_mqtt(
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    topic: typing.Any,
):
    global mqtt_initialized, subscribe_topic
    if mqtt_initialized:
        return
    mqtt_initialized = True
    subscribe_topic = topic

    def is_ip_address(host):
        try:
            parts = host.split(".")
            if len(parts) != 4:
                return False
            return all(0 <= int(p) < 256 for p in parts)
        except (ValueError, AttributeError):
            return False

    if broker not in ("localhost", "127.0.0.1") and not is_ip_address(broker):
        client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)

    connect(broker, port)
    click.secho(f"Connectted to MQTT broker: {broker}:{port}", fg="green")

    from .transport import start_request_processor
    start_request_processor()


def is_initialized() -> bool:
    return mqtt_initialized


def set_interactive_shell(value: bool):
    global in_interactive_shell
    in_interactive_shell = value


def is_interactive_shell() -> bool:
    return in_interactive_shell
