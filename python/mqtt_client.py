# -*- coding=utf-8 -*-

import ctypes
import dataclasses
import enum
import json
import queue
import os
import re
import struct
import threading
import time
import typing

import asyncio
import asyncclick as click
import paho.mqtt.client as mqtt
import utils

from data_types.port import (
    ClientPDStatus,
    FastChargingProtocol,
    PortConfig,
    PORTS,
    PortConfigInternal,
    PortType,
)
from data_types.service import SERVICE
from data_types.wifi import WiFiAuthMode, WiFiStateType
from usb_pd import pcap

msg_id = 0

# Create queues for requests and responses
request_queue = queue.Queue()
response_queue = queue.Queue()
ignore_push_msg: bool = True


class ManageFeature(enum.IntEnum):
    FPGA_POWER_CONTROL = 0


class ManageAction(enum.IntEnum):
    GET = 0
    SET = 1
    RESET = 2


@dataclasses.dataclass
class Message:
    command: int
    payload: bytes | bytearray = b""
    msg_id: int = 0

    def __post_init__(self):
        global msg_id
        self.msg_id = msg_id
        msg_id += 1
        msg_id &= 0xFFFF

    def serialize(self) -> bytes | bytearray:
        length = len(self.payload)
        data = bytearray(3 + length)
        struct.pack_into("<BH", data, 0, self.command, self.msg_id)
        if length:
            struct.pack_into(f"<{length}s", data, 3, self.payload)
        return data


@dataclasses.dataclass
class TelemetryMessage:
    command: int
    msg_id: int
    status: int
    payload: bytes = b""

    @classmethod
    def deserialize(cls, data: bytes):
        # Unpack the fixed part of the message (command, msg_id, status)
        command, msg_id, status = struct.unpack_from("<HHB", data, 0)
        if command > 0xFF:
            status = 0
            payload = data[4:]
        else:
            # The remaining part of the data is the payload
            payload = data[5:]
        return cls(command, msg_id, status, payload)

    def __str__(self):
        return f"Command: {SERVICE(self.command).name}, Msg ID: {self.msg_id}, Status: {self.status}, Payload: {self.payload.hex()}"


class PowerStatistics(ctypes.LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("protocol", ctypes.c_uint8),
        ("current", ctypes.c_uint8),
        ("voltage", ctypes.c_uint8),
        ("temperature", ctypes.c_uint8),
        ("last_full_cap", ctypes.c_uint16),
        ("present_cap", ctypes.c_uint16),
    ]

    def __str__(self) -> str:
        protocol = FastChargingProtocol(self.protocol).name
        current = self.current / 32.0
        voltage = self.voltage / 8.0
        last_full_cap = self.last_full_cap * 100
        present_cap = self.present_cap * 100
        return f"Protocol: {protocol}, Current: {current}A, Voltage: {voltage}V, Temperature: {self.temperature}C, Last full cap: {last_full_cap}mWh, Present cap: {present_cap}mWh"

    __repr__ = __str__


AllPowerStatistics = PowerStatistics * 5


def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected with result code " + str(rc))


def on_subscribe(client, userdata, mid, reasonCodes, properties):
    print(f"Subscribed: {mid}")


def on_message(client, userdata, msg):
    try:
        message = TelemetryMessage.deserialize(msg.payload)
        if handle_push_message(message):
            return
        response_queue.put(message)
    except Exception:
        print(msg.payload)


def handle_push_message(message) -> bool:
    if ignore_push_msg:
        return False
    return True


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
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


def validate_psn(ctx, param, value):
    if not re.match(r"^\d{16}$", value):
        raise click.BadParameter("PSN must be a 16-digit string.")
    return value


def send_message(topic: str, message: Message):
    request_queue.put((topic, message))


# Function to process outgoing requests
def process_requests():
    while True:
        try:
            topic, message = request_queue.get(timeout=0.1)
            client.publish(topic, payload=message.serialize())
            request_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error processing request: {e}")


# Start request processor thread
def start_request_processor():
    thread = threading.Thread(target=process_requests, daemon=True)
    thread.start()
    return thread


# Wait for a specific response with timeout
def wait_for_response(
    request: typing.Optional[Message] = None,
    cmd: typing.Optional[SERVICE] = None,
    timeout=5,
    return_all=False,
) -> typing.Optional[TelemetryMessage]:
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
            msg: TelemetryMessage = response_queue.get(block=True, timeout=0.1)
            if cmd and msg.command == cmd:
                return msg
            if (
                request
                and msg.msg_id == request.msg_id
                and msg.command == request.command
            ):
                return msg
            # Put back messages for different commands
            response_queue.put(msg)
        except queue.Empty:
            continue
    return None


def send_and_receive(
    topic: str,
    cmd: SERVICE,
    payload: bytes | bytearray = bytes(),
    timeout=5,
) -> typing.Optional[TelemetryMessage]:
    message = Message(cmd, payload)
    send_message(topic, message)
    response = wait_for_response(message, timeout=timeout)
    if not response:
        click.secho(
            f"Receive timeout for command: 0x{cmd:04X} after {timeout} seconds",
            fg="red",
        )
    return response


def cast_to_bool(ctx, params, value) -> bool:
    try:
        return utils.eval_true(value)
    except (ValueError, TypeError):
        raise click.BadParameter("Must be a boolean")
    return value


def cast_state_to_int(ctx, params, value) -> int:
    return 1 if value.lower() == "on" else 0


@click.group()
@click.option("--psn", type=str, callback=validate_psn)
@click.option("--broker", type=str, default="localhost")
@click.option("--port", type=int, default=1883)
@click.option("--ca_certs", type=str, default="certs/ca.crt")
@click.option("--certfile", type=str, default="certs/client.pem")
@click.option("--keyfile", type=str, default="certs/client.key")
@click.option("--timeout", type=int, default=15)
@click.pass_context
async def main(
    ctx,
    psn: str,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    timeout: int,
):
    ctx.ensure_object(dict)
    ctx.obj["topic"] = {
        "command": f"device/{psn}/command",
        "telemetry": f"device/{psn}/telemetry",
    }
    ctx.obj["timeout"] = timeout

    @ctx.call_on_close
    def disconnect_mqtt():
        click.secho("Disconnecting...", fg="yellow")
        disconnect()

    client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)
    connect(broker, port)
    subscribe(ctx.obj["topic"]["telemetry"])
    click.secho(f"Connectted to MQTT broker: {broker}:{port}", fg="green")
    start_request_processor()


@main.command()
@click.pass_context
def get_esp32_version(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_AP_VERSION,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Get ESP32 version: {response.payload[0]}.{response.payload[1]}.{response.payload[2]}",
            fg="green",
        )


@main.command()
@click.argument("version", type=str)
@click.pass_context
def trigger_esp32_ota(ctx, version: str):
    major, minor, patch = version.split(".")
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.PERFORM_WIFI_OTA,
        payload=bytes([int(major), int(minor), int(patch)]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Trigger ESP32 OTA", fg="green")


@main.command()
@click.argument("hash", type=str)
@click.pass_context
def confirm_esp32_ota(ctx, hash: str):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.CONFIRM_OTA,
        payload=bytes.fromhex(hash),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Confirm ESP32 OTA", fg="green")


@main.command()
@click.pass_context
def get_sw3566_version(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_BP_VERSION,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Get SW3566 version: {response.payload[0]}.{response.payload[1]}.{response.payload[2]}",
            fg="green",
        )


@main.command()
@click.argument("version", type=str)
@click.option("--force", is_flag=True)
@click.pass_context
def upgrade_sw3566(ctx, version: str, force: bool):
    major, minor, patch = version.split(".")
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.UPGRADE_SW3566,
        payload=bytes([int(major), int(minor), int(patch), int(force)]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Upgrade SW3566", fg="green")


@main.command()
@click.pass_context
def get_fpga_version(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_FPGA_VERSION,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Get FPGA version: {response.payload[0]}.{response.payload[1]}.{response.payload[2]}",
            fg="green",
        )


@main.command()
@click.argument("version", type=str)
@click.option("--force", is_flag=True)
@click.pass_context
def upgrade_fpga(ctx, version: str, force: bool):
    major, minor, patch = version.split(".")
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.UPGRADE_FPGA,
        payload=bytes([int(major), int(minor), int(patch), int(force)]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Upgrade FPGA", fg="green")


@main.command()
@click.argument("esp32_new_version", type=str)
@click.argument("sw3566_new_version", type=str)
@click.argument("fpga_new_version", type=str)
@click.option("--force", is_flag=True)
@click.pass_context
def start_ota(
    ctx,
    esp32_new_version: str,
    sw3566_new_version: str,
    fpga_new_version: str,
    force: bool,
):
    esp32_major, esp32_minor, esp32_patch = esp32_new_version.split(".")
    sw3566_major, sw3566_minor, sw3566_patch = sw3566_new_version.split(".")
    fpga_major, fpga_minor, fpga_patch = fpga_new_version.split(".")
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.START_OTA,
        payload=bytes(
            [
                int(sw3566_major),
                int(sw3566_minor),
                int(sw3566_patch),
                int(fpga_major),
                int(fpga_minor),
                int(fpga_patch),
                int(esp32_major),
                int(esp32_minor),
                int(esp32_patch),
                int(force),
            ]
        ),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Start OTA", fg="green")


@main.command()
@click.argument(
    "state",
    type=click.Choice(["on", "off"], case_sensitive=False),
    callback=cast_state_to_int,
)
@click.pass_context
def set_ble_state(ctx, state: int):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_BLE_STATE,
        payload=bytes([state]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Set BLE state to {state}", fg="green")


@main.command()
@click.argument(
    "state",
    type=click.Choice(["on", "off"], case_sensitive=False),
    callback=cast_state_to_int,
)
@click.pass_context
def set_syslog_state(ctx, state: int):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_SYSLOG_STATE,
        payload=bytes([state]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Set syslog state to {state}", fg="green")


@main.command()
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.pass_context
def get_port_pd_status(ctx, port: str):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_PORT_PD_STATUS,
        payload=bytearray([PORTS.index(port)]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        pd_status = ClientPDStatus.from_bytes(response.payload)
        click.secho(f"PD status: {pd_status}", fg="green")


@main.command()
@click.pass_context
def get_device_password(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DEVICE_PASSWORD,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Get password: {response.payload[0]}{response.payload[1]}{response.payload[2]}{response.payload[3]}",
            fg="green",
        )


@main.command()
@click.argument(
    "enabled",
    type=click.Choice(["on", "off"], case_sensitive=False),
    callback=cast_state_to_int,
)
@click.pass_context
def set_power_allocator_enabled(ctx, enabled: int):
    payload = bytearray()
    payload.append(1)
    payload.append(enabled)
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Set power allocator enabled to {bool(enabled)}", fg="green")


@main.command()
@click.pass_context
def get_power_allocator_enabled(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        payload=b"\x00",
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Get power allocator enabled: {bool(response.payload[0])}",
            fg="green",
        )


@main.command()
@click.pass_context
def del_power_allocator_enabled(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        payload=b"\x02",
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Delete power allocator enabled", fg="green")


@main.command()
@click.argument("adc_threshold_low", type=click.IntRange(0, 255))
@click.argument("adc_threshold_high", type=click.IntRange(0, 255))
@click.argument("action_deadzone", type=click.IntRange(0, 255))
@click.pass_context
def set_fpga_config(
    ctx,
    adc_threshold_low: int,
    adc_threshold_high: int,
    action_deadzone: int,
):
    payload = bytearray()
    payload.append(1)
    payload.append(adc_threshold_low)
    payload.append(adc_threshold_high)
    payload.append(action_deadzone)
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_FPGA_CONFIG,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(
            f"Set FPGA config: adc_threshold_low: {adc_threshold_low}, adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}",
            fg="green",
        )


@main.command()
@click.pass_context
def get_fpga_config(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_FPGA_CONFIG,
        payload=b"\x00",
        timeout=ctx.obj["timeout"],
    )
    if response:
        adc_threshold_low = response.payload[0]
        adc_threshold_high = response.payload[1]
        action_deadzone = response.payload[2]
        click.secho(
            f"Get FPGA config: adc_threshold_low: {adc_threshold_low}, adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}",
            fg="green",
        )


@main.command()
@click.pass_context
def del_fpga_config(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_FPGA_CONFIG,
        payload=b"\x02",
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Delete FPGA config", fg="green")


@dataclasses.dataclass
class PowerConfig:
    version: int
    power_budget: int

    device_high_temp_threshold: int
    device_high_temp_min_power: int
    device_low_temp_threshold: int
    device_over_temp_decrement: int
    device_under_temp_increment: int
    device_power_cooldown_time: int

    overprovisioning_threshold: int
    cap_increase_min_threshold: int
    cap_decrease_min_threshold: int
    port_power_cap_increase_threshold: int
    port_power_cap_increase_step: int
    port_power_cap_decrease_threshold: int
    high_power_usage_threshold: int
    low_power_usage_threshold: int
    port_power_adjustment_threshold: int

    @classmethod
    def deserialize(cls, payload: bytes):
        version = payload[0]
        match version:
            case 0:
                return cls(*struct.unpack_from("<17B", payload))
            case 1:
                return cls(*struct.unpack_from("<BH15B", payload))
        raise ValueError(f"Unsupported version: {version}")

    def serialize(self):
        payload = bytearray()
        payload.append(self.version)
        match self.version:
            case 0:
                payload.append(self.power_budget)
            case 1:
                payload.extend(struct.pack("<H", self.power_budget))
        payload.append(self.device_high_temp_threshold)
        payload.append(self.device_high_temp_min_power)
        payload.append(self.device_low_temp_threshold)
        payload.append(self.device_over_temp_decrement)
        payload.append(self.device_under_temp_increment)
        payload.append(self.device_power_cooldown_time)
        payload.append(self.overprovisioning_threshold)
        payload.append(self.cap_increase_min_threshold)
        payload.append(self.cap_decrease_min_threshold)
        payload.append(self.port_power_cap_increase_threshold)
        payload.append(self.port_power_cap_increase_step)
        payload.append(self.port_power_cap_decrease_threshold)
        payload.append(self.high_power_usage_threshold)
        payload.append(self.low_power_usage_threshold)
        payload.append(self.port_power_adjustment_threshold)
        return payload


@main.command()
@click.pass_context
def get_power_config(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_CONFIG,
        payload=b"\x00\x01",
        timeout=ctx.obj["timeout"],
    )
    if response:
        config = PowerConfig.deserialize(response.payload)
        click.secho(
            f"Get power config: {json.dumps(dataclasses.asdict(config), indent=4)}",
            fg="green",
        )


@main.command()
@click.pass_context
@click.argument("items", type=str, nargs=-1)
async def set_power_config(ctx, items: typing.List[str]):
    """Set power config

    ITEMS is a list of key=value pairs, e.g. power_budget=100
    """
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_CONFIG,
        payload=b"\x00\x01",
        timeout=ctx.obj["timeout"],
    )
    if not response:
        click.secho("Failed to get power config", fg="red")
        return

    config = PowerConfig.deserialize(response.payload)
    config_dict = dataclasses.asdict(config)
    for item in items:
        key, value = item.split("=")
        if key not in config_dict:
            raise ValueError(f"Invalid key {key}")
        setattr(config, key, int(value))
    print(
        "Setting power config: ",
        json.dumps(dataclasses.asdict(config), indent=4),
        flush=True,
    )
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_CONFIG,
        payload=b"\x01" + config.serialize(),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Set power config", fg="green")


@main.command()
@click.pass_context
def reset_power_config(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_POWER_CONFIG,
        payload=b"\x02\x01",
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Reset power config", fg="green")


@main.command()
@click.pass_context
def get_all_power_statistics(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"], SERVICE.GET_ALL_POWER_STATISTICS
    )
    if response:
        all_stats = AllPowerStatistics.from_buffer_copy(response.payload)
        for port, stat in enumerate(all_stats):
            click.secho(f"[{port}] {stat}", fg="green")


@main.command()
@click.pass_context
def set_system_time(ctx):
    ts = int(time.time())
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_SYSTEM_TIME,
        payload=struct.pack("<I", ts),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Set system time to {ts}", fg="green")


@main.command()
@click.pass_context
def get_power_supply_status(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_POWER_SUPPLY_STATUS,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Get power supply status: {response.payload.hex()}", fg="green")


@main.command()
@click.pass_context
def start_telemetry_stream(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.START_TELEMETRY_STREAM,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Start telemetry stream", fg="green")


@main.command()
@click.pass_context
def stop_telemetry_stream(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.STOP_TELEMETRY_STREAM,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("Stop telemetry stream", fg="green")


@main.command()
@click.pass_context
def reboot_device(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.REBOOT_DEVICE,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho("reboot device", fg="green")


@main.command()
@click.argument(
    "feature",
    type=click.Choice(list(ManageFeature.__members__.keys()), case_sensitive=False),
)
@click.argument(
    "action",
    type=click.Choice(list(ManageAction.__members__.keys()), case_sensitive=False),
)
@click.argument(
    "enabled", type=click.Choice(["on", "off"], case_sensitive=False), default="on"
)
@click.pass_context
def manage_feature_toggle(ctx, feature: str, action: str, enabled: str):
    payload = bytearray()
    payload.append(ManageFeature[feature.upper()].value)
    payload.append(ManageAction[action.upper()].value)
    payload.append(int(enabled == "on"))
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.MANAGE_FEATURE_TOGGLE,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0 and action == "get":
        click.secho(
            f"Manage feature toggle: {response.payload[0]} -> {enabled}", fg="green"
        )


@main.command()
@click.pass_context
def scan_wifi(ctx):
    message = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SCAN_WIFI,
        timeout=ctx.obj["timeout"],
    )
    if not message:
        click.secho("Failed to scan wifi", fg="red")
        return

    ssid_count = struct.unpack_from("B", message.payload, 0)[0]
    offset = 1
    ssid_list = []
    for _ in range(ssid_count):
        ssid_length = struct.unpack_from("B", message.payload, offset)[0]
        offset += 1
        ssid = struct.unpack_from(f"{ssid_length}s", message.payload, offset)[0]
        offset += ssid_length
        rssi = struct.unpack_from("b", message.payload, offset)[0]
        offset += 1
        auth_mode = struct.unpack_from("B", message.payload, offset)[0]
        offset += 1
        stored = struct.unpack_from("B", message.payload, offset)[0]
        offset += 1

        ssid_list.append(
            {
                "ssid_length": ssid_length,
                "ssid": ssid.decode("utf-8"),
                "rssi": rssi,
                "auth_mode": WiFiAuthMode.get_name_from_value(auth_mode),
                "stored": bool(stored),
            },
        )

    for item in ssid_list:
        click.secho(
            f"SSID: {item['ssid']} - RSSI: {item['rssi']} - Auth: {item['auth_mode']} - Stored: {item['stored']}",
            fg="green",
        )


@main.command()
@click.pass_context
def reset_device(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.RESET_DEVICE,
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Reset device: {response.status}", fg="green")


@main.command()
@click.pass_context
def get_device_model(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DEVICE_MODEL,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Device model: {response.payload.decode('utf-8')}", fg="green")


@main.command()
@click.argument("intensity", type=int, default=128)
@click.pass_context
async def set_display_intensity(ctx, intensity: int) -> None:
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DISPLAY_INTENSITY,
        payload=bytearray([intensity]),
        timeout=ctx.obj["timeout"],
    )
    if response:
        click.secho(f"Set display intensity to {intensity}", fg="green")


@main.command()
@click.pass_context
async def get_display_intensity(ctx) -> None:
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DISPLAY_INTENSITY,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Display intensity: {response.payload[0]}", fg="green")


@main.command()
@click.pass_context
async def get_display_mode(ctx) -> None:
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DISPLAY_MODE,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        mode = response.payload[0]
        mode_mapping = {0: "off", 1: "manual", 2: "power_meter"}
        click.secho(f"Display mode: {mode}({mode_mapping.get(mode)})", fg="green")


@main.command()
@click.argument(
    "mode",
    type=click.Choice(["off", "manual", "power_meter"], case_sensitive=False),
    default="power_meter",
)
@click.pass_context
async def set_display_mode(ctx, mode: str) -> None:
    mode_mapping = {"off": 0, "manual": 1, "power_meter": 2}
    payload = bytearray()
    payload.append(mode_mapping[mode])
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_DISPLAY_MODE,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Set display mode to {mode}", fg="green")


@main.command()
@click.argument(
    "flip",
    type=click.Choice(["normal", "flip"], case_sensitive=False),
    default="normal",
)
@click.pass_context
async def set_display_flip(ctx, flip: str) -> None:
    flip_mapping = {"normal": 0, "flip": 1}
    payload = bytearray()
    payload.append(flip_mapping[flip])
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_DISPLAY_FLIP,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Set display flip to {flip}", fg="green")


@main.command()
@click.argument("state", type=click.Choice(["off", "on_full", "reset"]))
@click.pass_context
async def set_display_state(ctx, state: str) -> None:
    state_mapping = {"off": 0, "on_full": 1, "reset": 2}
    payload = bytearray()
    payload.append(state_mapping[state])
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_DISPLAY_STATE,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Set display state to {state}", fg="green")


@main.command()
@click.pass_context
async def get_display_state(ctx) -> None:
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_DISPLAY_STATE,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        state = response.payload[0]
        state_mapping = {0: "off", 1: "on_full", 2: "reset"}
        click.secho(f"Display state: {state}({state_mapping.get(state)})", fg="green")


@main.command()
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.argument("command", type=click.IntRange(0, 0x10000, max_open=True))
@click.argument("data", type=str, default="")
@click.option("-a", "--address", type=str, default=None)
@click.pass_context
async def forward_port(ctx, port: str, command: int, data: str, address: str) -> None:
    payload = bytearray()
    payload.append(PORTS.index(port) & 0xFF)
    payload.append(command & 0xFF)
    payload.append((command >> 8) & 0xFF)
    if data:
        payload.extend(bytes.fromhex(data))
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.FORWARD_PORT,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Forward port: {response.payload.hex()}", fg="green")


@main.command()
@click.argument("ssid", type=str)
@click.argument("password", type=str, default="")
@click.option("-n", "--no-passwd", is_flag=True, default=False)
@click.pass_context
def set_wifi_ssid_and_password(ctx, ssid: str, password: str, no_passwd: bool):
    ssid_bytes = ssid.encode("utf-8")
    if len(ssid_bytes) > 255:
        raise ValueError("SSID is too long")
    if len(password) > 63:
        raise ValueError("Password is too long")
    payload = bytearray(
        bytes([len(ssid_bytes), int(not no_passwd)])
        + ssid_bytes
        + password.encode("utf-8")
    )
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_WIFI_SSID_AND_PASSWORD,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho("Set wifi ssid and password", fg="green")


@main.command()
@click.argument(
    "state",
    type=click.Choice(["on", "off"], case_sensitive=False),
    callback=cast_state_to_int,
)
@click.pass_context
def switch_device(ctx, state: int):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SWITCH_DEVICE,
        payload=bytes([state]),
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Switch device: {state}", fg="green")


@main.command()
@click.pass_context
def get_charging_status(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_CHARGING_STATUS,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Get charging status: {bin(response.payload[0])}", fg="green")


def get_wifi_status(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_WIFI_STATUS,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Get wifi status: {response.payload[0]}", fg="green")


@main.command()
@click.pass_context
def reset_wifi(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.RESET_WIFI,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho("Reset wifi", fg="green")


@main.command()
@click.pass_context
def get_wifi_records(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_WIFI_RECORDS,
        timeout=ctx.obj["timeout"],
    )
    if not response or response.status != 0:
        return

    payload = response.payload
    ssid_count = struct.unpack_from("B", payload, 0)[0]
    offset = 1
    ssid_list = []
    for _i in range(ssid_count):
        ssid_length = struct.unpack_from("B", payload, offset)[0]
        offset += 1
        ssid = struct.unpack_from(f"{ssid_length}s", payload, offset)[0]
        offset += ssid_length
        auth_mode = struct.unpack_from("B", payload, offset)[0]
        offset += 1

        ssid_list.append(
            {
                "ssid_length": ssid_length,
                "ssid": ssid.decode("utf-8"),
                "auth_mode": WiFiAuthMode.get_name_from_value(auth_mode),
            },
        )

    for item in ssid_list:
        click.secho(
            f"SSID: {item['ssid']} - Auth: {item['auth_mode']}",
            fg="green",
        )


@main.command()
@click.pass_context
def get_hardware_revision(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_HARDWARE_REVISION,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(
            f"Get hardware revision: {response.payload.decode('utf-8')}",
            fg="green",
        )


def handle_port_config_resp(response: typing.Optional[TelemetryMessage]) -> None:
    if not response or response.status != 0:
        return

    offset = 0
    size = 4
    for i in range(5):
        size = 4
        supply = b""
        if response.payload[offset] == 0:
            size = 3
            supply = b"\x01"
        config = PortConfig.from_buffer_copy(
            response.payload[offset : offset + size] + supply
        )
        click.secho(
            f"port {i}: {response.payload[offset : offset + size].hex()}",
            fg="green",
        )
        click.secho(f"{config}", fg="green")
        offset += size
        click.secho("")


@main.command()
@click.pass_context
def get_port_config(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_PORT_CONFIG,
        timeout=ctx.obj["timeout"],
    )
    handle_port_config_resp(response)


@main.command()
@click.pass_context
def get_port_config1(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_PORT_CONFIG1,
        payload=bytearray([1]),
        timeout=ctx.obj["timeout"],
    )
    handle_port_config_resp(response)


@main.command()
@click.argument("ports", type=click.Choice(PORTS, case_sensitive=False), nargs=-1)
@click.option("--tfcp", is_flag=True, default=False)
@click.option("--pe", is_flag=True, default=False)
@click.option("--qc2p0", is_flag=True, default=False)
@click.option("--qc3p0", is_flag=True, default=False)
@click.option("--qc3plus", is_flag=True, default=False)
@click.option("--afc", is_flag=True, default=False)
@click.option("--fcp", is_flag=True, default=False)
@click.option("--hvscp", is_flag=True, default=False)
@click.option("--lvscp", is_flag=True, default=False)
@click.option("--sfcp", is_flag=True, default=False)
@click.option("--apple", is_flag=True, default=False)
@click.option("--samsung", is_flag=True, default=False)
@click.option("--ufcs", is_flag=True, default=False)
@click.option("--pd", is_flag=True, default=False)
@click.option("--pdcompat", is_flag=True, default=False)
@click.option("--pd_lvpps", is_flag=True, default=False)
@click.option("--pdepr", is_flag=True, default=False)
@click.option("--pdrpi", is_flag=True, default=False)
@click.option("--pd_hvpps", is_flag=True, default=False)
@click.pass_context
def set_port_config1(
    ctx,
    ports: typing.Tuple[str],
    tfcp: bool,
    pe: bool,
    qc2p0: bool,
    qc3p0: bool,
    qc3plus: bool,
    afc: bool,
    fcp: bool,
    hvscp: bool,
    lvscp: bool,
    sfcp: bool,
    apple: bool,
    samsung: bool,
    ufcs: bool,
    pd: bool,
    pdcompat: bool,
    pd_lvpps: bool,
    pdepr: bool,
    pdrpi: bool,
    pd_hvpps: bool,
):
    version = 1
    default_value = [version, 0, 0]
    if version == 1:
        default_value = [version, 0, 0, 0]
    payload = bytearray()
    set_ports = 0
    for i, port in enumerate(PORTS):
        if port not in ports:
            payload.extend(default_value)
            continue
        set_ports |= 1 << i
        config = PortConfigInternal(
            version,
            tfcp,
            pe,
            qc2p0,
            qc3p0,
            qc3plus,
            afc,
            fcp,
            hvscp,
            lvscp,
            sfcp,
            apple,
            samsung,
            ufcs,
            pd,
            is_pd_compat_mode=pd and pdcompat,
            is_pd_lvpps_enabled=pd and pd_lvpps,
            is_pdepr_enabled=pd and pdepr,
            is_pdrpi_enabled=pd and pdrpi,
            is_pd_hvpps_enabled=pd and pd_hvpps,
        )
        payload.extend(config.serialize())
    for _ in range(8 - len(PORTS)):
        payload.extend(default_value)
    payload.insert(0, set_ports)

    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_PORT_CONFIG1,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho("Set port config1", fg="green")


@main.command()
@click.pass_context
def get_messages(ctx):
    global ignore_push_msg
    ignore_push_msg = False
    while True:
        message = wait_for_response(return_all=True)
        if message:
            click.secho(f"Message: {message}", fg="green")


@main.command()
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.pass_context
def turn_on_port(ctx, port: str):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.TURN_ON_PORT,
        payload=bytes([PORTS.index(port)]),
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Turn on port {port}", fg="green")


@main.command()
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.pass_context
def turn_off_port(ctx, port: str):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.TURN_OFF_PORT,
        payload=bytes([PORTS.index(port)]),
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Turn off port {port}", fg="green")


@main.command()
@click.argument(
    "strategy", type=click.Choice(["fast", "slow", "usba"], case_sensitive=False)
)
@click.pass_context
def set_charging_strategy(ctx, strategy: str):
    strategy_mapping = {
        "fast": 0,
        "slow": 1,
        "usba": 6,
    }
    payload = bytearray()
    payload.append(strategy_mapping[strategy])
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.SET_CHARGING_STRATEGY,
        payload=payload,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(f"Set charging strategy to {strategy}", fg="green")


@main.command()
@click.pass_context
def get_wifi_state_machine(ctx):
    response = send_and_receive(
        ctx.obj["topic"]["command"],
        SERVICE.GET_WIFI_STATE_MACHINE,
        timeout=ctx.obj["timeout"],
    )
    if response and response.status == 0:
        click.secho(
            f"Get wifi state machine: {WiFiStateType(response.payload[0]).name}",
            fg="green",
        )

if __name__ == "__main__":
    main()
