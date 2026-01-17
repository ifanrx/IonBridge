# -*- coding=utf-8 -*-

import ctypes
import dataclasses
import enum
import queue
import random
import re
import struct
import threading
import time
import typing
import concurrent.futures

import asyncclick as click
import paho.mqtt.client as mqtt

from data_types.port import FastChargingProtocol
import anyio
import asyncclick as click
import asyncio
import paho.mqtt.client as mqtt
import paho.mqtt.enums
import utils
from google.protobuf.json_format import MessageToJson, MessageToDict

import admin_pb2
import command_pb2
import data_types_pb2
import telemetry_pb2
from data_types.port import (
    CableCompensation,
    FastChargingProtocol,
    FixedVoltageOffset,
    FlashPpRequest,
    GetPortDetailsResponse,
    KeepAliveResponse,
    KeepAliveStatus,
    PortCommand,
    PORTS,
    SystemFlags,
    SetMaxPowerBudgetRequest,
)
from data_types.strategy import ChargingStrategy
from data_types.service import SERVICE, ServiceCommand, AdminCommand, TelemetryCommand, Topic
from data_types.wifi import WiFiAuthMode, WiFiStateType
from usb_pd import pcap

random.seed(time.time())

PORTS: list[str] = ["A", "C1", "C2", "C3", "C4"]


request_queue = queue.Queue()
response_queue = queue.Queue()
message_event = threading.Event()
receive_messages: typing.List["TelemetryMessage"] = []
device_esp32_version_map: typing.Dict[str, str] = {}
upgrade_storage: typing.Dict[str, bool] = {}
subscribe_topics: typing.Any = []
msg_id = 1


class ManageFeature(enum.IntEnum):
    FPGA_POWER_CONTROL = 0


class ManageAction(enum.IntEnum):
    GET = 0
    SET = 1
    RESET = 2


@dataclasses.dataclass
class Message:
    command: typing.Union[ServiceCommand, AdminCommand]
    msg_id: int = 0
    pb_message: typing.Union[
        command_pb2.CommandRequest, admin_pb2.AdminCommandRequest
    ] = dataclasses.field(default_factory=command_pb2.CommandRequest)

    def __post_init__(self):
        global msg_id
        self.msg_id = msg_id
        msg_id += 1
        msg_id &= 0xFFFF
        self.pb_message.id = self.msg_id

    def __str__(self):
        return f"Command: {self.command.name}, Msg ID: {self.msg_id}, Protobuf: {MessageToJson(self.pb_message)}"

    def serialize(self) -> bytes | bytearray:
        return self.pb_message.SerializeToString()


@dataclasses.dataclass
class TelemetryMessage:
    command: typing.Union[ServiceCommand, AdminCommand, TelemetryCommand]
    msg_id: int
    status: int
    pb_message: typing.Union[
        command_pb2.CommandResponse,
        admin_pb2.AdminCommandResponse,
        telemetry_pb2.TelemetryData,
    ]
    payload: bytes | bytearray = b""

    @classmethod
    def deserialize(cls, topic: str, data: bytes):
        topic_obj = Topic.parse(topic)
        pb_cls = command_pb2.CommandResponse
        match topic_obj.command:
            case ServiceCommand():
                pb_cls = command_pb2.CommandResponse
            case AdminCommand():
                pb_cls = admin_pb2.AdminCommandResponse
            case TelemetryCommand():
                pb_cls = telemetry_pb2.TelemetryData
        pb_message = pb_cls()
        pb_message.ParseFromString(data)
        msg_id = 0
        status = 0
        if isinstance(
            pb_message, (command_pb2.CommandResponse, admin_pb2.AdminCommandResponse)
        ):
            msg_id = pb_message.id
            status = pb_message.status
        payload = data
        return cls(topic_obj.command, msg_id, status, pb_message, payload)

    def __str__(self):
        return f"Command: {self.command.name}, Msg ID: {self.msg_id}, Status: {self.status}, Protobuf: {MessageToJson(self.pb_message)}"

    def to_json(self):
        return MessageToJson(self.pb_message, always_print_fields_with_no_presence=True)

    def command_pb(self) -> command_pb2.CommandResponse:
        if isinstance(self.pb_message, command_pb2.CommandResponse):
            return self.pb_message
        raise ValueError("Not a command response message")

    def telemetry_pb(self) -> telemetry_pb2.TelemetryData:
        if isinstance(self.pb_message, telemetry_pb2.TelemetryData):
            return self.pb_message
        raise ValueError("Not a telemetry message")

    def admin_pb(self) -> admin_pb2.AdminCommandResponse:
        if isinstance(self.pb_message, admin_pb2.AdminCommandResponse):
            return self.pb_message
        raise ValueError("Not an admin command response message")


class PowerStatistics(ctypes.BigEndianStructure):
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
    print('subscribing to topic: ', subscribe_topics)
    subscribe(subscribe_topics)


def on_subscribe(client, userdata, mid, reasonCodes, properties):
    print(f"Subscribed: {mid}")


def on_message(client, userdata, msg):
    psn = msg.topic.split("/")[1]
    try:
        message = TelemetryMessage.deserialize(msg.topic, msg.payload)
    except Exception:
        # print(f"{psn} Failed to deserialize message: {e}")
        # print(f"{psn} Payload: {msg.payload}")
        return
    if message.status != 0:
        print(f"{psn} Error: ", message)
        return

    global receive_messages
    receive_messages.append(message)
    message_event.set()
    match message.command:
        case ServiceCommand.GET_AP_VERSION:
            version = message.command_pb().get_ap_version.version
            print(f"{psn} AP version: {version.major}.{version.minor}.{version.patch}")
            device_esp32_version_map[psn] = (
                f"{version.major}.{version.minor}.{version.patch}"
            )
        # case Service.MANAGE_POWER_ALLOCATOR_ENABLED:
        #     if len(message.payload) != 1:
        #         return
        #     print(f"{psn} Get power allocator enabled: {bool(message.payload[0])}")
        # case Service.GET_DEVICE_MODEL:
        #     model = message.payload.decode("utf-8")
        #     print(f"{psn} Device model: {model}")
        case AdminCommand.MANAGE_FEATURE_TOGGLE:
            if message.status != 0:
                print(f"{psn} Failed to manage feature toggle")
                return
            feature = message.payload[0]
            action = message.payload[1]
            if action == ManageAction.GET:
                enabled = bool(message.payload[2])
                print(f"{psn} {ManageFeature(feature).name}: {enabled}")


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
    client.publish(topic, payload=message.serialize())


def send_command(psn: str, message: Message):
    topic_obj = Topic(psn, message.command, is_request=True)
    send_message(topic_obj.to_topic(), message)


def wait_online(psn: str) -> bool:
    req = command_pb2.CommandRequest()
    message = Message(ServiceCommand.GET_AP_VERSION, pb_message=req)
    send_command(psn, message)
    for _ in range(10):
        if device_esp32_version_map.get(psn) is not None:
            return True
        time.sleep(1)

    return False


def close_fpga_power_control(psn: str):
    req = admin_pb2.AdminCommandRequest()
    req.manage_feature_toggle.action = command_pb2.ManageAction.UPDATE
    req.manage_feature_toggle.feature = admin_pb2.ManageFeature.FPGA_POWER_CONTROL
    req.manage_feature_toggle.enabled = False

    message = Message(AdminCommand.MANAGE_FEATURE_TOGGLE, pb_message=req)
    send_command(psn, message)


def close_power_allocator(psn: str):
    req = admin_pb2.AdminCommandRequest()
    req.manage_feature_toggle.action = command_pb2.ManageAction.UPDATE
    req.manage_feature_toggle.feature = admin_pb2.ManageFeature.POWER_ALLOCATOR
    req.manage_feature_toggle.enabled = False

    message = Message(AdminCommand.MANAGE_FEATURE_TOGGLE, pb_message=req)
    send_command(psn, message)


def start_ota(
    psn: str,
    disable_power_control: bool,
    esp32_new_version: str,
    sw3566_new_version: str,
    fpga_new_version: str,
):
    force = False
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
    send_command(psn, Message(ServiceCommand.START_OTA, pb_message=req))


def start_ota_and_wait(
    psn: str,
    disable_power_control: bool,
    esp32_new_version: str,
    sw3566_new_version: str,
    fpga_new_version: str,
):
    timeout = time.time() + 4 * 60
    online = False
    print(f"{psn} Waiting for device to be online...")
    while time.time() < timeout:
        if wait_online(psn):
            online = True
            break

    if not online:
        print(f"{psn} Device is not online")
        upgrade_storage[psn] = False
        return
    upgrade_storage[psn] = False
    if device_esp32_version_map.get(psn) == esp32_new_version:
        if disable_power_control:
            print(f"{psn} Disable fpga power control and power allocator")
            close_fpga_power_control(psn)
            time.sleep(1)
            close_power_allocator(psn)
        print(f"{psn} Device is already at version {esp32_new_version}")
        upgrade_storage[psn] = True
        return

    force = False
    esp32_major, esp32_minor, esp32_patch = esp32_new_version.split(".")
    sw3566_major, sw3566_minor, sw3566_patch = sw3566_new_version.split(".")
    fpga_major, fpga_minor, fpga_patch = fpga_new_version.split(".")

    # print(f"{psn} Starting OTA to version {esp32_new_version} \
    #       SW3566 version: {sw3566_new_version} \
    #       FPGA version: {fpga_new_version}")

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
    send_command(psn, Message(ServiceCommand.START_OTA, pb_message=req))
    device_esp32_version_map.pop(psn, None)
    time.sleep(60)

    # wait for upgrade and reconnect
    timeout = time.time() + 9 * 60
    while time.time() < timeout:
        if wait_online(psn):
            break
        time.sleep(1)

    if device_esp32_version_map.get(psn) != esp32_new_version:
        print(f"{psn} Failed to update device to version {esp32_new_version}")
        upgrade_storage[psn] = False
        return

    upgrade_storage[psn] = True
    if disable_power_control:
        print(f"{psn} Disable fpga power control and power allocator")
        close_fpga_power_control(psn)
        time.sleep(1)
        close_power_allocator(psn)
        time.sleep(1)


@click.command()
@click.option(
    "--not-wait",
    is_flag=True,
    default=False,
    help="Do not wait for the device to be online, just send the OTA command.",
)
@click.option("--disable-power-control", is_flag=True, default=False)
@click.option("--broker", type=str, default="localhost")
@click.option("--port", type=int, default=1883)
@click.option("--ca_certs", type=str, default="certs/ca.crt")
@click.option("--certfile", type=str, default="certs/client.pem")
@click.option("--keyfile", type=str, default="certs/client.key")
@click.argument("versions", type=str, nargs=3)
@click.argument("psns", type=str, nargs=-1)
@click.pass_context
def main(
    ctx,
    not_wait: bool,
    disable_power_control: bool,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    versions: typing.List[str],
    psns: typing.List[str],
):
    ctx.ensure_object(dict)
    print("PSNs: ", psns)
    for psn in psns:
        subscribe_topics.append((f"device/{psn}/enduser/response/+", 0))
        subscribe_topics.append((f"device/{psn}/admin/response/+", 0))
        subscribe_topics.append((f"device/{psn}/telemetry/+", 0))
    print(subscribe_topics)

    client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)
    connect(broker, port)
    # subscribe(subscribe_topics)
    click.secho(f"Connectted to MQTT broker: {broker}:{port}", fg="green")

    ota_function = start_ota_and_wait
    if not_wait:
        ota_function = start_ota
    with concurrent.futures.ThreadPoolExecutor(max_workers=30) as executor:
        for psn in psns:
            executor.submit(ota_function, psn, disable_power_control, *versions)

    disconnect()
    print("\n\n")
    items = list(upgrade_storage.items())
    items.sort(key=lambda x: x[1], reverse=True)
    success_count = 0
    for k, v in items:
        if v:
            success_count += 1
            click.secho(f"{k}", fg="green")
            # click.secho(f"{k} Upgrade Succeeded", fg="green")
        else:
            click.secho(f"{k} Upgrade Failed", fg="red")
    click.secho(f"total count: {len(items)}, success: {success_count}")


if __name__ == "__main__":
    main()
