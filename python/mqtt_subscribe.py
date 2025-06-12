"""
MQTT 订阅，用来测试固件 MQTT 上报的消息
"""

import ctypes
import dataclasses
import datetime
import enum
import io
import random
import struct
import typing
from collections import namedtuple

import asyncclick as click
import paho.mqtt.client as mqtt_client

import ble_tester as ble
from data_types.port import PowerFeatures, PortType
from data_types.service import SERVICE


def info(msg: str) -> None:
    click.secho(msg, fg="green")


def error(msg: str) -> None:
    click.secho(msg, fg="red")


def warn(msg: str) -> None:
    click.secho(msg, fg="yellow")


def format_mac(raw: bytes) -> str:
    return ":".join(f"{b:02x}" for b in raw)


@dataclasses.dataclass
class Config:
    broker: str = "127.0.0.1"
    port: int = 1883
    topic: str = "device/{psn}/telemetry"
    command_topic: str = "device/{psn}/command"
    # generate client ID with pub prefix randomly
    client_id: str = f"python-mqtt-{random.randint(0, 100)}"
    subscribe_services: typing.List["SERVICE"] = dataclasses.field(default_factory=list)
    message_count: int = 0

    def set_topic(self, psn: str):
        self.topic = f"device/{psn}/telemetry"
        self.command_topic = f"device/{psn}/command"


@dataclasses.dataclass
class Global:
    client: typing.Optional[mqtt_client.Client] = None
    receive_count: int = 0

# 全局配置
config = Config()
g = Global()


@dataclasses.dataclass
class MessageHeader:
    service: int
    message_id: int

    @classmethod
    def from_data(cls, data: bytes):
        service, message_id = struct.unpack("<HH", data)
        return cls(service, message_id)


def bool_at_bit(value: int, index: int) -> bool:
    return bool((value >> index) & 0x1)


@dataclasses.dataclass
class PortFeatures:
    enable: bool
    ufcs: bool
    tfcp: bool

    @classmethod
    def from_uint8(cls, value: int):
        return cls(bool_at_bit(value, 0), bool_at_bit(value, 1), bool_at_bit(value, 2))

class PDStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("battery_vid", ctypes.c_uint16),
        ("battery_pid", ctypes.c_uint16),
        ("battery_design_capacity", ctypes.c_uint16),
        ("battery_last_full_charge_capacity", ctypes.c_uint16),
        ("battery_present_capacity", ctypes.c_uint16),
        ("battery_invalid", ctypes.c_bool, 1),
        ("battery_present", ctypes.c_bool, 1),
        ("battery_status", ctypes.c_uint8, 2),
        ("cable_is_active", ctypes.c_uint8, 1),
        ("cable_termination_type", ctypes.c_uint8, 1),
        ("cable_epr_mode_capable", ctypes.c_bool, 1),
        ("cable_active_phy_type", ctypes.c_uint8, 1),
        ("cable_latency", ctypes.c_uint8, 4),
        ("cable_max_vbus_voltage", ctypes.c_uint8, 2),
        ("cable_max_vbus_current", ctypes.c_uint8, 2),
        ("cable_usb_highest_speed", ctypes.c_uint8, 3),
        ("cable_active_element", ctypes.c_uint8, 1),
        ("cable_active_usb4", ctypes.c_bool, 1),
        ("cable_active_usb2p0", ctypes.c_bool, 1),
        ("cable_active_usb3p2", ctypes.c_bool, 1),
        ("cable_active_usb_lanes", ctypes.c_bool, 1),
        ("cable_active_optically_isolated", ctypes.c_bool, 1),
        ("cable_active_usb4_asym", ctypes.c_bool, 1),
        ("cable_active_usb_gen", ctypes.c_bool, 1),
        ("request_epr_mode_capable", ctypes.c_bool, 1),
        ("request_pdo_id", ctypes.c_uint8, 4),
        ("request_usb_communications_capable", ctypes.c_bool, 1),
        ("request_capability_mismatch", ctypes.c_bool, 1),
        ("sink_capabilities", ctypes.c_uint8, 4),
        ("sink_cap_pdo_count", ctypes.c_uint8, 2),
        ("status_temperature", ctypes.c_uint8),
        ("cable_vid", ctypes.c_uint16),
        ("cable_pid", ctypes.c_uint16),
        ("manufacturer_vid", ctypes.c_uint16),
        ("manufacturer_pid", ctypes.c_uint16),
        ("sink_minimum_pdp", ctypes.c_uint8),
        ("sink_operational_pdp", ctypes.c_uint8),
        ("sink_maximum_pdp", ctypes.c_uint8),
        ("unused0", ctypes.c_uint8),
        ("operating_current", ctypes.c_uint16, 10),
        ("pd_revision", ctypes.c_uint8, 2),
        ("pps_charging_supported", ctypes.c_uint8, 1),
        ("has_battery", ctypes.c_uint8, 1),
        ("dual_role_power", ctypes.c_uint8, 1),
        ("has_emarker", ctypes.c_uint8, 1),
        ("operating_voltage", ctypes.c_uint16, 15),
        ("request_ppsavs", ctypes.c_bool, 1)
    ]


class PortDetails(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("connected", ctypes.c_uint8),
        ("fc_protocol", ctypes.c_uint8),
        ("iout_value", ctypes.c_uint16),
        ("vout_value", ctypes.c_uint16),
        ("die_temperature", ctypes.c_uint16),
        ("vin_value", ctypes.c_uint16),
        ("session_id", ctypes.c_uint16),
        ("session_charge", ctypes.c_uint64),
        ("power_budget", ctypes.c_uint8),
    ]

    def __str__(self):
        return f"PortDetails(connected={self.connected}, fc_protocol={self.fc_protocol}, iout_value={self.iout_value}, vout_value={self.vout_value}, die_temperature={self.die_temperature}, vin_value={self.vin_value}, session_id={self.session_id}, session_charge={self.session_charge}, power_budget={self.power_budget})"


class PortStatusData(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("charging_minutes", ctypes.c_uint16),
        ("features", PowerFeatures),
        ("details", PortDetails),
        ("port_type", ctypes.c_uint8),
        ("unused1", ctypes.c_uint8 * 5),
    ]


class StreamPortStatus(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("status", ctypes.c_uint8),
        ("port_status_map", ctypes.c_uint8),
        ("padding", ctypes.c_uint16),
        ("ports", PortStatusData * 8),
    ]

    def show(self):
        port_names = ["A", "C1", "C2", "C3", "C4", "-", "-", "-"]
        print("StreamPortStatus:")
        print(f"  Status: {self.status}")
        print(f"  Port Status Map: {bin(self.port_status_map)}")
        print(f"  Padding: {self.padding}")
        print(f"  Ports:")
        for idx, port in enumerate(self.ports[:5]):
            open = ((self.port_status_map >> idx) & 0x1) == 0x1
            fg = None
            if open:
                fg = "green"
            if port.details.connected:
                fg = "magenta"
            click.secho(f"    Port {port_names[idx]}({idx}):", fg=fg)
            click.secho(f"      Features: {port.features}", fg=fg)
            click.secho(f"      Charging Minutes: {port.charging_minutes}", fg=fg)
            click.secho(f"      Details: {port.details}", fg=fg)
            click.secho(f"      Port Type: {PortType(port.port_type).name}", fg=fg)
            click.secho(f"      Unused1: {port.unused1}", fg=fg)
            click.secho("\n")


@dataclasses.dataclass
class Message:
    header: MessageHeader
    payload: bytes

    @classmethod
    def from_data(cls, data: bytes):
        header = MessageHeader.from_data(data[0:4])
        return cls(header, data[4:])

    def show(self):
        if self.header.service not in SERVICE:
            warn(f"Ignore unknown service: {self.header.service}")
            warn(f"    payload: {self.payload.hex()}")
            return
        if config.subscribe_services and self.header.service not in config.subscribe_services:
            return
        g.receive_count += 1
        if config.message_count and g.receive_count > config.message_count:
            assert g.client
            g.client.disconnect()
            return
        print(
            f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')} service: {SERVICE(self.header.service).name}, message_id: {self.header.message_id}"
        )
        format = ""
        fields = []
        port_names = ["A", "C1", "C2", "C3", "C4", "-", "-", "-"]
        if not format:
            print(f"    payload: {self.payload.hex()}")
            return
        try:
            result = struct.unpack(format, self.payload)
        except Exception:
            error(f"    payload({len(self.payload)}): {self.payload.hex()}")
            raise
        if len(result) != len(fields):
            error("Invalid format")
        for field, value in zip(fields, result):
            if isinstance(value, bytes):
                try:
                    s = value.decode("utf-8").rstrip("\x00")
                    if s.isalnum():
                        value = s
                    else:
                        value = value.hex().upper()
                except UnicodeDecodeError:
                    value = value.hex().upper()
            elif field.endswith("bits"):
                value = bin(value)

            info(f"    {field}: {value}")


def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connected to MQTT Broker!")
        print(f"Subscribing to topic: {config.topic}")
        client.subscribe(config.topic)
    else:
        print(f"Failed to connect, return code {rc}\n")


def on_message(client, userdata, msg):
    rmsg = Message.from_data(msg.payload)
    rmsg.show()
    # print(f"Received `{msg.payload}` from `{msg.topic}` topic")


def connect_mqtt(
    broker: str, port: int, ca_certs: str, certfile: str, keyfile: str
) -> mqtt_client.Client:
    client = mqtt_client.Client(
        mqtt_client.CallbackAPIVersion.VERSION2, protocol=mqtt_client.MQTTv5
    )
    client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    return client


@click.command()
@click.option("--psn", type=str)
@click.option("--broker", type=str, default="localhost")
@click.option("--port", type=int, default=1883)
@click.option("--ca_certs", type=str, default="certs/ca.crt")
@click.option("--certfile", type=str, default="certs/client.pem")
@click.option("--keyfile", type=str, default="certs/client.key")
@click.option("--count", type=int, default=0, help="Number of messages to receive")
@click.option("-s", "--start", is_flag=True, help="Start telemetry stream")
@click.argument("services", type=click.Choice(list(SERVICE.__members__.keys()), case_sensitive=False), nargs=-1)
def run(
    psn: str,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    count: int,
    start: bool,
    services: typing.List[str],
):
    config.broker = broker
    config.port = port
    config.set_topic(psn)
    config.subscribe_services = [SERVICE[s] for s in services]
    config.message_count = count
    client = connect_mqtt(broker, port, ca_certs, certfile, keyfile)
    g.client = client
    if start:
        client.publish(config.command_topic, bytes([SERVICE.START_TELEMETRY_STREAM, 12, 34]))
    client.loop_forever()


if __name__ == "__main__":
    run()
