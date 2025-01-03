# -*- coding=utf-8 -*-

import ctypes
import dataclasses
import enum
import json
import random
import re
import struct
import threading
import time
import typing

import asyncclick as click
import paho.mqtt.client as mqtt

from ble_tester import FastChargingProtocol

random.seed(time.time())

PORTS: list[str] = ["A", "C1", "C2", "C3", "C4"]

message_event = threading.Event()
receive_messages: typing.List["TelemetryMessage"] = []


class ManageFeature(enum.IntEnum):
    FPGA_POWER_CONTROL = 0


class ManageAction(enum.IntEnum):
    GET = 0
    SET = 1
    RESET = 2


class WifiAuthMode(enum.IntEnum):
    WIFI_AUTH_OPEN = 0  # authenticate mode : open
    WIFI_AUTH_WEP = 1  # authenticate mode : WEP
    WIFI_AUTH_WPA_PSK = 2  # authenticate mode : WPA_PSK
    WIFI_AUTH_WPA2_PSK = 3  # authenticate mode : WPA2_PSK
    WIFI_AUTH_WPA_WPA2_PSK = 4  # authenticate mode : WPA_WPA2_PSK
    WIFI_AUTH_ENTERPRISE = 5  # authenticate mode : WiFi EAP security
    WIFI_AUTH_WPA2_ENTERPRISE = (
        5  # authenticate mode : WiFi EAP security (same as WIFI_AUTH_ENTERPRISE)
    )
    WIFI_AUTH_WPA3_PSK = 6  # authenticate mode : WPA3_PSK
    WIFI_AUTH_WPA2_WPA3_PSK = 7  # authenticate mode : WPA2_WPA3_PSK
    WIFI_AUTH_WAPI_PSK = 8  # authenticate mode : WAPI_PSK
    WIFI_AUTH_OWE = 9  # authenticate mode : OWE
    WIFI_AUTH_WPA3_ENT_192 = 10  # authenticate mode : WPA3_ENT_SUITE_B_192_BIT
    WIFI_AUTH_MAX = 11  # Max value for authentication modes

    @classmethod
    def get_name_from_value(cls, value: int) -> str:
        """Return the name of the enum corresponding to the given value."""
        try:
            return cls(value).name
        except ValueError:
            return f"Unknown: {value}"


@dataclasses.dataclass
class Message:
    command: int
    payload: bytes | bytearray = b""
    msg_id: int = 0

    def __post_init__(self):
        self.msg_id = random.randint(0, 65535)

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

    def __post_init__(self):
        if not hasattr(self, "msg_id") or self.msg_id == 0:
            self.msg_id = random.randint(0, 65535)

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
        return f"Command: {Service(self.command).name}, Msg ID: {self.msg_id}, Status: {self.status}, Payload: {self.payload.hex()}"


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


class Service(enum.IntEnum):
    GET_DEVICE_PASSWORD = 0x05
    MANAGE_FPGA_CONFIG = 0x08  # 管理 FPGA 配置
    MANAGE_POWER_ALLOCATOR_ENABLED = 0x09  # 管理功率分配器是否启用
    MANAGE_POWER_CONFIG = 0x0A  # 管理功率配置
    MANAGE_FEATURE_TOGGLE = 0x0B  # 管理 feature toggle

    REBOOT_DEVICE = 0x11  # 重启设备
    RESET_DEVICE = 0x12  # 重置设备
    GET_ESP32_VERSION = 0x15
    GET_SW3566_VERSION = 0x16
    GET_FPGA_VERSION = 0x17
    SWITCH_DEVICE = 0x1A
    GET_DEVICE_MODEL = 0x1C
    TRIGGER_ESP32_OTA = 0x21
    CONFIRM_ESP32_OTA = 0x23
    SCAN_WIFI = 0x30  # 扫描WIFI
    SET_WIFI_SSID_AND_PASSWORD = 0x36  # 设置WIFI SSID和密码
    GET_POWER_SUPPLY_STATUS = 0x42  # 获取端口供电状态（是否供电）
    GET_PORT_PD_STATUS = 0x49
    GET_ALL_POWER_STATISTICS = 0x4A
    SET_DISPLAY_STATE = 0x77
    GET_DISPLAY_STATE = 0x78

    # MQTT only
    START_TELEMETRY_STREAM = 0x90
    STOP_TELEMETRY_STREAM = 0x91
    SET_BLE_STATE = 0x98
    SET_SYSLOG_STATE = 0x99
    SET_SYSTEM_TIME = 0x9A  # 设置系统时间
    START_OTA = 0x9C

    # 内部信息 0x0000 - 0x0110
    PING = 0x0100

    # 基础设备信息 0x0110 - 0x0120
    DEVICE_BOOT_INFO = 0x0110
    DEVICE_MEMORY_INFO = 0x0111
    DEVICE_SYSTEM_STATE = 0x0112

    # 功率相关信息
    POWER_HISTORICAL_DATA = 0x0130
    POWER_CONSUMPTION_DATA = 0x0131  # 用电统计

    # OTA
    ESP32_UPGRADE_DATA = 0x0140
    OTA_CONFIRM_DATA = 0x0143

    # WIFI
    WIFI_STATS_DATA = 0x0150

    # ALERT
    PD_CHARGING_ALERT = 0x0162


class ClientPDStatus(ctypes.LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("battery_vid", ctypes.c_uint16),
        ("battery_pid", ctypes.c_uint16),  # 4 bytes
        ("battery_design_capacity", ctypes.c_uint16),
        ("battery_last_full_charge_capacity", ctypes.c_uint16),  # 8 bytes
        ("battery_present_capacity", ctypes.c_uint16),
        ("battery_invalid", ctypes.c_uint8, 1),
        ("battery_present", ctypes.c_uint8, 1),
        ("battery_status", ctypes.c_uint8, 2),
        ("cable_is_active", ctypes.c_uint8, 1),
        ("cable_termination_type", ctypes.c_uint8, 1),
        ("cable_epr_mode_capable", ctypes.c_uint8, 1),
        ("cable_active_phy_type", ctypes.c_uint8, 1),
        ("cable_latency", ctypes.c_uint8, 4),
        ("cable_max_vbus_voltage", ctypes.c_uint8, 2),
        ("cable_max_vbus_current", ctypes.c_uint8, 2),
        ("cable_usb_highest_speed", ctypes.c_uint8, 3),
        ("cable_active_element", ctypes.c_uint8, 1),
        ("cable_active_usb4", ctypes.c_uint8, 1),
        ("cable_active_usb2p0", ctypes.c_uint8, 1),
        ("cable_active_usb3p2", ctypes.c_uint8, 1),
        ("cable_active_usb_lanes", ctypes.c_uint8, 1),
        ("cable_active_optically_isolated", ctypes.c_uint8, 1),
        ("cable_active_usb4_asym", ctypes.c_uint8, 1),
        ("cable_active_usb_gen", ctypes.c_uint8, 1),
        ("request_epr_mode_capable", ctypes.c_uint8, 1),
        ("request_pdo_id", ctypes.c_uint8, 4),
        ("request_usb_communications_capable", ctypes.c_uint8, 1),
        ("request_capability_mismatch", ctypes.c_uint8, 1),
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
        ("unused1", ctypes.c_uint8, 4),
        ("operating_voltage", ctypes.c_uint16, 15),
        ("request_ppsavs", ctypes.c_uint16, 1),
        ("cable_xid", ctypes.c_uint32),
        ("bcd_device", ctypes.c_uint16),
        ("unused2", ctypes.c_uint16),
        ("unused3", ctypes.c_uint32 * 14),
    ]

    @classmethod
    def from_bytes(cls, data: bytes | bytearray):
        """Convert incoming data bytes to a ClientPDStatus instance."""
        expected_size = ctypes.sizeof(cls)
        actual_size = len(data)
        if actual_size != expected_size:
            raise ValueError(f"Expected {expected_size} bytes, got {actual_size}")
        return cls.from_buffer_copy(data)

    def __str__(self):
        """Human-readable string representation of the structure."""
        fields_str = []
        for field_name, field_type, *rest in self._fields_:
            value = getattr(self, field_name)
            if isinstance(field_type, ctypes.Array):
                value = list(value)
            fields_str.append(f"  {field_name}={value}")
        return "ClientPDStatus(\n" + ",\n".join(fields_str) + "\n)"


def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected with result code " + str(rc))


def on_subscribe(client, userdata, mid, reasonCodes, properties):
    print(f"Subscribed: {mid}")


def on_message(client, userdata, msg):
    print(f"Received message from {msg.topic}, length: {len(msg.payload)}")
    try:
        message = TelemetryMessage.deserialize(msg.payload)
    except Exception:
        print(msg.payload)
        return
    if message.status != 0:
        print("Error: ", message)
        return

    global receive_messages
    receive_messages.append(message)
    message_event.set()
    match message.command:
        case Service.GET_ESP32_VERSION:
            print(
                f"Get ESP32 version: {message.payload[0]}.{message.payload[1]}.{message.payload[2]}"
            )
        case Service.GET_SW3566_VERSION:
            print(
                f"Get SW3566 version: {message.payload[0]}.{message.payload[1]}.{message.payload[2]}"
            )
        case Service.GET_FPGA_VERSION:
            print(
                f"Get FPGA version: {message.payload[0]}.{message.payload[1]}.{message.payload[2]}"
            )
        case Service.TRIGGER_ESP32_OTA:
            print("Trigger ESP32 OTA")
        case Service.CONFIRM_ESP32_OTA:
            print("Confirm ESP32 OTA")
        case Service.OTA_CONFIRM_DATA:
            hash_length = 32
            print(
                f"OTA Confirm Data, hash: {message.payload[:hash_length].hex()}, esp32 version: {message.payload[hash_length]}.{message.payload[hash_length+1]}.{message.payload[hash_length+2]}"
            )
        case Service.GET_DEVICE_PASSWORD:
            print(
                f"Get password: {message.payload[0]}{message.payload[1]}{message.payload[2]}{message.payload[3]}"
            )
        case Service.MANAGE_POWER_ALLOCATOR_ENABLED:
            if len(message.payload) != 1:
                return
            print(f"Get power allocator enabled: {bool(message.payload[0])}")
        case Service.MANAGE_FPGA_CONFIG:
            if len(message.payload) != 3:
                return
            adc_threshold_low = message.payload[0]
            adc_threshold_high = message.payload[1]
            action_deadzone = message.payload[2]
            print(
                f"Get FPGA config: adc_threshold_low: {adc_threshold_low}, adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}"
            )
        case Service.MANAGE_POWER_CONFIG:
            if message.status != 0:
                print("Failed to manage power config")
                return
            if len(message.payload) == 0:
                return
            if len(message.payload) != 17:
                print(f"Invalid power config length: {len(message.payload)}")
                return
            config = PowerConfig.deserialize(message.payload)
            print(
                f"Get power config: {json.dumps(dataclasses.asdict(config), indent=4)}"
            )
        case Service.GET_ALL_POWER_STATISTICS:
            if len(message.payload) != ctypes.sizeof(AllPowerStatistics):
                print(f"Invalid power statistics data length: {len(message.payload)}")
            all_stats = AllPowerStatistics.from_buffer_copy(message.payload)
            for port, stat in enumerate(all_stats):
                print(f"[{port}] {stat}")
        case Service.GET_POWER_SUPPLY_STATUS:
            print(f"Get power supply status: {message.payload.hex()}")
        case Service.MANAGE_FEATURE_TOGGLE:
            if message.status != 0:
                print("Failed to manage feature toggle")
                return
            feature = message.payload[0]
            action = message.payload[1]
            if action == ManageAction.GET:
                enabled = bool(message.payload[2])
                print(f"{ManageFeature(feature).name}: {enabled}")
        case Service.SCAN_WIFI:
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
                        "auth_mode": WifiAuthMode.get_name_from_value(auth_mode),
                        "stored": bool(stored),
                    },
                )

            for item in ssid_list:
                click.secho(
                    f"SSID: {item['ssid']} - RSSI: {item['rssi']} - Auth: {item['auth_mode']} - Stored: {item['stored']}",
                    fg="green",
                )
        case Service.GET_DEVICE_MODEL:
            model = message.payload.decode("utf-8")
            click.secho(f"Device model: {model}", fg="green")
        case Service.WIFI_STATS_DATA:
            fmt = "<HHbBHHHHHII"
            fields = struct.unpack_from(fmt, message.payload)
            click.secho(f"associationCount: {fields[0]}", fg="green")
            click.secho(f"disassociationCount: {fields[1]}", fg="green")
            click.secho(f"rssi: {fields[2]}", fg="green")
            click.secho(f"channel: {fields[3]}", fg="green")
            click.secho(f"wifiConnectionTime: {fields[4]}", fg="green")
            click.secho(f"mqttConnectionTime: {fields[5]}", fg="green")
            click.secho(f"connectivityFailureCount: {fields[6]}", fg="green")
            click.secho(f"dnsResolutionFailureCount: {fields[7]}", fg="green")
            click.secho(f"mqttConnectionCount: {fields[8]}", fg="green")
            click.secho(f"mqttMessageTxCount: {fields[9]}", fg="green")
            click.secho(f"mqttMessageRxCount: {fields[10]}", fg="green")
        case Service.DEVICE_BOOT_INFO:
            fmt = "<3B 3B 3B 3B 3B B 6B 4B 16B Q H I H b B I I i 8s 8s 8s"
            # Unpack all fields at once
            unpacked = struct.unpack_from(fmt, message.payload)

            # Map the unpacked values to the fields
            esp32 = unpacked[0:3]
            sw3566 = unpacked[3:6]
            fpga = unpacked[6:9]
            zrlib = unpacked[9:12]
            ESPIdfVersion = unpacked[12:15]
            chipCores = unpacked[15]
            bleAddress = unpacked[16:22]
            ipv4 = unpacked[22:26]
            ipv6 = unpacked[26:42]
            uptime = unpacked[42]
            chipModel = unpacked[43]
            chipFeatures = unpacked[44]
            chipRevision = unpacked[45]
            wifiMaxTxPower = unpacked[46]
            activeSW3566Count = unpacked[47]
            fsTotalSize = unpacked[48]
            fsUsedSize = unpacked[49]
            resetReason = unpacked[50]
            hardwareRev = (unpacked[51].decode("utf-8").strip("\x00"),)
            deviceModel = (unpacked[52].decode("utf-8").strip("\x00"),)
            productFamily = (unpacked[53].decode("utf-8").strip("\x00"),)
            click.secho(f"ESP32: {esp32}", fg="green")
            click.secho(f"SW3566: {sw3566}", fg="green")
            click.secho(f"FPGA: {fpga}", fg="green")
            click.secho(f"ZRLib: {zrlib}", fg="green")
            click.secho(f"ESP IDF Version: {ESPIdfVersion}", fg="green")
            click.secho(f"Chip Cores: {chipCores}", fg="green")
            click.secho(f"BLE Address: {bleAddress}", fg="green")
            click.secho(f"IPv4: {ipv4}", fg="green")
            click.secho(f"IPv6: {ipv6}", fg="green")
            click.secho(f"Uptime: {uptime}", fg="green")
            click.secho(f"Chip Model: {chipModel}", fg="green")
            click.secho(f"Chip Features: {chipFeatures}", fg="green")
            click.secho(f"Chip Revision: {chipRevision}", fg="green")
            click.secho(f"Wifi Max Tx Power: {wifiMaxTxPower}", fg="green")
            click.secho(f"Active SW3566 Count: {activeSW3566Count}", fg="green")
            click.secho(f"FS Total Size: {fsTotalSize}", fg="green")
            click.secho(f"FS Used Size: {fsUsedSize}", fg="green")
            click.secho(f"Reset Reason: {resetReason}", fg="green")
            click.secho(f"Hardware Revision: {hardwareRev}", fg="green")
            click.secho(f"Device Model: {deviceModel}", fg="green")
            click.secho(f"Product Family: {productFamily}", fg="green")
        case Service.PD_CHARGING_ALERT:
            fmt = "<B H H H H B"
            fields = struct.unpack_from(fmt, message.payload)
            click.secho(f"Port: {fields[0]}", fg="green")
            click.secho(f"PD RX Soft Reset Count: {fields[1]}", fg="green")
            click.secho(f"PD RX Hard Reset Count: {fields[2]}", fg="green")
            click.secho(f"PD RX Error Count: {fields[3]}", fg="green")
            click.secho(f"PD RX Cable Reset Count: {fields[4]}", fg="green")
        case Service.GET_PORT_PD_STATUS:
            pd_status = ClientPDStatus.from_bytes(message.payload)
            click.secho(f"PD status: {pd_status}", fg="green")
        case Service.GET_DISPLAY_STATE:
            state = message.payload[0]
            state_mapping = {0: "off", 1: "on_full", 2: "reset"}
            click.secho(
                f"Display state: {state}({state_mapping.get(state)})", fg="green"
            )


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


@click.group()
@click.option("--psn", type=str, callback=validate_psn)
@click.option("--broker", type=str, default="localhost")
@click.option("--port", type=int, default=1883)
@click.option("--ca_certs", type=str, default="certs/ca.crt")
@click.option("--certfile", type=str, default="certs/client.pem")
@click.option("--keyfile", type=str, default="certs/client.key")
@click.pass_context
def main(
    ctx,
    psn: str,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
):
    ctx.ensure_object(dict)
    ctx.obj["topic"] = {
        "command": f"device/{psn}/command",
        "telemetry": f"device/{psn}/telemetry",
    }

    @ctx.call_on_close
    def disconnect_mqtt():
        click.secho("Press Ctrl+C to exit...", fg="green")
        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
            click.secho("Disconnecting...", fg="yellow")
        disconnect()

    client.tls_set(ca_certs=ca_certs, certfile=certfile, keyfile=keyfile)
    connect(broker, port)
    subscribe(ctx.obj["topic"]["telemetry"])
    click.secho(f"Connectted to MQTT broker: {broker}:{port}", fg="green")


@main.command()
@click.pass_context
def get_esp32_version(ctx):
    message = Message(Service.GET_ESP32_VERSION)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("version", type=str)
@click.pass_context
def trigger_esp32_ota(ctx, version: str):
    major, minor, patch = version.split(".")
    message = Message(
        Service.TRIGGER_ESP32_OTA,
        payload=bytes([int(major), int(minor), int(patch)]),
    )
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("hash", type=str)
@click.pass_context
def confirm_esp32_ota(ctx, hash: str):
    message = Message(Service.CONFIRM_ESP32_OTA, payload=bytes.fromhex(hash))
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_sw3566_version(ctx):
    message = Message(Service.GET_SW3566_VERSION)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_fpga_version(ctx):
    message = Message(Service.GET_FPGA_VERSION)
    send_message(ctx.obj["topic"]["command"], message)


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
    message = Message(
        Service.START_OTA,
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
    )
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("state", type=click.Choice(["on", "off"], case_sensitive=False))
@click.pass_context
def set_ble_state(ctx, state: str):
    match state:
        case "on":
            message = Message(Service.SET_BLE_STATE, payload=b"\x01")
        case "off":
            message = Message(Service.SET_BLE_STATE, payload=b"\x00")
        case _:
            raise click.ClickException(f"Unknown state: {state}")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument(
    "state",
    type=click.Choice(["on", "off"], case_sensitive=False),
    callback=lambda _, __, v: v.lower(),
)
@click.pass_context
def set_syslog_state(ctx, state: typing.Literal["on", "off"]):
    message = Message(
        Service.SET_SYSLOG_STATE,
        payload=b"\x01" if state == "on" else b"\x00",
    )
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.pass_context
def get_port_pd_status(ctx, port: str):
    message = Message(
        Service.GET_PORT_PD_STATUS, payload=bytearray([PORTS.index(port)])
    )
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_device_password(ctx):
    message = Message(Service.GET_DEVICE_PASSWORD)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("enabled", type=click.Choice(["on", "off"], case_sensitive=False))
@click.pass_context
def set_power_allocator_enabled(ctx, enabled: str):
    payload = bytearray()
    payload.append(1)
    payload.append(int(enabled == "on"))
    message = Message(Service.MANAGE_POWER_ALLOCATOR_ENABLED, payload=payload)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_power_allocator_enabled(ctx):
    message = Message(Service.MANAGE_POWER_ALLOCATOR_ENABLED, payload=b"\x00")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def del_power_allocator_enabled(ctx):
    message = Message(Service.MANAGE_POWER_ALLOCATOR_ENABLED, payload=b"\x02")
    send_message(ctx.obj["topic"]["command"], message)


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
    message = Message(Service.MANAGE_FPGA_CONFIG, payload=payload)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_fpga_config(ctx):
    message = Message(Service.MANAGE_FPGA_CONFIG, payload=b"\x00")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def del_fpga_config(ctx):
    message = Message(Service.MANAGE_FPGA_CONFIG, payload=b"\x02")
    send_message(ctx.obj["topic"]["command"], message)


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
        return cls(*struct.unpack_from("<17B", payload))

    def serialize(self):
        payload = bytearray()
        payload.append(self.version)
        payload.append(self.power_budget)
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
    message = Message(Service.MANAGE_POWER_CONFIG, payload=b"\x00")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
@click.argument("items", type=str, nargs=-1)
async def set_power_config(ctx, items: typing.List[str]):
    """Set power config

    ITEMS is a list of key=value pairs, e.g. power_budget=100
    """
    message = Message(Service.MANAGE_POWER_CONFIG, payload=b"\x00")
    send_message(ctx.obj["topic"]["command"], message)
    print("Waiting for power config...")
    receive_message = None
    while True:
        message_event.wait(10)
        message_event.clear()
        for message in reversed(receive_messages):
            if message.command == Service.MANAGE_POWER_CONFIG:
                receive_message = message
                break
        if receive_message is not None:
            break
    if receive_message.status != 0:
        print("Error: ", receive_message)
        return
    config = PowerConfig.deserialize(receive_message.payload)
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
    message = Message(Service.MANAGE_POWER_CONFIG, payload=b"\x01" + config.serialize())
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def reset_power_config(ctx):
    message = Message(Service.MANAGE_POWER_CONFIG, payload=b"\x02")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_all_power_statistics(ctx):
    message = Message(Service.GET_ALL_POWER_STATISTICS, payload=b"")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def set_system_time(ctx):
    ts = int(time.time())
    message = Message(Service.SET_SYSTEM_TIME, payload=struct.pack("<I", ts))
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_power_supply_status(ctx):
    message = Message(Service.GET_POWER_SUPPLY_STATUS)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def start_telemetry_stream(ctx):
    message = Message(Service.START_TELEMETRY_STREAM, payload=b"")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def stop_telemetry_stream(ctx):
    message = Message(Service.STOP_TELEMETRY_STREAM, payload=b"")
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def reboot_device(ctx):
    message = Message(Service.REBOOT_DEVICE)
    send_message(ctx.obj["topic"]["command"], message)


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
    message = Message(Service.MANAGE_FEATURE_TOGGLE, payload=payload)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def scan_wifi(ctx):
    message = Message(Service.SCAN_WIFI)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def reset_device(ctx):
    message = Message(Service.RESET_DEVICE)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
def get_device_model(ctx):
    message = Message(Service.GET_DEVICE_MODEL)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("state", type=click.Choice(["off", "on_full", "reset"]))
@click.pass_context
async def set_display_state(ctx, state: str) -> None:
    state_mapping = {"off": 0, "on_full": 1, "reset": 2}
    payload = bytearray()
    payload.append(state_mapping[state])
    message = Message(Service.SET_DISPLAY_STATE, payload=payload)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.pass_context
async def get_display_state(ctx) -> None:
    message = Message(Service.GET_DISPLAY_STATE)
    send_message(ctx.obj["topic"]["command"], message)


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
    message = Message(Service.SET_WIFI_SSID_AND_PASSWORD, payload=payload)
    send_message(ctx.obj["topic"]["command"], message)


@main.command()
@click.argument("state", type=click.Choice(["on", "off"], case_sensitive=False))
@click.pass_context
def switch_device(ctx, state: str):
    message = Message(Service.SWITCH_DEVICE, payload=bytes([int(state == "on")]))
    send_message(ctx.obj["topic"]["command"], message)


if __name__ == "__main__":
    main()
