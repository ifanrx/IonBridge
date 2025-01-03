# -*- coding=utf-8 -*-

"""
See https://github.com/hbldh/bleak/blob/master/examples/uart_service.py
"""

from __future__ import annotations

import asyncio
import binascii
import ctypes
import enum
import json
import os
import pathlib
import random
import re
import struct
import sys
import time
import typing
from dataclasses import dataclass, field
from itertools import count, takewhile
from pprint import pprint
from typing import Any, Iterator, Type

import asyncclick as click
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

SERVICE_UUID = "048e3f2e-e1a6-4707-9e74-a930e898a1ea"
TX_CHAR_UUID = "148e3f2e-e1a6-4707-9e74-a930e898a1ea"
RX_CHAR_UUID = "248e3f2e-e1a6-4707-9e74-a930e898a1ea"

MAX_MSG_SIZE = 8192
FRAG_SIZE = 499

VERSION_PAT = re.compile(r"(\d+)\.(\d+)\.(\d+)")
ENVIRON_BLE_ADDRESS = os.environ.get("BLE_ADDRESS") or ""

RX_HANDLER = Type[typing.Callable[[BleakGATTCharacteristic, bytearray], None]]

PORTS: list[str] = ["A", "C1", "C2", "C3", "C4"]
PORTS_NUM = len(PORTS)
PORT_FEATURES: list[str] = ["overclock", "tfcp", "ufcs"]


class SERVICE(enum.IntEnum):
    @property
    def command(self: "SERVICE") -> str:
        return self.name.lower().replace("_", "-")

    # Internal
    BLE_ECHO_TEST = 0x00  # BLE 回显测试
    GET_DEBUG_LOG = 0x01  # 获取调试日志
    GET_SECURE_BOOT_DIGEST = 0x02  # 获取 Secure Boot Digest
    PING_MQTT_TELEMETRY = 0x03  # ping MQTT telemetry
    PING_HTTP = 0x04  # ping HTTP
    GET_DEVICE_PASSWORD = 0x05  # 获取密码
    SET_TEST_MODE_A = 0x06  # 设置测试模式 A 定频发射
    SET_TEST_MODE_B = 0x07  # 设置测试模式 B Single Tone
    MANAGE_FPGA_CONFIG = 0x08  # 管理 FPGA 配置
    MANAGE_POWER_ALLOCATOR_ENABLED = 0x09  # 管理 Power Allocator Enabled

    # Device
    ASSOCIATE_DEVICE = 0x10  # 关联设备
    REBOOT_DEVICE = 0x11  # 重启设备
    RESET_DEVICE = 0x12  # 重置设备
    GET_DEVICE_SERIAL_NO = 0x13  # 获取设备序列号
    GET_DEVICE_UPTIME = 0x14  # 获取设备运行时间
    GET_AP_VERSION = 0x15  # 获取 ESP32 固件版本
    GET_BP_VERSION = 0x16  # 获取 SW3566 固件版本
    GET_FPGA_VERSION = 0x17  # 获取 FPGA 固件版本
    GET_ZRLIB_VERSION = 0x18  # 获取 ZRLIB 版本
    GET_DEVICE_BLE_ADDR = 0x19  # 获取设备BLE地址
    SWITCH_DEVICE = 0x1A  # 开关设备
    GET_DEVICE_SWITCH = 0x1B  # 获取开关状态
    GET_DEVICE_MODEL = 0x1C  # 获取设备型号
    PUSH_LICENSE = 0x1D  # 推送 license
    GET_BLE_RSSI = 0x1E  # 获取 BLE RSSI

    # OTA
    PERFORM_BLE_OTA = 0x20  # 执行BLE OTA
    PERFORM_WIFI_OTA = 0x21  # 执行WIFI OTA
    GET_WIFI_OTA_PROGRESS = 0x22  # 获取WIFI OTA进度
    CONFIRM_OTA = 0x23  # 确认OTA

    # WIFI
    SCAN_WIFI = 0x30  # 扫描WIFI
    SET_WIFI_SSID = 0x31  # 设置WIFI SSID
    SET_WIFI_PASSWORD = 0x32  # 设置WIFI密码
    RESET_WIFI = 0x33  # 重置WIFI
    GET_WIFI_STATUS = 0x34  # 获取连接的 WIFI SSID BSSID RSSI CHANNEL PROTOCOL
    GET_DEVICE_WIFI_ADDR = 0x35  # 获取设备WIFI地址
    SET_WIFI_SSID_AND_PASSWORD = 0x36  # 设置WIFI SSID和密码
    GET_WIFI_RECORDS = 0x37  # 获取WIFI记录
    OPERATE_WIFI_RECORD = 0x38  # 操作WIFI记录

    # Power
    TOGGLE_PORT_POWER = 0x40  # 切换端口
    GET_POWER_STATISTICS = 0x41  # 获取电源统计
    GET_POWER_SUPPLY_STATUS = 0x42  # 获取端口供电状态（是否供电）
    SET_CHARGING_STRATEGY = 0x43  # 设置充电策略，有两种模式，分别是高速充电和智能慢充
    GET_CHARGING_STATUS = 0x44  # 获取端口充电状态
    GET_POWER_HISTORICAL_STATS = 0x45  # 获取历史电源统计
    SET_PORT_PRIORITY = 0x46  # 设置端口优先级
    GET_PORT_PRIORITY = 0x47  # 获取端口优先级
    GET_CHARGING_STRATEGY = 0x48  # 获取充电策略
    GET_PORT_PD_STATUS = 0x49  # 获取 PD 信息
    GET_ALL_POWER_STATISTICS = 0x4A  # 获取全部端口电源统计
    GET_START_CHARGE_TIMESTAMP = 0x4B  # 获取开始充电的时间戳
    TURN_ON_PORT = 0x4C  # 打开端口
    TURN_OFF_PORT = 0x4D  # 关闭端口
    SET_PORT_OVERCLOCK_FEATURE = 0x4F
    GET_PORT_OVERCLOCK_FEATURE = 0x50
    SET_PORT_TFCP_FEATURE = 0x51  # 切换 TFCP
    GET_PORT_TFCP_FEATURE = 0x52  # 查询 TFCP 状态
    SET_PORT_UFCS_FEATURE = 0x53  # 切换 UFCS
    GET_PORT_UFCS_FEATURE = 0x54  # 查询 UFCS 状态
    SET_STATIC_ALLOCATOR = 0x55  # 设置老式充充电策略
    GET_STATIC_ALLOCATOR = 0x56  # 获取老式充充电策略
    SET_PORT_CONFIG = 0x57  # 设置端口配置
    GET_PORT_CONFIG = 0x58  # 获取端口配置
    SET_PORT_COMPATIBILITY_SETTINGS = 0x59  # 设置端口兼容性设置
    GET_PORT_COMPATIBILITY_SETTINGS = 0x5A  # 获取端口兼容性设置
    SET_TEMPERATURE_MODE = 0x5B  # 设置温度模式
    SET_TEMPORARY_ALLOCATOR = 0x5C  # 设置临时充电策略

    # Display
    SET_DISPLAY_INTENSITY = 0x70
    SET_DISPLAY_MODE = 0x71
    GET_DISPLAY_INTENSITY = 0x72
    GET_DISPLAY_MODE = 0x73
    SET_DISPLAY_FLIP = 0x74  # 设置屏幕/灯正反方向
    GET_DISPLAY_FLIP = 0x75  # 获取屏幕/灯正反方向
    SET_DISPLAY_CONFIG = 0x76  # 设置屏幕/灯配置
    SET_DISPLAY_STATE = 0x77  # 设置屏幕/灯状态
    GET_DISPLAY_STATE = 0x78  # 获取屏幕/灯状态

    # Telemetry Stream
    START_TELEMETRY_STREAM = 0x90
    STOP_TELEMETRY_STREAM = 0x91

    GET_DEVICE_INFO = 0x92  # 获取设备信息


class BLEFLAGS(enum.IntEnum):
    NONE = 0x0
    SYN = 0x1
    ACK = 0x2
    FIN = 0x3
    RST = 0x4
    SYN_ACK = 0x5


class FRAGFLAGS(enum.IntEnum):
    NO_FRAG = 0x0
    FIRST_FRAG = 0x1
    MORE_FRAG = 0x2
    LAST_FRAG = 0x3


class FastChargingProtocol(enum.IntEnum):
    NONE = 0
    QC2 = 1
    QC3 = 2
    QC3P = 3
    SFCP = 4
    AFC = 5
    FCP = 6
    SCP = 7
    VOOC1P0 = 8
    VOOC4P0 = 9
    SVOOC2P0 = 10
    TFCP = 11
    UFCS = 12
    PE1 = 13
    PE2 = 14
    PD_FIX5V = 15
    PD_FIXHV = 16
    PD_SPR_AVS = 17
    PD_PPS = 18
    PD_EPR_HV = 19
    PD_AVS = 20
    NOT_CHARGING = 0xFF


class ChargingStrategy(enum.IntEnum):
    FAST_CHARGING = 0
    SLOW_CHARGING = 1
    STATIC_CHARGING = 2
    TEMPORARY_CHARGING = 3  # 临时分配模式


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


def to_int8(b: int) -> int:
    return struct.unpack(">b", (b & 0xFF).to_bytes(1, "big"))[0]


def to_uint8(b: int) -> int:
    return struct.unpack(">B", (b & 0xFF).to_bytes(1, "big"))[0]


def to_uint32(b: int) -> typing.Sequence[int]:
    return struct.unpack(">BBBB", (b & 0xFFFFFFFF).to_bytes(4, "big"))


def bytes_to_int(sequence: typing.Sequence[int]) -> int:
    # Ensure the sequence has exactly 4 elements
    if len(sequence) != 4:
        raise ValueError("Sequence must have exactly 4 elements.")

    # Pack the sequence of bytes into a single 32-bit integer
    return struct.unpack(">I", struct.pack("4B", *sequence))[0]


@dataclass
class BLEHeader:
    version: int
    msg_id: int
    service: int
    sequence: int
    flags: int
    size_t: typing.Sequence[int] = (0, 0, 0, 0)

    def __len__(self) -> int:
        # 9 bytes
        return 9

    @property
    def frag_flag(self: "BLEHeader") -> int:
        return (self.flags >> 3) & 0x3

    @property
    def tcp_flag(self: "BLEHeader") -> int:
        return self.flags & 0x7

    @property
    def size(self: "BLEHeader") -> int:
        return bytes_to_int(self.size_t)

    @size.setter
    def size(self: "BLEHeader", value: int) -> None:
        self.size_t = to_uint32(value & 0xFFFFFFFF)

    @property
    def checksum(self: "BLEHeader") -> int:
        return self.calc_checksum()

    @checksum.setter
    def checksum(self: "BLEHeader", value: int) -> None:
        if value != self.calc_checksum():
            raise ValueError("Invalid checksum")

    def __post_init__(self: "BLEHeader") -> None:
        self.version = to_uint8(self.version)
        self.msg_id = to_uint8(self.msg_id)
        self.service = to_int8(self.service)
        self.sequence = to_uint8(self.sequence)
        self.flags = to_uint8(self.flags)

    def to_bytes(self: "BLEHeader") -> bytearray:
        data = bytearray()
        data.append(self.version)
        data.append(self.msg_id)
        data.append(self.service)
        data.append(self.sequence)
        data.append(self.flags)
        data.extend(self.size_t[1:4])
        data.append(self.checksum)
        return data

    def calc_checksum(self: "BLEHeader") -> int:
        return to_uint8(
            sum(
                [
                    self.version,
                    self.msg_id,
                    self.service,
                    self.sequence,
                    self.flags,
                    *self.size_t[1:4],
                ],
            ),
        )

    @classmethod
    def from_bytes(cls: Type["BLEHeader"], data: bytearray) -> "BLEHeader":
        if len(data) < 9:
            raise ValueError("Invalid data length")
        inst = cls(
            version=data[0],
            msg_id=data[1],
            service=data[2],
            sequence=data[3],
            flags=data[4],
        )
        inst.size = (data[5] << 16) + (data[6] << 8) + data[7]
        inst.checksum = data[8]
        return inst

    def __repr__(self) -> str:
        flags_str = f"<TCP: {BLEFLAGS(self.tcp_flag).name}, FRAG: {FRAGFLAGS(self.frag_flag).name}>"
        return "<BLEHeader: version={}, msg_id={}, service={}, sequence={}, flags={}, size={}, checksum={}>".format(
            self.version,
            self.msg_id,
            self.service,
            self.sequence,
            flags_str,
            self.size,
            self.checksum,
        )

    __str__ = __repr__


@dataclass
class Message:
    header: BLEHeader
    payload: bytearray = field(default_factory=bytearray)

    def __post_init__(self: "Message") -> None:
        self.header.size = len(self.payload)

    def populate_token(self: "Message") -> None:
        if (token_file := pathlib.Path(".token")).exists():
            # insert token before payload
            self.payload.insert(0, token_file.read_bytes()[0])
            self.header.size = len(self.payload)

    def to_bytes(self: "Message") -> bytearray:
        data = self.header.to_bytes()
        for i in range(self.header.size):
            data.append(self.payload[i] & 0xFF)
        return data

    @classmethod
    def from_bytes(cls: Type["Message"], data: bytearray) -> "Message":
        header = BLEHeader.from_bytes(data)
        return cls(header, data[9 : 9 + header.size])

    def __repr__(self) -> str:
        suffix = ""
        if len(self.payload) > 32:
            suffix = f"... (+{len(self.payload)-32} bytes)"
        return "<Message: headler={}, payload={}{}({})>".format(
            self.header,
            binascii.hexlify(self.payload[:32]),
            suffix,
            len(self.payload),
        )

    __str__ = __repr__

    def __len__(self) -> int:
        return len(self.header) + len(self.payload)


def sliced(data: bytearray, n: int) -> Iterator[bytes | bytearray]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))


def detection_callback(
    _: BLEDevice,
    advertisement_data: AdvertisementData,
) -> bool:
    return SERVICE_UUID.lower() in advertisement_data.service_uuids


def handle_disconnect(_: BleakClient) -> None:
    click.secho("Device was disconnected, goodbye.")


def get_rx_char(client: BleakClient) -> BleakGATTCharacteristic:
    service = client.services.get_service(SERVICE_UUID)
    if not service:
        raise RuntimeError(f"No such service: {SERVICE_UUID}")
    rx_char = service.get_characteristic(RX_CHAR_UUID)
    if not rx_char:
        raise RuntimeError(f"No such characteristic: {RX_CHAR_UUID}")
    return rx_char


async def scan_device(
    address: typing.Optional[str] = None,
    echo: bool = True,
    exit_if_no_device: bool = True,
) -> typing.Optional[BLEDevice]:
    if echo:
        click.secho("Scanning for 30 secs, please wait...")
    device = None
    if address:
        device = await BleakScanner.find_device_by_address(address, timeout=30)
    else:
        device = await BleakScanner.find_device_by_filter(
            detection_callback, timeout=30
        )
    if not device:
        click.secho(
            f"No matched device for service: {SERVICE_UUID}",
            fg="red",
            err=True,
        )
        if exit_if_no_device:
            sys.exit(1)
    return device


async def send_msg(
    rx_char: BleakGATTCharacteristic,
    client: BleakClient,
    msg: Message,
    echo: bool = False,
    need_token: bool = True,
) -> None:
    if need_token:
        msg.populate_token()
    if echo:
        click.secho(f"Sent: {msg}, total bytes: {len(msg)}")
    await client.write_gatt_char(rx_char, msg.to_bytes())


async def send_ack_msg(
    rx_char: BleakGATTCharacteristic,
    client: BleakClient,
    payload: bytearray,
    msg_id: int,
    service_id: int = 0,
    echo: bool = False,
    need_token: bool = True,
    mtu_size: int = 512,
) -> None:
    reserved_size = 9 + 1 + 3  # msg header + token + ble header
    offset = 0
    payload_size = len(payload)
    frag_size = mtu_size - reserved_size
    frag_flag = FRAGFLAGS.FIRST_FRAG if frag_size < payload_size else FRAGFLAGS.NO_FRAG
    sequence = 0

    while offset <= payload_size:
        header = BLEHeader(
            0,
            msg_id,
            service_id,
            sequence,
            BLEFLAGS.ACK | (frag_flag << 3),
        )
        frag = payload[offset : offset + frag_size]
        msg = Message(header, frag)
        await send_msg(rx_char, client, msg, echo, need_token)
        if frag_flag in (FRAGFLAGS.LAST_FRAG, FRAGFLAGS.NO_FRAG):
            break

        sequence += 1
        offset += frag_size
        if offset + frag_size < payload_size:
            frag_flag = FRAGFLAGS.MORE_FRAG
        else:
            frag_flag = FRAGFLAGS.LAST_FRAG


def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
    msg = Message.from_bytes(data)
    click.secho(f"Received: {binascii.hexlify(data)} => {msg}")


def check_success_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
    msg = Message.from_bytes(data)
    status = msg.payload[0]
    if status != 0:
        click.secho(
            "Failed to perform service {}. Status: {}".format(
                SERVICE(abs(msg.header.service)).name, status
            ),
            err=True,
            fg="red",
        )
    if msg.header.tcp_flag == BLEFLAGS.FIN:
        click.secho("Received FIN")


async def send_cmd(
    service_id: int,
    payload: bytearray = bytearray(),
    address: typing.Optional[str] = None,
    handle_rx: RX_HANDLER = handle_msg_rx,
    echo: bool = True,
    delay: float = 0.5,
    need_token: bool = True,
) -> None:
    address = address or ENVIRON_BLE_ADDRESS
    device = await scan_device(address, echo)
    if not device:
        click.secho(f"No device found for address: {address}", err=True, fg="red")
        return

    random.seed(time.time())
    async with BleakClient(
        device,
        services=[SERVICE_UUID],
        disconnected_callback=handle_disconnect if echo else None,
        timeout=60,
    ) as client:
        await client.start_notify(TX_CHAR_UUID, handle_rx)
        rx_char = get_rx_char(client)

        mtu_size = min(client.mtu_size, 512)
        if echo:
            click.secho(f"MTU size: {mtu_size}")
        msg_id = random.randint(0, 255)

        header = BLEHeader(
            0,
            msg_id,
            service_id,
            0,
            BLEFLAGS.SYN | (FRAGFLAGS.NO_FRAG << 3),
        )
        msg = Message(header, bytearray())
        await send_msg(rx_char, client, msg, echo=echo, need_token=need_token)
        await send_ack_msg(
            rx_char,
            client,
            payload,
            msg_id + 1,
            service_id,
            echo=echo,
            need_token=need_token,
            mtu_size=mtu_size,
        )
        await asyncio.sleep(delay)


@click.group()
async def main() -> None:
    if ENVIRON_BLE_ADDRESS:
        print(
            f'Using BLE address: {ENVIRON_BLE_ADDRESS} from environment variable "BLE_ADDRESS"',  # noqa: E501
        )
    pass


@main.command("scan")
async def scan() -> None:
    click.secho("Scanning for 10 secs, please wait...")
    devices = await BleakScanner.discover(
        timeout=10,
        return_adv=True,
        service_uuids=[SERVICE_UUID],
    )
    manufacturer_vid = 0x36E9
    model_mapping = {0x00: "devb", 0x01: "pro", 0x02: "ultra"}
    family_mapping = {0x00: "CP02"}
    color_mapping = {0x00: "none", 0x01: "white", 0x02: "gray", 0x03: "black"}
    for k, (device, adv) in devices.items():
        ble_addr = device.address
        model = "unknown"
        model_byte = -1
        family = "unknown"
        family_byte = -1
        color = "none"
        color_byte = -1
        if (manufacturer_data := adv.manufacturer_data) and (
            manufacturer_vid in manufacturer_data
        ):
            data = manufacturer_data[manufacturer_vid]
            data_len = len(data)
            if data_len == 7:
                # older version advertisement data
                # 6 bytes mac address, 1 byte model
                ble_addr = ":".join(f"{b:02X}" for b in data[:6])
                model_byte = data[6]
            elif 5 <= data_len <= 6:
                # new version advertisement data
                # 3 bytes mac address, 1 byte family, 1 byte model, 1 byte color
                ble_addr_prefix = "6C:80:AB"
                ble_addr_suffix = ":".join(f"{b:02X}" for b in data[:3])
                family_byte = data[3]
                family = family_mapping.get(family_byte, "invalid")
                model_byte = data[4]
                if model_byte == 0:
                    ble_addr_prefix = "48:31:b7"
                ble_addr = ble_addr_prefix + ":" + ble_addr_suffix
                if data_len == 6:
                    color_byte = data[5]
                    color = color_mapping.get(color_byte, "invalid")
            else:
                continue
            model = model_mapping.get(model_byte, "invalid")
        print(
            f"id: {k:<36}, name: {str(adv.local_name):<11}, rssi: {adv.rssi:<3}"
            f", address: {ble_addr}, model: {model}({model_byte}), "
            f"family: {family}({family_byte}) color: {color}({color_byte})",
        )


@main.command(SERVICE.BLE_ECHO_TEST.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("message", type=str, nargs=-1)
async def ble_echo_test(address: str, message: str | tuple[str]) -> None:
    finished: bool = False
    received_data: bytearray = bytearray()

    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {msg}")

        if msg.header.tcp_flag not in (BLEFLAGS.ACK, BLEFLAGS.FIN):
            return

        nonlocal received_data, finished
        received_data.extend(msg.payload)
        finished = msg.header.tcp_flag == BLEFLAGS.FIN

    message = " ".join(message)
    click.secho(f"Sending {len(message)} bytes message")
    await send_cmd(
        SERVICE.BLE_ECHO_TEST,
        bytearray(message.encode()),
        address,
        handle_rx=handle_msg_rx,
    )
    while not finished:
        await asyncio.sleep(0)

    click.secho(f"Received {len(received_data)} bytes message")
    if received_data[0] != 0:
        click.secho(
            "Something went wrong, please check the ESP32 logs",
            err=True,
            fg="red",
        )
        return
    click.secho(f"Received message: {received_data.decode()}")


async def read_log_loop(
    address: typing.Optional[str] = None,
    handle_rx: RX_HANDLER = handle_msg_rx,
    echo: bool = True,
    delay: float = 0.5,
):
    service_id = SERVICE.GET_DEBUG_LOG
    need_token = True
    payload = bytearray()
    click.echo(f"Scanning device {address} ...")
    while True:
        device = await scan_device(address, echo, exit_if_no_device=False)
        if device is not None:
            break
    click.echo("Connecting ...")
    random.seed(time.time())
    async with BleakClient(
        device,
        services=[SERVICE_UUID],
        disconnected_callback=handle_disconnect if echo else None,
        timeout=60,
    ) as client:
        await client.start_notify(TX_CHAR_UUID, handle_rx)
        rx_char = get_rx_char(client)

        mtu_size = min(client.mtu_size, 512)
        if echo:
            click.secho(f"MTU size: {mtu_size}")
        click.echo("Reading log ...")
        while True:
            msg_id = random.randint(0, 255)

            header = BLEHeader(
                0,
                msg_id,
                service_id,
                0,
                BLEFLAGS.SYN | (FRAGFLAGS.NO_FRAG << 3),
            )
            msg = Message(header, bytearray())
            await send_msg(rx_char, client, msg, echo=echo, need_token=need_token)
            await send_ack_msg(
                rx_char,
                client,
                payload,
                msg_id + 1,
                service_id,
                echo=echo,
                need_token=need_token,
                mtu_size=mtu_size,
            )
            await asyncio.sleep(delay)


@main.command(SERVICE.GET_DEBUG_LOG.command)
@click.option("-a", "--address", type=str, default=None)
async def get_debug_log(address: str) -> None:
    def log_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        # click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        # click.echo(f"Status: {msg.payload[0]}, size: {len(msg.payload) - 1}")
        print(msg.payload[1:].decode(), end="")

    address = address or ENVIRON_BLE_ADDRESS
    echo = False
    handle_rx = log_rx
    while True:
        try:
            await read_log_loop(address, handle_rx, echo)
        except Exception as e:
            click.echo(f"Read log error: {e}, retrying...")


@main.command(SERVICE.GET_SECURE_BOOT_DIGEST.command)
@click.option("-a", "--address", type=str, default=None)
async def get_secure_boot_digest(address: str) -> None:
    expected_size = 1 + 32  # 1字节状态 + 32字节 digest
    payload = bytearray()

    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        payload.extend(msg.payload)
        print(f"Got {len(payload)} bytes {binascii.hexlify(payload)}")
        if len(payload) != expected_size:
            return
        status = payload[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return
        click.secho(
            f"Secure boot digest: {binascii.hexlify(payload[1:])}",
            fg="green",
        )

    await send_cmd(
        SERVICE.GET_SECURE_BOOT_DIGEST, bytearray(), address, handle_rx=handle_msg_rx
    )


@main.command(SERVICE.PING_MQTT_TELEMETRY.command)
@click.option("-a", "--address", type=str, default=None)
async def ping_mqtt_telemetry(address: str) -> None:
    await send_cmd(
        SERVICE.PING_MQTT_TELEMETRY,
        bytearray(),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.PING_HTTP.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("url", type=str, nargs=1)
async def ping_http(address: str, url: str) -> None:
    await send_cmd(
        SERVICE.PING_HTTP,
        bytearray(url.encode()),
        address,
        handle_rx=check_success_rx,
        delay=10,
    )


@main.command(SERVICE.SET_TEST_MODE_A.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("params", type=int, nargs=6)
async def set_test_mode_a(address: str, params: typing.Tuple[int]) -> None:
    payload = bytearray()
    for param in params:
        payload.extend(struct.pack("<I", param))
    await send_cmd(
        SERVICE.SET_TEST_MODE_A,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.SET_TEST_MODE_B.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("params", type=int, nargs=6)
async def set_test_mode_b(address: str, params: typing.Tuple[int]) -> None:
    payload = bytearray()
    for param in params:
        payload.extend(struct.pack("<I", param))
    await send_cmd(
        SERVICE.SET_TEST_MODE_B,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
async def del_fpga_config(
    address: str,
) -> None:
    payload = bytearray()
    payload.append(2)
    await send_cmd(
        SERVICE.MANAGE_FPGA_CONFIG,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
@click.argument("adc_threshold_low", type=click.IntRange(0, 255))
@click.argument("adc_threshold_high", type=click.IntRange(0, 255))
@click.argument("action_deadzone", type=click.IntRange(0, 255))
async def set_fpga_config(
    address: str,
    adc_threshold_low: int,
    adc_threshold_high: int,
    action_deadzone: int,
) -> None:
    payload = bytearray()
    payload.append(1)
    payload.append(adc_threshold_low)
    payload.append(adc_threshold_high)
    payload.append(action_deadzone)
    await send_cmd(
        SERVICE.MANAGE_FPGA_CONFIG,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
async def get_fpga_config(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        status = msg.payload[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return
        adc_threshold_low = msg.payload[1]
        adc_threshold_high = msg.payload[2]
        action_deadzone = msg.payload[3]
        click.secho(
            f"adc_threshold_low: {adc_threshold_low}, adc_threshold_high: {adc_threshold_high}, action_deadzone: {action_deadzone}",
            fg="green",
        )

    await send_cmd(
        SERVICE.MANAGE_FPGA_CONFIG, bytearray([0]), address, handle_rx=handle_msg_rx
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
async def get_power_allocator_enabled(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        status = msg.payload[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return
        enabled = msg.payload[1]
        click.secho(f"Power allocator enabled: {bool(enabled)}", fg="green")

    await send_cmd(
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        bytearray([0]),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
@click.argument("enable", type=click.Choice(["on", "off"], case_sensitive=False))
async def set_power_allocator_enabled(address: str, enable: str) -> None:
    await send_cmd(
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        bytearray([1, int(enable == "on")]),
        address,
        handle_rx=check_success_rx,
    )


@main.command()
@click.option("-a", "--address", type=str, default=None)
async def del_power_allocator_enabled(address: str) -> None:
    await send_cmd(
        SERVICE.MANAGE_POWER_ALLOCATOR_ENABLED,
        bytearray([2]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_DEVICE_UPTIME.command)
@click.option("-a", "--address", type=str, default=None)
async def uptime(address: str) -> None:
    await send_cmd(SERVICE.GET_DEVICE_UPTIME, bytearray(), address)


@main.command(SERVICE.GET_DEVICE_BLE_ADDR.command)
@click.option("-a", "--address", type=str, default=None)
async def ble_addr(address: str) -> None:
    await send_cmd(SERVICE.GET_DEVICE_BLE_ADDR, bytearray(), address)


@main.command(SERVICE.GET_DEVICE_WIFI_ADDR.command)
@click.option("-a", "--address", type=str, default=None)
async def wifi_addr(address: str) -> None:
    await send_cmd(SERVICE.GET_DEVICE_WIFI_ADDR, bytearray(), address)


@main.command(SERVICE.SET_WIFI_SSID_AND_PASSWORD.command)
@click.argument("ssid", type=str)
@click.argument("password", type=str, default="")
@click.option("-n", "--no-passwd", is_flag=True, default=False)
@click.option("-a", "--address", type=str, default=None)
async def wifi_ssid_and_password(
    ssid: str, password: str, no_passwd: bool, address: str
) -> None:
    ssid_bytes = ssid.encode("utf-8")
    if len(ssid_bytes) > 255:
        raise ValueError("SSID is too long")
    if len(password) > 63:
        raise ValueError("Password is too long")
    await send_cmd(
        SERVICE.SET_WIFI_SSID_AND_PASSWORD,
        bytearray(
            bytes([len(ssid_bytes), int(not no_passwd)])
            + ssid_bytes
            + password.encode("utf-8")
        ),
        address,
    )


@main.command(SERVICE.GET_WIFI_RECORDS.command)
@click.option("-a", "--address", type=str, default=None)
async def get_wifi_records(address: str) -> None:
    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if len(msg.payload) < 2:
            return
        status = struct.unpack_from("B", msg.payload, 0)[0]
        if status != 0:
            click.secho("Something went wrong, please check the ESP32 logs")
            return
        ssid_count = struct.unpack_from("B", msg.payload, 1)[0]
        if ssid_count == 0:
            click.secho("No SSID found", fg="green")
            return
        offset = 2
        ssid_list = []
        for _i in range(ssid_count):
            ssid_length = struct.unpack_from("B", msg.payload, offset)[0]
            offset += 1
            ssid = struct.unpack_from(f"{ssid_length}s", msg.payload, offset)[0]
            offset += ssid_length
            auth_mode = struct.unpack_from("B", msg.payload, offset)[0]
            offset += 1

            ssid_list.append(
                {
                    "ssid_length": ssid_length,
                    "ssid": ssid.decode("utf-8"),
                    "auth_mode": WifiAuthMode.get_name_from_value(auth_mode),
                },
            )

        for item in ssid_list:
            click.secho(
                f"SSID: {item['ssid']} - Auth: {item['auth_mode']}",
                fg="green",
            )

    await send_cmd(SERVICE.GET_WIFI_RECORDS, bytearray(), address, handle_rx=handle_rx)


@main.command(SERVICE.OPERATE_WIFI_RECORD.command)
@click.argument(
    "operation",
    type=click.Choice(["create", "update", "delete"], case_sensitive=False),
    nargs=1,
)
@click.argument("ssid", type=str, nargs=1)
@click.argument("password", type=str, nargs=1, default="")
@click.option("-a", "--address", type=str, default=None)
async def operate_wifi_record(
    address: str, operation: str, ssid: str, password: str
) -> None:
    operation_v = 0
    match operation:
        case "create":
            operation_v = 0
        case "update":
            operation_v = 1
        case "delete":
            operation_v = 2
    payload = bytearray()
    payload.append(operation_v)
    payload.append(len(ssid.encode("utf-8")))
    payload.append(bool(password))
    payload.extend(ssid.encode("utf-8"))
    payload.extend(password.encode("utf-8"))
    await send_cmd(
        SERVICE.OPERATE_WIFI_RECORD, payload, address, handle_rx=check_success_rx
    )


@main.command(SERVICE.GET_DEVICE_SERIAL_NO.command)
@click.option("-a", "--address", type=str, default=None)
async def serial_number(address: str) -> None:
    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if len(msg.payload) < 2:
            return
        click.secho(f"Serial number: {msg.payload[1:].decode()}")

    await send_cmd(SERVICE.GET_DEVICE_SERIAL_NO, bytearray(), address, handle_rx)


@main.command(SERVICE.SWITCH_DEVICE.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("device_switch", type=click.Choice(["open", "close"]))
async def switch_device(address: str, device_switch: str) -> None:
    device_switch_v = 1
    if device_switch == "close":
        device_switch_v = 0
    await send_cmd(SERVICE.SWITCH_DEVICE, bytearray([device_switch_v]), address)


@main.command(SERVICE.GET_DEVICE_SWITCH.command)
@click.option("-a", "--address", type=str, default=None)
async def get_device_switch(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            return
        v = msg.payload[1]
        if v == 1:
            click.secho("Device is open", fg="green")
        else:
            click.secho("Device is closed", fg="yellow")

    await send_cmd(
        SERVICE.GET_DEVICE_SWITCH, bytearray(), address, handle_rx=handle_msg_rx
    )


@main.command(SERVICE.CONFIRM_OTA.command)
@click.argument("validation_hash", type=str)
@click.option("-a", "--address", type=str, default=None)
async def confirm(validation_hash: str, address: str) -> None:
    await send_cmd(SERVICE.CONFIRM_OTA, bytearray.fromhex(validation_hash), address)


@main.command(SERVICE.SCAN_WIFI.command)
@click.option("-a", "--address", type=str, default=None)
async def wifi_scan(address: str) -> None:
    address = address or ENVIRON_BLE_ADDRESS
    print(f"Scanning for {address}...")
    finished = False
    received_data = bytearray()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        print(f"Received: {binascii.hexlify(data)}")
        nonlocal finished
        msg = Message.from_bytes(data)
        if msg.payload == b"\0":
            return
        received_data.extend(msg.payload)
        frag_flag = msg.header.frag_flag
        if frag_flag in (FRAGFLAGS.NO_FRAG, FRAGFLAGS.LAST_FRAG):
            finished = True

    device = await scan_device(address)
    if not device:
        click.secho(f"Device {address} not found", err=True, fg="red")
        return
    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(TX_CHAR_UUID, handle_rx)
        rx_char = get_rx_char(client)

        msg_id = 0
        header = BLEHeader(
            0,
            msg_id,
            SERVICE.SCAN_WIFI,
            0,
            BLEFLAGS.SYN | (FRAGFLAGS.NO_FRAG << 3),
        )
        msg = Message(header)
        await send_msg(rx_char, client, msg)

        await send_ack_msg(
            rx_char,
            client,
            bytearray(),
            msg_id + 1,
            service_id=SERVICE.SCAN_WIFI,
        )
        while not finished:
            await asyncio.sleep(0)

        print("parse wifi scan result...")
        scan_status = struct.unpack_from("B", received_data, 0)[0]
        if scan_status != 0:
            click.secho("Something went wrong, please check the ESP32 logs")
            return
        ssid_count = struct.unpack_from("B", received_data, 1)[0]
        offset = 2
        ssid_list = []
        for _ in range(ssid_count):
            ssid_length = struct.unpack_from("B", received_data, offset)[0]
            offset += 1
            ssid = struct.unpack_from(f"{ssid_length}s", received_data, offset)[0]
            offset += ssid_length
            rssi = struct.unpack_from("b", received_data, offset)[0]
            offset += 1
            auth_mode = struct.unpack_from("B", received_data, offset)[0]
            offset += 1
            stored = struct.unpack_from("B", received_data, offset)[0]
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


@main.command(SERVICE.SET_WIFI_SSID.command)
@click.argument("ssid", type=str)
@click.option("-a", "--address", type=str, default=None)
async def wifi_ssid(ssid: str, address: str) -> None:
    await send_cmd(SERVICE.SET_WIFI_SSID, bytearray(ssid.encode("utf-8")), address)


@main.command(SERVICE.SET_WIFI_PASSWORD.command)
@click.argument("password", type=str)
@click.option("-a", "--address", type=str, default=None)
async def wifi_password(password: str, address: str) -> None:
    await send_cmd(
        SERVICE.SET_WIFI_PASSWORD,
        bytearray(password.encode("utf-8")),
        address,
    )


@main.command(SERVICE.RESET_WIFI.command)
@click.option("-a", "--address", type=str, default=None)
async def wifi_reset(address: str) -> None:
    await send_cmd(SERVICE.RESET_WIFI, bytearray(), address)


@main.command(SERVICE.GET_WIFI_STATUS.command)
@click.option("-a", "--address", type=str, default=None)
async def wifi_query(address: str) -> None:
    def deserialize_ssid_detail(serialized_data: bytes | bytearray) -> dict[str, Any]:
        # Unpack fixed-size prefix (bssid, channel, rssi, flags)
        bssid, channel, rssi, flags = struct.unpack_from("6sBbB", serialized_data, 0)
        # Decode the SSID part, which may be variable length up to 32 chars plus
        # null terminator
        ssid = serialized_data[9:].split(b"\x00", 1)[0].decode("utf-8")

        # Extract bit fields from flags
        phy_11b = flags & 0x01
        phy_11g = (flags >> 1) & 0x01
        phy_11n = (flags >> 2) & 0x01
        phy_lr = (flags >> 3) & 0x01
        phy_11ax = (flags >> 4) & 0x01
        wps = (flags >> 5) & 0x01
        ftm_responder = (flags >> 6) & 0x01
        ftm_initiator = (flags >> 7) & 0x01

        return {
            "bssid": binascii.hexlify(bssid),
            "channel": channel,
            "rssi": rssi,
            "phy_11b": phy_11b,
            "phy_11g": phy_11g,
            "phy_11n": phy_11n,
            "phy_lr": phy_lr,
            "phy_11ax": phy_11ax,
            "wps": wps,
            "ftm_responder": ftm_responder,
            "ftm_initiator": ftm_initiator,
            "ssid": ssid,
        }

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size > 1 and msg.payload[0] == 0:
            detail = deserialize_ssid_detail(msg.payload[1:])
            pprint(detail)

    await send_cmd(SERVICE.GET_WIFI_STATUS, bytearray(), address, handle_rx)


def version_rx_handler(service: SERVICE, name: str) -> RX_HANDLER:
    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != service:
            return
        if msg.header.size < 2:
            return

        status = struct.unpack_from("<B", msg.payload)[0]
        if status != 0:
            click.secho("Something went wrong, please check the ESP32 logs")
            return

        major, minor, patch = struct.unpack_from("<BBB", msg.payload, 1)
        click.secho(f"{name} version: {major}.{minor}.{patch}", fg="green")

    return handle_rx


@main.command(SERVICE.GET_AP_VERSION.command)
@click.option("-a", "--address", type=str, default=None)
async def get_ap_version(address: str) -> None:
    await send_cmd(
        SERVICE.GET_AP_VERSION,
        address=address,
        handle_rx=version_rx_handler(SERVICE.GET_AP_VERSION, "AP"),
    )


@main.command(SERVICE.GET_BP_VERSION.command)
@click.option("-a", "--address", type=str, default=None)
async def get_bp_version(address: str) -> None:
    await send_cmd(
        SERVICE.GET_BP_VERSION,
        address=address,
        handle_rx=version_rx_handler(SERVICE.GET_BP_VERSION, "BP"),
    )


@main.command(SERVICE.GET_FPGA_VERSION.command)
@click.option("-a", "--address", type=str, default=None)
async def get_fpga_version(address: str) -> None:
    await send_cmd(
        SERVICE.GET_FPGA_VERSION,
        address=address,
        handle_rx=version_rx_handler(SERVICE.GET_FPGA_VERSION, "FPGA"),
    )


@main.command(SERVICE.GET_ZRLIB_VERSION.command)
@click.option("-a", "--address", type=str, default=None)
async def get_zrlib_version(address: str) -> None:
    await send_cmd(
        SERVICE.GET_ZRLIB_VERSION,
        address=address,
        handle_rx=version_rx_handler(SERVICE.GET_ZRLIB_VERSION, "ZRLib"),
    )


@main.command(SERVICE.GET_POWER_SUPPLY_STATUS.command)
@click.option("-a", "--address", type=str, default=None)
async def get_power_supply_status(address: str) -> None:
    await send_cmd(SERVICE.GET_POWER_SUPPLY_STATUS, address=address)


@main.command(SERVICE.GET_CHARGING_STATUS.command)
@click.option("-a", "--address", type=str, default=None)
async def get_charging_status(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != SERVICE.GET_CHARGING_STATUS:
            return
        if msg.header.tcp_flag != BLEFLAGS.ACK:
            return

        status, charging_status = struct.unpack_from("<2B", msg.payload)
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return

        for i, port_name in enumerate(PORTS):
            charging = (charging_status >> i) & 0x01 == 1
            if charging:
                click.secho(f"Charging port: {port_name}", fg="green")
        if charging_status == 0:
            click.secho("Not charging", fg="green")

    await send_cmd(
        SERVICE.GET_CHARGING_STATUS,
        address=address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.GET_POWER_STATISTICS.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def get_power_statistics(port: str, address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size != 5:
            return

        status, protocol, current, voltage, temperature = struct.unpack(
            "<5B",
            msg.payload,
        )
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return

        actual_current = current / 32.0
        actual_voltage = voltage / 8.0
        protocol_item = FastChargingProtocol(protocol)
        click.secho(
            f"Protocol: {protocol_item.name}, Current: {actual_current}A, "
            f"Voltage: {actual_voltage}V, Temperature: {temperature}°C",
            fg="green",
        )

    await send_cmd(
        SERVICE.GET_POWER_STATISTICS,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.GET_POWER_HISTORICAL_STATS.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.argument("offset", type=int)
@click.option("-a", "--address", type=str, default=None)
async def get_power_historical_stats(port: str, offset: int, address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size < 2:
            return

        status = struct.unpack_from("<B", msg.payload)[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return

        offset_ = struct.unpack_from("<H", msg.payload, 1)[0]
        click.secho("Points:", fg="green")
        for i in range((msg.header.size - 3) // 2):
            current = struct.unpack_from("<B", msg.payload, 3 + 2 * i)[0]
            voltage = struct.unpack_from("<B", msg.payload, 4 + 2 * i)[0]
            click.secho(
                f"\t[{i}] Current: {current / 32.0:.2f}A, Voltage: {voltage / 8.0:.2f}V",
                fg="green",
            )
        click.secho(f"Next offset: {offset_}", fg="green")

    byte_array = bytearray([PORTS.index(port) & 0xFF])
    byte_array.extend(struct.pack("<H", offset))
    await send_cmd(
        SERVICE.GET_POWER_HISTORICAL_STATS,
        byte_array,
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.SET_PORT_PRIORITY.command)
@click.argument("priority", type=int)
@click.option("-a", "--address", type=str, default=None)
async def set_port_priority(priority: int, address: str) -> None:
    byte_array = bytearray([])
    while priority > 0:
        val = int(priority % 10)
        priority = int(priority / 10)
        byte_array.append(val)
    await send_cmd(SERVICE.SET_PORT_PRIORITY, byte_array, address)


@main.command(SERVICE.GET_PORT_PRIORITY.command)
@click.option("-a", "--address", type=str, default=None)
async def get_port_priority(address: str) -> None:
    await send_cmd(SERVICE.GET_PORT_PRIORITY, bytearray(), address)


@main.command(SERVICE.GET_CHARGING_STRATEGY.command)
@click.option("-a", "--address", type=str, default=None)
async def get_charging_strategy(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != SERVICE.GET_CHARGING_STRATEGY:
            return
        if msg.header.size < 2:
            return

        status, strategy = struct.unpack_from("<BB", msg.payload)
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return

        click.secho(f"Charging strategy: {ChargingStrategy(strategy).name}", fg="green")

    await send_cmd(
        SERVICE.GET_CHARGING_STRATEGY,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.GET_PORT_PD_STATUS.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def get_pd_status(address: str, port: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != SERVICE.GET_PORT_PD_STATUS:
            return
        if msg.header.size < 2:
            return

        status = struct.unpack_from("<B", msg.payload)[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return

        pd_status = ClientPDStatus.from_bytes(msg.payload[1:])
        click.secho(f"Port: {port}, PD status: {pd_status}", fg="green")

    await send_cmd(
        SERVICE.GET_PORT_PD_STATUS,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.GET_ALL_POWER_STATISTICS.command)
@click.option("-a", "--address", type=str, default=None)
async def get_all_power_statistics(address: str) -> None:
    # 1 字节状态 + 5 个充电端口的 8 字节数据
    expected_size = 1 + 5 * 8
    payload = bytearray()

    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        payload.extend(msg.payload)
        if len(payload) != expected_size:
            return
        status = payload[0]
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return
        offset = 1
        size = 8
        for i in range(5):
            result = struct.unpack(
                "<4B2H",
                payload[offset : offset + size],
            )
            protocol = result[0]
            current = result[1]
            voltage = result[2]
            temperature = result[3]
            battery_last_full_charge_capacity = result[4]
            battery_present_capacity = result[5]
            actual_current = current / 32.0
            actual_voltage = voltage / 8.0
            protocol_item = FastChargingProtocol(protocol)
            click.secho(
                f"Port: {i}, Protocol: {protocol_item.name}, Current: {actual_current}A, "
                f"Voltage: {actual_voltage}V, Temperature: {temperature}°C, "
                f"Last Full Cap: {battery_last_full_charge_capacity * 100}mWh, "
                f"Present Cap: {battery_present_capacity * 100}mWh, ",
                fg="green",
            )
            offset += size

    await send_cmd(
        SERVICE.GET_ALL_POWER_STATISTICS,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.GET_START_CHARGE_TIMESTAMP.command)
@click.option("-a", "--address", type=str, default=None)
async def get_start_charge_timestamp(address: str) -> None:
    def handle_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            return
        status, timestamp = struct.unpack("<BL", msg.payload)
        if status != 0:
            click.secho(
                "Something went wrong, please check the ESP32 logs",
                err=True,
                fg="red",
            )
            return
        s = ""
        if timestamp > 0:
            s = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
        click.secho(
            f"Start charge timestamp: {timestamp} '{hex(timestamp)}' ({s})", fg="green"
        )

    await send_cmd(
        SERVICE.GET_START_CHARGE_TIMESTAMP, bytearray(), address, handle_rx=handle_rx
    )


@main.command(SERVICE.TOGGLE_PORT_POWER.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def toggle_port_power(port: str, address: str) -> None:
    await send_cmd(
        SERVICE.TOGGLE_PORT_POWER,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
    )


@main.command(SERVICE.ASSOCIATE_DEVICE.command)
@click.argument("passcode", type=str, default="0000")
@click.option("-a", "--address", type=str, default=None)
@click.option("--reset-data", is_flag=True, default=False)
async def associate_device(passcode: str, reset_data: bool, address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.flags != BLEFLAGS.FIN:
            return
        if msg.header.size > 2:
            return

        status = struct.unpack_from("B", msg.payload, 0)[0]
        if status != 0:
            click.secho(
                f"Failed to associate device, please check the passcode: {passcode}",
                err=True,
                fg="red",
            )
            return
        token = struct.unpack_from("B", msg.payload, 1)[0]
        with open(".token", "wb") as f:
            click.secho(f"Got token: 0x{token:02x}({token})", fg="green")
            f.write(token.to_bytes(1, "big"))

    if not (len(passcode) == 4 and passcode.isnumeric()):
        click.secho("Passcode must be 4 digits", err=True, fg="red")
        return
    payload = bytearray(int(d) & 0xFF for d in passcode)
    payload.append(int(not reset_data))
    await send_cmd(
        SERVICE.ASSOCIATE_DEVICE,
        payload,
        address,
        handle_msg_rx,
        need_token=False,
    )


@main.command(SERVICE.RESET_DEVICE.command)
@click.option("-a", "--address", type=str, default=None)
async def reset(address: str) -> None:
    await send_cmd(SERVICE.RESET_DEVICE, bytearray(), address)


@main.command(SERVICE.PERFORM_WIFI_OTA.command)
@click.argument("version", type=str)
@click.option("-a", "--address", type=str, default=None)
async def wifi_ota(version: str, address: str) -> None:
    matched = VERSION_PAT.match(version)
    if not matched:
        click.secho("Invalid version", err=True, fg="red")
        return
    major, minor, patch = matched.groups()
    await send_cmd(
        SERVICE.PERFORM_WIFI_OTA,
        bytearray([int(major), int(minor), int(patch)]),
        address,
    )


@main.command(SERVICE.GET_WIFI_OTA_PROGRESS.command)
@click.option("-a", "--address", type=str, default=None)
@click.option("-p", "--period", type=int, default=0)
async def wifi_ota_progress(address: str, period: int) -> None:
    previous_progress = 0
    current_progress = 0
    t = 0

    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        nonlocal previous_progress, current_progress, t
        msg = Message.from_bytes(data)
        status = struct.unpack_from("B", msg.payload, 0)[0]
        if status != 0:
            click.secho("Failed to perform OTA, please check the ESP32 logs")
            return

        if msg.header.size < 5:
            return

        current_progress = struct.unpack_from("<i", msg.payload, 1)[0]
        curr_t = time.time()
        speed = (current_progress - previous_progress) / (curr_t - t) / 1024
        click.secho(
            f"[{int(curr_t)}] Progress: {current_progress} Speed: {speed:.2f} KiB/s",
        )
        previous_progress = current_progress
        t = curr_t

        if len(msg.payload) > 5:
            failed_code = struct.unpack_from("<i", msg.payload, 5)[0]
            click.secho(f"Failed: {failed_code}", err=True, fg="red")

    t = int(time.time())
    while True:
        await send_cmd(
            SERVICE.GET_WIFI_OTA_PROGRESS,
            bytearray(),
            address,
            handle_msg_rx,
            echo=period == 0,
        )
        if period == 0:
            break
        await asyncio.sleep(period)


@main.command(SERVICE.REBOOT_DEVICE.command)
@click.option("-a", "--address", type=str, default=None)
async def reboot(address: str) -> None:
    await send_cmd(SERVICE.REBOOT_DEVICE, bytearray(), address)


def cast_to_int(ctx, params, value) -> int:
    try:
        if isinstance(value, str) and value.lower().startswith("0x"):
            value = int(value, 16)  # Attempt to parse as hex if it starts with '0x'
        else:
            value = int(value)  # Fallback to regular integer parsing
    except (ValueError, TypeError):
        raise click.BadParameter("Must be an integer")  # noqa: B904
    return value


@main.command(SERVICE.SET_CHARGING_STRATEGY.command)
@click.argument("strategy", type=click.Choice(["fast", "slow"]))
@click.option("-a", "--address", type=str, default=None)
async def set_charging_strategy(strategy: str, address: str) -> None:
    payload = bytearray([0])
    if strategy.lower() == "slow":
        payload[0] = 1
    await send_cmd(
        SERVICE.SET_CHARGING_STRATEGY, payload, address, handle_rx=check_success_rx
    )



@main.command(SERVICE.SET_DISPLAY_INTENSITY.command)
@click.argument("intensity", callback=cast_to_int)
@click.option("-a", "--address", type=str, default=None)
async def set_display_intensity(intensity: int, address: str) -> None:
    await send_cmd(
        SERVICE.SET_DISPLAY_INTENSITY,
        bytearray([intensity % 256]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.SET_DISPLAY_MODE.command)
@click.argument("mode", type=click.Choice(["off", "manual", "power_meter"]))
@click.option("-a", "--address", type=str, default=None)
async def set_display_mode(mode: str, address: str) -> None:
    mode_byte = 0
    match mode:
        case "off":
            mode_byte = 0
        case "manual":
            mode_byte = 1
        case "power_meter":
            mode_byte = 2
    await send_cmd(
        SERVICE.SET_DISPLAY_MODE,
        bytearray([mode_byte]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_DISPLAY_INTENSITY.command)
@click.option("-a", "--address", type=str, default=None)
async def get_display_intensity(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size != 2:
            return
        status, intensity = struct.unpack("<BB", msg.payload)
        if status != 0:
            click.secho(
                f"Error getting display intensity: {status}", err=True, fg="red"
            )
            return

        click.secho(f"Display intensity: {intensity}({hex(intensity)})", fg="green")

    await send_cmd(
        SERVICE.GET_DISPLAY_INTENSITY, bytearray(), address, handle_rx=handle_msg_rx
    )


@main.command(SERVICE.GET_DISPLAY_MODE.command)
@click.option("-a", "--address", type=str, default=None)
async def get_display_mode(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size != 2:
            return
        status, mode = struct.unpack("<BB", msg.payload)
        if status != 0:
            click.secho(f"Error getting display mode: {status}", err=True, fg="red")
            return

        mode_mapping = {0: "OFF", 1: "MANUAL", 2: "POWER_METER"}
        click.secho(f"Display mode: {mode}({mode_mapping.get(mode)})", fg="green")

    await send_cmd(
        SERVICE.GET_DISPLAY_MODE, bytearray(), address, handle_rx=handle_msg_rx
    )


@main.command(SERVICE.SET_DISPLAY_FLIP.command)
@click.argument("flip", type=click.Choice(["normal", "flip"]))
@click.option("-a", "--address", type=str, default=None)
async def set_display_flip(flip: str, address: str) -> None:
    flip_byte = 0
    match flip:
        case "normal":
            flip_byte = 0
        case "flip":
            flip_byte = 1

    await send_cmd(
        SERVICE.SET_DISPLAY_FLIP,
        bytearray([flip_byte]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_DISPLAY_FLIP.command)
@click.option("-a", "--address", type=str, default=None)
async def get_display_flip(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size != 2:
            return
        status, flip = struct.unpack("<BB", msg.payload)
        if status != 0:
            click.secho(f"Error getting display flip: {status}", err=True, fg="red")
            return

        flip_mapping = {0: "normal", 1: "flip"}
        click.secho(f"Display flip: {flip}({flip_mapping.get(flip)})", fg="green")

    await send_cmd(
        SERVICE.GET_DISPLAY_FLIP,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.SET_DISPLAY_CONFIG.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("intensity", callback=cast_to_int)
@click.argument("flip", type=click.Choice(["normal", "flip"]))
@click.argument("mode", type=click.Choice(["off", "manual", "power_meter"]))
@click.argument("mode_param", callback=cast_to_int)
async def set_display_config(
    address: str, intensity: int, flip: str, mode: str, mode_param: int
) -> None:
    flip_byte = 0
    match flip:
        case "normal":
            flip_byte = 0
        case "flip":
            flip_byte = 1
    mode_byte = 0
    match mode:
        case "off":
            mode_byte = 0
        case "manual":
            mode_byte = 1
        case "power_meter":
            mode_byte = 2
    payload = bytearray([intensity % 256, flip_byte, mode_byte])
    payload.extend(struct.pack("<H", mode_param))
    print(len(payload))
    await send_cmd(
        SERVICE.SET_DISPLAY_CONFIG,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.SET_DISPLAY_STATE.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("state", type=click.Choice(["off", "on_full", "reset"]))
async def set_display_state(address: str, state: str) -> None:
    state_mapping = {"off": 0, "on_full": 1, "reset": 2}
    payload = bytearray()
    payload.append(state_mapping[state])
    await send_cmd(
        SERVICE.SET_DISPLAY_STATE,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_DISPLAY_STATE.command)
@click.option("-a", "--address", type=str, default=None)
async def get_display_state(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size != 2:
            return
        status, state = struct.unpack(">BB", msg.payload)
        if status != 0:
            click.secho(f"Error getting display flip: {status}", err=True, fg="red")
            return

        state_mapping = {0: "off", 1: "on_full", 2: "reset"}
        click.secho(f"Display state: {state}({state_mapping.get(state)})", fg="green")

    payload = bytearray()
    await send_cmd(
        SERVICE.GET_DISPLAY_STATE,
        payload,
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.TURN_ON_PORT.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def turn_on_port(port: str, address: str) -> None:
    await send_cmd(
        SERVICE.TURN_ON_PORT,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.TURN_OFF_PORT.command)
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def turn_off_port(port: str, address: str) -> None:
    await send_cmd(
        SERVICE.TURN_OFF_PORT,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_DEVICE_MODEL.command)
@click.option("-a", "--address", type=str, default=None)
async def get_device_model(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != SERVICE.GET_DEVICE_MODEL:
            return
        if msg.header.size < 2:
            return

        status, model = struct.unpack(f"<B{msg.header.size - 1}s", msg.payload)
        if status != 0:
            click.secho(f"Error getting device model: {status}", err=True, fg="red")
            return

        click.secho(f"Device model: {model.decode('utf-8')}", fg="green")

    await send_cmd(
        SERVICE.GET_DEVICE_MODEL,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.PUSH_LICENSE.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("license_file", type=click.Path(exists=True), nargs=1)
async def push_license(address: str, license_file: str) -> None:
    payload = bytearray()
    payload.extend(pathlib.Path(license_file).read_bytes())
    await send_cmd(
        SERVICE.PUSH_LICENSE,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_BLE_RSSI.command)
@click.option("-a", "--address", type=str, default=None)
async def get_ble_rssi(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != SERVICE.GET_BLE_RSSI:
            return
        if msg.header.size < 2:
            return

        status, rssi = struct.unpack("<Bb", msg.payload)
        if status != 0:
            click.secho(f"Error getting ble rssi: {status}", err=True, fg="red")
            return

        click.secho(f"RSSI: {rssi}", fg="green")

    await send_cmd(SERVICE.GET_BLE_RSSI, bytearray(), address, handle_rx=handle_msg_rx)


@main.command("set-port-feature")
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.argument("feature", type=click.Choice(PORT_FEATURES, case_sensitive=False))
@click.argument("enable", type=click.Choice(["on", "off"], case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def set_port_feature(port: str, feature: str, enable: str, address: str) -> None:
    match feature:
        case "overclock":
            cmd = SERVICE.SET_PORT_OVERCLOCK_FEATURE
        case "tfcp":
            cmd = SERVICE.SET_PORT_TFCP_FEATURE
        case "ufcs":
            cmd = SERVICE.SET_PORT_UFCS_FEATURE
        case _:
            raise click.ClickException(f"Unknown feature: {feature}")

    await send_cmd(
        cmd,
        bytearray([PORTS.index(port) & 0xFF, enable == "on"]),
        address,
        handle_rx=check_success_rx,
    )
    click.secho("Done", fg="green")


@main.command("get-port-feature")
@click.argument("port", type=click.Choice(PORTS, case_sensitive=False))
@click.argument("feature", type=click.Choice(PORT_FEATURES, case_sensitive=False))
@click.option("-a", "--address", type=str, default=None)
async def get_port_feature(port: str, feature: str, address: str) -> None:
    match feature:
        case "overclock":
            cmd = SERVICE.GET_PORT_OVERCLOCK_FEATURE
        case "tfcp":
            cmd = SERVICE.GET_PORT_TFCP_FEATURE
        case "ufcs":
            cmd = SERVICE.GET_PORT_UFCS_FEATURE
        case _:
            raise click.ClickException(f"Unknown feature: {feature}")

    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if abs(msg.header.service) != cmd:
            return
        if msg.header.flags != BLEFLAGS.ACK:
            return

        status, enabled = struct.unpack("<B?", msg.payload)
        if status != 0:
            click.secho(
                f"Error getting port {feature} status: {status}",
                err=True,
                fg="red",
            )
            return

        click.secho(f"Port {port} {feature} enabled: {enabled}", fg="green")

    await send_cmd(
        cmd,
        bytearray([PORTS.index(port) & 0xFF]),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.SET_STATIC_ALLOCATOR.command)
@click.argument("identifier", type=click.IntRange(0, 0xFFFF))
@click.argument("version", type=click.IntRange(0, 0xFF))
@click.argument("file", type=click.Path(exists=True, readable=True), nargs=1)
@click.option("-a", "--address", type=str, default=None)
async def set_static_allocator(
    identifier: int, version: int, file: str, address: str
) -> None:
    payload = bytearray()
    payload.extend(struct.pack("<H", identifier))
    payload.append(version & 0xFF)
    power_allocation_chart = json.loads(pathlib.Path(file).read_text())
    for allocation in power_allocation_chart:
        if len(allocation) != 5:
            raise ValueError("invalid allocation")
        for n in allocation:
            payload.extend(struct.pack("<H", n))
    await send_cmd(
        SERVICE.SET_STATIC_ALLOCATOR,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_STATIC_ALLOCATOR.command)
@click.option("-a", "--address", type=str, default=None)
async def get_static_allocator(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        status = msg.payload[0]
        if status != 0:
            click.secho(
                "Failed to perform service {}. Status: {}".format(
                    SERVICE(abs(msg.header.service)).name, status
                ),
                err=True,
                fg="red",
            )
            return
        identifier, version = struct.unpack("<HB", msg.payload[1:])
        click.secho(f"identifier: {identifier}, version: {version}", fg="green")

    await send_cmd(
        SERVICE.GET_STATIC_ALLOCATOR,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@dataclass
class PortConfig:
    version: int
    is_tfcp_enabled: bool
    is_pe_enabled: bool
    is_qc2p0_enabled: bool
    is_qc3p0_enabled: bool
    is_qc3plus_enabled: bool
    is_afc_enabled: bool
    is_fcp_enabled: bool
    is_hvscp_enabled: bool
    is_lvscp_enabled: bool
    is_sfcp_enabled: bool
    is_apple_enabled: bool
    is_samsung_enabled: bool
    is_ufcs_enabled: bool
    is_pd_enabled: bool

    def serialize(self) -> bytearray:
        data = bytearray([self.version])
        n = 0
        n |= self.is_tfcp_enabled << 0
        n |= self.is_pe_enabled << 1
        n |= self.is_qc2p0_enabled << 2
        n |= self.is_qc3p0_enabled << 3
        n |= self.is_qc3plus_enabled << 4
        n |= self.is_afc_enabled << 5
        n |= self.is_fcp_enabled << 6
        n |= self.is_hvscp_enabled << 7
        n |= self.is_lvscp_enabled << 8
        n |= self.is_sfcp_enabled << 9
        n |= self.is_apple_enabled << 10
        n |= self.is_samsung_enabled << 11
        n |= self.is_ufcs_enabled << 12
        n |= self.is_pd_enabled << 13
        data.extend(struct.pack("<H", n))
        return data

    @classmethod
    def deserialize(cls, data: bytearray) -> "PortConfig":
        version, n1, n2 = struct.unpack("<BBB", data)
        return cls(
            version=version,
            is_tfcp_enabled=((n1 >> 0) & 0x01) != 0,
            is_pe_enabled=((n1 >> 1) & 0x01) != 0,
            is_qc2p0_enabled=((n1 >> 2) & 0x01) != 0,
            is_qc3p0_enabled=((n1 >> 3) & 0x01) != 0,
            is_qc3plus_enabled=((n1 >> 4) & 0x01) != 0,
            is_afc_enabled=((n1 >> 5) & 0x01) != 0,
            is_fcp_enabled=((n1 >> 6) & 0x01) != 0,
            is_hvscp_enabled=((n1 >> 7) & 0x01) != 0,
            is_lvscp_enabled=((n2 >> 0) & 0x01) != 0,
            is_sfcp_enabled=((n2 >> 1) & 0x01) != 0,
            is_apple_enabled=((n2 >> 2) & 0x01) != 0,
            is_samsung_enabled=((n2 >> 3) & 0x01) != 0,
            is_ufcs_enabled=((n2 >> 4) & 0x01) != 0,
            is_pd_enabled=((n2 >> 5) & 0x01) != 0,
        )

    def __str__(self) -> str:
        return f"""version: {self.version}
        is_tfcp_enabled: {self.is_tfcp_enabled}
        is_pe_enabled: {self.is_pe_enabled}
        is_qc2p0_enabled: {self.is_qc2p0_enabled}
        is_qc3p0_enabled: {self.is_qc3p0_enabled}
        is_qc3plus_enabled: {self.is_qc3plus_enabled}
        is_afc_enabled: {self.is_afc_enabled}
        is_fcp_enabled: {self.is_fcp_enabled}
        is_hvscp_enabled: {self.is_hvscp_enabled}
        is_lvscp_enabled: {self.is_lvscp_enabled}
        is_sfcp_enabled: {self.is_sfcp_enabled}
        is_apple_enabled: {self.is_apple_enabled}
        is_samsung_enabled: {self.is_samsung_enabled}
        is_ufcs_enabled: {self.is_ufcs_enabled}
        is_pd_enabled: {self.is_pd_enabled}
        """


@main.command(SERVICE.SET_PORT_CONFIG.command)
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
@click.option("-a", "--address", type=str, default=None)
async def set_port_config(
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
    address: str,
) -> None:
    payload = bytearray()
    set_ports = 0
    for i, port in enumerate(PORTS):
        if port not in ports:
            payload.extend([0, 0, 0])
            continue
        set_ports |= 1 << i
        config = PortConfig(
            0,
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
        )
        payload.extend(config.serialize())
    for _ in range(8 - len(PORTS)):
        payload.extend([0, 0, 0])
    payload.insert(0, set_ports)
    await send_cmd(
        SERVICE.SET_PORT_CONFIG,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_PORT_CONFIG.command)
@click.option("-a", "--address", type=str, default=None)
async def get_port_config(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        status = msg.payload[0]
        if status != 0:
            click.secho(
                "Failed to perform service {}. Status: {}".format(
                    SERVICE(abs(msg.header.service)).name, status
                ),
                err=True,
                fg="red",
            )
            return
        offset = 1
        size = 3
        for i in range(5):
            config = PortConfig.deserialize(msg.payload[offset : offset + size])
            click.secho(
                f"port {i}: {msg.payload[offset:offset+size].hex()}", fg="green"
            )
            click.secho(f"{config}", fg="green")
            offset += size

    await send_cmd(
        SERVICE.GET_PORT_CONFIG,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@dataclass
class CompatibilitySettings:
    is_tfcp_enabled: bool
    is_fcp_enabled: bool
    is_ufcs_enabled: bool
    is_hvscp_enabled: bool
    is_lvscp_enabled: bool

    def serialize(self) -> bytearray:
        n = (
            0
            | (self.is_tfcp_enabled << 0)
            | (self.is_fcp_enabled << 1)
            | (self.is_ufcs_enabled << 2)
            | (self.is_hvscp_enabled << 3)
            | (self.is_lvscp_enabled << 4)
        )
        return bytearray([n])

    @classmethod
    def deserialize(cls, data: bytearray) -> "CompatibilitySettings":
        n = data[0]
        return cls(
            is_tfcp_enabled=(n & (1 << 0)) != 0,
            is_fcp_enabled=(n & (1 << 1)) != 0,
            is_ufcs_enabled=(n & (1 << 2)) != 0,
            is_hvscp_enabled=(n & (1 << 3)) != 0,
            is_lvscp_enabled=(n & (1 << 4)) != 0,
        )


@main.command(SERVICE.SET_PORT_COMPATIBILITY_SETTINGS.command)
@click.option("--tfcp", is_flag=True, default=False)
@click.option("--fcp", is_flag=True, default=False)
@click.option("--ufcs", is_flag=True, default=False)
@click.option("--hvscp", is_flag=True, default=False)
@click.option("--lvscp", is_flag=True, default=False)
@click.option("-a", "--address", type=str, default=None)
async def set_port_compatibility_settings(
    tfcp: bool, fcp: bool, ufcs: bool, hvscp: bool, lvscp: bool, address: str
) -> None:
    settings = CompatibilitySettings(tfcp, fcp, ufcs, hvscp, lvscp)
    await send_cmd(
        SERVICE.SET_PORT_COMPATIBILITY_SETTINGS,
        settings.serialize(),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.GET_PORT_COMPATIBILITY_SETTINGS.command)
@click.option("-a", "--address", type=str, default=None)
async def get_port_compatibility_settings(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        status = msg.payload[0]
        if status != 0:
            click.secho(
                "Failed to perform service {}. Status: {}".format(
                    SERVICE(abs(msg.header.service)).name, status
                ),
                err=True,
                fg="red",
            )
            return
        settings = CompatibilitySettings.deserialize(msg.payload[1:])
        click.secho(f"Settings: {settings}", fg="green")

    await send_cmd(
        SERVICE.GET_PORT_COMPATIBILITY_SETTINGS,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


@main.command(SERVICE.SET_TEMPERATURE_MODE.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument(
    "mode",
    type=click.Choice(["POWER_PRIORITY", "TEMPERATURE_PRIORITY"], case_sensitive=False),
)
async def set_temperature_mode(mode: str, address: str) -> None:
    mode_byte = 0
    match mode:
        case "POWER_PRIORITY":
            mode_byte = 0
        case "TEMPERATURE_PRIORITY":
            mode_byte = 1
        case _:
            raise ValueError("Invalid mode: {}".format(mode))
    await send_cmd(
        SERVICE.SET_TEMPERATURE_MODE,
        bytearray([mode_byte]),
        address,
        handle_rx=check_success_rx,
    )


@main.command(SERVICE.SET_TEMPORARY_ALLOCATOR.command)
@click.option("-a", "--address", type=str, default=None)
@click.argument("power_allocation", type=click.IntRange(0, 256), nargs=5)
async def set_temporary_allocator(
    power_allocation: typing.Tuple[int, int, int, int, int], address: str
) -> None:
    payload = bytearray()
    payload.extend(power_allocation)
    await send_cmd(
        SERVICE.SET_TEMPORARY_ALLOCATOR,
        payload,
        address,
        handle_rx=check_success_rx,
    )


@main.command("start-telemetry-stream")
@click.option("-a", "--address", type=str, default=None)
async def start_telemetry_stream(address: str) -> None:
    await send_cmd(
        SERVICE.START_TELEMETRY_STREAM,
        bytearray(),
        address,
        handle_rx=check_success_rx,
    )


@main.command("stop-telemetry-stream")
@click.option("-a", "--address", type=str, default=None)
async def stop_telemetry_stream(address: str) -> None:
    await send_cmd(
        SERVICE.STOP_TELEMETRY_STREAM,
        bytearray(),
        address,
        handle_rx=check_success_rx,
    )


@main.command("get-device-info")
@click.option("-a", "--address", type=str, default=None)
async def get_device_info(address: str) -> None:
    def handle_msg_rx(_: BleakGATTCharacteristic, data: bytearray) -> None:
        msg = Message.from_bytes(data)

        click.secho(f"Received: {binascii.hexlify(data)} => {msg}")
        if msg.header.size == 1:
            # 去掉中间阶段的空成功响应
            return

        # click.secho(f"Device info: {msg.payload}")

    await send_cmd(
        SERVICE.GET_DEVICE_INFO,
        bytearray(),
        address,
        handle_rx=handle_msg_rx,
    )


if __name__ == "__main__":
    main()
