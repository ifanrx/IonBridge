"""
MQTT 订阅，用来测试固件 MQTT 上报的消息
"""

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
import ctypes


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
    # generate client ID with pub prefix randomly
    client_id: str = f"python-mqtt-{random.randint(0, 100)}"
    subscribe_services: typing.List["SERVICE"] = dataclasses.field(default_factory=list)


# 全局配置
config = Config()


class SERVICE(enum.IntEnum):
    STREAM_PORTS_STATUS = 0x0080
    STREAM_DEVICE_STATUS = 0x0081
    STREAM_PORT_PD_STATUS = 0x0082

    GET_DEVICE_INFO = 0x0092

    # 内部信息 0x0000 - 0x0110
    PING = 0x0100

    # 基础设备信息 0x0110 - 0x0120
    DEVICE_BOOT_INFO = 0x0110
    DEVICE_MEMORY_INFO = 0x0111
    DEVICE_SYSTEM_STATE = 0x0112

    # 功率相关信息
    POWER_HISTORICAL_DATA = 0x0130
    POWER_CONSUMPTION_DATA = 0x0131  # 用电统计
    POWER_ALLOCATION_DATA = 0x0132

    # OTA
    ESP32_UPGRADE_DATA = 0x0140
    OTA_CONFIRM_DATA = 0x0143

    WIFI_STATS_DATA = 0x0150
    REQUEST_LICENSE = 0x0151
    UART_METRICS_DATA = 0x0152

    # alert
    OUT_OF_MEMORY_ALERT = 0x0160
    OVER_TEMPERATURE_ALERT = 0x0161
    OVER_POWER_ALERT = 0x0162  # 功率超限


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
        ("unused1", ctypes.c_uint8, 4),
        ("operating_voltage", ctypes.c_uint16, 15),
        ("request_ppsavs", ctypes.c_bool, 1)
    ]


class StreamPortStatusParser:
    # Named tuples for structured data access
    PowerFeatures = namedtuple("PowerFeatures", [
        "EnableTfcp", "EnablePe", "EnableQc2p0", "EnableQc3p0",
        "EnableQc3plus", "EnableAfc", "EnableFcp", "EnableHvScp",
        "EnableLvScp", "EnableSfcp", "EnableApple", "EnableSamsung",
        "EnableUfcs", "EnablePd", "EnableOverCompensation", "LimitedCurrentMode"
    ])

    PortDetails = namedtuple("PortDetails", [
        "connected", "fc_protocol", "iout_value", "vout_value",
        "die_temperature", "vin_value", "session_id", "session_charge"
    ])

    PortStatusData = namedtuple("PortStatusData", [
        "features",      # PowerFeatures struct
        "unused0",       # 6 bytes
        "details",       # PortDetails struct
        "charging_minutes",
        "unused1"
    ])

    StreamPortStatus = namedtuple("StreamPortStatus", [
        "status",
        "port_status_map",
        "padding",
        "ports"
    ])

    PORT_COUNT = 8

    def __init__(self, data):
        self.data = data
        self.result = None

        self.POWER_FEATURES_SIZE = 2

        self.PORT_DETAILS_FORMAT = "<B B H H H H H Q"
        self.PORT_DETAILS_SIZE = struct.calcsize(self.PORT_DETAILS_FORMAT)  # Should be 20

        self.PORT_STATUS_DATA_SIZE = 32

        self.STREAM_PORT_STATUS_HEADER_FORMAT = "<B B H"
        self.STREAM_PORT_STATUS_HEADER_SIZE = struct.calcsize(self.STREAM_PORT_STATUS_HEADER_FORMAT)

    def parse_power_features(self, data):
        """Parse PowerFeatures from 2 bytes."""
        # Two bytes represent 16 bits for the features
        val = data[0] | (data[1] << 8)
        bits = [(val >> i) & 1 for i in range(16)]
        return self.PowerFeatures(*bits)

    def parse_port_details(self, data):
        """Parse PortDetails from 20 bytes."""
        # Expect data to be 20 bytes
        unpacked = struct.unpack(self.PORT_DETAILS_FORMAT, data)
        return self.PortDetails(*unpacked)

    def parse_port_status_data(self, data):
        """Parse PortStatusData from 32 bytes."""
        # Extract PowerFeatures (2 bytes)
        pf_data = data[0:2]
        features = self.parse_power_features(pf_data)

        # unused0 (6 bytes)
        unused0 = data[2:2+6]

        # PortDetails (20 bytes)
        details_data = data[8:8+20]
        details = self.parse_port_details(details_data)

        # charging_minutes (2 bytes) after details
        # details ends at offset 8+20=28
        charging_minutes = struct.unpack("<H", data[28:30])[0]

        # unused1 (2 bytes)
        unused1 = data[30:32]

        return self.PortStatusData(features, unused0, details, charging_minutes, unused1)

    def parse(self):
        """Parse the entire StreamPortStatus from self.data."""
        # Parse header
        header = struct.unpack(self.STREAM_PORT_STATUS_HEADER_FORMAT, self.data[:self.STREAM_PORT_STATUS_HEADER_SIZE])
        status, port_status_map, padding = header
        offset = self.STREAM_PORT_STATUS_HEADER_SIZE

        ports = []
        for i in range(self.PORT_COUNT):
            port_data = self.data[offset:offset+self.PORT_STATUS_DATA_SIZE]
            port_parsed = self.parse_port_status_data(port_data)
            ports.append(port_parsed)
            offset += self.PORT_STATUS_DATA_SIZE

        self.result = self.StreamPortStatus(status, port_status_map, padding, ports)

    def display(self):
        if self.result is None:
            print("No data parsed. Call parse() first.")
            return

        port_names = ["A", "C1", "C2", "C3", "C4", "-", "-", "-"]
        print("StreamPortStatus:")
        print(f"  Status: {self.result.status}")
        print(f"  Port Status Map: {bin(self.result.port_status_map)}")
        print(f"  Padding: {self.result.padding}")
        print(f"  Ports:")
        for idx, port in enumerate(self.result.ports[:5]):
            open = ((self.result.port_status_map >> idx) & 0x1) == 0x1
            fg = None
            if open:
                fg = "green"
            if port.details.connected:
                fg = "magenta"
            click.secho(f"    Port {port_names[idx]}({idx}):", fg=fg)
            click.secho(f"      Features: {port.features}", fg=fg)
            click.secho(f"      Charging Minutes: {port.charging_minutes}", fg=fg)
            click.secho(f"      Details: {port.details}", fg=fg)


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
        print(
            f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')} service: {SERVICE(self.header.service).name}, message_id: {self.header.message_id}"
        )
        format = ""
        fields = []
        port_names = ["A", "C1", "C2", "C3", "C4", "-", "-", "-"]
        match self.header.service:
            case SERVICE.STREAM_PORTS_STATUS:
                parser = StreamPortStatusParser(self.payload)
                parser.parse()
                parser.display()
                return
            case SERVICE.STREAM_DEVICE_STATUS:
                format = "<5BHIHHHB"
                fields = [
                    "status",
                    "strategy",
                    "display_intensity",
                    "display_flip_mode",
                    "display_mode",
                    "mode_param",
                    "device_uptime",
                    "feature_flag_bits",
                    "charging_minutes",
                    "power_allocator_param",
                    "temperature_mode",
                ]
            case SERVICE.GET_DEVICE_INFO:
                reader = io.BytesIO(self.payload)
                status = struct.unpack("<B", reader.read(1))[0]
                psn, model = struct.unpack("<32s8s", reader.read(40))
                esp32_major, esp32_minor, esp32_patch = struct.unpack(
                    "<3B",
                    reader.read(3),
                )
                sw3566_major, sw3566_minor, sw3566_patch = struct.unpack(
                    "<3B",
                    reader.read(3),
                )
                fpga_major, fpga_minor, fpga_patch = struct.unpack(
                    "<3B",
                    reader.read(3),
                )
                zrlib_major, zrlib_minor, zrlib_patch = struct.unpack(
                    "<3B",
                    reader.read(3),
                )
                ble_mac = reader.read(6)
                wifi_mac = reader.read(6)
                bssid = reader.read(6)
                ssid = struct.unpack("<64s", reader.read(64))[0]
                rssi = struct.unpack("<b", reader.read(1))[0]
                wifi_protocol = struct.unpack("<B", reader.read(1))[0]
                channel = struct.unpack("<B", reader.read(1))[0]
                info(f"    Status: {status}")
                info(f"    PSN: {psn.decode('utf-8')}")
                info(f"    Model: {model.decode('utf-8')}")
                info(f"    ESP32: {esp32_major}.{esp32_minor}.{esp32_patch}")
                info(f"    SW3566: {sw3566_major}.{sw3566_minor}.{sw3566_patch}")
                info(f"    FPGA: {fpga_major}.{fpga_minor}.{fpga_patch}")
                info(f"    ZRLIB: {zrlib_major}.{zrlib_minor}.{zrlib_patch}")
                info(f"    BLE MAC: {format_mac(ble_mac)}")
                info(f"    WIFI MAC: {format_mac(wifi_mac)}")
                info(f"    BSSID: {bssid.hex()}")
                info(f"    SSID: {ssid.decode('utf-8')}")
                info(f"    RSSI: {rssi}")
                info(f"    WIFI Protocol: {wifi_protocol}")
                info(f"    Channel: {channel}")
                return
            case SERVICE.PING:
                format = ""
                fields = []
            case SERVICE.DEVICE_BOOT_INFO:
                fmt = "<3B 3B 3B 3B 3B B 6B 4B 16B Q H I H b B I I i 8s 8s 8s"
                # Unpack all fields at once
                unpacked = struct.unpack_from(fmt, self.payload)

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
                hardwareRev = unpacked[51].decode("utf-8").strip("\x00"),
                deviceModel = unpacked[52].decode("utf-8").strip("\x00"),
                productFamily = unpacked[53].decode("utf-8").strip("\x00"),
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
            case SERVICE.DEVICE_MEMORY_INFO:
                format = "<B3I"
                fields = [
                    "initialBoot",
                    "freeHeapSize",
                    "freeInternalHeapSize",
                    "minimumFreeHeapSize",
                ]
            case SERVICE.DEVICE_SYSTEM_STATE:
                initialBoot, uptime, taskCount = struct.unpack(
                    "<BqH", self.payload[:11]
                )
                print(f"    initialBoot: {initialBoot}")
                print(f"    uptime: {uptime}")
                print(f"    taskCount: {taskCount}")
                offset = 11
                size = 40
                print("        name             runtimePercent    stackHighWaterMark")
                for _i in range(taskCount):
                    result = struct.unpack(
                        "<32s2I", self.payload[offset : offset + size]
                    )
                    name = result[0].rstrip(b"\x00").decode("utf-8")
                    percentage = f"{result[1] / 100:.2f}%"
                    print(
                        f"        {name.ljust(16)} {percentage.ljust(14)}    {result[2]}bytes"
                    )
                    offset += size
                temperature = struct.unpack("<B", self.payload[offset : offset + 1])[0]
                print(f"    temperature: {temperature}°C")
                return
            case SERVICE.POWER_HISTORICAL_DATA:
                port, length = struct.unpack("<BH", self.payload[:3])
                print(f"    port: {port}")
                print(f"    length: {length}")
                offset = 3
                size = 4
                data = []
                for _i in range(length):
                    result = struct.unpack("<4B", self.payload[offset : offset + size])
                    data.append(result)
                    offset += size

                print("        voltage amperage temperature vin")
                prev = ()
                duplicates = 0
                for result in data:
                    amperage = f"{result[0] / 32:.2f}A"
                    voltage = f"{result[1] / 8:.2f}V"
                    temperature = f"{result[2]}°C"
                    vin = f"{result[3] / 8}V"
                    if prev == result:
                        duplicates += 1
                        continue

                    if duplicates > 0:
                        print(f"        ... {duplicates} duplicates")
                        duplicates = 0

                    print(
                        f"        {voltage.ljust(7)} {amperage.ljust(8)} {temperature.ljust(11)} {vin}"
                    )
                    prev = result
                if duplicates > 0:
                    print(f"        ... {duplicates} duplicates")
                    duplicates = 0
                return
            case SERVICE.STREAM_PORT_PD_STATUS:
                print(f"    payload: {self.payload.hex()}")
                port_index = self.payload[1]
                pd_status_bytes = self.payload[2:]

                # Assuming pd_status_bytes is a byte array containing the data to be parsed
                pd_status = PDStatus.from_buffer_copy(pd_status_bytes)

                # Access the parsed fields
                print(f"    Battery VID: {pd_status.battery_vid}")
                print(f"    Battery PID: {pd_status.battery_pid}")
                print(f"    Battery Design Capacity: {pd_status.battery_design_capacity}")
                print(f"    Battery Last Full Charge Capacity: {pd_status.battery_last_full_charge_capacity}")
                print(f"    Battery Present Capacity: {pd_status.battery_present_capacity}")
                print(f"    Battery Invalid: {pd_status.battery_invalid}")
                print(f"    Battery Present: {pd_status.battery_present}")
                print(f"    Battery Status: {pd_status.battery_status}")
                print(f"    Cable Is Active: {pd_status.cable_is_active}")
                print(f"    Cable Termination Type: {pd_status.cable_termination_type}")
                print(f"    Cable EPR Mode Capable: {pd_status.cable_epr_mode_capable}")
                print(f"    Cable Active PHY Type: {pd_status.cable_active_phy_type}")
                print(f"    Cable Latency: {pd_status.cable_latency}")
                print(f"    Cable Max VBUS Voltage: {pd_status.cable_max_vbus_voltage}")
                print(f"    Cable Max VBUS Current: {pd_status.cable_max_vbus_current}")
                print(f"    Cable USB Highest Speed: {pd_status.cable_usb_highest_speed}")
                print(f"    Cable Active Element: {pd_status.cable_active_element}")
                print(f"    Cable Active USB4: {pd_status.cable_active_usb4}")
                print(f"    Cable Active USB2p0: {pd_status.cable_active_usb2p0}")
                print(f"    Cable Active USB3p2: {pd_status.cable_active_usb3p2}")
                print(f"    Cable Active USB Lanes: {pd_status.cable_active_usb_lanes}")
                print(f"    Cable Active Optically Isolated: {pd_status.cable_active_optically_isolated}")
                print(f"    Cable Active USB4 Asym: {pd_status.cable_active_usb4_asym}")
                print(f"    Cable Active USB Gen: {pd_status.cable_active_usb_gen}")
                print(f"    Request EPR Mode Capable: {pd_status.request_epr_mode_capable}")
                print(f"    Request PDO ID: {pd_status.request_pdo_id}")
                print(f"    Request USB Communications Capable: {pd_status.request_usb_communications_capable}")
                print(f"    Request Capability Mismatch: {pd_status.request_capability_mismatch}")
                print(f"    Sink Capabilities: {pd_status.sink_capabilities}")
                print(f"    Sink Cap PDO Count: {pd_status.sink_cap_pdo_count}")
                print(f"    Status Temperature: {pd_status.status_temperature}")
                print(f"    Cable VID: {pd_status.cable_vid}")
                print(f"    Cable PID: {pd_status.cable_pid}")
                print(f"    Manufacturer VID: {pd_status.manufacturer_vid}")
                print(f"    Manufacturer PID: {pd_status.manufacturer_pid}")
                print(f"    Sink Minimum PDP: {pd_status.sink_minimum_pdp}")
                print(f"    Sink Operational PDP: {pd_status.sink_operational_pdp}")
                print(f"    Sink Maximum PDP: {pd_status.sink_maximum_pdp}")
                print(f"    Unused0: {pd_status.unused0}")
                print(f"    Operating Current: {pd_status.operating_current}")
                print(f"    PD Revision: {pd_status.pd_revision}")
                print(f"    Unused1: {pd_status.unused1}")
                print(f"    Operating Voltage: {pd_status.operating_voltage}")
                print(f"    Request PPSAVS: {pd_status.request_ppsavs}")
                                
                for i in range(len(pd_status_bytes) // 4):
                    val = struct.unpack("<I", pd_status_bytes[i*4:(i+1)*4])[0]
                    setattr(pd_status, f"field{i}", val)

                print(f"    Port Index: {port_index}")
                return 
            case SERVICE.POWER_ALLOCATION_DATA:
                power_budget, remaining_power, adc_value, temperature, unused = struct.unpack("<3H B B", self.payload[:8])
                print(f"    power_budget: {power_budget}")
                print(f"    temperature: {temperature}")
                print(f"    remaining_power: {remaining_power}")
                print(f"    adc_value: {adc_value}")
                for i in range(8):
                    offset = 8 + i * 2
                    power_budget, actual_power = struct.unpack("<2B", self.payload[offset : offset + 2])
                    print(f"    port {i}: power_budget: {power_budget}, actual_power: {actual_power}")
                return
            case SERVICE.ESP32_UPGRADE_DATA:
                format = "<5Bi"
                fields = ["major", "minor", "revision", "status", "reason", "esp_err"]
            case SERVICE.WIFI_STATS_DATA:
                format = "<2H b B 5H 2I"
                fields = [
                    "associationCount",
                    "disassociationCount",
                    "rssi",
                    "channel",
                    "wifiConnectionTime",
                    "mqttConnectionTime",
                    "connectivityFailureCount",
                    "dnsResolutionFailureCount",
                    "mqttConnectionCount",
                    "mqttMessageTxCount",
                    "mqttMessageRxCount",
                ]
            case SERVICE.REQUEST_LICENSE:
                pass
            case SERVICE.UART_METRICS_DATA:
                format = "<4I"
                fields = ["reset_state_count", "resend_count", "sent_failed_count", "sent_count"]
            case SERVICE.OUT_OF_MEMORY_ALERT:
                format = "<4I"
                fields = [
                    "freeHeapSize",
                    "freeInternalHeapSize",
                    "minimumFreeHeapSize",
                    "thresholdFreeHeapSize",
                ]
            case SERVICE.OVER_TEMPERATURE_ALERT:
                format = "<2B"
                fields = ["temperature", "threshold"]
            case SERVICE.OVER_POWER_ALERT:
                pass

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
@click.argument("services", type=click.Choice(list(SERVICE.__members__.keys()), case_sensitive=False), nargs=-1)
def run(
    psn: str,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    services: typing.List[str],
):
    config.broker = broker
    config.port = port
    config.topic = config.topic.format(psn=psn)
    config.subscribe_services = [SERVICE[s] for s in services]
    client = connect_mqtt(broker, port, ca_certs, certfile, keyfile)
    client.loop_forever()


if __name__ == "__main__":
    run()
