#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import ctypes
import enum
import struct
import typing

import dataclasses

PORTS: list[str] = ["A", "C1", "C2", "C3", "C4"]


class GenericStatus(enum.IntEnum):
    OK = 0
    FAILED = 1


class KeepAliveStatus(enum.IntEnum):
    BOOTLOADER_ALIVE = 0
    USER_APPLICATION_ALIVE = 1
    USER_APPLICATION_SAFE_MODE = 2


class Subscriptions(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("EnablePortDetailsUpdate", ctypes.c_uint8, 1),
        ("EnablePDStatusUpdate", ctypes.c_uint8, 1),
    ]


class ClientPDStatus(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("battery_vid", ctypes.c_uint16),
        ("battery_pid", ctypes.c_uint16),
        ("battery_design_capacity", ctypes.c_uint16),
        ("battery_last_full_charge_capacity", ctypes.c_uint16),
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
        ("sink_cap_pod_count", ctypes.c_uint8, 2),
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
        ("request_ppsavs", ctypes.c_uint8, 1),
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
            raise ValueError(
                f"Invalid data size: expected {expected_size}, got {actual_size}"
            )
        return cls.from_buffer_copy(data)

    def __str__(self):
        """Human-readable representation of the ClientPDStatus instance."""
        fields_str = []
        for field_name, field_type, *_ in self._fields_:
            if field_name.startswith("unused"):
                continue
            field_value = getattr(self, field_name)
            if isinstance(field_value, ctypes.Array):
                field_value = list(field_value)
            fields_str.append(f" {field_name}={field_value}")
        return "ClientPDStatus(\n" + ",\n".join(fields_str) + "\n)"


class PowerFeatures(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        # Byte 1
        ("EnableTfcp", ctypes.c_uint8, 1),
        ("EnablePe", ctypes.c_uint8, 1),
        ("EnableQc2p0", ctypes.c_uint8, 1),
        ("EnableQc3p0", ctypes.c_uint8, 1),
        ("EnableQc3plus", ctypes.c_uint8, 1),
        ("EnableAfc", ctypes.c_uint8, 1),
        ("EnableFcp", ctypes.c_uint8, 1),
        ("EnableHvScp", ctypes.c_uint8, 1),
        # Byte 2
        ("EnableLvScp", ctypes.c_uint8, 1),
        ("EnableSfcp", ctypes.c_uint8, 1),
        ("EnableApple", ctypes.c_uint8, 1),
        ("EnableSamsung", ctypes.c_uint8, 1),
        ("EnableUfcs", ctypes.c_uint8, 1),
        ("EnablePd", ctypes.c_uint8, 1),
        ("EnablePdCompatMode", ctypes.c_uint8, 1),
        ("LimitedCurrentMode", ctypes.c_uint8, 1),
        # Byte 3
        ("EnablePdLVPPS", ctypes.c_uint8, 1),
        ("EnablePdEPR", ctypes.c_uint8, 1),
        ("EnablePd5V5A", ctypes.c_uint8, 1),
        ("EnablePdHVPPS", ctypes.c_uint8, 1),
        ("reserved", ctypes.c_uint8, 4),
    ]

    def __str__(self):
        return (
            f"PowerFeatures(EnableTfcp={self.EnableTfcp}, EnablePe={self.EnablePe}, "
            f"EnableQc2p0={self.EnableQc2p0}, EnableQc3p0={self.EnableQc3p0}, "
            f"EnableQc3plus={self.EnableQc3plus}, EnableAfc={self.EnableAfc}, "
            f"EnableFcp={self.EnableFcp}, EnableHvScp={self.EnableHvScp}, "
            f"EnableLvScp={self.EnableLvScp}, EnableSfcp={self.EnableSfcp}, "
            f"EnableApple={self.EnableApple}, EnableSamsung={self.EnableSamsung}, "
            f"EnableUfcs={self.EnableUfcs}, EnablePd={self.EnablePd}, "
            f"EnablePdCompatMode={self.EnablePdCompatMode}, "
            f"LimitedCurrentMode={self.LimitedCurrentMode}, "
            f"EnablePdLVPPS={self.EnablePdLVPPS}, EnablePdEPR={self.EnablePdEPR}, "
            f"EnablePd5V5A={self.EnablePd5V5A}, EnablePdHVPPS={self.EnablePdHVPPS}, reserved={self.reserved})"
        )

    def pack(self):
        return bytes(
            bytearray(ctypes.string_at(ctypes.byref(self), ctypes.sizeof(self)))
        )

    @staticmethod
    def all() -> list[str]:
        return [
            "tfcp",
            "pe",
            "qc2",
            "qc3",
            "qc3p",
            "afc",
            "fcp",
            "hvscp",
            "lvscp",
            "sfcp",
            "apple",
            "samsung",
            "ufcs",
            "pd",
            "pd_compat_mode",
            "limited_current_mode",
            "pd_pps",
            "pd_epr",
            "pd_rpi",
            "pd_lv_pps",
            "pd_hv_pps",
        ]

    def set_feature(self, feature: str, enable: bool):
        feature = feature.lower()
        match feature:
            case "tfcp":
                self.EnableTfcp = enable
            case "pe":
                self.EnablePe = enable
            case "qc2":
                self.EnableQc2p0 = enable
            case "qc3":
                self.EnableQc3p0 = enable
            case "qc3p":
                self.EnableQc3plus = enable
            case "afc":
                self.EnableAfc = enable
            case "fcp":
                self.EnableFcp = enable
            case "hvscp":
                self.EnableHvScp = enable
            case "lvscp":
                self.EnableLvScp = enable
            case "sfcp":
                self.EnableSfcp = enable
            case "apple":
                self.EnableApple = enable
            case "samsung":
                self.EnableSamsung = enable
            case "ufcs":
                self.EnableUfcs = enable
            case "pd":
                self.EnablePd = enable
            case "pd_compat_mode":
                self.EnablePdCompatMode = enable
            case "limited_current_mode":
                self.LimitedCurrentMode = enable
            case "pd_pps":
                self.EnablePdLVPPS = enable
                self.EnablePdHVPPS = enable
            case "pd_lv_pps":
                self.EnablePdLVPPS = enable
            case "pd_hv_pps":
                self.EnablePdHVPPS = enable
            case "pd_epr":
                self.EnablePdEPR = enable
            case "pd_rpi":
                self.EnablePd5V5A = enable
            case _:
                raise ValueError(f"Invalid feature value: {feature}")

    def set_all(self, enable: bool):
        for feature in self.all():
            self.set_feature(feature, enable)


class UARTMetrics(ctypes.LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("overrun", ctypes.c_uint32),
        ("valid_msg", ctypes.c_uint32),
        ("invalid_msg", ctypes.c_uint32),
        ("read", ctypes.c_uint32),
        ("processed_at", ctypes.c_uint32),
        ("responded_at", ctypes.c_uint32),
        ("latency", ctypes.c_uint32),
        ("command", ctypes.c_uint16),
    ]

    def __str__(self):
        # Creating a human-readable output
        output = [
            f"Overrun: {self.overrun}",
            f"Valid Messages: {self.valid_msg}",
            f"Invalid Messages: {self.invalid_msg}",
            f"Read: {self.read}",
            f"Processed At: {self.processed_at}",
            f"Responded At: {self.responded_at}",
            f"Latency: {self.latency}",
            f"Command: 0x{self.command:04X}",
        ]

        # Join the list into a readable string
        return "\n".join(output)


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


@dataclasses.dataclass
class PortConfigInternal:
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
    is_pd_compat_mode: bool = False
    is_limited_current_mode: bool = False
    is_pd_lvpps_enabled: bool = True
    is_pdepr_enabled: bool = True
    is_pdrpi_enabled: bool = False
    is_pd_hvpps_enabled: bool = True

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
        n |= self.is_pd_compat_mode << 14
        n |= self.is_limited_current_mode << 15
        data.extend(struct.pack("<H", n))
        if self.version == 1:
            n = 0
            n |= self.is_pd_lvpps_enabled << 0
            n |= self.is_pdepr_enabled << 1
            n |= self.is_pdrpi_enabled << 2
            n |= self.is_pd_hvpps_enabled << 3
            data.extend(struct.pack("<B", n))
        return data

    @classmethod
    def deserialize(cls, data: bytearray) -> "PortConfigInternal":
        version = data[0]
        n3 = 0b0000_0001
        if version == 0:
            version, n1, n2 = struct.unpack("<BBB", data)
        else:
            version, n1, n2, n3 = struct.unpack("<BBBB", data)
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
            is_pd_compat_mode=((n2 >> 6) & 0x01) != 0,
            is_limited_current_mode=((n2 >> 7) & 0x01) != 0,
            is_pd_lvpps_enabled=((n3 >> 0) & 0x01) != 0,
            is_pdepr_enabled=((n3 >> 1) & 0x01) != 0,
            is_pdrpi_enabled=((n3 >> 2) & 0x01) != 0,
            is_pd_hvpps_enabled=((n3 >> 3) & 0x01) != 0,
        )

    def __str__(self) -> str:
        lines = []
        version1_fields = [
            "is_pdepr_enabled",
            "is_pdrpi_enabled",
            "is_pd_lvpps_enabled",
            "is_pd_hvpps_enabled",
        ]
        for field in dataclasses.fields(self):
            if self.version < 1 and field.name in version1_fields:
                continue
            lines.append(f"    {field.name}: {getattr(self, field.name)}")
        return "\n".join(lines)


class UARTDebugCounter(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("processing", ctypes.c_uint16),
        ("preamble0", ctypes.c_uint16),
        ("preamble1", ctypes.c_uint16),
        ("addr", ctypes.c_uint16),
        ("command0", ctypes.c_uint16),
        ("command1", ctypes.c_uint16),
        ("length", ctypes.c_uint16),
        ("value", ctypes.c_uint16),
        ("checksum0", ctypes.c_uint16),
        ("checksum1", ctypes.c_uint16),
        ("postamble0", ctypes.c_uint16),
        ("postamble1", ctypes.c_uint16),
        ("reset_reason", ctypes.c_uint16 * 12),
    ]

    def __str__(self):
        # Creating a human-readable output
        output = [
            f"Processing: {bool(self.processing)}",
            f"Preamble: 0x{self.preamble0:02X}{self.preamble1:02X}",
            f"Address: {self.addr}",
            f"Command: 0x{self.command0:02X}{self.command1:02X}",
            f"Length: {self.length}",
            f"Value: {self.value}",
            f"Checksum: 0x{self.checksum0:02X}{self.checksum1:02X}",
            f"Postamble: 0x{self.postamble0:02X}{self.postamble1:02X}",
        ]

        # Adding the reset_reason array
        reset_reason_str = (
            "Reset Reason: ["
            + ", ".join(str(self.reset_reason[i]) for i in range(12))
            + "]"
        )
        output.append(reset_reason_str)

        # Joining the list into a readable string
        return "\n".join(output)


class PortConfig(ctypes.LittleEndianStructure):
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("features", PowerFeatures),
    ]

    def __str__(self):
        """Human-readable string representation of the structure."""
        fields_str = []
        for field_name, field_type, *rest in self._fields_:
            value = getattr(self, field_name)
            if isinstance(field_type, ctypes.Array):
                value = list(value)
            fields_str.append(f"  {field_name}={value}")
        return "PortConfig(\n" + ",\n".join(fields_str) + "\n)"


class FixedVoltageOffset:
    """Class to handle fixed voltage offset conversions and validations."""

    # Mapping between enum values and human-readable strings
    VOLTAGE_MAP = {0: "0mV", 1: "100mV", 2: "150mV", 3: "200mV"}

    # Reverse mapping for lookups
    STRING_TO_VALUE = {v: k for k, v in VOLTAGE_MAP.items()}

    @classmethod
    def parse(cls, value) -> int:
        """
        Parse a value into the corresponding enum value (0-3).

        Args:
            value: Can be string like "100mV", integer 0-3, or voltage value

        Returns:
            Integer value 0-3 representing the voltage offset
        """
        # If it's already an integer in the valid range
        if isinstance(value, int) and 0 <= value <= 3:
            return value

        # If it's a string that matches our known formats directly
        if isinstance(value, str) and value in cls.STRING_TO_VALUE:
            return cls.STRING_TO_VALUE[value]

        # If it's a string with a number, extract and convert
        if isinstance(value, str):
            try:
                import re

                # Extract numeric part (strip non-digits)
                digits = re.sub(r"\D", "", value)
                num_value = int(digits) if digits else 0

                # Map common voltage values to enum values
                if num_value == 0:
                    return 0
                elif num_value == 100:
                    return 1
                elif num_value == 150:
                    return 2
                elif num_value == 200:
                    return 3
            except ValueError:
                pass

        raise ValueError(
            f"Invalid fixed_voltage_offset value: {value}. "
            f"Expected one of: {list(cls.VOLTAGE_MAP.values())} or integer 0-3."
        )

    @classmethod
    def to_string(cls, value: int) -> str:
        """Convert enum value to human-readable string."""
        if value not in cls.VOLTAGE_MAP:
            raise ValueError(f"Invalid voltage offset value: {value}. Expected 0-3.")
        return cls.VOLTAGE_MAP[value]

    @classmethod
    def get_choices(cls) -> list:
        """Return list of valid string choices for CLI."""
        return list(cls.VOLTAGE_MAP.values())

    @classmethod
    def click_callback(cls, ctx, param, value):
        """Callback function for Click to convert string to enum value."""
        if not value:
            return 0  # Default value
        return cls.parse(value)


class SystemFlags(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("commit", ctypes.c_uint8),  # Must be 0x55 to commit
        ("unused0", ctypes.c_uint8, 1),
        ("port_type", ctypes.c_uint8, 1),
        ("enable_cable_compensation", ctypes.c_uint8, 1),
        ("cable_compensation", ctypes.c_uint8, 1),
        ("fixed_voltage_offset", ctypes.c_uint8, 2),
        ("reserved", ctypes.c_uint8, 2),
    ]

    def __str__(self) -> str:
        """Human-readable string representation of the structure."""
        commit = self.commit == 0x55
        unused0 = self.unused0 == 1
        port_type = "A" if self.port_type == 0 else "C"
        enable_cable_compensation = self.enable_cable_compensation == 1
        cable_compensation = "65mOhm" if self.cable_compensation == 1 else "100mOhm"
        fixed_voltage_offset = FixedVoltageOffset.to_string(self.fixed_voltage_offset)

        return (
            "SystemFlags(\n"
            f"\tcommit: {commit},\n"
            f"\tunused0: {unused0}\n"
            f"\tport_type: {port_type}\n"
            f"\tenable_cable_compensation: {enable_cable_compensation}\n"
            f"\tcable_compensation: {cable_compensation}\n"
            f"\tfixed_voltage_offset: {fixed_voltage_offset}\n"
            ")"
        )



class PortType(enum.IntEnum):
    PORT_A = 0
    PORT_C = 1


class KeepAliveResponse(ctypes.LittleEndianStructure):
    _pack_ = 1
    _anonymous_ = ("_union",)

    class _Union(ctypes.Union):
        class _Struct(ctypes.LittleEndianStructure):
            _pack_ = 1
            _fields_ = [
                ("uptimeMS", ctypes.c_uint32),
                ("rebootReason", ctypes.c_uint32),
            ]

        _fields_ = [
            ("_struct", _Struct),
        ]

    _fields_ = [
        ("nonce", ctypes.c_uint8),
        ("status", ctypes.c_uint8),  # Underlying type for KeepAliveStatus
        ("_union", _Union),
    ]

    @property
    def uptimeMS(self):
        return self._union._struct.uptimeMS

    @uptimeMS.setter
    def uptimeMS(self, value):
        self._union._struct.uptimeMS = value

    @property
    def rebootReason(self):
        return self._union._struct.rebootReason

    @rebootReason.setter
    def rebootReason(self, value):
        self._union._struct.rebootReason = value

    def __str__(self) -> str:
        """Human-readable string representation of the structure."""
        try:
            status_str = KeepAliveStatus(self.status).name
        except ValueError:
            status_str = f"UNKNOWN_STATUS ({self.status})"

        return (
            "KeepAliveResponse(\n"
            f"\tnonce: {self.nonce},\n"
            f"\tstatus: {status_str},\n"
            f"\tuptimeMS: {self.uptimeMS},\n"
            f"\trebootReason: 0x{self.rebootReason:08x}\n"
            ")"
        )
