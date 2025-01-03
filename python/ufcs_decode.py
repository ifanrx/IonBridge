#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import ctypes
import enum
import struct


class Device(enum.IntEnum):
    SOURCE = 0b001
    SINK = 0b010
    CABLE = 0b011


class MsgType(enum.IntEnum):
    CTRL = 0b000
    DATA = 0b001
    CUSTOM = 0b010


class ControlCommand(enum.IntEnum):
    PING = 0x00
    ACK = 0x01
    NCK = 0x02
    ACCEPT = 0x03
    SOFT_RESET = 0x04
    POWER_READY = 0x05
    GET_OUTPUT_CAPABILITIES = 0x06
    GET_SOURCE_INFO = 0x07
    GET_SINK_INFO = 0x08
    GET_CABLE_INFO = 0x09
    GET_DEVICE_INFO = 0x0A
    GET_ERROR_INFO = 0x0B
    DETECT_CABLE_INFO = 0x0C
    START_CABLE_DETECT = 0x0D
    END_CABLE_DETECT = 0x0E
    EXIT_UFCS_MODE = 0x0F


class UFCSMessageHeader(ctypes.BigEndianStructure):
    _fields_ = [
        ("device_addr", ctypes.c_uint16, 3),  # 3 bits wide
        ("msg_num", ctypes.c_uint16, 4),  # 4 bits wide
        ("protocol_ver", ctypes.c_uint16, 6),  # 6 bits wide
        ("msg_type", ctypes.c_uint16, 3),  # 3 bits wide
    ]

    def __str__(self):
        device_addr_str = (
            Device(self.device_addr).name
            if self.device_addr in Device._value2member_map_
            else "RESERVED"
        )
        msg_type_str = (
            MsgType(self.msg_type).name
            if self.msg_type in MsgType._value2member_map_
            else "RESERVED"
        )
        protocol_ver_str = "1.0" if self.protocol_ver == 1 else "RESERVED"
        return (
            "<<UFCS Header - "
            f"Device Address: {device_addr_str} "
            f"Message Number: {self.msg_num} "
            f"Protocol Version: {protocol_ver_str} "
            f"Message Type: {msg_type_str}"
            ">>"
        )


class UFCSMessage(ctypes.BigEndianStructure):
    _fields_ = [
        ("prefix", ctypes.c_uint8),
        ("header", UFCSMessageHeader),
        ("data", ctypes.c_uint8 * 61),  # Body can be up to 61 bytes
        ("crc", ctypes.c_uint8),
    ]


header = UFCSMessageHeader()
struct.pack_into("!H", header, 0, 0x4A09)
print(header)
