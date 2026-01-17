# -*- coding: utf-8 -*-

from typing import Any, Dict, List, Tuple

import csv
import ctypes
import enum
import struct

from usb_pd import pd_hla


class PcapDirection(enum.IntEnum):
    PCAP_DIR_TX = 0
    PCAP_DIR_RX = 1


class PcapPDPacketHeader(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("direction", ctypes.c_uint8, 1),
        ("sop", ctypes.c_uint8, 2),
        ("serial", ctypes.c_uint8, 3),
        ("reserved", ctypes.c_uint8, 2),
    ]

    @property
    def sop_str(self) -> str:
        # Map sop values to their string representations
        sop_mapping = {1: "SOP", 2: "SOP_PRIME", 3: "SOP_PRIME_PRIME"}
        return sop_mapping.get(self.sop, f"Unknown ({self.sop})")


class PcapEntry(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint16),
        ("header", PcapPDPacketHeader),
        ("length", ctypes.c_uint8),
        ("vout", ctypes.c_uint16),
        ("iout", ctypes.c_uint16),
        # Flexible array requires special handling
    ]

    payload = bytes()

    @staticmethod
    def parse(data):
        # Size of the fixed part of the structure
        header_size = ctypes.sizeof(PcapEntry)

        # Create a structure from the fixed part
        entry = PcapEntry.from_buffer_copy(data[:header_size])

        # Extract payload as a Python bytes object and attach as attribute
        payload_size = entry.length
        entry.payload = bytes(data[header_size : header_size + payload_size])

        return entry


class PDPcapParser:
    """Parser for PD (Power Delivery) packet capture files."""

    PD_HDR = [
        "Sno",
        "Ok",
        "SOP",
        "Message",
        "Msg Id",
        "Data Role",
        "Power Role",
        "Obj Count",
        "Rev",
        "Duration",
        "Delta",
        "Vbus(V)",
        "Data",
        "Start Time",
        "End Time",
    ]

    def __init__(self, filename: str):
        self._filename = filename

    def __enter__(self):
        self.pd_file = open(self._filename, "w+")
        self.pd_writer = csv.writer(self.pd_file)
        self.pd_writer.writerow(self.PD_HDR)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.pd_file.flush()
        self.pd_file.close()

    @staticmethod
    def parse(data: bytes) -> Tuple[List[Any], Dict[str, Any]]:
        pcap_entry = PcapEntry.parse(data)
        header: PcapPDPacketHeader = pcap_entry.header
        payload = pcap_entry.payload

        hla = pd_hla.Hla()
        pd_header: int = struct.unpack("<I", payload[:4])[0]
        hdr = hla.decode_header(pd_header, header.sop_str)

        if hdr["extended"]:
            first_u16 = int.from_bytes(payload[0:2], "little")
            second_u16 = int.from_bytes(payload[2:4], "little")
            remaining_hex = " ".join(f"0x{b:02X}" for b in payload[4:])
            payload_hex = "0x%04X 0x%04X %s" % (first_u16, second_u16, remaining_hex)
        else:
            # Convert the first 16-bit segment
            first_u16 = int.from_bytes(payload[0:2], "little")

            # Prepare to encode all remaining data as u32[]
            remaining_payload = payload[2:]
            encoded_u32_list = []

            # Process each set of 4 bytes as a u32, padding with 0 if necessary
            for i in range(0, hdr["number_of_objects"] * 4, 4):
                segment = remaining_payload[i : i + 4]
                if len(segment) < 4:
                    # Pad with zeros if the segment is less than 4 bytes
                    segment = segment.rjust(4, b"\x00")

                encoded_u32 = int.from_bytes(segment, "little")
                encoded_u32_list.append(encoded_u32)

            # Convert list to string format
            encoded_u32_str = " ".join(f"0x{u32:08X}" for u32 in encoded_u32_list)

            # Format the string such that it includes the first u16 and all encoded u32 values
            payload_hex = f"0x{first_u16:04X} {encoded_u32_str}"

        direction_str = "RX" if header.direction else "TX"
        human_data = {
            "timestamp": pcap_entry.timestamp,
            "direction": direction_str,
            "sop": header.sop_str,
            "serial": header.serial,
            "length": pcap_entry.length,
            "vout": pcap_entry.vout,
            "iout": pcap_entry.iout,
            "header": hdr,
        }

        ezpd_data = [
            header.serial,
            direction_str,
            header.sop_str,
            hdr["msg_type"].upper(),
            hdr["message_id"],
            hdr["port_data_role"] or "RSVD",
            hdr["power_port_role"] or hdr["cable_plug"],
            hdr["number_of_objects"],
            hdr["spec_revision"],
            0,
            0,
            pcap_entry.vout,
            payload_hex.strip(),
            pcap_entry.timestamp * 16 * 1000,
            pcap_entry.timestamp * 16 * 1000 + 1,
        ]
        return ezpd_data, human_data

    def parse_pcap(self, data: bytes) -> None:
        ezpd_data, _ = self.parse(data)
        self.pd_writer.writerow(ezpd_data)
