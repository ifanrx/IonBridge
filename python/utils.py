#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import struct
import typing

from itertools import count, takewhile


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


def sliced(data: bytearray, n: int) -> typing.Iterator[bytes | bytearray]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))


def eval_true(s: str) -> bool:
    return s.lower() in ("true", "1", "t", "y", "yes", "enable")
