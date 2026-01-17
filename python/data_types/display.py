# -*- coding: utf-8 -*-

import dataclasses
import enum


class DisplayTestKind(enum.IntEnum):
    TEST_IGNORE = 0
    TEST_EXIT = 1
    TEST_ENTER = 2
    TEST_LIGHTNESS = 3
    TEST_AUTO = 4
    TEST_SCREEN = 5
    TEST_AUTO_ONCE = 6


class DisplayTestScreen(enum.IntEnum):
    TEST_SCREEN_DEFAULT = 0
    TEST_SCREEN_RED = 1
    TEST_SCREEN_GREEN = 2
    TEST_SCREEN_BLUE = 3
    TEST_SCREEN_WHITE = 4
    TEST_SCREEN_BLACK = 5
    TEST_SCREEN_GREY = 6
    TEST_SCREEN_OFF = 7
    TEST_SCREEN_LOW_LIGHT = 8
    TEST_SCREEN_HIGH_LIGHT = 9
    TEST_SCREEN_1 = 10
    TEST_SCREEN_2 = 11
    TEST_SCREEN_3 = 12


@dataclasses.dataclass
class DisplayTestEvent:
    kind: DisplayTestKind
    param: int | None = None

    @classmethod
    def deserialize(cls, data: bytes) -> "DisplayTestEvent":
        if len(data) < 2:
            raise ValueError("Data too short to deserialize")
        kind = DisplayTestKind(data[0])
        param = int.from_bytes(data[1:3], "big") if len(data) > 2 else None
        return cls(kind, param)

    def serialize(self) -> bytearray:
        data = bytearray()
        data.append(self.kind.value)
        if self.param is not None:
            data.extend(self.param.to_bytes())
        return data
