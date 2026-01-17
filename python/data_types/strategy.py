#!/usr/bin/env python3
# -*- coding=utf-8 -*-


class ChargingStrategy:
    STRATEGY_MAP = {
        1: "slow",
        2: "static",
        3: "temporary",
        5: "usba",
        7: "high_performance",
        8: "ultra_fast_single_port",
    }
    STRING_TO_VALUE = {v: k for k, v in STRATEGY_MAP.items()}

    @classmethod
    def parse(cls, value) -> int:
        if isinstance(value, int) and value in cls.STRATEGY_MAP:
            return value
        if isinstance(value, str) and value.lower() in cls.STRING_TO_VALUE:
            return cls.STRING_TO_VALUE[value.lower()]

        raise ValueError(
            f"Invalid charging strategy: {value}. Valid values are: {list(cls.STRING_TO_VALUE.keys())}."
        )

    @classmethod
    def to_string(cls, value: int) -> str:
        if value in cls.STRATEGY_MAP:
            return cls.STRATEGY_MAP[value]
        raise ValueError(
            f"Invalid charging strategy value: {value}. Valid values are: {list(cls.STRATEGY_MAP.keys())}."
        )

    @classmethod
    def get_choices(cls) -> list:
        return list(cls.STRING_TO_VALUE.keys())

    @classmethod
    def click_callback(cls, ctx, param, value):
        if not value:
            return 1
        return cls.parse(value)
