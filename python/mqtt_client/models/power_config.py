#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Power configuration model."""

import dataclasses
import struct


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
        version = payload[0]
        match version:
            case 0:
                return cls(*struct.unpack_from("<17B", payload))
            case 1:
                return cls(*struct.unpack_from("<BH15B", payload))
        raise ValueError(f"Unsupported version: {version}")

    def serialize(self):
        payload = bytearray()
        payload.append(self.version)
        match self.version:
            case 0:
                payload.append(self.power_budget)
            case 1:
                payload.extend(struct.pack("<H", self.power_budget))
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
