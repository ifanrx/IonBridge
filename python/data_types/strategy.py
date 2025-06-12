#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import enum


class ChargingStrategy(enum.IntEnum):
    SLOW_CHARGING = 1
    STATIC_CHARGING = 2
    TEMPORARY_CHARGING = 3  # 临时分配模式
