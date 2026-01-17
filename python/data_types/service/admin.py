#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import enum


class AdminCommand(enum.IntEnum):
    GET_DEVICE_PASSWORD = 0x00
    MANAGE_FPGA_CONFIG = 0x10
    MANAGE_POWER_CONFIG = 0x11
    MANAGE_FEATURE_TOGGLE = 0x12
    SET_TELEMETRY_DEBUG = 0x20
    SET_SYSLOG_STATE = 0x21

    REQUEST_SYSTEM_TIME = 0x30
    SET_SYSTEM_TIME = 0x31
    REQUEST_LICENSE = 0x32
    PUSH_LICENSE = 0x33

    REQUEST_SHUTDOWN_BLE = 0x40

    FORWARD_PORT = 0x50  # Forwarding request to port directly
    READ_ADC = 0x51
    SET_MACHINE_CONFIG = 0x52
    CLEANUP_NVS_CONFIGS = 0x53
    TOGGLE_MCU_GPIO = 0x54

    SCHEDULE_TASK = 0x60
    GET_TASK_STATUS = 0x61

    