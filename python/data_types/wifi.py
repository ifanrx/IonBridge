#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import enum


class WiFiAuthMode(enum.IntEnum):
    WIFI_AUTH_OPEN = 0  # authenticate mode : open
    WIFI_AUTH_WEP = 1  # authenticate mode : WEP
    WIFI_AUTH_WPA_PSK = 2  # authenticate mode : WPA_PSK
    WIFI_AUTH_WPA2_PSK = 3  # authenticate mode : WPA2_PSK
    WIFI_AUTH_WPA_WPA2_PSK = 4  # authenticate mode : WPA_WPA2_PSK
    WIFI_AUTH_ENTERPRISE = 5  # authenticate mode : WiFi EAP security
    WIFI_AUTH_WPA2_ENTERPRISE = (
        5  # authenticate mode : WiFi EAP security (same as WIFI_AUTH_ENTERPRISE)
    )
    WIFI_AUTH_WPA3_PSK = 6  # authenticate mode : WPA3_PSK
    WIFI_AUTH_WPA2_WPA3_PSK = 7  # authenticate mode : WPA2_WPA3_PSK
    WIFI_AUTH_WAPI_PSK = 8  # authenticate mode : WAPI_PSK
    WIFI_AUTH_OWE = 9  # authenticate mode : OWE
    WIFI_AUTH_WPA3_ENT_192 = 10  # authenticate mode : WPA3_ENT_SUITE_B_192_BIT
    WIFI_AUTH_MAX = 11  # Max value for authentication modes

    @classmethod
    def get_name_from_value(cls, value: int) -> str:
        """Return the name of the enum corresponding to the given value."""
        try:
            return cls(value).name
        except ValueError:
            return f"Unknown: {value}"


class WiFiStateType(enum.IntEnum):
    CONFIGURATION = 0
    SCAN_AND_CONNECT = 1
    CONNECTING = 2
    CONNECTED = 3
    CHECKING_CONN = 4
    IDLE = 5
    DISCONNECTED = 6
    RECONNECTING = 7
    WAITING_FOR_NEXT = 8
    SCANNING = 9
    CONNECTION_ABORT = 10
    WAITING_FOR_DEFAULT = 11
    COUNT = 12

    @classmethod
    def from_str(cls, state: str) -> int:
        """Return the enum corresponding to the given string."""
        return cls[state.upper()]

    @classmethod
    def from_int(cls, state: int) -> int:
        """Return the enum corresponding to the given integer."""
        if state >= cls.COUNT:
            raise ValueError(f"Invalid state: {state}")
        return cls(state)
