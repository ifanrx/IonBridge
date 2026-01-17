#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Reset device options model."""

import dataclasses
import typing

import asyncclick as click
import command_pb2


@dataclasses.dataclass
class ResetDeviceOptions:
    """Data class to handle reset device options"""

    # NVS namespaces to keep after reset
    keep_wifi_nvs: typing.Optional[bool] = None
    keep_mqtt_nvs: typing.Optional[bool] = None
    keep_ble_nvs: typing.Optional[bool] = None
    keep_display_nvs: typing.Optional[bool] = None
    keep_power_nvs: typing.Optional[bool] = None
    keep_system_nvs: typing.Optional[bool] = None
    keep_license_nvs: typing.Optional[bool] = None
    keep_syslog_nvs: typing.Optional[bool] = None
    keep_fpga_nvs: typing.Optional[bool] = None
    keep_ota_nvs: typing.Optional[bool] = None

    # Reset operations
    prevent_reboot: typing.Optional[bool] = None
    reset_ports: typing.Optional[bool] = None
    reset_power_strategy: typing.Optional[bool] = None
    reset_wifi: typing.Optional[bool] = None
    reset_display: typing.Optional[bool] = None
    reset_fc_protocol: typing.Optional[bool] = None
    reset_temperature_mode: typing.Optional[bool] = None
    reset_cable_compensation: typing.Optional[bool] = None

    @classmethod
    def from_kwargs(cls, **kwargs) -> "ResetDeviceOptions":
        """Create ResetDeviceOptions from CLI kwargs"""
        # Handle the keep-all-nvs shortcut
        if kwargs.pop("keep_all_nvs", False):
            for key in kwargs:
                if key.startswith("keep_") and key.endswith("_nvs"):
                    kwargs[key] = True

        # Convert CLI flag values (False when not provided) to None for optional behavior
        def flag_to_optional(value):
            return value if value else None

        return cls(
            keep_wifi_nvs=flag_to_optional(kwargs.get("keep_wifi_nvs")),
            keep_mqtt_nvs=flag_to_optional(kwargs.get("keep_mqtt_nvs")),
            keep_ble_nvs=flag_to_optional(kwargs.get("keep_ble_nvs")),
            keep_display_nvs=flag_to_optional(kwargs.get("keep_display_nvs")),
            keep_power_nvs=flag_to_optional(kwargs.get("keep_power_nvs")),
            keep_system_nvs=flag_to_optional(kwargs.get("keep_system_nvs")),
            keep_license_nvs=flag_to_optional(kwargs.get("keep_license_nvs")),
            keep_syslog_nvs=flag_to_optional(kwargs.get("keep_syslog_nvs")),
            keep_fpga_nvs=flag_to_optional(kwargs.get("keep_fpga_nvs")),
            keep_ota_nvs=flag_to_optional(kwargs.get("keep_ota_nvs")),
            prevent_reboot=flag_to_optional(kwargs.get("prevent_reboot")),
            reset_ports=flag_to_optional(kwargs.get("reset_ports")),
            reset_power_strategy=flag_to_optional(kwargs.get("reset_power_strategy")),
            reset_wifi=flag_to_optional(kwargs.get("reset_wifi")),
            reset_display=flag_to_optional(kwargs.get("reset_display")),
            reset_fc_protocol=flag_to_optional(kwargs.get("reset_fc_protocol")),
            reset_temperature_mode=flag_to_optional(
                kwargs.get("reset_temperature_mode")
            ),
            reset_cable_compensation=flag_to_optional(
                kwargs.get("reset_cable_compensation")
            ),
        )

    def display_summary(self):
        """Display a summary of what will be preserved and reset"""
        click.echo("\nReset Summary:")

        # Show NVS namespaces that will be kept
        kept_nvs = []
        for field_name, value in self.__dict__.items():
            if field_name.startswith("keep_") and field_name.endswith("_nvs") and value:
                namespace = field_name.replace("keep_", "").replace("_nvs", "")
                kept_nvs.append(namespace)

        if kept_nvs:
            click.secho(f"  NVS to keep: {', '.join(kept_nvs)}", fg="green")
        else:
            click.secho("  All NVS namespaces will be cleared", fg="red")

        # Show reset operations
        reset_ops = []
        if self.reset_ports:
            reset_ops.append("ports")
        if self.reset_power_strategy:
            reset_ops.append("power strategy")
        if self.reset_wifi:
            reset_ops.append("wifi")
        if self.reset_display:
            reset_ops.append("display")
        if self.reset_fc_protocol:
            reset_ops.append("fc protocol")
        if self.reset_temperature_mode:
            reset_ops.append("temperature mode")
        if self.reset_cable_compensation:
            reset_ops.append("cable compensation")

        if reset_ops:
            click.secho(f"  Additional resets: {', '.join(reset_ops)}", fg="yellow")

        if self.prevent_reboot:
            click.secho("  Device will NOT reboot after reset", fg="blue")
        else:
            click.secho("  Device will reboot after reset", fg="red")

        click.echo()

    def to_protobuf(self, req: command_pb2.CommandRequest):
        """Apply options to the protobuf request"""
        # NVS namespace options
        if self.keep_wifi_nvs is not None:
            req.reset_device.keep_wifi_nvs = self.keep_wifi_nvs
        if self.keep_mqtt_nvs is not None:
            req.reset_device.keep_mqtt_nvs = self.keep_mqtt_nvs
        if self.keep_ble_nvs is not None:
            req.reset_device.keep_ble_nvs = self.keep_ble_nvs
        if self.keep_display_nvs is not None:
            req.reset_device.keep_display_nvs = self.keep_display_nvs
        if self.keep_power_nvs is not None:
            req.reset_device.keep_power_nvs = self.keep_power_nvs
        if self.keep_system_nvs is not None:
            req.reset_device.keep_system_nvs = self.keep_system_nvs
        if self.keep_license_nvs is not None:
            req.reset_device.keep_license_nvs = self.keep_license_nvs
        if self.keep_syslog_nvs is not None:
            req.reset_device.keep_syslog_nvs = self.keep_syslog_nvs
        if self.keep_fpga_nvs is not None:
            req.reset_device.keep_fpga_nvs = self.keep_fpga_nvs
        if self.keep_ota_nvs is not None:
            req.reset_device.keep_ota_nvs = self.keep_ota_nvs

        # Reset operation options
        if self.prevent_reboot is not None:
            req.reset_device.prevent_reboot = self.prevent_reboot
        if self.reset_ports is not None:
            req.reset_device.reset_ports = self.reset_ports
        if self.reset_power_strategy is not None:
            req.reset_device.reset_power_strategy = self.reset_power_strategy
        if self.reset_wifi is not None:
            req.reset_device.reset_wifi = self.reset_wifi
        if self.reset_display is not None:
            req.reset_device.reset_display = self.reset_display
        if self.reset_fc_protocol is not None:
            req.reset_device.reset_fc_protocol = self.reset_fc_protocol
        if self.reset_temperature_mode is not None:
            req.reset_device.reset_temperature_mode = self.reset_temperature_mode
        if self.reset_cable_compensation is not None:
            req.reset_device.reset_cable_compensation = self.reset_cable_compensation
