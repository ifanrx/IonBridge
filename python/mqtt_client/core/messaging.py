#!/usr/bin/env python3
# -*- coding=utf-8 -*-
"""Message dataclasses for MQTT communication."""

import dataclasses
import typing

from google.protobuf.json_format import MessageToJson

import admin_pb2
import command_pb2
import telemetry_pb2
from data_types.service import AdminCommand, ServiceCommand, TelemetryCommand, Topic

# Global message ID counter
msg_id = 1


@dataclasses.dataclass
class Message:
    command: typing.Union[ServiceCommand, AdminCommand]
    msg_id: int = 0
    pb_message: typing.Union[
        command_pb2.CommandRequest, admin_pb2.AdminCommandRequest
    ] = dataclasses.field(default_factory=command_pb2.CommandRequest)

    def __post_init__(self):
        global msg_id
        self.msg_id = msg_id
        msg_id += 1
        msg_id &= 0xFFFF
        self.pb_message.id = self.msg_id

    def __str__(self):
        return f"Command: {self.command.name}, Msg ID: {self.msg_id}, Protobuf: {MessageToJson(self.pb_message)}"

    def serialize(self) -> bytes | bytearray:
        return self.pb_message.SerializeToString()


@dataclasses.dataclass
class ResponseMessage:
    command: typing.Union[ServiceCommand, AdminCommand, TelemetryCommand]
    msg_id: int
    status: int
    pb_message: typing.Union[
        command_pb2.CommandResponse,
        admin_pb2.AdminCommandResponse,
        telemetry_pb2.TelemetryData,
    ]
    payload: bytes | bytearray = b""

    @classmethod
    def deserialize(cls, topic: str, data: bytes):
        topic_obj = Topic.parse(topic)
        pb_cls = command_pb2.CommandResponse
        match topic_obj.command:
            case ServiceCommand():
                pb_cls = command_pb2.CommandResponse
            case AdminCommand():
                pb_cls = admin_pb2.AdminCommandResponse
            case TelemetryCommand():
                pb_cls = telemetry_pb2.TelemetryData
        pb_message = pb_cls()
        pb_message.ParseFromString(data)
        msg_id = 0
        status = 0
        if isinstance(
            pb_message, (command_pb2.CommandResponse, admin_pb2.AdminCommandResponse)
        ):
            msg_id = pb_message.id
            status = pb_message.status
        payload = data
        return cls(topic_obj.command, msg_id, status, pb_message, payload)

    def __str__(self):
        return f"Command: {self.command.name}, Msg ID: {self.msg_id}, Status: {self.status}, Protobuf: {MessageToJson(self.pb_message)}"

    def to_json(self):
        return MessageToJson(self.pb_message, always_print_fields_with_no_presence=True)

    def command_pb(self) -> command_pb2.CommandResponse:
        if isinstance(self.pb_message, command_pb2.CommandResponse):
            return self.pb_message
        raise ValueError("Not a command response message")

    def telemetry_pb(self) -> telemetry_pb2.TelemetryData:
        if isinstance(self.pb_message, telemetry_pb2.TelemetryData):
            return self.pb_message
        raise ValueError("Not a telemetry message")

    def admin_pb(self) -> admin_pb2.AdminCommandResponse:
        if isinstance(self.pb_message, admin_pb2.AdminCommandResponse):
            return self.pb_message
        raise ValueError("Not an admin command response message")
