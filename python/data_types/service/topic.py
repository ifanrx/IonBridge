#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import dataclasses
import typing

from .admin import AdminCommand
from .commands import ServiceCommand
from .telemetry import TelemetryCommand


@dataclasses.dataclass
class Topic:
    psn: str
    command: typing.Union[ServiceCommand, TelemetryCommand, AdminCommand]
    is_request: bool

    @classmethod
    def parse(cls, topic: str) -> "Topic":
        """
        解析 topic
        :param topic: topic
        :return: Topic
        """
        # device/<psn>/enduser/request/<command>
        # device/<psn>/enduser/response/<command>
        # device/<psn>/admin/request/<command>
        # device/<psn>/admin/response/<command>
        # device/<psn>/telemetry/<command>
        parts = topic.split("/")
        psn = parts[1]
        source = parts[2]
        command = parts[-1]
        command = int(command)
        is_request = False
        match source:
            case "enduser":
                if len(parts) != 5 or parts[3] not in ("request", "response"):
                    raise ValueError(f"Invalid topic: {topic}")
                is_request = parts[3] == "request"
                command = ServiceCommand(command)
            case "admin":
                if len(parts) != 5 or parts[3] not in ("request", "response"):
                    raise ValueError(f"Invalid topic: {topic}")
                is_request = parts[3] == "request"
                command = AdminCommand(command)
            case "telemetry":
                if len(parts) != 4:
                    raise ValueError(f"Invalid topic: {topic}")
                is_request = False
                command = TelemetryCommand(command)
            case _:
                raise ValueError(f"Invalid topic: {topic}")
        return cls(psn=psn, command=command, is_request=is_request)

    def to_topic(self) -> str:
        """
        将 Topic 转换为 topic
        :return: topic
        """
        part = "request" if self.is_request else "response"
        match self.command:
            case ServiceCommand():
                return f"device/{self.psn}/enduser/{part}/{self.command.value}"
            case AdminCommand():
                return f"device/{self.psn}/admin/{part}/{self.command.value}"
            case TelemetryCommand():
                return f"device/{self.psn}/telemetry/{self.command.value}"
        raise ValueError(f"Invalid command: {self.command}")
