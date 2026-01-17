#!/usr/bin/env python3
"""Config Sync Test Script - Cloud service for testing device config sync."""

import asyncio
import dataclasses
import enum
import json
import os
import random
import signal
import time
import typing
from pathlib import Path
from collections import deque

import asyncclick as click
import paho.mqtt.client as mqtt_client
import paho.mqtt.enums
from watchdog.events import FileSystemEventHandler, FileSystemEvent
from watchdog.observers import Observer

import admin_pb2
import data_types_pb2
import telemetry_pb2
from data_types.service import TelemetryCommand, Topic, AdminCommand


def info(msg: str) -> None:
    click.secho(msg, fg="green")


def error(msg: str) -> None:
    click.secho(msg, fg="red")


def warn(msg: str) -> None:
    click.secho(msg, fg="yellow")


def debug(msg: str) -> None:
    click.secho(msg, fg="cyan")


def state_change(msg: str) -> None:
    click.secho(f"[STATE] {msg}", fg="magenta", bold=True)


class CloudSyncState(enum.Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    REQUESTING_CONFIG = "requesting_config"
    AWAITING_CONFIG = "awaiting_config"
    SYNCING_TO_DEVICE = "syncing_to_device"
    PROCESSING_REPORT = "processing_report"
    CLEANING_NVS = "cleaning_nvs"
    ERROR = "error"


@dataclasses.dataclass
class MqttConnectedEvent:
    pass


@dataclasses.dataclass
class MqttDisconnectedEvent:
    pass


@dataclasses.dataclass
class ConfigRequestEvent:
    psn: str
    reason: str


@dataclasses.dataclass
class ConfigReportEvent:
    psn: str
    config: data_types_pb2.MachineConfig


@dataclasses.dataclass
class FileChangeEvent:
    psn: str
    file_path: str


@dataclasses.dataclass
class TimeoutEvent:
    operation: str


@dataclasses.dataclass
class ErrorEvent:
    error_msg: str


@dataclasses.dataclass
class CleanupRequestEvent:
    force: bool = False


class CloudSyncStateMachine:
    """Simple state machine for config sync operations."""

    def __init__(self):
        self._state = CloudSyncState.DISCONNECTED
        self._pending_events = deque()
        self._timeout_handle = None

    @property
    def state(self) -> CloudSyncState:
        return self._state

    def transition(self, to_state: CloudSyncState, reason: str = "") -> None:
        old_state = self._state
        self._state = to_state
        reason_str = f" ({reason})" if reason else ""
        state_change(f"{old_state.value} -> {to_state.value}{reason_str}")

        if self._timeout_handle:
            self._timeout_handle.cancel()
            self._timeout_handle = None

    def is_state(self, *states: CloudSyncState) -> bool:
        return self._state in states

    def queue_event(self, event) -> None:
        self._pending_events.append(event)

    def get_pending_event(self):
        return self._pending_events.popleft() if self._pending_events else None

    def clear_pending_events(self) -> None:
        self._pending_events.clear()

    def set_timeout(self, seconds: float, callback, loop=None) -> None:
        if self._timeout_handle:
            self._timeout_handle.cancel()
        if loop:
            self._timeout_handle = loop.call_later(seconds, callback)


def config_to_dict(config: data_types_pb2.MachineConfig) -> dict:
    """Convert protobuf MachineConfig to dictionary."""
    result = {"timestamp": config.timestamp, "ports": []}

    for port in config.ports:
        port_dict = {
            "port_id": port.port_id,
            "config": {
                "enabled": port.config.enabled,
                "features": {},
                "custom_pdo_voltage_mv": port.config.custom_pdo_voltage_mv,
            },
        }

        features = port.config.features
        if features:
            port_dict["config"]["features"] = {
                "tfcp": features.enable_tfcp,
                "pe": features.enable_pe,
                "qc2p0": features.enable_qc2p0,
                "qc3p0": features.enable_qc3p0,
                "qc3plus": features.enable_qc3plus,
                "afc": features.enable_afc,
                "fcp": features.enable_fcp,
                "hv_scp": features.enable_hv_scp,
                "lv_scp": features.enable_lv_scp,
                "sfcp": features.enable_sfcp,
                "apple": features.enable_apple,
                "samsung": features.enable_samsung,
                "ufcs": features.enable_ufcs,
                "pd": features.enable_pd,
                "pd_compat_mode": features.enable_pd_compat_mode,
                "limited_current_mode": features.limited_current_mode,
                "pd_lvpps": features.enable_pd_lvpps,
                "pd_epr": features.enable_pd_epr,
                "pd_rpi": features.enable_pd_rpi,
                "pd_hvpps": features.enable_pd_hvpps,
                "pd_hard_reset_on_refusal": features.enable_pd_hard_reset_on_refusal,
                "pd_spr_avs": features.enable_pd_spr_avs,
            }

        result["ports"].append(port_dict)

    if config.HasField("charging"):
        charging = config.charging
        charging_dict = {"strategy_type": charging.strategy_type}

        if charging.HasField("identifier"):
            charging_dict["identifier"] = charging.identifier

        if charging.HasField("static_table"):
            charging_dict["static_table"] = {
                "version": charging.static_table.version,
                "rows": [
                    {"allocations": list(entry.power_allocation)}
                    for entry in charging.static_table.entries
                ],
            }

        result["charging"] = charging_dict

    if config.HasField("display"):
        display = config.display
        result["display"] = {
            "mode": display.mode,
            "intensity": display.intensity,
            "flip": display.flip,
            "rotation": display.rotation,
        }

    if config.temperature_mode:
        result["temperature_mode"] = config.temperature_mode

    return result


def dict_to_config(data: dict) -> data_types_pb2.MachineConfig:
    """Convert dictionary to protobuf MachineConfig."""
    config = data_types_pb2.MachineConfig()
    config.timestamp = int(data.get("timestamp", time.time()))

    for port_data in data.get("ports", []):
        port_config = config.ports.add()
        port_config.port_id = port_data["port_id"]
        port_config.config.enabled = port_data["config"]["enabled"]
        port_config.config.custom_pdo_voltage_mv = port_data["config"].get(
            "custom_pdo_voltage_mv", 0
        )

        features = port_data["config"].get("features", {})
        if features:
            f = port_config.config.features
            f.enable_tfcp = features.get("tfcp", False)
            f.enable_pe = features.get("pe", False)
            f.enable_qc2p0 = features.get("qc2p0", False)
            f.enable_qc3p0 = features.get("qc3p0", False)
            f.enable_qc3plus = features.get("qc3plus", False)
            f.enable_afc = features.get("afc", False)
            f.enable_fcp = features.get("fcp", False)
            f.enable_hv_scp = features.get("hv_scp", False)
            f.enable_lv_scp = features.get("lv_scp", False)
            f.enable_sfcp = features.get("sfcp", False)
            f.enable_apple = features.get("apple", False)
            f.enable_samsung = features.get("samsung", False)
            f.enable_ufcs = features.get("ufcs", False)
            f.enable_pd = features.get("pd", False)
            f.enable_pd_compat_mode = features.get("pd_compat_mode", False)
            f.limited_current_mode = features.get("limited_current_mode", False)
            f.enable_pd_lvpps = features.get("pd_lvpps", False)
            f.enable_pd_epr = features.get("pd_epr", False)
            f.enable_pd_rpi = features.get("pd_rpi", False)
            f.enable_pd_hvpps = features.get("pd_hvpps", False)
            f.enable_pd_hard_reset_on_refusal = features.get(
                "pd_hard_reset_on_refusal", False
            )
            f.enable_pd_spr_avs = features.get("pd_spr_avs", False)

    charging_data = data.get("charging", {})
    if charging_data:
        config.charging.strategy_type = charging_data.get("strategy_type", 0)
        config.charging.identifier = charging_data.get("identifier", 0)

        static_table_data = charging_data.get("static_table", {})
        if static_table_data:
            config.charging.static_table.version = static_table_data.get("version", 0)
            for row_data in static_table_data.get("rows", []):
                entry = config.charging.static_table.entries.add()
                entry.power_allocation.extend(row_data.get("allocations", []))

    display_data = data.get("display", {})
    if display_data:
        config.display.mode = display_data.get("mode", 0)
        config.display.intensity = display_data.get("intensity", 0)
        config.display.flip = display_data.get("flip", 0)
        config.display.rotation = display_data.get("rotation", 0)

    temperature_mode = data.get("temperature_mode", 0)
    if temperature_mode:
        config.temperature_mode = temperature_mode

    return config


@dataclasses.dataclass
class ConfigStore:
    """Simple config storage handler."""

    base_dir: Path = Path("device_configs")
    _internal_saves: dict = dataclasses.field(default_factory=dict)

    def __post_init__(self):
        self.base_dir.mkdir(exist_ok=True)

    def get_config_path(self, psn: str) -> Path:
        return self.base_dir / f"{psn}_config.json"

    def save_config(
        self, psn: str, config: data_types_pb2.MachineConfig, internal: bool = False
    ) -> None:
        config_path = self.get_config_path(psn)

        if internal:
            self._internal_saves[str(config_path)] = time.time()

        with open(config_path, "w") as f:
            json.dump(config_to_dict(config), f, indent=2)

        info(f"Saved config to {config_path}")

    def load_config(self, psn: str) -> typing.Optional[data_types_pb2.MachineConfig]:
        config_path = self.get_config_path(psn)

        if not config_path.exists():
            return None

        try:
            with open(config_path, "r") as f:
                return dict_to_config(json.load(f))
        except Exception as e:
            error(f"Failed to load config: {e}")
            return None

    def is_internal_save(self, file_path: str) -> bool:
        if file_path not in self._internal_saves:
            return False

        save_time = self._internal_saves[file_path]
        if time.time() - save_time < 5.0:
            del self._internal_saves[file_path]
            return True

        del self._internal_saves[file_path]
        return False

    def has_config(self, psn: str) -> bool:
        return self.get_config_path(psn).exists()


class ConfigFileHandler(FileSystemEventHandler):
    """File watcher for config changes."""

    def __init__(self, event_callback, config_store: ConfigStore):
        self.event_callback = event_callback
        self.config_store = config_store
        self.last_modified = {}
        self.debounce_delay = 2.0
        self.last_saved_timestamps = {}

    def on_modified(self, event: FileSystemEvent) -> None:
        if event.is_directory:
            return

        src_path = str(event.src_path)
        filename = os.path.basename(src_path)
        if not filename.endswith("_config.json"):
            return

        psn = filename.replace("_config.json", "")

        current_time = time.time()
        if current_time - self.last_modified.get(psn, 0) < self.debounce_delay:
            return

        self.last_modified[psn] = current_time
        self.event_callback(FileChangeEvent(psn=psn, file_path=src_path))

    def record_saved_config(self, psn: str, timestamp: int):
        self.last_saved_timestamps[psn] = timestamp

    def is_our_save(self, psn: str, timestamp: int) -> bool:
        return self.last_saved_timestamps.get(psn) == timestamp


@dataclasses.dataclass
class MQTTHandler:
    """MQTT communication handler."""

    broker: str
    port: int
    psn: str
    ca_certs: str
    certfile: str
    keyfile: str
    client: typing.Optional[mqtt_client.Client] = None
    event_callback: typing.Optional[typing.Callable] = None

    def connect(self):
        if self.client is None:
            self.client = mqtt_client.Client(
                paho.mqtt.enums.CallbackAPIVersion.VERSION2,
                protocol=mqtt_client.MQTTv5,
                client_id=f"config-sync-test-{random.randint(0, 1000)}",
            )

            self.client.on_connect = self._on_connect
            self.client.on_message = self._on_message
            self.client.on_disconnect = self._on_disconnect

            if self.broker not in (
                "localhost",
                "127.0.0.1",
            ) and not self._is_ip_address(self.broker):
                self.client.tls_set(
                    ca_certs=self.ca_certs, certfile=self.certfile, keyfile=self.keyfile
                )

            self.client.loop_start()

        try:
            self.client.connect(self.broker, self.port, 60)
        except Exception as e:
            error(f"Failed to connect to MQTT: {e}")

    def _is_ip_address(self, host: str) -> bool:
        try:
            parts = host.split(".")
            return len(parts) == 4 and all(0 <= int(p) < 256 for p in parts)
        except (ValueError, AttributeError):
            return False

    def disconnect(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, properties):
        if rc == 0:
            info("Connected to MQTT broker")
            topic = f"device/{self.psn}/telemetry/+"
            client.subscribe(topic, 0)
            info(f"Subscribed to {topic}")

            if self.event_callback:
                self.event_callback(MqttConnectedEvent())
        else:
            error(f"Failed to connect, return code {rc}")
            if self.event_callback:
                self.event_callback(ErrorEvent(f"MQTT connection failed: {rc}"))

    def _on_disconnect(self, client, userdata, rc, properties=None, flags=None):
        warn(f"Disconnected from MQTT broker (rc={rc})")
        if self.event_callback:
            self.event_callback(MqttDisconnectedEvent())

    def _on_message(self, client, userdata, msg):
        try:
            topic_obj = Topic.parse(msg.topic)

            if not isinstance(topic_obj.command, TelemetryCommand):
                return

            telemetry_msg = telemetry_pb2.TelemetryData()
            telemetry_msg.ParseFromString(msg.payload)

            if topic_obj.command == TelemetryCommand.CONFIG_REQUEST:
                debug("Received ConfigRequest")
                if self.event_callback:
                    reason = (
                        telemetry_msg.config_request.reason
                        if telemetry_msg.config_request.reason
                        else ""
                    )
                    self.event_callback(
                        ConfigRequestEvent(psn=topic_obj.psn, reason=reason)
                    )

            elif topic_obj.command == TelemetryCommand.CONFIG_REPORT:
                debug("Received ConfigReport")
                if self.event_callback and telemetry_msg.config_report.config:
                    self.event_callback(
                        ConfigReportEvent(
                            psn=topic_obj.psn, config=telemetry_msg.config_report.config
                        )
                    )

        except Exception as e:
            error(f"Error handling message: {e}")
            if self.event_callback:
                self.event_callback(ErrorEvent(f"Message handling error: {e}"))

    def send_set_machine_config(
        self,
        operation: data_types_pb2.ConfigOperation,
        config: typing.Optional[data_types_pb2.MachineConfig] = None,
    ) -> bool:
        if not self.client:
            error("MQTT client not connected")
            return False

        req = admin_pb2.AdminCommandRequest()
        req.set_machine_config.operation = operation

        if operation == data_types_pb2.ConfigOperation.CONFIG_UPDATE and config:
            req.set_machine_config.config.CopyFrom(config)

        topic = (
            f"device/{self.psn}/admin/request/{AdminCommand.SET_MACHINE_CONFIG.value}"
        )
        payload = req.SerializeToString()

        result = self.client.publish(topic, payload, qos=1)

        if result.rc == mqtt_client.MQTT_ERR_SUCCESS:
            if operation == data_types_pb2.ConfigOperation.CONFIG_UPDATE and config:
                info(f"Sent CONFIG_UPDATE with timestamp {config.timestamp}")
            else:
                info("Sent CONFIG_REQUEST_UPLOAD")
            return True
        else:
            error(f"Failed to send message: {result.rc}")
            return False


class ConfigSyncCloud:
    """Main config sync orchestrator."""

    def __init__(
        self,
        psn: str,
        broker: str = "localhost",
        port: int = 1883,
        ca_certs: str = "certs/ca.crt",
        certfile: str = "certs/client.pem",
        keyfile: str = "certs/client.key",
        auto_cleanup: bool = True,
    ):
        self.psn = psn
        self.config_store = ConfigStore()
        self.state_machine = CloudSyncStateMachine()
        self.mqtt_handler = MQTTHandler(broker, port, psn, ca_certs, certfile, keyfile)
        self.auto_cleanup_enabled = auto_cleanup
        self.mqtt_handler.event_callback = self.queue_event
        self.file_observer = Observer()
        self.file_handler = None
        self.running = False
        self.loop = None

        self.CONFIG_REQUEST_TIMEOUT = 30.0
        self.RECONNECT_DELAY = 5.0

    async def start(self):
        self.running = True
        self.loop = asyncio.get_running_loop()

        def signal_handler(signum, frame):
            if self.running:
                info("\nReceived interrupt signal, shutting down gracefully...")
                self.running = False

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        state_change(f"Initial state: {self.state_machine.state.value}")

        self.state_machine.transition(CloudSyncState.CONNECTING, "Starting service")
        self.mqtt_handler.connect()

        self.file_handler = ConfigFileHandler(self.queue_event, self.config_store)
        self.file_observer.schedule(
            self.file_handler, str(self.config_store.base_dir), recursive=False
        )
        self.file_observer.start()

        info(f"Config sync cloud started for device {self.psn}")
        info(f"Config directory: {self.config_store.base_dir}")
        info(f"Auto-cleanup: {'ENABLED' if self.auto_cleanup_enabled else 'DISABLED'}")
        info("Press Ctrl+C to stop...\n")

        while self.running:
            await asyncio.sleep(0.1)

            event = self.state_machine.get_pending_event()
            if event:
                await self.handle_event_async(event)

    def stop(self):
        self.running = False
        self.file_observer.stop()
        self.file_observer.join()
        self.mqtt_handler.disconnect()
        self.state_machine.transition(CloudSyncState.DISCONNECTED, "Service stopped")
        info("Config sync cloud stopped")

    def cleanup_local_files(self):
        config_path = self.config_store.get_config_path(self.psn)
        if config_path.exists():
            info(f"  Removing config file: {config_path.name}")
            config_path.unlink()
        else:
            info(f"  No config file found for PSN {self.psn}")

    def queue_event(self, event):
        if self.loop and not self.loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.handle_event_async(event), self.loop)
        else:
            warn("No event loop available, dropping event")

    async def handle_event_async(self, event):
        """Handle events using match statement."""
        debug(
            f"Event: {type(event).__name__} in state {self.state_machine.state.value}"
        )

        match event:
            case MqttConnectedEvent():
                await self._handle_mqtt_connected()
            case MqttDisconnectedEvent():
                await self._handle_mqtt_disconnected()
            case ConfigRequestEvent():
                await self._handle_config_request(event)
            case ConfigReportEvent():
                await self._handle_config_report(event)
            case FileChangeEvent():
                await self._handle_file_change(event)
            case CleanupRequestEvent():
                await self._handle_cleanup_request(event)
            case TimeoutEvent():
                await self._handle_timeout(event)
            case ErrorEvent():
                await self._handle_error(event)

    async def _handle_mqtt_connected(self):
        if self.state_machine.is_state(
            CloudSyncState.CONNECTING, CloudSyncState.DISCONNECTED
        ):
            self.state_machine.transition(CloudSyncState.IDLE, "MQTT connected")
        else:
            warn(f"Unexpected MQTT connected in state {self.state_machine.state.value}")

    async def _handle_mqtt_disconnected(self):
        if not self.state_machine.is_state(CloudSyncState.DISCONNECTED):
            self.state_machine.transition(
                CloudSyncState.DISCONNECTED, "MQTT disconnected"
            )
            self.state_machine.clear_pending_events()

            if self.loop:
                info(f"Will attempt reconnection in {self.RECONNECT_DELAY} seconds")
                self.loop.call_later(self.RECONNECT_DELAY, self.attempt_reconnect)

    async def _handle_config_request(self, event: ConfigRequestEvent):
        if not self.state_machine.is_state(CloudSyncState.IDLE):
            self.state_machine.queue_event(event)
            return

        info(f"Device {event.psn} requested config (reason: '{event.reason}')")

        if self.config_store.has_config(event.psn):
            stored_config = self.config_store.load_config(event.psn)
            if stored_config:
                await self._send_config_to_device(stored_config)
            else:
                error("Failed to load stored config")
                self.request_device_upload(event.psn)
        else:
            self.request_device_upload(event.psn)

    async def _send_config_to_device(self, config: data_types_pb2.MachineConfig):
        self.state_machine.transition(
            CloudSyncState.SYNCING_TO_DEVICE, "Sending stored config"
        )
        info(f"Sending stored config (timestamp: {config.timestamp})")

        if self.mqtt_handler.send_set_machine_config(
            data_types_pb2.ConfigOperation.CONFIG_UPDATE, config
        ):
            self.state_machine.transition(CloudSyncState.IDLE, "Config sent")
        else:
            self.state_machine.transition(CloudSyncState.ERROR, "Failed to send config")

    async def _handle_config_report(self, event: ConfigReportEvent):
        if self.state_machine.is_state(CloudSyncState.AWAITING_CONFIG):
            self.state_machine.transition(
                CloudSyncState.PROCESSING_REPORT, "Received expected config"
            )
        elif self.state_machine.is_state(CloudSyncState.IDLE):
            self.state_machine.transition(
                CloudSyncState.PROCESSING_REPORT, "Processing unsolicited config"
            )
        else:
            self.state_machine.queue_event(event)
            return

        info(
            f"Device {event.psn} reported config (timestamp: {event.config.timestamp})"
        )

        is_first_config = not self.config_store.has_config(event.psn)

        if self.file_handler:
            self.file_handler.record_saved_config(event.psn, event.config.timestamp)

        self.config_store.save_config(event.psn, event.config, internal=True)
        self.print_config_summary(event.config)

        if is_first_config and self.auto_cleanup_enabled:
            info("=== FIRST CONFIG RECEIVED FROM DEVICE ===")
            info("Scheduling automatic NVS cleanup after 2 seconds...")
            if self.loop:
                self.loop.call_later(
                    2.0, lambda: self.queue_event(CleanupRequestEvent(force=False))
                )

        self.state_machine.transition(CloudSyncState.IDLE, "Config processed")

    async def _handle_file_change(self, event: FileChangeEvent):
        if event.psn != self.psn:
            return

        if not self.state_machine.is_state(CloudSyncState.IDLE):
            if self.state_machine.is_state(CloudSyncState.PROCESSING_REPORT):
                debug("Ignoring file change during config processing")
                return
            self.state_machine.queue_event(event)
            return

        config = self.config_store.load_config(event.psn)
        if not config:
            error("Failed to load updated config")
            return

        if self.file_handler and self.file_handler.is_our_save(
            event.psn, config.timestamp
        ):
            debug(f"Ignoring our own save (timestamp: {config.timestamp})")
            return

        info(f"Config file changed for device {event.psn}")
        config.timestamp = int(time.time())

        self.state_machine.transition(
            CloudSyncState.SYNCING_TO_DEVICE, "Sending file change"
        )
        info(f"Sending updated config (timestamp: {config.timestamp})")

        if self.mqtt_handler.send_set_machine_config(
            data_types_pb2.ConfigOperation.CONFIG_UPDATE, config
        ):
            self.state_machine.transition(CloudSyncState.IDLE, "File change sent")
        else:
            self.state_machine.transition(
                CloudSyncState.ERROR, "Failed to send file change"
            )

    async def _handle_cleanup_request(self, event: CleanupRequestEvent):
        if not self.state_machine.is_state(CloudSyncState.IDLE):
            self.state_machine.queue_event(event)
            return

        info(f"Sending NVS cleanup command (force={event.force})")
        self.state_machine.transition(CloudSyncState.CLEANING_NVS, "Sending cleanup")

        req = admin_pb2.AdminCommandRequest()
        req.cleanup_nvs_configs.force = event.force

        try:
            result = self.mqtt_handler.client.publish(
                f"device/{self.psn}/admin/request/{AdminCommand.CLEANUP_NVS_CONFIGS.value}",
                req.SerializeToString(),
            )
            if result.rc == 0:
                info("Cleanup command sent successfully")
            else:
                warn(f"Cleanup command sent with result code: {result.rc}")
            self.state_machine.transition(CloudSyncState.IDLE, "Cleanup sent")
        except Exception as e:
            error(f"Failed to send cleanup command: {e}")
            self.state_machine.transition(
                CloudSyncState.ERROR, "Failed to send cleanup"
            )

    async def _handle_timeout(self, event: TimeoutEvent):
        warn(f"Timeout occurred: {event.operation}")

        if self.state_machine.is_state(CloudSyncState.AWAITING_CONFIG):
            warn("Device did not respond to config request")
            self.state_machine.transition(CloudSyncState.IDLE, "Timeout - no response")

    async def _handle_error(self, event: ErrorEvent):
        error(f"Error: {event.error_msg}")

        if not self.state_machine.is_state(CloudSyncState.ERROR):
            self.state_machine.transition(CloudSyncState.ERROR, event.error_msg)

        if self.loop:
            self.loop.call_later(5.0, self.attempt_recovery)

    def attempt_reconnect(self):
        if self.state_machine.is_state(CloudSyncState.DISCONNECTED):
            info("Attempting to reconnect to MQTT...")
            self.state_machine.transition(CloudSyncState.CONNECTING, "Reconnecting")
            self.mqtt_handler.connect()

    def request_device_upload(self, psn: str):
        if not self.state_machine.is_state(CloudSyncState.IDLE):
            warn(f"Cannot request upload in state {self.state_machine.state.value}")
            return

        info("No stored config found, requesting device upload")
        self.state_machine.transition(
            CloudSyncState.REQUESTING_CONFIG, "Requesting device config"
        )

        if self.mqtt_handler.send_set_machine_config(
            data_types_pb2.ConfigOperation.CONFIG_REQUEST_UPLOAD
        ):
            self.state_machine.transition(
                CloudSyncState.AWAITING_CONFIG, "Waiting for response"
            )
            self.state_machine.set_timeout(
                self.CONFIG_REQUEST_TIMEOUT,
                lambda: self.queue_event(TimeoutEvent("CONFIG_REQUEST")),
                self.loop,
            )
        else:
            self.state_machine.transition(
                CloudSyncState.ERROR, "Failed to request config"
            )

    def attempt_recovery(self):
        if self.state_machine.is_state(CloudSyncState.ERROR):
            info("Attempting to recover from error state")
            if self.mqtt_handler.client and self.mqtt_handler.client.is_connected():
                self.state_machine.transition(CloudSyncState.IDLE, "Recovered")
            else:
                self.state_machine.transition(
                    CloudSyncState.DISCONNECTED, "Need reconnection"
                )
                self.attempt_reconnect()

    def print_config_summary(self, config: data_types_pb2.MachineConfig):
        info("Config summary:")
        info(f"  - Timestamp: {config.timestamp}")
        info(f"  - Ports: {len(config.ports)}")
        for port in config.ports:
            enabled_str = "enabled" if port.config.enabled else "disabled"
            info(f"    - Port {port.port_id}: {enabled_str}")
        if config.charging:
            info(
                f"  - Charging strategy: type={config.charging.strategy_type}, "
                f"id={config.charging.identifier}"
            )
        if config.HasField("display"):
            info(f"  - Display: mode={config.display.mode}, intensity={config.display.intensity}, "
                 f"flip={config.display.flip}, rotation={config.display.rotation}")
        info(f"  - Temperature mode: {config.temperature_mode}")


@click.command()
@click.option("--psn", required=True, help="Device PSN to sync with")
@click.option("--broker", default="localhost", help="MQTT broker address")
@click.option("--port", default=1883, type=int, help="MQTT broker port")
@click.option("--ca-certs", default="certs/ca.crt", help="CA certificate file")
@click.option("--certfile", default="certs/client.pem", help="Client certificate file")
@click.option("--keyfile", default="certs/client.key", help="Client key file")
@click.option(
    "--auto-cleanup/--no-auto-cleanup",
    default=True,
    help="Automatically cleanup NVS after first config upload",
)
@click.option(
    "--cleanup-files-on-exit/--keep-files-on-exit",
    default=False,
    help="Clean up local config files when program exits",
)
async def main(
    psn: str,
    broker: str,
    port: int,
    ca_certs: str,
    certfile: str,
    keyfile: str,
    auto_cleanup: bool,
    cleanup_files_on_exit: bool,
):
    """Config Sync Test - Cloud service for testing device config sync."""
    cloud = ConfigSyncCloud(
        psn, broker, port, ca_certs, certfile, keyfile, auto_cleanup
    )

    try:
        await cloud.start()
    except asyncio.CancelledError:
        info("\nService cancelled")
    except Exception as e:
        error(f"Unexpected error: {e}")
    finally:
        info("\nPerforming cleanup...")
        cloud.stop()

        if cleanup_files_on_exit:
            info("Cleaning up local config files...")
            cloud.cleanup_local_files()
            info("Config files cleaned up")

        info("Shutdown complete")


if __name__ == "__main__":
    main()
