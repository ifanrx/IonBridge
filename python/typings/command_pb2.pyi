import data_types_pb2 as _data_types_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CommandRequest(_message.Message):
    __slots__ = ("id", "lock_swd", "unlock_swd", "enable_release_mode", "reboot_device", "reset_device", "get_device_psn", "get_ap_version", "set_power_state", "get_power_state", "get_device_info", "get_hardware_revision", "scan_wifi", "set_wifi_ssid", "set_wifi_password", "reset_wifi", "get_wifi_status", "get_wifi_mac", "set_wifi_ssid_and_password", "get_wifi_records", "operate_wifi_record", "get_wifi_state_machine", "set_wifi_state_machine", "set_charging_strategy", "get_charging_status", "get_power_historical_stats", "set_port_priority", "get_port_priority", "get_charging_strategy", "get_port_pd_status", "get_all_power_statistics", "turn_on_port", "turn_off_port", "set_static_allocator", "get_static_allocator", "set_port_config", "get_port_config", "set_port_compatibility_settings", "get_port_compatibility_settings", "set_temperature_mode", "set_temporary_allocator", "subscribe_pdpcap_streaming", "set_port_type", "set_custom_pdo_voltage", "set_cable_compensation", "get_cable_compensation", "set_display_intensity", "set_display_mode", "get_display_intensity", "get_display_mode", "set_display_flip", "get_display_flip", "set_display_config", "get_display_rotation", "set_display_rotation", "set_display_setup", "get_display_setup", "start_telemetry_stream", "stop_telemetry_stream", "set_ble_state", "start_ota", "start_programmer", "clear_factory_dm_flag")
    ID_FIELD_NUMBER: _ClassVar[int]
    LOCK_SWD_FIELD_NUMBER: _ClassVar[int]
    UNLOCK_SWD_FIELD_NUMBER: _ClassVar[int]
    ENABLE_RELEASE_MODE_FIELD_NUMBER: _ClassVar[int]
    REBOOT_DEVICE_FIELD_NUMBER: _ClassVar[int]
    RESET_DEVICE_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_PSN_FIELD_NUMBER: _ClassVar[int]
    GET_AP_VERSION_FIELD_NUMBER: _ClassVar[int]
    SET_POWER_STATE_FIELD_NUMBER: _ClassVar[int]
    GET_POWER_STATE_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    GET_HARDWARE_REVISION_FIELD_NUMBER: _ClassVar[int]
    SCAN_WIFI_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_SSID_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    RESET_WIFI_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_MAC_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_SSID_AND_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_RECORDS_FIELD_NUMBER: _ClassVar[int]
    OPERATE_WIFI_RECORD_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_STATE_MACHINE_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_STATE_MACHINE_FIELD_NUMBER: _ClassVar[int]
    SET_CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    GET_CHARGING_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_POWER_HISTORICAL_STATS_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_PRIORITY_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_PRIORITY_FIELD_NUMBER: _ClassVar[int]
    GET_CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_PD_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_ALL_POWER_STATISTICS_FIELD_NUMBER: _ClassVar[int]
    TURN_ON_PORT_FIELD_NUMBER: _ClassVar[int]
    TURN_OFF_PORT_FIELD_NUMBER: _ClassVar[int]
    SET_STATIC_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    GET_STATIC_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_CONFIG_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_CONFIG_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_COMPATIBILITY_SETTINGS_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_COMPATIBILITY_SETTINGS_FIELD_NUMBER: _ClassVar[int]
    SET_TEMPERATURE_MODE_FIELD_NUMBER: _ClassVar[int]
    SET_TEMPORARY_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    SUBSCRIBE_PDPCAP_STREAMING_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_TYPE_FIELD_NUMBER: _ClassVar[int]
    SET_CUSTOM_PDO_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    SET_CABLE_COMPENSATION_FIELD_NUMBER: _ClassVar[int]
    GET_CABLE_COMPENSATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_INTENSITY_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_MODE_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_INTENSITY_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_MODE_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_FLIP_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_FLIP_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_CONFIG_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_ROTATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_ROTATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_SETUP_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_SETUP_FIELD_NUMBER: _ClassVar[int]
    START_TELEMETRY_STREAM_FIELD_NUMBER: _ClassVar[int]
    STOP_TELEMETRY_STREAM_FIELD_NUMBER: _ClassVar[int]
    SET_BLE_STATE_FIELD_NUMBER: _ClassVar[int]
    START_OTA_FIELD_NUMBER: _ClassVar[int]
    START_PROGRAMMER_FIELD_NUMBER: _ClassVar[int]
    CLEAR_FACTORY_DM_FLAG_FIELD_NUMBER: _ClassVar[int]
    id: int
    lock_swd: LockSWDReq
    unlock_swd: UnlockSWDReq
    enable_release_mode: EnableReleaseModeReq
    reboot_device: RebootDeviceReq
    reset_device: ResetDeviceReq
    get_device_psn: GetDevicePSNReq
    get_ap_version: GetAPVersionReq
    set_power_state: SetDevicePowerStateReq
    get_power_state: GetDevicePowerStateReq
    get_device_info: GetDeviceInfoReq
    get_hardware_revision: GetHardwareRevisionReq
    scan_wifi: ScanWiFiReq
    set_wifi_ssid: SetWiFiSSIDReq
    set_wifi_password: SetWiFiPasswordReq
    reset_wifi: ResetWiFiReq
    get_wifi_status: GetWiFiStatusReq
    get_wifi_mac: GetWiFiMACReq
    set_wifi_ssid_and_password: SetWiFiSSIDAndPasswordReq
    get_wifi_records: GetWiFiRecordsReq
    operate_wifi_record: OperateWiFiRecordReq
    get_wifi_state_machine: GetWiFiStateMachineReq
    set_wifi_state_machine: SetWiFiStateMachineReq
    set_charging_strategy: SetChargingStrategyReq
    get_charging_status: GetChargingStatusReq
    get_power_historical_stats: GetPowerHistoricalStatsReq
    set_port_priority: SetPortPriorityReq
    get_port_priority: GetPortPriorityReq
    get_charging_strategy: GetChargingStrategyReq
    get_port_pd_status: GetPortPDStatusReq
    get_all_power_statistics: GetAllPowerStatisticsReq
    turn_on_port: TurnOnPortReq
    turn_off_port: TurnOffPortReq
    set_static_allocator: SetStaticAllocatorReq
    get_static_allocator: GetStaticAllocatorReq
    set_port_config: SetPortConfigReq
    get_port_config: GetPortConfigReq
    set_port_compatibility_settings: SetPortCompatibilitySettingsReq
    get_port_compatibility_settings: GetPortCompatibilitySettingsReq
    set_temperature_mode: SetTemperatureModeReq
    set_temporary_allocator: SetTemporaryAllocatorReq
    subscribe_pdpcap_streaming: SubscribePDPcapStreamingReq
    set_port_type: SetPortTypeReq
    set_custom_pdo_voltage: SetCustomPdoVoltageReq
    set_cable_compensation: SetCableCompensationReq
    get_cable_compensation: GetCableCompensationReq
    set_display_intensity: SetDisplayIntensityReq
    set_display_mode: SetDisplayModeReq
    get_display_intensity: GetDisplayIntensityReq
    get_display_mode: GetDisplayModeReq
    set_display_flip: SetDisplayFlipReq
    get_display_flip: GetDisplayFlipReq
    set_display_config: SetDisplayConfigReq
    get_display_rotation: GetDisplayRotationReq
    set_display_rotation: SetDisplayRotationReq
    set_display_setup: SetDisplaySetupReq
    get_display_setup: GetDisplaySetupReq
    start_telemetry_stream: StartTelemetryStreamReq
    stop_telemetry_stream: StopTelemetryStreamReq
    set_ble_state: SetBleStateReq
    start_ota: StartOTAReq
    start_programmer: StartProgrammerReq
    clear_factory_dm_flag: ClearFactoryDMFlagReq
    def __init__(self, id: _Optional[int] = ..., lock_swd: _Optional[_Union[LockSWDReq, _Mapping]] = ..., unlock_swd: _Optional[_Union[UnlockSWDReq, _Mapping]] = ..., enable_release_mode: _Optional[_Union[EnableReleaseModeReq, _Mapping]] = ..., reboot_device: _Optional[_Union[RebootDeviceReq, _Mapping]] = ..., reset_device: _Optional[_Union[ResetDeviceReq, _Mapping]] = ..., get_device_psn: _Optional[_Union[GetDevicePSNReq, _Mapping]] = ..., get_ap_version: _Optional[_Union[GetAPVersionReq, _Mapping]] = ..., set_power_state: _Optional[_Union[SetDevicePowerStateReq, _Mapping]] = ..., get_power_state: _Optional[_Union[GetDevicePowerStateReq, _Mapping]] = ..., get_device_info: _Optional[_Union[GetDeviceInfoReq, _Mapping]] = ..., get_hardware_revision: _Optional[_Union[GetHardwareRevisionReq, _Mapping]] = ..., scan_wifi: _Optional[_Union[ScanWiFiReq, _Mapping]] = ..., set_wifi_ssid: _Optional[_Union[SetWiFiSSIDReq, _Mapping]] = ..., set_wifi_password: _Optional[_Union[SetWiFiPasswordReq, _Mapping]] = ..., reset_wifi: _Optional[_Union[ResetWiFiReq, _Mapping]] = ..., get_wifi_status: _Optional[_Union[GetWiFiStatusReq, _Mapping]] = ..., get_wifi_mac: _Optional[_Union[GetWiFiMACReq, _Mapping]] = ..., set_wifi_ssid_and_password: _Optional[_Union[SetWiFiSSIDAndPasswordReq, _Mapping]] = ..., get_wifi_records: _Optional[_Union[GetWiFiRecordsReq, _Mapping]] = ..., operate_wifi_record: _Optional[_Union[OperateWiFiRecordReq, _Mapping]] = ..., get_wifi_state_machine: _Optional[_Union[GetWiFiStateMachineReq, _Mapping]] = ..., set_wifi_state_machine: _Optional[_Union[SetWiFiStateMachineReq, _Mapping]] = ..., set_charging_strategy: _Optional[_Union[SetChargingStrategyReq, _Mapping]] = ..., get_charging_status: _Optional[_Union[GetChargingStatusReq, _Mapping]] = ..., get_power_historical_stats: _Optional[_Union[GetPowerHistoricalStatsReq, _Mapping]] = ..., set_port_priority: _Optional[_Union[SetPortPriorityReq, _Mapping]] = ..., get_port_priority: _Optional[_Union[GetPortPriorityReq, _Mapping]] = ..., get_charging_strategy: _Optional[_Union[GetChargingStrategyReq, _Mapping]] = ..., get_port_pd_status: _Optional[_Union[GetPortPDStatusReq, _Mapping]] = ..., get_all_power_statistics: _Optional[_Union[GetAllPowerStatisticsReq, _Mapping]] = ..., turn_on_port: _Optional[_Union[TurnOnPortReq, _Mapping]] = ..., turn_off_port: _Optional[_Union[TurnOffPortReq, _Mapping]] = ..., set_static_allocator: _Optional[_Union[SetStaticAllocatorReq, _Mapping]] = ..., get_static_allocator: _Optional[_Union[GetStaticAllocatorReq, _Mapping]] = ..., set_port_config: _Optional[_Union[SetPortConfigReq, _Mapping]] = ..., get_port_config: _Optional[_Union[GetPortConfigReq, _Mapping]] = ..., set_port_compatibility_settings: _Optional[_Union[SetPortCompatibilitySettingsReq, _Mapping]] = ..., get_port_compatibility_settings: _Optional[_Union[GetPortCompatibilitySettingsReq, _Mapping]] = ..., set_temperature_mode: _Optional[_Union[SetTemperatureModeReq, _Mapping]] = ..., set_temporary_allocator: _Optional[_Union[SetTemporaryAllocatorReq, _Mapping]] = ..., subscribe_pdpcap_streaming: _Optional[_Union[SubscribePDPcapStreamingReq, _Mapping]] = ..., set_port_type: _Optional[_Union[SetPortTypeReq, _Mapping]] = ..., set_custom_pdo_voltage: _Optional[_Union[SetCustomPdoVoltageReq, _Mapping]] = ..., set_cable_compensation: _Optional[_Union[SetCableCompensationReq, _Mapping]] = ..., get_cable_compensation: _Optional[_Union[GetCableCompensationReq, _Mapping]] = ..., set_display_intensity: _Optional[_Union[SetDisplayIntensityReq, _Mapping]] = ..., set_display_mode: _Optional[_Union[SetDisplayModeReq, _Mapping]] = ..., get_display_intensity: _Optional[_Union[GetDisplayIntensityReq, _Mapping]] = ..., get_display_mode: _Optional[_Union[GetDisplayModeReq, _Mapping]] = ..., set_display_flip: _Optional[_Union[SetDisplayFlipReq, _Mapping]] = ..., get_display_flip: _Optional[_Union[GetDisplayFlipReq, _Mapping]] = ..., set_display_config: _Optional[_Union[SetDisplayConfigReq, _Mapping]] = ..., get_display_rotation: _Optional[_Union[GetDisplayRotationReq, _Mapping]] = ..., set_display_rotation: _Optional[_Union[SetDisplayRotationReq, _Mapping]] = ..., set_display_setup: _Optional[_Union[SetDisplaySetupReq, _Mapping]] = ..., get_display_setup: _Optional[_Union[GetDisplaySetupReq, _Mapping]] = ..., start_telemetry_stream: _Optional[_Union[StartTelemetryStreamReq, _Mapping]] = ..., stop_telemetry_stream: _Optional[_Union[StopTelemetryStreamReq, _Mapping]] = ..., set_ble_state: _Optional[_Union[SetBleStateReq, _Mapping]] = ..., start_ota: _Optional[_Union[StartOTAReq, _Mapping]] = ..., start_programmer: _Optional[_Union[StartProgrammerReq, _Mapping]] = ..., clear_factory_dm_flag: _Optional[_Union[ClearFactoryDMFlagReq, _Mapping]] = ...) -> None: ...

class CommandResponse(_message.Message):
    __slots__ = ("id", "status", "lock_swd", "unlock_swd", "enable_release_mode", "reboot_device", "reset_device", "get_device_psn", "get_ap_version", "set_power_state", "get_power_state", "get_device_info", "get_hardware_revision", "scan_wifi", "set_wifi_ssid", "set_wifi_password", "reset_wifi", "get_wifi_status", "get_wifi_mac", "set_wifi_ssid_and_password", "get_wifi_records", "operate_wifi_record", "get_wifi_state_machine", "set_wifi_state_machine", "set_charging_strategy", "get_charging_status", "get_power_historical_stats", "set_port_priority", "get_port_priority", "get_charging_strategy", "get_port_pd_status", "get_all_power_statistics", "turn_on_port", "turn_off_port", "set_static_allocator", "get_static_allocator", "set_port_config", "get_port_config", "set_port_compatibility_settings", "get_port_compatibility_settings", "set_temperature_mode", "set_temporary_allocator", "subscribe_pdpcap_streaming", "set_port_type", "set_custom_pdo_voltage", "set_cable_compensation", "get_cable_compensation", "set_display_intensity", "set_display_mode", "get_display_intensity", "get_display_mode", "set_display_flip", "get_display_flip", "set_display_config", "get_display_rotation", "set_display_rotation", "set_display_setup", "get_display_setup", "start_telemetry_stream", "stop_telemetry_stream", "stream_port_status", "stream_device_status", "stream_port_pd_status", "stream_programmer_status", "set_ble_state", "start_ota", "start_programmer", "clear_factory_dm_flag")
    ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    LOCK_SWD_FIELD_NUMBER: _ClassVar[int]
    UNLOCK_SWD_FIELD_NUMBER: _ClassVar[int]
    ENABLE_RELEASE_MODE_FIELD_NUMBER: _ClassVar[int]
    REBOOT_DEVICE_FIELD_NUMBER: _ClassVar[int]
    RESET_DEVICE_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_PSN_FIELD_NUMBER: _ClassVar[int]
    GET_AP_VERSION_FIELD_NUMBER: _ClassVar[int]
    SET_POWER_STATE_FIELD_NUMBER: _ClassVar[int]
    GET_POWER_STATE_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_INFO_FIELD_NUMBER: _ClassVar[int]
    GET_HARDWARE_REVISION_FIELD_NUMBER: _ClassVar[int]
    SCAN_WIFI_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_SSID_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    RESET_WIFI_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_MAC_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_SSID_AND_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_RECORDS_FIELD_NUMBER: _ClassVar[int]
    OPERATE_WIFI_RECORD_FIELD_NUMBER: _ClassVar[int]
    GET_WIFI_STATE_MACHINE_FIELD_NUMBER: _ClassVar[int]
    SET_WIFI_STATE_MACHINE_FIELD_NUMBER: _ClassVar[int]
    SET_CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    GET_CHARGING_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_POWER_HISTORICAL_STATS_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_PRIORITY_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_PRIORITY_FIELD_NUMBER: _ClassVar[int]
    GET_CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_PD_STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_ALL_POWER_STATISTICS_FIELD_NUMBER: _ClassVar[int]
    TURN_ON_PORT_FIELD_NUMBER: _ClassVar[int]
    TURN_OFF_PORT_FIELD_NUMBER: _ClassVar[int]
    SET_STATIC_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    GET_STATIC_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_CONFIG_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_CONFIG_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_COMPATIBILITY_SETTINGS_FIELD_NUMBER: _ClassVar[int]
    GET_PORT_COMPATIBILITY_SETTINGS_FIELD_NUMBER: _ClassVar[int]
    SET_TEMPERATURE_MODE_FIELD_NUMBER: _ClassVar[int]
    SET_TEMPORARY_ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    SUBSCRIBE_PDPCAP_STREAMING_FIELD_NUMBER: _ClassVar[int]
    SET_PORT_TYPE_FIELD_NUMBER: _ClassVar[int]
    SET_CUSTOM_PDO_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    SET_CABLE_COMPENSATION_FIELD_NUMBER: _ClassVar[int]
    GET_CABLE_COMPENSATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_INTENSITY_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_MODE_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_INTENSITY_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_MODE_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_FLIP_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_FLIP_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_CONFIG_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_ROTATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_ROTATION_FIELD_NUMBER: _ClassVar[int]
    SET_DISPLAY_SETUP_FIELD_NUMBER: _ClassVar[int]
    GET_DISPLAY_SETUP_FIELD_NUMBER: _ClassVar[int]
    START_TELEMETRY_STREAM_FIELD_NUMBER: _ClassVar[int]
    STOP_TELEMETRY_STREAM_FIELD_NUMBER: _ClassVar[int]
    STREAM_PORT_STATUS_FIELD_NUMBER: _ClassVar[int]
    STREAM_DEVICE_STATUS_FIELD_NUMBER: _ClassVar[int]
    STREAM_PORT_PD_STATUS_FIELD_NUMBER: _ClassVar[int]
    STREAM_PROGRAMMER_STATUS_FIELD_NUMBER: _ClassVar[int]
    SET_BLE_STATE_FIELD_NUMBER: _ClassVar[int]
    START_OTA_FIELD_NUMBER: _ClassVar[int]
    START_PROGRAMMER_FIELD_NUMBER: _ClassVar[int]
    CLEAR_FACTORY_DM_FLAG_FIELD_NUMBER: _ClassVar[int]
    id: int
    status: _data_types_pb2.CommandStatus
    lock_swd: LockSWDResp
    unlock_swd: UnlockSWDResp
    enable_release_mode: EnableReleaseModeResp
    reboot_device: RebootDeviceResp
    reset_device: ResetDeviceResp
    get_device_psn: GetDevicePSNResp
    get_ap_version: GetAPVersionResp
    set_power_state: SetDevicePowerStateResp
    get_power_state: GetDevicePowerStateResp
    get_device_info: GetDeviceInfoResp
    get_hardware_revision: GetHardwareRevisionResp
    scan_wifi: ScanWiFiResp
    set_wifi_ssid: SetWiFiSSIDResp
    set_wifi_password: SetWiFiPasswordResp
    reset_wifi: ResetWiFiResp
    get_wifi_status: GetWiFiStatusResp
    get_wifi_mac: GetWiFiMACResp
    set_wifi_ssid_and_password: SetWiFiSSIDAndPasswordResp
    get_wifi_records: GetWiFiRecordsResp
    operate_wifi_record: OperateWiFiRecordResp
    get_wifi_state_machine: GetWiFiStateMachineResp
    set_wifi_state_machine: SetWiFiStateMachineResp
    set_charging_strategy: SetChargingStrategyResp
    get_charging_status: GetChargingStatusResp
    get_power_historical_stats: GetPowerHistoricalStatsResp
    set_port_priority: SetPortPriorityResp
    get_port_priority: GetPortPriorityResp
    get_charging_strategy: GetChargingStrategyResp
    get_port_pd_status: GetPortPDStatusResp
    get_all_power_statistics: GetAllPowerStatisticsResp
    turn_on_port: TurnOnPortResp
    turn_off_port: TurnOffPortResp
    set_static_allocator: SetStaticAllocatorResp
    get_static_allocator: GetStaticAllocatorResp
    set_port_config: SetPortConfigResp
    get_port_config: GetPortConfigResp
    set_port_compatibility_settings: SetPortCompatibilitySettingsResp
    get_port_compatibility_settings: GetPortCompatibilitySettingsResp
    set_temperature_mode: SetTemperatureModeResp
    set_temporary_allocator: SetTemporaryAllocatorResp
    subscribe_pdpcap_streaming: SubscribePDPcapStreamingResp
    set_port_type: SetPortTypeResp
    set_custom_pdo_voltage: SetCustomPdoVoltageResp
    set_cable_compensation: SetCableCompensationResp
    get_cable_compensation: GetCableCompensationResp
    set_display_intensity: SetDisplayIntensityResp
    set_display_mode: SetDisplayModeResp
    get_display_intensity: GetDisplayIntensityResp
    get_display_mode: GetDisplayModeResp
    set_display_flip: SetDisplayFlipResp
    get_display_flip: GetDisplayFlipResp
    set_display_config: SetDisplayConfigResp
    get_display_rotation: GetDisplayRotationResp
    set_display_rotation: SetDisplayRotationResp
    set_display_setup: SetDisplaySetupResp
    get_display_setup: GetDisplaySetupResp
    start_telemetry_stream: StartTelemetryStreamResp
    stop_telemetry_stream: StopTelemetryStreamResp
    stream_port_status: StreamPortStatus
    stream_device_status: StreamDeviceStatus
    stream_port_pd_status: StreamPortPDStatus
    stream_programmer_status: StreamProgrammerStatus
    set_ble_state: SetBleStateResp
    start_ota: StartOTAResp
    start_programmer: StartProgrammerResp
    clear_factory_dm_flag: ClearFactoryDMFlagResp
    def __init__(self, id: _Optional[int] = ..., status: _Optional[_Union[_data_types_pb2.CommandStatus, str]] = ..., lock_swd: _Optional[_Union[LockSWDResp, _Mapping]] = ..., unlock_swd: _Optional[_Union[UnlockSWDResp, _Mapping]] = ..., enable_release_mode: _Optional[_Union[EnableReleaseModeResp, _Mapping]] = ..., reboot_device: _Optional[_Union[RebootDeviceResp, _Mapping]] = ..., reset_device: _Optional[_Union[ResetDeviceResp, _Mapping]] = ..., get_device_psn: _Optional[_Union[GetDevicePSNResp, _Mapping]] = ..., get_ap_version: _Optional[_Union[GetAPVersionResp, _Mapping]] = ..., set_power_state: _Optional[_Union[SetDevicePowerStateResp, _Mapping]] = ..., get_power_state: _Optional[_Union[GetDevicePowerStateResp, _Mapping]] = ..., get_device_info: _Optional[_Union[GetDeviceInfoResp, _Mapping]] = ..., get_hardware_revision: _Optional[_Union[GetHardwareRevisionResp, _Mapping]] = ..., scan_wifi: _Optional[_Union[ScanWiFiResp, _Mapping]] = ..., set_wifi_ssid: _Optional[_Union[SetWiFiSSIDResp, _Mapping]] = ..., set_wifi_password: _Optional[_Union[SetWiFiPasswordResp, _Mapping]] = ..., reset_wifi: _Optional[_Union[ResetWiFiResp, _Mapping]] = ..., get_wifi_status: _Optional[_Union[GetWiFiStatusResp, _Mapping]] = ..., get_wifi_mac: _Optional[_Union[GetWiFiMACResp, _Mapping]] = ..., set_wifi_ssid_and_password: _Optional[_Union[SetWiFiSSIDAndPasswordResp, _Mapping]] = ..., get_wifi_records: _Optional[_Union[GetWiFiRecordsResp, _Mapping]] = ..., operate_wifi_record: _Optional[_Union[OperateWiFiRecordResp, _Mapping]] = ..., get_wifi_state_machine: _Optional[_Union[GetWiFiStateMachineResp, _Mapping]] = ..., set_wifi_state_machine: _Optional[_Union[SetWiFiStateMachineResp, _Mapping]] = ..., set_charging_strategy: _Optional[_Union[SetChargingStrategyResp, _Mapping]] = ..., get_charging_status: _Optional[_Union[GetChargingStatusResp, _Mapping]] = ..., get_power_historical_stats: _Optional[_Union[GetPowerHistoricalStatsResp, _Mapping]] = ..., set_port_priority: _Optional[_Union[SetPortPriorityResp, _Mapping]] = ..., get_port_priority: _Optional[_Union[GetPortPriorityResp, _Mapping]] = ..., get_charging_strategy: _Optional[_Union[GetChargingStrategyResp, _Mapping]] = ..., get_port_pd_status: _Optional[_Union[GetPortPDStatusResp, _Mapping]] = ..., get_all_power_statistics: _Optional[_Union[GetAllPowerStatisticsResp, _Mapping]] = ..., turn_on_port: _Optional[_Union[TurnOnPortResp, _Mapping]] = ..., turn_off_port: _Optional[_Union[TurnOffPortResp, _Mapping]] = ..., set_static_allocator: _Optional[_Union[SetStaticAllocatorResp, _Mapping]] = ..., get_static_allocator: _Optional[_Union[GetStaticAllocatorResp, _Mapping]] = ..., set_port_config: _Optional[_Union[SetPortConfigResp, _Mapping]] = ..., get_port_config: _Optional[_Union[GetPortConfigResp, _Mapping]] = ..., set_port_compatibility_settings: _Optional[_Union[SetPortCompatibilitySettingsResp, _Mapping]] = ..., get_port_compatibility_settings: _Optional[_Union[GetPortCompatibilitySettingsResp, _Mapping]] = ..., set_temperature_mode: _Optional[_Union[SetTemperatureModeResp, _Mapping]] = ..., set_temporary_allocator: _Optional[_Union[SetTemporaryAllocatorResp, _Mapping]] = ..., subscribe_pdpcap_streaming: _Optional[_Union[SubscribePDPcapStreamingResp, _Mapping]] = ..., set_port_type: _Optional[_Union[SetPortTypeResp, _Mapping]] = ..., set_custom_pdo_voltage: _Optional[_Union[SetCustomPdoVoltageResp, _Mapping]] = ..., set_cable_compensation: _Optional[_Union[SetCableCompensationResp, _Mapping]] = ..., get_cable_compensation: _Optional[_Union[GetCableCompensationResp, _Mapping]] = ..., set_display_intensity: _Optional[_Union[SetDisplayIntensityResp, _Mapping]] = ..., set_display_mode: _Optional[_Union[SetDisplayModeResp, _Mapping]] = ..., get_display_intensity: _Optional[_Union[GetDisplayIntensityResp, _Mapping]] = ..., get_display_mode: _Optional[_Union[GetDisplayModeResp, _Mapping]] = ..., set_display_flip: _Optional[_Union[SetDisplayFlipResp, _Mapping]] = ..., get_display_flip: _Optional[_Union[GetDisplayFlipResp, _Mapping]] = ..., set_display_config: _Optional[_Union[SetDisplayConfigResp, _Mapping]] = ..., get_display_rotation: _Optional[_Union[GetDisplayRotationResp, _Mapping]] = ..., set_display_rotation: _Optional[_Union[SetDisplayRotationResp, _Mapping]] = ..., set_display_setup: _Optional[_Union[SetDisplaySetupResp, _Mapping]] = ..., get_display_setup: _Optional[_Union[GetDisplaySetupResp, _Mapping]] = ..., start_telemetry_stream: _Optional[_Union[StartTelemetryStreamResp, _Mapping]] = ..., stop_telemetry_stream: _Optional[_Union[StopTelemetryStreamResp, _Mapping]] = ..., stream_port_status: _Optional[_Union[StreamPortStatus, _Mapping]] = ..., stream_device_status: _Optional[_Union[StreamDeviceStatus, _Mapping]] = ..., stream_port_pd_status: _Optional[_Union[StreamPortPDStatus, _Mapping]] = ..., stream_programmer_status: _Optional[_Union[StreamProgrammerStatus, _Mapping]] = ..., set_ble_state: _Optional[_Union[SetBleStateResp, _Mapping]] = ..., start_ota: _Optional[_Union[StartOTAResp, _Mapping]] = ..., start_programmer: _Optional[_Union[StartProgrammerResp, _Mapping]] = ..., clear_factory_dm_flag: _Optional[_Union[ClearFactoryDMFlagResp, _Mapping]] = ...) -> None: ...

class EnableReleaseModeReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class EnableReleaseModeResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetHardwareRevisionReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetHardwareRevisionResp(_message.Message):
    __slots__ = ("revision",)
    REVISION_FIELD_NUMBER: _ClassVar[int]
    revision: str
    def __init__(self, revision: _Optional[str] = ...) -> None: ...

class RebootDeviceReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class RebootDeviceResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ResetDeviceReq(_message.Message):
    __slots__ = ("keep_wifi_nvs", "keep_mqtt_nvs", "keep_ble_nvs", "keep_display_nvs", "keep_power_nvs", "keep_system_nvs", "keep_license_nvs", "keep_syslog_nvs", "keep_fpga_nvs", "keep_ota_nvs", "prevent_reboot", "reset_ports", "reset_power_strategy", "reset_wifi", "reset_display", "reset_fc_protocol", "reset_temperature_mode", "reset_cable_compensation")
    KEEP_WIFI_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_MQTT_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_BLE_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_DISPLAY_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_POWER_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_SYSTEM_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_LICENSE_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_SYSLOG_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_FPGA_NVS_FIELD_NUMBER: _ClassVar[int]
    KEEP_OTA_NVS_FIELD_NUMBER: _ClassVar[int]
    PREVENT_REBOOT_FIELD_NUMBER: _ClassVar[int]
    RESET_PORTS_FIELD_NUMBER: _ClassVar[int]
    RESET_POWER_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    RESET_WIFI_FIELD_NUMBER: _ClassVar[int]
    RESET_DISPLAY_FIELD_NUMBER: _ClassVar[int]
    RESET_FC_PROTOCOL_FIELD_NUMBER: _ClassVar[int]
    RESET_TEMPERATURE_MODE_FIELD_NUMBER: _ClassVar[int]
    RESET_CABLE_COMPENSATION_FIELD_NUMBER: _ClassVar[int]
    keep_wifi_nvs: bool
    keep_mqtt_nvs: bool
    keep_ble_nvs: bool
    keep_display_nvs: bool
    keep_power_nvs: bool
    keep_system_nvs: bool
    keep_license_nvs: bool
    keep_syslog_nvs: bool
    keep_fpga_nvs: bool
    keep_ota_nvs: bool
    prevent_reboot: bool
    reset_ports: bool
    reset_power_strategy: bool
    reset_wifi: bool
    reset_display: bool
    reset_fc_protocol: bool
    reset_temperature_mode: bool
    reset_cable_compensation: bool
    def __init__(self, keep_wifi_nvs: bool = ..., keep_mqtt_nvs: bool = ..., keep_ble_nvs: bool = ..., keep_display_nvs: bool = ..., keep_power_nvs: bool = ..., keep_system_nvs: bool = ..., keep_license_nvs: bool = ..., keep_syslog_nvs: bool = ..., keep_fpga_nvs: bool = ..., keep_ota_nvs: bool = ..., prevent_reboot: bool = ..., reset_ports: bool = ..., reset_power_strategy: bool = ..., reset_wifi: bool = ..., reset_display: bool = ..., reset_fc_protocol: bool = ..., reset_temperature_mode: bool = ..., reset_cable_compensation: bool = ...) -> None: ...

class ResetDeviceResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDevicePSNReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDevicePSNResp(_message.Message):
    __slots__ = ("psn",)
    PSN_FIELD_NUMBER: _ClassVar[int]
    psn: str
    def __init__(self, psn: _Optional[str] = ...) -> None: ...

class GetAPVersionReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetAPVersionResp(_message.Message):
    __slots__ = ("version",)
    VERSION_FIELD_NUMBER: _ClassVar[int]
    version: _data_types_pb2.Version
    def __init__(self, version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ...) -> None: ...

class SetDevicePowerStateReq(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: _data_types_pb2.PowerState
    def __init__(self, state: _Optional[_Union[_data_types_pb2.PowerState, str]] = ...) -> None: ...

class SetDevicePowerStateResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDevicePowerStateReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDevicePowerStateResp(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: _data_types_pb2.PowerState
    def __init__(self, state: _Optional[_Union[_data_types_pb2.PowerState, str]] = ...) -> None: ...

class LockSWDReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class LockSWDResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class UnlockSWDReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class UnlockSWDResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ScanWiFiReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ScanWiFiResp(_message.Message):
    __slots__ = ("ssid_list",)
    SSID_LIST_FIELD_NUMBER: _ClassVar[int]
    ssid_list: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.SSIDInfo]
    def __init__(self, ssid_list: _Optional[_Iterable[_Union[_data_types_pb2.SSIDInfo, _Mapping]]] = ...) -> None: ...

class SetWiFiSSIDReq(_message.Message):
    __slots__ = ("ssid",)
    SSID_FIELD_NUMBER: _ClassVar[int]
    ssid: str
    def __init__(self, ssid: _Optional[str] = ...) -> None: ...

class SetWiFiSSIDResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetWiFiPasswordReq(_message.Message):
    __slots__ = ("password",)
    PASSWORD_FIELD_NUMBER: _ClassVar[int]
    password: str
    def __init__(self, password: _Optional[str] = ...) -> None: ...

class SetWiFiPasswordResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ResetWiFiReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ResetWiFiResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiStatusReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiStatusResp(_message.Message):
    __slots__ = ("bssid", "rssi", "channel", "protocol", "ssid")
    BSSID_FIELD_NUMBER: _ClassVar[int]
    RSSI_FIELD_NUMBER: _ClassVar[int]
    CHANNEL_FIELD_NUMBER: _ClassVar[int]
    PROTOCOL_FIELD_NUMBER: _ClassVar[int]
    SSID_FIELD_NUMBER: _ClassVar[int]
    bssid: str
    rssi: int
    channel: int
    protocol: int
    ssid: str
    def __init__(self, bssid: _Optional[str] = ..., rssi: _Optional[int] = ..., channel: _Optional[int] = ..., protocol: _Optional[int] = ..., ssid: _Optional[str] = ...) -> None: ...

class GetWiFiMACReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiMACResp(_message.Message):
    __slots__ = ("mac",)
    MAC_FIELD_NUMBER: _ClassVar[int]
    mac: str
    def __init__(self, mac: _Optional[str] = ...) -> None: ...

class SetWiFiSSIDAndPasswordReq(_message.Message):
    __slots__ = ("ssid", "password", "auth_mode")
    SSID_FIELD_NUMBER: _ClassVar[int]
    PASSWORD_FIELD_NUMBER: _ClassVar[int]
    AUTH_MODE_FIELD_NUMBER: _ClassVar[int]
    ssid: str
    password: str
    auth_mode: _data_types_pb2.WiFiAuthMode
    def __init__(self, ssid: _Optional[str] = ..., password: _Optional[str] = ..., auth_mode: _Optional[_Union[_data_types_pb2.WiFiAuthMode, str]] = ...) -> None: ...

class SetWiFiSSIDAndPasswordResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiRecordsReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiRecordsResp(_message.Message):
    __slots__ = ("records",)
    RECORDS_FIELD_NUMBER: _ClassVar[int]
    records: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.WiFiRecord]
    def __init__(self, records: _Optional[_Iterable[_Union[_data_types_pb2.WiFiRecord, _Mapping]]] = ...) -> None: ...

class OperateWiFiRecordReq(_message.Message):
    __slots__ = ("action", "ssid", "password", "auth_mode")
    ACTION_FIELD_NUMBER: _ClassVar[int]
    SSID_FIELD_NUMBER: _ClassVar[int]
    PASSWORD_FIELD_NUMBER: _ClassVar[int]
    AUTH_MODE_FIELD_NUMBER: _ClassVar[int]
    action: _data_types_pb2.ManageAction
    ssid: str
    password: str
    auth_mode: _data_types_pb2.WiFiAuthMode
    def __init__(self, action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., ssid: _Optional[str] = ..., password: _Optional[str] = ..., auth_mode: _Optional[_Union[_data_types_pb2.WiFiAuthMode, str]] = ...) -> None: ...

class OperateWiFiRecordResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiStateMachineReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetWiFiStateMachineResp(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: _data_types_pb2.WiFiState
    def __init__(self, state: _Optional[_Union[_data_types_pb2.WiFiState, str]] = ...) -> None: ...

class SetWiFiStateMachineReq(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: _data_types_pb2.WiFiState
    def __init__(self, state: _Optional[_Union[_data_types_pb2.WiFiState, str]] = ...) -> None: ...

class SetWiFiStateMachineResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetChargingStrategyReq(_message.Message):
    __slots__ = ("charging_strategy",)
    CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    charging_strategy: _data_types_pb2.ChargingStrategy
    def __init__(self, charging_strategy: _Optional[_Union[_data_types_pb2.ChargingStrategy, str]] = ...) -> None: ...

class SetChargingStrategyResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetChargingStatusReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetChargingStatusResp(_message.Message):
    __slots__ = ("status",)
    STATUS_FIELD_NUMBER: _ClassVar[int]
    status: int
    def __init__(self, status: _Optional[int] = ...) -> None: ...

class GetPowerHistoricalStatsReq(_message.Message):
    __slots__ = ("port", "offset")
    PORT_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    port: int
    offset: int
    def __init__(self, port: _Optional[int] = ..., offset: _Optional[int] = ...) -> None: ...

class GetPowerHistoricalStatsResp(_message.Message):
    __slots__ = ("offset", "data")
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    offset: int
    data: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.PowerHistoricalStats]
    def __init__(self, offset: _Optional[int] = ..., data: _Optional[_Iterable[_Union[_data_types_pb2.PowerHistoricalStats, _Mapping]]] = ...) -> None: ...

class SetPortPriorityReq(_message.Message):
    __slots__ = ("priority",)
    PRIORITY_FIELD_NUMBER: _ClassVar[int]
    priority: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, priority: _Optional[_Iterable[int]] = ...) -> None: ...

class SetPortPriorityResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetPortPriorityReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetPortPriorityResp(_message.Message):
    __slots__ = ("priority",)
    PRIORITY_FIELD_NUMBER: _ClassVar[int]
    priority: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, priority: _Optional[_Iterable[int]] = ...) -> None: ...

class GetChargingStrategyReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetChargingStrategyResp(_message.Message):
    __slots__ = ("charging_strategy",)
    CHARGING_STRATEGY_FIELD_NUMBER: _ClassVar[int]
    charging_strategy: _data_types_pb2.ChargingStrategy
    def __init__(self, charging_strategy: _Optional[_Union[_data_types_pb2.ChargingStrategy, str]] = ...) -> None: ...

class GetPortPDStatusReq(_message.Message):
    __slots__ = ("port",)
    PORT_FIELD_NUMBER: _ClassVar[int]
    port: int
    def __init__(self, port: _Optional[int] = ...) -> None: ...

class GetPortPDStatusResp(_message.Message):
    __slots__ = ("pd_status",)
    PD_STATUS_FIELD_NUMBER: _ClassVar[int]
    pd_status: _data_types_pb2.ClientPDStatus
    def __init__(self, pd_status: _Optional[_Union[_data_types_pb2.ClientPDStatus, _Mapping]] = ...) -> None: ...

class GetAllPowerStatisticsReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetAllPowerStatisticsResp(_message.Message):
    __slots__ = ("stats",)
    STATS_FIELD_NUMBER: _ClassVar[int]
    stats: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.PowerStatistics]
    def __init__(self, stats: _Optional[_Iterable[_Union[_data_types_pb2.PowerStatistics, _Mapping]]] = ...) -> None: ...

class TurnOnPortReq(_message.Message):
    __slots__ = ("ports",)
    PORTS_FIELD_NUMBER: _ClassVar[int]
    ports: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, ports: _Optional[_Iterable[int]] = ...) -> None: ...

class TurnOnPortResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class TurnOffPortReq(_message.Message):
    __slots__ = ("ports",)
    PORTS_FIELD_NUMBER: _ClassVar[int]
    ports: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, ports: _Optional[_Iterable[int]] = ...) -> None: ...

class TurnOffPortResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetStaticAllocatorReq(_message.Message):
    __slots__ = ("identifier", "table")
    IDENTIFIER_FIELD_NUMBER: _ClassVar[int]
    TABLE_FIELD_NUMBER: _ClassVar[int]
    identifier: int
    table: _data_types_pb2.PowerAllocationTable
    def __init__(self, identifier: _Optional[int] = ..., table: _Optional[_Union[_data_types_pb2.PowerAllocationTable, _Mapping]] = ...) -> None: ...

class SetStaticAllocatorResp(_message.Message):
    __slots__ = ("identifier",)
    IDENTIFIER_FIELD_NUMBER: _ClassVar[int]
    identifier: int
    def __init__(self, identifier: _Optional[int] = ...) -> None: ...

class GetStaticAllocatorReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetStaticAllocatorResp(_message.Message):
    __slots__ = ("identifier", "version")
    IDENTIFIER_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    identifier: int
    version: int
    def __init__(self, identifier: _Optional[int] = ..., version: _Optional[int] = ...) -> None: ...

class SetPortConfigReq(_message.Message):
    __slots__ = ("ports", "configs")
    PORTS_FIELD_NUMBER: _ClassVar[int]
    CONFIGS_FIELD_NUMBER: _ClassVar[int]
    ports: int
    configs: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.PortConfig]
    def __init__(self, ports: _Optional[int] = ..., configs: _Optional[_Iterable[_Union[_data_types_pb2.PortConfig, _Mapping]]] = ...) -> None: ...

class SetPortConfigResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetPortConfigReq(_message.Message):
    __slots__ = ("version",)
    VERSION_FIELD_NUMBER: _ClassVar[int]
    version: int
    def __init__(self, version: _Optional[int] = ...) -> None: ...

class GetPortConfigResp(_message.Message):
    __slots__ = ("configs",)
    CONFIGS_FIELD_NUMBER: _ClassVar[int]
    configs: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.PortConfig]
    def __init__(self, configs: _Optional[_Iterable[_Union[_data_types_pb2.PortConfig, _Mapping]]] = ...) -> None: ...

class SetPortCompatibilitySettingsReq(_message.Message):
    __slots__ = ("settings",)
    SETTINGS_FIELD_NUMBER: _ClassVar[int]
    settings: _data_types_pb2.CompatibilitySettings
    def __init__(self, settings: _Optional[_Union[_data_types_pb2.CompatibilitySettings, _Mapping]] = ...) -> None: ...

class SetPortCompatibilitySettingsResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetPortCompatibilitySettingsReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetPortCompatibilitySettingsResp(_message.Message):
    __slots__ = ("settings",)
    SETTINGS_FIELD_NUMBER: _ClassVar[int]
    settings: _data_types_pb2.CompatibilitySettings
    def __init__(self, settings: _Optional[_Union[_data_types_pb2.CompatibilitySettings, _Mapping]] = ...) -> None: ...

class SetTemperatureModeReq(_message.Message):
    __slots__ = ("mode",)
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: _data_types_pb2.TemperatureMode
    def __init__(self, mode: _Optional[_Union[_data_types_pb2.TemperatureMode, str]] = ...) -> None: ...

class SetTemperatureModeResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetTemporaryAllocatorReq(_message.Message):
    __slots__ = ("power_allocation",)
    POWER_ALLOCATION_FIELD_NUMBER: _ClassVar[int]
    power_allocation: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, power_allocation: _Optional[_Iterable[int]] = ...) -> None: ...

class SetTemporaryAllocatorResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SubscribePDPcapStreamingReq(_message.Message):
    __slots__ = ("port", "subscribe")
    PORT_FIELD_NUMBER: _ClassVar[int]
    SUBSCRIBE_FIELD_NUMBER: _ClassVar[int]
    port: int
    subscribe: bool
    def __init__(self, port: _Optional[int] = ..., subscribe: bool = ...) -> None: ...

class SubscribePDPcapStreamingResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetPortTypeReq(_message.Message):
    __slots__ = ("port", "port_type")
    PORT_FIELD_NUMBER: _ClassVar[int]
    PORT_TYPE_FIELD_NUMBER: _ClassVar[int]
    port: int
    port_type: _data_types_pb2.PortType
    def __init__(self, port: _Optional[int] = ..., port_type: _Optional[_Union[_data_types_pb2.PortType, str]] = ...) -> None: ...

class SetPortTypeResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetCustomPdoVoltageReq(_message.Message):
    __slots__ = ("port", "voltage_mv")
    PORT_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_MV_FIELD_NUMBER: _ClassVar[int]
    port: int
    voltage_mv: int
    def __init__(self, port: _Optional[int] = ..., voltage_mv: _Optional[int] = ...) -> None: ...

class SetCustomPdoVoltageResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetCableCompensationReq(_message.Message):
    __slots__ = ("port_mask", "disable", "resistance", "voltage_offset")
    PORT_MASK_FIELD_NUMBER: _ClassVar[int]
    DISABLE_FIELD_NUMBER: _ClassVar[int]
    RESISTANCE_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_OFFSET_FIELD_NUMBER: _ClassVar[int]
    port_mask: int
    disable: bool
    resistance: _data_types_pb2.CableCompensation
    voltage_offset: _data_types_pb2.FixedVoltageOffset
    def __init__(self, port_mask: _Optional[int] = ..., disable: bool = ..., resistance: _Optional[_Union[_data_types_pb2.CableCompensation, str]] = ..., voltage_offset: _Optional[_Union[_data_types_pb2.FixedVoltageOffset, str]] = ...) -> None: ...

class SetCableCompensationResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetCableCompensationReq(_message.Message):
    __slots__ = ("port",)
    PORT_FIELD_NUMBER: _ClassVar[int]
    port: int
    def __init__(self, port: _Optional[int] = ...) -> None: ...

class GetCableCompensationResp(_message.Message):
    __slots__ = ("enable", "resistance", "voltage_offset")
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    RESISTANCE_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_OFFSET_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    resistance: _data_types_pb2.CableCompensation
    voltage_offset: _data_types_pb2.FixedVoltageOffset
    def __init__(self, enable: bool = ..., resistance: _Optional[_Union[_data_types_pb2.CableCompensation, str]] = ..., voltage_offset: _Optional[_Union[_data_types_pb2.FixedVoltageOffset, str]] = ...) -> None: ...

class SetDisplayIntensityReq(_message.Message):
    __slots__ = ("intensity",)
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    intensity: int
    def __init__(self, intensity: _Optional[int] = ...) -> None: ...

class SetDisplayIntensityResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetDisplayModeReq(_message.Message):
    __slots__ = ("mode",)
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: _data_types_pb2.DisplayMode
    def __init__(self, mode: _Optional[_Union[_data_types_pb2.DisplayMode, str]] = ...) -> None: ...

class SetDisplayModeResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayIntensityReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayIntensityResp(_message.Message):
    __slots__ = ("intensity",)
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    intensity: int
    def __init__(self, intensity: _Optional[int] = ...) -> None: ...

class GetDisplayModeReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayModeResp(_message.Message):
    __slots__ = ("mode",)
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: _data_types_pb2.DisplayMode
    def __init__(self, mode: _Optional[_Union[_data_types_pb2.DisplayMode, str]] = ...) -> None: ...

class SetDisplayFlipReq(_message.Message):
    __slots__ = ("flip",)
    FLIP_FIELD_NUMBER: _ClassVar[int]
    flip: _data_types_pb2.DisplayFlip
    def __init__(self, flip: _Optional[_Union[_data_types_pb2.DisplayFlip, str]] = ...) -> None: ...

class SetDisplayFlipResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayFlipReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayFlipResp(_message.Message):
    __slots__ = ("flip",)
    FLIP_FIELD_NUMBER: _ClassVar[int]
    flip: _data_types_pb2.DisplayFlip
    def __init__(self, flip: _Optional[_Union[_data_types_pb2.DisplayFlip, str]] = ...) -> None: ...

class SetDisplayConfigReq(_message.Message):
    __slots__ = ("config",)
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    config: _data_types_pb2.DisplayConfiguration
    def __init__(self, config: _Optional[_Union[_data_types_pb2.DisplayConfiguration, _Mapping]] = ...) -> None: ...

class SetDisplayConfigResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayRotationReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplayRotationResp(_message.Message):
    __slots__ = ("rotation",)
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    rotation: _data_types_pb2.DisplayRotation
    def __init__(self, rotation: _Optional[_Union[_data_types_pb2.DisplayRotation, str]] = ...) -> None: ...

class SetDisplayRotationReq(_message.Message):
    __slots__ = ("rotation",)
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    rotation: _data_types_pb2.DisplayRotation
    def __init__(self, rotation: _Optional[_Union[_data_types_pb2.DisplayRotation, str]] = ...) -> None: ...

class SetDisplayRotationResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetDisplaySetupReq(_message.Message):
    __slots__ = ("setup",)
    SETUP_FIELD_NUMBER: _ClassVar[int]
    setup: _data_types_pb2.DisplaySetup
    def __init__(self, setup: _Optional[_Union[_data_types_pb2.DisplaySetup, _Mapping]] = ...) -> None: ...

class SetDisplaySetupResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplaySetupReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDisplaySetupResp(_message.Message):
    __slots__ = ("setup",)
    SETUP_FIELD_NUMBER: _ClassVar[int]
    setup: _data_types_pb2.DisplaySetup
    def __init__(self, setup: _Optional[_Union[_data_types_pb2.DisplaySetup, _Mapping]] = ...) -> None: ...

class StartTelemetryStreamReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StartTelemetryStreamResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StopTelemetryStreamReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StopTelemetryStreamResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDeviceInfoReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDeviceInfoResp(_message.Message):
    __slots__ = ("info",)
    INFO_FIELD_NUMBER: _ClassVar[int]
    info: _data_types_pb2.DeviceInfo
    def __init__(self, info: _Optional[_Union[_data_types_pb2.DeviceInfo, _Mapping]] = ...) -> None: ...

class SetBleStateReq(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class SetBleStateResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StartOTAReq(_message.Message):
    __slots__ = ("sw3566_version", "fpga_version", "esp32_version", "force")
    SW3566_VERSION_FIELD_NUMBER: _ClassVar[int]
    FPGA_VERSION_FIELD_NUMBER: _ClassVar[int]
    ESP32_VERSION_FIELD_NUMBER: _ClassVar[int]
    FORCE_FIELD_NUMBER: _ClassVar[int]
    sw3566_version: _data_types_pb2.Version
    fpga_version: _data_types_pb2.Version
    esp32_version: _data_types_pb2.Version
    force: bool
    def __init__(self, sw3566_version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ..., fpga_version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ..., esp32_version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ..., force: bool = ...) -> None: ...

class StartOTAResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StreamPortStatus(_message.Message):
    __slots__ = ("port_status_map", "ports")
    PORT_STATUS_MAP_FIELD_NUMBER: _ClassVar[int]
    PORTS_FIELD_NUMBER: _ClassVar[int]
    port_status_map: int
    ports: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.PortStatusData]
    def __init__(self, port_status_map: _Optional[int] = ..., ports: _Optional[_Iterable[_Union[_data_types_pb2.PortStatusData, _Mapping]]] = ...) -> None: ...

class StreamPortPDStatus(_message.Message):
    __slots__ = ("port", "pd_status")
    PORT_FIELD_NUMBER: _ClassVar[int]
    PD_STATUS_FIELD_NUMBER: _ClassVar[int]
    port: int
    pd_status: _data_types_pb2.ClientPDStatus
    def __init__(self, port: _Optional[int] = ..., pd_status: _Optional[_Union[_data_types_pb2.ClientPDStatus, _Mapping]] = ...) -> None: ...

class StreamDeviceStatus(_message.Message):
    __slots__ = ("allocator", "display_config", "device_uptime", "feature_flag", "charging_minutes", "power_allocator_param", "temperature_mode", "programmer_mode")
    ALLOCATOR_FIELD_NUMBER: _ClassVar[int]
    DISPLAY_CONFIG_FIELD_NUMBER: _ClassVar[int]
    DEVICE_UPTIME_FIELD_NUMBER: _ClassVar[int]
    FEATURE_FLAG_FIELD_NUMBER: _ClassVar[int]
    CHARGING_MINUTES_FIELD_NUMBER: _ClassVar[int]
    POWER_ALLOCATOR_PARAM_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_MODE_FIELD_NUMBER: _ClassVar[int]
    PROGRAMMER_MODE_FIELD_NUMBER: _ClassVar[int]
    allocator: int
    display_config: _data_types_pb2.DisplayConfiguration
    device_uptime: int
    feature_flag: _data_types_pb2.FeatureFlag
    charging_minutes: int
    power_allocator_param: int
    temperature_mode: _data_types_pb2.TemperatureMode
    programmer_mode: bool
    def __init__(self, allocator: _Optional[int] = ..., display_config: _Optional[_Union[_data_types_pb2.DisplayConfiguration, _Mapping]] = ..., device_uptime: _Optional[int] = ..., feature_flag: _Optional[_Union[_data_types_pb2.FeatureFlag, _Mapping]] = ..., charging_minutes: _Optional[int] = ..., power_allocator_param: _Optional[int] = ..., temperature_mode: _Optional[_Union[_data_types_pb2.TemperatureMode, str]] = ..., programmer_mode: bool = ...) -> None: ...

class StartProgrammerReq(_message.Message):
    __slots__ = ("version", "product_family")
    VERSION_FIELD_NUMBER: _ClassVar[int]
    PRODUCT_FAMILY_FIELD_NUMBER: _ClassVar[int]
    version: _data_types_pb2.Version
    product_family: str
    def __init__(self, version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ..., product_family: _Optional[str] = ...) -> None: ...

class StartProgrammerResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class StreamProgrammerStatus(_message.Message):
    __slots__ = ("start", "success", "error_reason")
    START_FIELD_NUMBER: _ClassVar[int]
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    ERROR_REASON_FIELD_NUMBER: _ClassVar[int]
    start: bool
    success: bool
    error_reason: _data_types_pb2.ProgrammerState
    def __init__(self, start: bool = ..., success: bool = ..., error_reason: _Optional[_Union[_data_types_pb2.ProgrammerState, str]] = ...) -> None: ...

class ClearFactoryDMFlagReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ClearFactoryDMFlagResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...
