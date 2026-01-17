import data_types_pb2 as _data_types_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class AdminCommandRequest(_message.Message):
    __slots__ = ("id", "get_device_password", "download_firmware", "port_diagnostic", "manage_fpga_config", "manage_power_config", "manage_feature_toggle", "manage_device_config", "manage_phy_data", "set_telemetry_debug", "set_syslog_state", "set_system_time", "set_license", "schedule_task", "get_task_status", "forward_port", "read_adc", "set_machine_config", "cleanup_nvs_configs", "toggle_mcu_gpio", "toggle_temperature_protect", "write_file", "read_file", "delete_file", "list_dir")
    ID_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    DOWNLOAD_FIRMWARE_FIELD_NUMBER: _ClassVar[int]
    PORT_DIAGNOSTIC_FIELD_NUMBER: _ClassVar[int]
    MANAGE_FPGA_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_POWER_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_FEATURE_TOGGLE_FIELD_NUMBER: _ClassVar[int]
    MANAGE_DEVICE_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_PHY_DATA_FIELD_NUMBER: _ClassVar[int]
    SET_TELEMETRY_DEBUG_FIELD_NUMBER: _ClassVar[int]
    SET_SYSLOG_STATE_FIELD_NUMBER: _ClassVar[int]
    SET_SYSTEM_TIME_FIELD_NUMBER: _ClassVar[int]
    SET_LICENSE_FIELD_NUMBER: _ClassVar[int]
    SCHEDULE_TASK_FIELD_NUMBER: _ClassVar[int]
    GET_TASK_STATUS_FIELD_NUMBER: _ClassVar[int]
    FORWARD_PORT_FIELD_NUMBER: _ClassVar[int]
    READ_ADC_FIELD_NUMBER: _ClassVar[int]
    SET_MACHINE_CONFIG_FIELD_NUMBER: _ClassVar[int]
    CLEANUP_NVS_CONFIGS_FIELD_NUMBER: _ClassVar[int]
    TOGGLE_MCU_GPIO_FIELD_NUMBER: _ClassVar[int]
    TOGGLE_TEMPERATURE_PROTECT_FIELD_NUMBER: _ClassVar[int]
    WRITE_FILE_FIELD_NUMBER: _ClassVar[int]
    READ_FILE_FIELD_NUMBER: _ClassVar[int]
    DELETE_FILE_FIELD_NUMBER: _ClassVar[int]
    LIST_DIR_FIELD_NUMBER: _ClassVar[int]
    id: int
    get_device_password: GetDevicePasswordReq
    download_firmware: DownloadFirmwareReq
    port_diagnostic: PortDiagnosticReq
    manage_fpga_config: ManageFPGAConfigReq
    manage_power_config: ManagePowerConfigReq
    manage_feature_toggle: ManageFeatureToggleReq
    manage_device_config: ManageDeviceConfigReq
    manage_phy_data: ManagePhyDataReq
    set_telemetry_debug: SetTelemetryDebugReq
    set_syslog_state: SetSyslogStateReq
    set_system_time: SetSystemTimeReq
    set_license: SetLicenseReq
    schedule_task: ScheduleTaskRequest
    get_task_status: GetTaskStatusRequest
    forward_port: ForwardPortReq
    read_adc: ReadADCReq
    set_machine_config: SetMachineConfigReq
    cleanup_nvs_configs: CleanupNVSConfigsReq
    toggle_mcu_gpio: ToggleMcuGPIOReq
    toggle_temperature_protect: ToggleTemperatureProtectReq
    write_file: WriteFileReq
    read_file: ReadFileReq
    delete_file: DeleteFileReq
    list_dir: ListDirReq
    def __init__(self, id: _Optional[int] = ..., get_device_password: _Optional[_Union[GetDevicePasswordReq, _Mapping]] = ..., download_firmware: _Optional[_Union[DownloadFirmwareReq, _Mapping]] = ..., port_diagnostic: _Optional[_Union[PortDiagnosticReq, _Mapping]] = ..., manage_fpga_config: _Optional[_Union[ManageFPGAConfigReq, _Mapping]] = ..., manage_power_config: _Optional[_Union[ManagePowerConfigReq, _Mapping]] = ..., manage_feature_toggle: _Optional[_Union[ManageFeatureToggleReq, _Mapping]] = ..., manage_device_config: _Optional[_Union[ManageDeviceConfigReq, _Mapping]] = ..., manage_phy_data: _Optional[_Union[ManagePhyDataReq, _Mapping]] = ..., set_telemetry_debug: _Optional[_Union[SetTelemetryDebugReq, _Mapping]] = ..., set_syslog_state: _Optional[_Union[SetSyslogStateReq, _Mapping]] = ..., set_system_time: _Optional[_Union[SetSystemTimeReq, _Mapping]] = ..., set_license: _Optional[_Union[SetLicenseReq, _Mapping]] = ..., schedule_task: _Optional[_Union[ScheduleTaskRequest, _Mapping]] = ..., get_task_status: _Optional[_Union[GetTaskStatusRequest, _Mapping]] = ..., forward_port: _Optional[_Union[ForwardPortReq, _Mapping]] = ..., read_adc: _Optional[_Union[ReadADCReq, _Mapping]] = ..., set_machine_config: _Optional[_Union[SetMachineConfigReq, _Mapping]] = ..., cleanup_nvs_configs: _Optional[_Union[CleanupNVSConfigsReq, _Mapping]] = ..., toggle_mcu_gpio: _Optional[_Union[ToggleMcuGPIOReq, _Mapping]] = ..., toggle_temperature_protect: _Optional[_Union[ToggleTemperatureProtectReq, _Mapping]] = ..., write_file: _Optional[_Union[WriteFileReq, _Mapping]] = ..., read_file: _Optional[_Union[ReadFileReq, _Mapping]] = ..., delete_file: _Optional[_Union[DeleteFileReq, _Mapping]] = ..., list_dir: _Optional[_Union[ListDirReq, _Mapping]] = ...) -> None: ...

class AdminCommandResponse(_message.Message):
    __slots__ = ("id", "status", "get_device_password", "download_firmware", "port_diagnosing", "manage_fpga_config", "manage_power_config", "manage_feature_toggle", "manage_device_config", "manage_phy_data", "set_telemetry_debug", "set_syslog_state", "request_system_time", "set_system_time", "request_license_info", "set_license", "schedule_task", "get_task_status", "request_shutdown_ble_info", "forward_port", "read_adc", "set_machine_config", "cleanup_nvs_configs", "toggle_mcu_gpio", "toggle_temperature_protect", "write_file", "read_file", "delete_file", "list_dir")
    ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    GET_DEVICE_PASSWORD_FIELD_NUMBER: _ClassVar[int]
    DOWNLOAD_FIRMWARE_FIELD_NUMBER: _ClassVar[int]
    PORT_DIAGNOSING_FIELD_NUMBER: _ClassVar[int]
    MANAGE_FPGA_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_POWER_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_FEATURE_TOGGLE_FIELD_NUMBER: _ClassVar[int]
    MANAGE_DEVICE_CONFIG_FIELD_NUMBER: _ClassVar[int]
    MANAGE_PHY_DATA_FIELD_NUMBER: _ClassVar[int]
    SET_TELEMETRY_DEBUG_FIELD_NUMBER: _ClassVar[int]
    SET_SYSLOG_STATE_FIELD_NUMBER: _ClassVar[int]
    REQUEST_SYSTEM_TIME_FIELD_NUMBER: _ClassVar[int]
    SET_SYSTEM_TIME_FIELD_NUMBER: _ClassVar[int]
    REQUEST_LICENSE_INFO_FIELD_NUMBER: _ClassVar[int]
    SET_LICENSE_FIELD_NUMBER: _ClassVar[int]
    SCHEDULE_TASK_FIELD_NUMBER: _ClassVar[int]
    GET_TASK_STATUS_FIELD_NUMBER: _ClassVar[int]
    REQUEST_SHUTDOWN_BLE_INFO_FIELD_NUMBER: _ClassVar[int]
    FORWARD_PORT_FIELD_NUMBER: _ClassVar[int]
    READ_ADC_FIELD_NUMBER: _ClassVar[int]
    SET_MACHINE_CONFIG_FIELD_NUMBER: _ClassVar[int]
    CLEANUP_NVS_CONFIGS_FIELD_NUMBER: _ClassVar[int]
    TOGGLE_MCU_GPIO_FIELD_NUMBER: _ClassVar[int]
    TOGGLE_TEMPERATURE_PROTECT_FIELD_NUMBER: _ClassVar[int]
    WRITE_FILE_FIELD_NUMBER: _ClassVar[int]
    READ_FILE_FIELD_NUMBER: _ClassVar[int]
    DELETE_FILE_FIELD_NUMBER: _ClassVar[int]
    LIST_DIR_FIELD_NUMBER: _ClassVar[int]
    id: int
    status: _data_types_pb2.CommandStatus
    get_device_password: GetDevicePasswordResp
    download_firmware: DownloadFirmwareResp
    port_diagnosing: PortDiagnosticResp
    manage_fpga_config: ManageFPGAConfigResp
    manage_power_config: ManagePowerConfigResp
    manage_feature_toggle: ManageFeatureToggleResp
    manage_device_config: ManageDeviceConfigResp
    manage_phy_data: ManagePhyDataResp
    set_telemetry_debug: SetTelemetryDebugResp
    set_syslog_state: SetSyslogStateResp
    request_system_time: RequestSystemTime
    set_system_time: SetSystemTimeResp
    request_license_info: RequestLicenseInfo
    set_license: SetLicenseResp
    schedule_task: ScheduleTaskResponse
    get_task_status: GetTaskStatusResponse
    request_shutdown_ble_info: RequestShutdownBleInfo
    forward_port: ForwardPortResp
    read_adc: ReadADCResp
    set_machine_config: SetMachineConfigResp
    cleanup_nvs_configs: CleanupNVSConfigsResp
    toggle_mcu_gpio: ToggleMcuGPIOResp
    toggle_temperature_protect: ToggleTemperatureProtectResp
    write_file: WriteFileResp
    read_file: ReadFileResp
    delete_file: DeleteFileResp
    list_dir: ListDirResp
    def __init__(self, id: _Optional[int] = ..., status: _Optional[_Union[_data_types_pb2.CommandStatus, str]] = ..., get_device_password: _Optional[_Union[GetDevicePasswordResp, _Mapping]] = ..., download_firmware: _Optional[_Union[DownloadFirmwareResp, _Mapping]] = ..., port_diagnosing: _Optional[_Union[PortDiagnosticResp, _Mapping]] = ..., manage_fpga_config: _Optional[_Union[ManageFPGAConfigResp, _Mapping]] = ..., manage_power_config: _Optional[_Union[ManagePowerConfigResp, _Mapping]] = ..., manage_feature_toggle: _Optional[_Union[ManageFeatureToggleResp, _Mapping]] = ..., manage_device_config: _Optional[_Union[ManageDeviceConfigResp, _Mapping]] = ..., manage_phy_data: _Optional[_Union[ManagePhyDataResp, _Mapping]] = ..., set_telemetry_debug: _Optional[_Union[SetTelemetryDebugResp, _Mapping]] = ..., set_syslog_state: _Optional[_Union[SetSyslogStateResp, _Mapping]] = ..., request_system_time: _Optional[_Union[RequestSystemTime, _Mapping]] = ..., set_system_time: _Optional[_Union[SetSystemTimeResp, _Mapping]] = ..., request_license_info: _Optional[_Union[RequestLicenseInfo, _Mapping]] = ..., set_license: _Optional[_Union[SetLicenseResp, _Mapping]] = ..., schedule_task: _Optional[_Union[ScheduleTaskResponse, _Mapping]] = ..., get_task_status: _Optional[_Union[GetTaskStatusResponse, _Mapping]] = ..., request_shutdown_ble_info: _Optional[_Union[RequestShutdownBleInfo, _Mapping]] = ..., forward_port: _Optional[_Union[ForwardPortResp, _Mapping]] = ..., read_adc: _Optional[_Union[ReadADCResp, _Mapping]] = ..., set_machine_config: _Optional[_Union[SetMachineConfigResp, _Mapping]] = ..., cleanup_nvs_configs: _Optional[_Union[CleanupNVSConfigsResp, _Mapping]] = ..., toggle_mcu_gpio: _Optional[_Union[ToggleMcuGPIOResp, _Mapping]] = ..., toggle_temperature_protect: _Optional[_Union[ToggleTemperatureProtectResp, _Mapping]] = ..., write_file: _Optional[_Union[WriteFileResp, _Mapping]] = ..., read_file: _Optional[_Union[ReadFileResp, _Mapping]] = ..., delete_file: _Optional[_Union[DeleteFileResp, _Mapping]] = ..., list_dir: _Optional[_Union[ListDirResp, _Mapping]] = ...) -> None: ...

class GetDevicePasswordReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetDevicePasswordResp(_message.Message):
    __slots__ = ("password",)
    PASSWORD_FIELD_NUMBER: _ClassVar[int]
    password: bytes
    def __init__(self, password: _Optional[bytes] = ...) -> None: ...

class ManageFPGAConfigReq(_message.Message):
    __slots__ = ("action", "config", "level")
    ACTION_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    action: _data_types_pb2.ManageAction
    config: _data_types_pb2.FPGAConfig
    level: int
    def __init__(self, action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., config: _Optional[_Union[_data_types_pb2.FPGAConfig, _Mapping]] = ..., level: _Optional[int] = ...) -> None: ...

class ManageFPGAConfigResp(_message.Message):
    __slots__ = ("config",)
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    config: _data_types_pb2.FPGAConfig
    def __init__(self, config: _Optional[_Union[_data_types_pb2.FPGAConfig, _Mapping]] = ...) -> None: ...

class ManagePowerConfigReq(_message.Message):
    __slots__ = ("action", "config")
    ACTION_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    action: _data_types_pb2.ManageAction
    config: _data_types_pb2.PowerConfig
    def __init__(self, action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., config: _Optional[_Union[_data_types_pb2.PowerConfig, _Mapping]] = ...) -> None: ...

class ManagePowerConfigResp(_message.Message):
    __slots__ = ("config",)
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    config: _data_types_pb2.PowerConfig
    def __init__(self, config: _Optional[_Union[_data_types_pb2.PowerConfig, _Mapping]] = ...) -> None: ...

class ManageFeatureToggleReq(_message.Message):
    __slots__ = ("feature", "action", "enabled")
    FEATURE_FIELD_NUMBER: _ClassVar[int]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    feature: _data_types_pb2.ManageFeature
    action: _data_types_pb2.ManageAction
    enabled: bool
    def __init__(self, feature: _Optional[_Union[_data_types_pb2.ManageFeature, str]] = ..., action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., enabled: bool = ...) -> None: ...

class ManageFeatureToggleResp(_message.Message):
    __slots__ = ("feature", "action", "enabled")
    FEATURE_FIELD_NUMBER: _ClassVar[int]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    feature: _data_types_pb2.ManageFeature
    action: _data_types_pb2.ManageAction
    enabled: bool
    def __init__(self, feature: _Optional[_Union[_data_types_pb2.ManageFeature, str]] = ..., action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., enabled: bool = ...) -> None: ...

class ManageDeviceConfigReq(_message.Message):
    __slots__ = ("entity", "action", "value")
    ENTITY_FIELD_NUMBER: _ClassVar[int]
    ACTION_FIELD_NUMBER: _ClassVar[int]
    VALUE_FIELD_NUMBER: _ClassVar[int]
    entity: _data_types_pb2.DeviceConfigEntity
    action: _data_types_pb2.ManageAction
    value: str
    def __init__(self, entity: _Optional[_Union[_data_types_pb2.DeviceConfigEntity, str]] = ..., action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., value: _Optional[str] = ...) -> None: ...

class ManageDeviceConfigResp(_message.Message):
    __slots__ = ("entity", "value")
    ENTITY_FIELD_NUMBER: _ClassVar[int]
    VALUE_FIELD_NUMBER: _ClassVar[int]
    entity: _data_types_pb2.DeviceConfigEntity
    value: str
    def __init__(self, entity: _Optional[_Union[_data_types_pb2.DeviceConfigEntity, str]] = ..., value: _Optional[str] = ...) -> None: ...

class ManagePhyDataReq(_message.Message):
    __slots__ = ("action", "data", "expected_crc32")
    ACTION_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    EXPECTED_CRC32_FIELD_NUMBER: _ClassVar[int]
    action: _data_types_pb2.ManageAction
    data: bytes
    expected_crc32: int
    def __init__(self, action: _Optional[_Union[_data_types_pb2.ManageAction, str]] = ..., data: _Optional[bytes] = ..., expected_crc32: _Optional[int] = ...) -> None: ...

class ManagePhyDataResp(_message.Message):
    __slots__ = ("data", "crc32")
    DATA_FIELD_NUMBER: _ClassVar[int]
    CRC32_FIELD_NUMBER: _ClassVar[int]
    data: bytes
    crc32: int
    def __init__(self, data: _Optional[bytes] = ..., crc32: _Optional[int] = ...) -> None: ...

class SetSyslogStateReq(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class SetSyslogStateResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetSystemTimeReq(_message.Message):
    __slots__ = ("timestamp",)
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    def __init__(self, timestamp: _Optional[int] = ...) -> None: ...

class SetSystemTimeResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetTelemetryDebugReq(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class SetTelemetryDebugResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetLicenseReq(_message.Message):
    __slots__ = ("data",)
    DATA_FIELD_NUMBER: _ClassVar[int]
    data: _containers.RepeatedCompositeFieldContainer[_data_types_pb2.LicenseData]
    def __init__(self, data: _Optional[_Iterable[_Union[_data_types_pb2.LicenseData, _Mapping]]] = ...) -> None: ...

class SetLicenseResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ToggleTemperatureProtectReq(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class ToggleTemperatureProtectResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class RequestLicenseInfo(_message.Message):
    __slots__ = ("nonce",)
    NONCE_FIELD_NUMBER: _ClassVar[int]
    nonce: bytes
    def __init__(self, nonce: _Optional[bytes] = ...) -> None: ...

class RequestSystemTime(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class RequestShutdownBleInfo(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ForwardPortReq(_message.Message):
    __slots__ = ("port", "command", "data")
    PORT_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    port: int
    command: int
    data: bytes
    def __init__(self, port: _Optional[int] = ..., command: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class ForwardPortResp(_message.Message):
    __slots__ = ("command", "data")
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    command: int
    data: bytes
    def __init__(self, command: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class DownloadFirmwareReq(_message.Message):
    __slots__ = ("sw3566_version", "fpga_version")
    SW3566_VERSION_FIELD_NUMBER: _ClassVar[int]
    FPGA_VERSION_FIELD_NUMBER: _ClassVar[int]
    sw3566_version: _data_types_pb2.Version
    fpga_version: _data_types_pb2.Version
    def __init__(self, sw3566_version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ..., fpga_version: _Optional[_Union[_data_types_pb2.Version, _Mapping]] = ...) -> None: ...

class DownloadFirmwareResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class PortDiagnosticReq(_message.Message):
    __slots__ = ("port", "diagnostic")
    PORT_FIELD_NUMBER: _ClassVar[int]
    DIAGNOSTIC_FIELD_NUMBER: _ClassVar[int]
    port: int
    diagnostic: bool
    def __init__(self, port: _Optional[int] = ..., diagnostic: bool = ...) -> None: ...

class PortDiagnosticResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ScheduleTaskRequest(_message.Message):
    __slots__ = ("task",)
    TASK_FIELD_NUMBER: _ClassVar[int]
    task: _data_types_pb2.Task
    def __init__(self, task: _Optional[_Union[_data_types_pb2.Task, _Mapping]] = ...) -> None: ...

class ScheduleTaskResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetTaskStatusRequest(_message.Message):
    __slots__ = ("task_id",)
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    task_id: int
    def __init__(self, task_id: _Optional[int] = ...) -> None: ...

class GetTaskStatusResponse(_message.Message):
    __slots__ = ("status",)
    STATUS_FIELD_NUMBER: _ClassVar[int]
    status: _data_types_pb2.TaskExecutionStatus
    def __init__(self, status: _Optional[_Union[_data_types_pb2.TaskExecutionStatus, str]] = ...) -> None: ...

class ReadADCReq(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class ReadADCResp(_message.Message):
    __slots__ = ("adc_value",)
    ADC_VALUE_FIELD_NUMBER: _ClassVar[int]
    adc_value: int
    def __init__(self, adc_value: _Optional[int] = ...) -> None: ...

class SetMachineConfigReq(_message.Message):
    __slots__ = ("operation", "config")
    OPERATION_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    operation: _data_types_pb2.ConfigOperation
    config: _data_types_pb2.MachineConfig
    def __init__(self, operation: _Optional[_Union[_data_types_pb2.ConfigOperation, str]] = ..., config: _Optional[_Union[_data_types_pb2.MachineConfig, _Mapping]] = ...) -> None: ...

class SetMachineConfigResp(_message.Message):
    __slots__ = ("success",)
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    def __init__(self, success: bool = ...) -> None: ...

class ToggleMcuGPIOReq(_message.Message):
    __slots__ = ("port",)
    PORT_FIELD_NUMBER: _ClassVar[int]
    port: int
    def __init__(self, port: _Optional[int] = ...) -> None: ...

class ToggleMcuGPIOResp(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class CleanupNVSConfigsReq(_message.Message):
    __slots__ = ("force",)
    FORCE_FIELD_NUMBER: _ClassVar[int]
    force: bool
    def __init__(self, force: bool = ...) -> None: ...

class CleanupNVSConfigsResp(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class WriteFileReq(_message.Message):
    __slots__ = ("path", "offset", "data")
    PATH_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    path: str
    offset: int
    data: bytes
    def __init__(self, path: _Optional[str] = ..., offset: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class WriteFileResp(_message.Message):
    __slots__ = ("success", "bytes_written")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    BYTES_WRITTEN_FIELD_NUMBER: _ClassVar[int]
    success: bool
    bytes_written: int
    def __init__(self, success: bool = ..., bytes_written: _Optional[int] = ...) -> None: ...

class ReadFileReq(_message.Message):
    __slots__ = ("path", "offset", "size")
    PATH_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    SIZE_FIELD_NUMBER: _ClassVar[int]
    path: str
    offset: int
    size: int
    def __init__(self, path: _Optional[str] = ..., offset: _Optional[int] = ..., size: _Optional[int] = ...) -> None: ...

class ReadFileResp(_message.Message):
    __slots__ = ("success", "data", "total_size")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    TOTAL_SIZE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    data: bytes
    total_size: int
    def __init__(self, success: bool = ..., data: _Optional[bytes] = ..., total_size: _Optional[int] = ...) -> None: ...

class DeleteFileReq(_message.Message):
    __slots__ = ("path", "recursive")
    PATH_FIELD_NUMBER: _ClassVar[int]
    RECURSIVE_FIELD_NUMBER: _ClassVar[int]
    path: str
    recursive: bool
    def __init__(self, path: _Optional[str] = ..., recursive: bool = ...) -> None: ...

class DeleteFileResp(_message.Message):
    __slots__ = ("success",)
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    success: bool
    def __init__(self, success: bool = ...) -> None: ...

class ListDirReq(_message.Message):
    __slots__ = ("path",)
    PATH_FIELD_NUMBER: _ClassVar[int]
    path: str
    def __init__(self, path: _Optional[str] = ...) -> None: ...

class ListDirResp(_message.Message):
    __slots__ = ("success", "entries")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    ENTRIES_FIELD_NUMBER: _ClassVar[int]
    success: bool
    entries: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, success: bool = ..., entries: _Optional[_Iterable[str]] = ...) -> None: ...
