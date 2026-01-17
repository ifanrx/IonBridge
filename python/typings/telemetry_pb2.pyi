import data_types_pb2 as _data_types_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class TelemetryData(_message.Message):
    __slots__ = ("ping_info", "coredump_slice", "device_boot_info", "device_stats_data", "power_historical_data", "power_allocation_data", "pdp_cap_data", "upgrade_data", "uart_metrics_data", "sw3566_bootloader_info", "out_of_memory_alert_info", "over_temperature_alert_info", "charging_alert_info", "acdc_self_test_info", "task_execution_result", "config_report", "config_request", "programmer_start", "programmer_finish")
    PING_INFO_FIELD_NUMBER: _ClassVar[int]
    COREDUMP_SLICE_FIELD_NUMBER: _ClassVar[int]
    DEVICE_BOOT_INFO_FIELD_NUMBER: _ClassVar[int]
    DEVICE_STATS_DATA_FIELD_NUMBER: _ClassVar[int]
    POWER_HISTORICAL_DATA_FIELD_NUMBER: _ClassVar[int]
    POWER_ALLOCATION_DATA_FIELD_NUMBER: _ClassVar[int]
    PDP_CAP_DATA_FIELD_NUMBER: _ClassVar[int]
    UPGRADE_DATA_FIELD_NUMBER: _ClassVar[int]
    UART_METRICS_DATA_FIELD_NUMBER: _ClassVar[int]
    SW3566_BOOTLOADER_INFO_FIELD_NUMBER: _ClassVar[int]
    OUT_OF_MEMORY_ALERT_INFO_FIELD_NUMBER: _ClassVar[int]
    OVER_TEMPERATURE_ALERT_INFO_FIELD_NUMBER: _ClassVar[int]
    CHARGING_ALERT_INFO_FIELD_NUMBER: _ClassVar[int]
    ACDC_SELF_TEST_INFO_FIELD_NUMBER: _ClassVar[int]
    TASK_EXECUTION_RESULT_FIELD_NUMBER: _ClassVar[int]
    CONFIG_REPORT_FIELD_NUMBER: _ClassVar[int]
    CONFIG_REQUEST_FIELD_NUMBER: _ClassVar[int]
    PROGRAMMER_START_FIELD_NUMBER: _ClassVar[int]
    PROGRAMMER_FINISH_FIELD_NUMBER: _ClassVar[int]
    ping_info: _data_types_pb2.PingInfo
    coredump_slice: _data_types_pb2.CoredumpPartitionSlice
    device_boot_info: _data_types_pb2.DeviceBootInfo
    device_stats_data: _data_types_pb2.DeviceStatsData
    power_historical_data: _data_types_pb2.PowerHistoricalData
    power_allocation_data: _data_types_pb2.PowerAllocationData
    pdp_cap_data: _data_types_pb2.PDPCapData
    upgrade_data: _data_types_pb2.UpgradeData
    uart_metrics_data: _data_types_pb2.UartMetricsData
    sw3566_bootloader_info: _data_types_pb2.SW3566BootloaderInfo
    out_of_memory_alert_info: _data_types_pb2.OutOfMemoryAlertInfo
    over_temperature_alert_info: _data_types_pb2.OverTemperatureAlertInfo
    charging_alert_info: _data_types_pb2.ChargingAlertInfo
    acdc_self_test_info: _data_types_pb2.ACDCSelfTestInfo
    task_execution_result: _data_types_pb2.TaskExecutionResult
    config_report: _data_types_pb2.ConfigReport
    config_request: _data_types_pb2.ConfigRequest
    programmer_start: _data_types_pb2.ProgrammerStart
    programmer_finish: _data_types_pb2.ProgrammerFinish
    def __init__(self, ping_info: _Optional[_Union[_data_types_pb2.PingInfo, _Mapping]] = ..., coredump_slice: _Optional[_Union[_data_types_pb2.CoredumpPartitionSlice, _Mapping]] = ..., device_boot_info: _Optional[_Union[_data_types_pb2.DeviceBootInfo, _Mapping]] = ..., device_stats_data: _Optional[_Union[_data_types_pb2.DeviceStatsData, _Mapping]] = ..., power_historical_data: _Optional[_Union[_data_types_pb2.PowerHistoricalData, _Mapping]] = ..., power_allocation_data: _Optional[_Union[_data_types_pb2.PowerAllocationData, _Mapping]] = ..., pdp_cap_data: _Optional[_Union[_data_types_pb2.PDPCapData, _Mapping]] = ..., upgrade_data: _Optional[_Union[_data_types_pb2.UpgradeData, _Mapping]] = ..., uart_metrics_data: _Optional[_Union[_data_types_pb2.UartMetricsData, _Mapping]] = ..., sw3566_bootloader_info: _Optional[_Union[_data_types_pb2.SW3566BootloaderInfo, _Mapping]] = ..., out_of_memory_alert_info: _Optional[_Union[_data_types_pb2.OutOfMemoryAlertInfo, _Mapping]] = ..., over_temperature_alert_info: _Optional[_Union[_data_types_pb2.OverTemperatureAlertInfo, _Mapping]] = ..., charging_alert_info: _Optional[_Union[_data_types_pb2.ChargingAlertInfo, _Mapping]] = ..., acdc_self_test_info: _Optional[_Union[_data_types_pb2.ACDCSelfTestInfo, _Mapping]] = ..., task_execution_result: _Optional[_Union[_data_types_pb2.TaskExecutionResult, _Mapping]] = ..., config_report: _Optional[_Union[_data_types_pb2.ConfigReport, _Mapping]] = ..., config_request: _Optional[_Union[_data_types_pb2.ConfigRequest, _Mapping]] = ..., programmer_start: _Optional[_Union[_data_types_pb2.ProgrammerStart, _Mapping]] = ..., programmer_finish: _Optional[_Union[_data_types_pb2.ProgrammerFinish, _Mapping]] = ...) -> None: ...
