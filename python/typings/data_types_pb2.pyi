from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CommandStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    SUCCESS: _ClassVar[CommandStatus]
    FAILURE: _ClassVar[CommandStatus]
    FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE: _ClassVar[CommandStatus]

class ConfigOperation(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CONFIG_UPDATE: _ClassVar[ConfigOperation]
    CONFIG_REQUEST_UPLOAD: _ClassVar[ConfigOperation]

class PowerState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    POWER_OFF: _ClassVar[PowerState]
    POWER_ON: _ClassVar[PowerState]

class WiFiState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CONFIGURATION: _ClassVar[WiFiState]
    SCAN_AND_CONNECT: _ClassVar[WiFiState]
    CONNECTING: _ClassVar[WiFiState]
    CONNECTED: _ClassVar[WiFiState]
    CHECKING_CONN: _ClassVar[WiFiState]
    IDLE: _ClassVar[WiFiState]
    DISCONNECTED: _ClassVar[WiFiState]
    RECONNECTING: _ClassVar[WiFiState]
    WAITING_FOR_NEXT: _ClassVar[WiFiState]
    SCANNING: _ClassVar[WiFiState]
    CONNECTION_ABORT: _ClassVar[WiFiState]
    WAITING_FOR_DEFAULT: _ClassVar[WiFiState]

class TemperatureMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    POWER_PRIORITY: _ClassVar[TemperatureMode]
    TEMPERATURE_PRIORITY: _ClassVar[TemperatureMode]

class DisplayMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    OFF: _ClassVar[DisplayMode]
    MANUAL: _ClassVar[DisplayMode]
    POWER_METER: _ClassVar[DisplayMode]

class CableCompensation(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    RESISTANCE_65_MOHM: _ClassVar[CableCompensation]
    RESISTANCE_100_MOHM: _ClassVar[CableCompensation]

class FixedVoltageOffset(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    OFFSET_0_MV: _ClassVar[FixedVoltageOffset]
    OFFSET_100_MV: _ClassVar[FixedVoltageOffset]
    OFFSET_150_MV: _ClassVar[FixedVoltageOffset]
    OFFSET_200_MV: _ClassVar[FixedVoltageOffset]

class DisplayFlip(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    NORMAL: _ClassVar[DisplayFlip]
    FLIP: _ClassVar[DisplayFlip]

class FastChargingProtocol(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    NONE: _ClassVar[FastChargingProtocol]
    QC2: _ClassVar[FastChargingProtocol]
    QC3: _ClassVar[FastChargingProtocol]
    QC3P: _ClassVar[FastChargingProtocol]
    SFCP: _ClassVar[FastChargingProtocol]
    AFC: _ClassVar[FastChargingProtocol]
    FCP: _ClassVar[FastChargingProtocol]
    SCP: _ClassVar[FastChargingProtocol]
    VOOC1P0: _ClassVar[FastChargingProtocol]
    VOOC4P0: _ClassVar[FastChargingProtocol]
    SVOOC2P0: _ClassVar[FastChargingProtocol]
    TFCP: _ClassVar[FastChargingProtocol]
    UFCS: _ClassVar[FastChargingProtocol]
    PE1: _ClassVar[FastChargingProtocol]
    PE2: _ClassVar[FastChargingProtocol]
    PD_FIX5V: _ClassVar[FastChargingProtocol]
    PD_FIXHV: _ClassVar[FastChargingProtocol]
    PD_SPR_AVS: _ClassVar[FastChargingProtocol]
    PD_PPS: _ClassVar[FastChargingProtocol]
    PD_EPR_HV: _ClassVar[FastChargingProtocol]
    PD_AVS: _ClassVar[FastChargingProtocol]
    NOT_CHARGING: _ClassVar[FastChargingProtocol]

class PortType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    TYPE_A: _ClassVar[PortType]
    TYPE_C: _ClassVar[PortType]

class TaskExecutionStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    PENDING: _ClassVar[TaskExecutionStatus]
    SUCCEEDED: _ClassVar[TaskExecutionStatus]
    FAILED: _ClassVar[TaskExecutionStatus]

class DisplayRotation(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    ROTATION_0: _ClassVar[DisplayRotation]
    ROTATION_90: _ClassVar[DisplayRotation]
    ROTATION_180: _ClassVar[DisplayRotation]
    ROTATION_270: _ClassVar[DisplayRotation]

class IdleAnimationType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    IDLE_NONE: _ClassVar[IdleAnimationType]
    IDLE_METEOR: _ClassVar[IdleAnimationType]
    IDLE_GAME_OF_LIFE: _ClassVar[IdleAnimationType]

class ManageAction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CREATE: _ClassVar[ManageAction]
    UPDATE: _ClassVar[ManageAction]
    DELETE: _ClassVar[ManageAction]
    READ: _ClassVar[ManageAction]

class WiFiAuthMode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    WIFI_AUTH_OPEN: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WEP: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA2_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA_WPA2_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_ENTERPRISE: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA3_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA2_WPA3_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WAPI_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_OWE: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA3_ENT_192: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA3_EXT_PSK: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_DPP: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA3_ENTERPRISE: _ClassVar[WiFiAuthMode]
    WIFI_AUTH_WPA2_WPA3_ENTERPRISE: _ClassVar[WiFiAuthMode]

class ChargingStrategy(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    FAST_CHARGING: _ClassVar[ChargingStrategy]
    SLOW_CHARGING: _ClassVar[ChargingStrategy]
    HIGH_PERFORMANCE_CHARGING: _ClassVar[ChargingStrategy]
    ULTRA_FAST_SINGLE_PORT_CHARGING: _ClassVar[ChargingStrategy]

class UpgradeStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    UPGRADE_START: _ClassVar[UpgradeStatus]
    UPGRADE_FAIL: _ClassVar[UpgradeStatus]
    UPGRADE_SUCCESS: _ClassVar[UpgradeStatus]

class UpgradeFailureReason(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    REASON_NONE: _ClassVar[UpgradeFailureReason]
    REASON_ESP_ERROR: _ClassVar[UpgradeFailureReason]
    REASON_CONFIRM_FAILED: _ClassVar[UpgradeFailureReason]
    REASON_CONFIRM_TIMEOUT: _ClassVar[UpgradeFailureReason]

class ManageFeature(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    FPGA_POWER_CONTROL: _ClassVar[ManageFeature]
    POWER_ALLOCATOR: _ClassVar[ManageFeature]

class ProgrammerState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    STATE_INIT: _ClassVar[ProgrammerState]
    SEND_METADATA: _ClassVar[ProgrammerState]
    WAIT_METADATA_ACK: _ClassVar[ProgrammerState]
    SEND_BLOCKS: _ClassVar[ProgrammerState]
    WAIT_BLOCKS_ACK: _ClassVar[ProgrammerState]
    SEND_IUM: _ClassVar[ProgrammerState]
    WAIT_IUM_ACK: _ClassVar[ProgrammerState]
    DONE: _ClassVar[ProgrammerState]
    ERROR: _ClassVar[ProgrammerState]

class DeviceModel(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DEVB: _ClassVar[DeviceModel]
    PRO: _ClassVar[DeviceModel]
    ULTRA: _ClassVar[DeviceModel]
    BIRD: _ClassVar[DeviceModel]
    FAKE: _ClassVar[DeviceModel]

class DeviceConfigEntity(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    ENTITY_HARDWARE_REVISION: _ClassVar[DeviceConfigEntity]
    ENTITY_PRODUCT_FAMILY: _ClassVar[DeviceConfigEntity]
    ENTITY_DEVICE_MODEL: _ClassVar[DeviceConfigEntity]
    ENTITY_DEVICE_COLOR: _ClassVar[DeviceConfigEntity]
    ENTITY_DEVICE_NAME: _ClassVar[DeviceConfigEntity]

class ACDCSelfTestStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    ACDC_SELF_TEST_NOT_RUN: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_PASSED: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_NO_PORT: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_FAIL_L0: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_FAIL_L1: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_FAIL_L2: _ClassVar[ACDCSelfTestStatus]
    ACDC_SELF_TEST_FAIL_L3: _ClassVar[ACDCSelfTestStatus]
SUCCESS: CommandStatus
FAILURE: CommandStatus
FLASH_NOT_WRITABLE_AT_HIGH_TEMPERATURE: CommandStatus
CONFIG_UPDATE: ConfigOperation
CONFIG_REQUEST_UPLOAD: ConfigOperation
POWER_OFF: PowerState
POWER_ON: PowerState
CONFIGURATION: WiFiState
SCAN_AND_CONNECT: WiFiState
CONNECTING: WiFiState
CONNECTED: WiFiState
CHECKING_CONN: WiFiState
IDLE: WiFiState
DISCONNECTED: WiFiState
RECONNECTING: WiFiState
WAITING_FOR_NEXT: WiFiState
SCANNING: WiFiState
CONNECTION_ABORT: WiFiState
WAITING_FOR_DEFAULT: WiFiState
POWER_PRIORITY: TemperatureMode
TEMPERATURE_PRIORITY: TemperatureMode
OFF: DisplayMode
MANUAL: DisplayMode
POWER_METER: DisplayMode
RESISTANCE_65_MOHM: CableCompensation
RESISTANCE_100_MOHM: CableCompensation
OFFSET_0_MV: FixedVoltageOffset
OFFSET_100_MV: FixedVoltageOffset
OFFSET_150_MV: FixedVoltageOffset
OFFSET_200_MV: FixedVoltageOffset
NORMAL: DisplayFlip
FLIP: DisplayFlip
NONE: FastChargingProtocol
QC2: FastChargingProtocol
QC3: FastChargingProtocol
QC3P: FastChargingProtocol
SFCP: FastChargingProtocol
AFC: FastChargingProtocol
FCP: FastChargingProtocol
SCP: FastChargingProtocol
VOOC1P0: FastChargingProtocol
VOOC4P0: FastChargingProtocol
SVOOC2P0: FastChargingProtocol
TFCP: FastChargingProtocol
UFCS: FastChargingProtocol
PE1: FastChargingProtocol
PE2: FastChargingProtocol
PD_FIX5V: FastChargingProtocol
PD_FIXHV: FastChargingProtocol
PD_SPR_AVS: FastChargingProtocol
PD_PPS: FastChargingProtocol
PD_EPR_HV: FastChargingProtocol
PD_AVS: FastChargingProtocol
NOT_CHARGING: FastChargingProtocol
TYPE_A: PortType
TYPE_C: PortType
PENDING: TaskExecutionStatus
SUCCEEDED: TaskExecutionStatus
FAILED: TaskExecutionStatus
ROTATION_0: DisplayRotation
ROTATION_90: DisplayRotation
ROTATION_180: DisplayRotation
ROTATION_270: DisplayRotation
IDLE_NONE: IdleAnimationType
IDLE_METEOR: IdleAnimationType
IDLE_GAME_OF_LIFE: IdleAnimationType
CREATE: ManageAction
UPDATE: ManageAction
DELETE: ManageAction
READ: ManageAction
WIFI_AUTH_OPEN: WiFiAuthMode
WIFI_AUTH_WEP: WiFiAuthMode
WIFI_AUTH_WPA_PSK: WiFiAuthMode
WIFI_AUTH_WPA2_PSK: WiFiAuthMode
WIFI_AUTH_WPA_WPA2_PSK: WiFiAuthMode
WIFI_AUTH_ENTERPRISE: WiFiAuthMode
WIFI_AUTH_WPA3_PSK: WiFiAuthMode
WIFI_AUTH_WPA2_WPA3_PSK: WiFiAuthMode
WIFI_AUTH_WAPI_PSK: WiFiAuthMode
WIFI_AUTH_OWE: WiFiAuthMode
WIFI_AUTH_WPA3_ENT_192: WiFiAuthMode
WIFI_AUTH_WPA3_EXT_PSK: WiFiAuthMode
WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE: WiFiAuthMode
WIFI_AUTH_DPP: WiFiAuthMode
WIFI_AUTH_WPA3_ENTERPRISE: WiFiAuthMode
WIFI_AUTH_WPA2_WPA3_ENTERPRISE: WiFiAuthMode
FAST_CHARGING: ChargingStrategy
SLOW_CHARGING: ChargingStrategy
HIGH_PERFORMANCE_CHARGING: ChargingStrategy
ULTRA_FAST_SINGLE_PORT_CHARGING: ChargingStrategy
UPGRADE_START: UpgradeStatus
UPGRADE_FAIL: UpgradeStatus
UPGRADE_SUCCESS: UpgradeStatus
REASON_NONE: UpgradeFailureReason
REASON_ESP_ERROR: UpgradeFailureReason
REASON_CONFIRM_FAILED: UpgradeFailureReason
REASON_CONFIRM_TIMEOUT: UpgradeFailureReason
FPGA_POWER_CONTROL: ManageFeature
POWER_ALLOCATOR: ManageFeature
STATE_INIT: ProgrammerState
SEND_METADATA: ProgrammerState
WAIT_METADATA_ACK: ProgrammerState
SEND_BLOCKS: ProgrammerState
WAIT_BLOCKS_ACK: ProgrammerState
SEND_IUM: ProgrammerState
WAIT_IUM_ACK: ProgrammerState
DONE: ProgrammerState
ERROR: ProgrammerState
DEVB: DeviceModel
PRO: DeviceModel
ULTRA: DeviceModel
BIRD: DeviceModel
FAKE: DeviceModel
ENTITY_HARDWARE_REVISION: DeviceConfigEntity
ENTITY_PRODUCT_FAMILY: DeviceConfigEntity
ENTITY_DEVICE_MODEL: DeviceConfigEntity
ENTITY_DEVICE_COLOR: DeviceConfigEntity
ENTITY_DEVICE_NAME: DeviceConfigEntity
ACDC_SELF_TEST_NOT_RUN: ACDCSelfTestStatus
ACDC_SELF_TEST_PASSED: ACDCSelfTestStatus
ACDC_SELF_TEST_NO_PORT: ACDCSelfTestStatus
ACDC_SELF_TEST_FAIL_L0: ACDCSelfTestStatus
ACDC_SELF_TEST_FAIL_L1: ACDCSelfTestStatus
ACDC_SELF_TEST_FAIL_L2: ACDCSelfTestStatus
ACDC_SELF_TEST_FAIL_L3: ACDCSelfTestStatus

class Version(_message.Message):
    __slots__ = ("major", "minor", "patch")
    MAJOR_FIELD_NUMBER: _ClassVar[int]
    MINOR_FIELD_NUMBER: _ClassVar[int]
    PATCH_FIELD_NUMBER: _ClassVar[int]
    major: int
    minor: int
    patch: int
    def __init__(self, major: _Optional[int] = ..., minor: _Optional[int] = ..., patch: _Optional[int] = ...) -> None: ...

class PowerFeatures(_message.Message):
    __slots__ = ("enable_tfcp", "enable_pe", "enable_qc2p0", "enable_qc3p0", "enable_qc3plus", "enable_afc", "enable_fcp", "enable_hv_scp", "enable_lv_scp", "enable_sfcp", "enable_apple", "enable_samsung", "enable_ufcs", "enable_pd", "enable_pd_compat_mode", "limited_current_mode", "enable_pd_lvpps", "enable_pd_epr", "enable_pd_rpi", "enable_pd_hvpps", "enable_pd_hard_reset_on_refusal", "enable_pd_spr_avs")
    ENABLE_TFCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PE_FIELD_NUMBER: _ClassVar[int]
    ENABLE_QC2P0_FIELD_NUMBER: _ClassVar[int]
    ENABLE_QC3P0_FIELD_NUMBER: _ClassVar[int]
    ENABLE_QC3PLUS_FIELD_NUMBER: _ClassVar[int]
    ENABLE_AFC_FIELD_NUMBER: _ClassVar[int]
    ENABLE_FCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_HV_SCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_LV_SCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_SFCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_APPLE_FIELD_NUMBER: _ClassVar[int]
    ENABLE_SAMSUNG_FIELD_NUMBER: _ClassVar[int]
    ENABLE_UFCS_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_COMPAT_MODE_FIELD_NUMBER: _ClassVar[int]
    LIMITED_CURRENT_MODE_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_LVPPS_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_EPR_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_RPI_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_HVPPS_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_HARD_RESET_ON_REFUSAL_FIELD_NUMBER: _ClassVar[int]
    ENABLE_PD_SPR_AVS_FIELD_NUMBER: _ClassVar[int]
    enable_tfcp: bool
    enable_pe: bool
    enable_qc2p0: bool
    enable_qc3p0: bool
    enable_qc3plus: bool
    enable_afc: bool
    enable_fcp: bool
    enable_hv_scp: bool
    enable_lv_scp: bool
    enable_sfcp: bool
    enable_apple: bool
    enable_samsung: bool
    enable_ufcs: bool
    enable_pd: bool
    enable_pd_compat_mode: bool
    limited_current_mode: bool
    enable_pd_lvpps: bool
    enable_pd_epr: bool
    enable_pd_rpi: bool
    enable_pd_hvpps: bool
    enable_pd_hard_reset_on_refusal: bool
    enable_pd_spr_avs: bool
    def __init__(self, enable_tfcp: bool = ..., enable_pe: bool = ..., enable_qc2p0: bool = ..., enable_qc3p0: bool = ..., enable_qc3plus: bool = ..., enable_afc: bool = ..., enable_fcp: bool = ..., enable_hv_scp: bool = ..., enable_lv_scp: bool = ..., enable_sfcp: bool = ..., enable_apple: bool = ..., enable_samsung: bool = ..., enable_ufcs: bool = ..., enable_pd: bool = ..., enable_pd_compat_mode: bool = ..., limited_current_mode: bool = ..., enable_pd_lvpps: bool = ..., enable_pd_epr: bool = ..., enable_pd_rpi: bool = ..., enable_pd_hvpps: bool = ..., enable_pd_hard_reset_on_refusal: bool = ..., enable_pd_spr_avs: bool = ...) -> None: ...

class ClientPDStatus(_message.Message):
    __slots__ = ("battery_vid", "battery_pid", "battery_design_capacity", "battery_last_full_charge_capacity", "battery_present_capacity", "battery_invalid", "battery_present", "battery_status", "cable_is_active", "cable_termination_type", "cable_epr_mode_capable", "cable_active_phy_type", "cable_latency", "cable_max_vbus_voltage", "cable_max_vbus_current", "cable_usb_highest_speed", "cable_active_element", "cable_active_usb4", "cable_active_usb2p0", "cable_active_usb3p2", "cable_active_usb_lanes", "cable_active_optically_isolated", "cable_active_usb4_asym", "cable_active_usb_gen", "request_epr_mode_capable", "request_pdo_id", "request_usb_communications_capable", "request_capability_mismatch", "sink_capabilities", "sink_cap_pdo_count", "status_temperature", "cable_vid", "cable_pid", "manufacturer_vid", "manufacturer_pid", "sink_minimum_pdp", "sink_operational_pdp", "sink_maximum_pdp", "operating_current", "pd_revision", "pps_charging_supported", "has_battery", "dual_role_power", "has_emarker", "operating_voltage", "request_ppsavs", "cable_xid", "bcd_device")
    BATTERY_VID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_DESIGN_CAPACITY_FIELD_NUMBER: _ClassVar[int]
    BATTERY_LAST_FULL_CHARGE_CAPACITY_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PRESENT_CAPACITY_FIELD_NUMBER: _ClassVar[int]
    BATTERY_INVALID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PRESENT_FIELD_NUMBER: _ClassVar[int]
    BATTERY_STATUS_FIELD_NUMBER: _ClassVar[int]
    CABLE_IS_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    CABLE_TERMINATION_TYPE_FIELD_NUMBER: _ClassVar[int]
    CABLE_EPR_MODE_CAPABLE_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_PHY_TYPE_FIELD_NUMBER: _ClassVar[int]
    CABLE_LATENCY_FIELD_NUMBER: _ClassVar[int]
    CABLE_MAX_VBUS_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    CABLE_MAX_VBUS_CURRENT_FIELD_NUMBER: _ClassVar[int]
    CABLE_USB_HIGHEST_SPEED_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_ELEMENT_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB4_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB2P0_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB3P2_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB_LANES_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_OPTICALLY_ISOLATED_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB4_ASYM_FIELD_NUMBER: _ClassVar[int]
    CABLE_ACTIVE_USB_GEN_FIELD_NUMBER: _ClassVar[int]
    REQUEST_EPR_MODE_CAPABLE_FIELD_NUMBER: _ClassVar[int]
    REQUEST_PDO_ID_FIELD_NUMBER: _ClassVar[int]
    REQUEST_USB_COMMUNICATIONS_CAPABLE_FIELD_NUMBER: _ClassVar[int]
    REQUEST_CAPABILITY_MISMATCH_FIELD_NUMBER: _ClassVar[int]
    SINK_CAPABILITIES_FIELD_NUMBER: _ClassVar[int]
    SINK_CAP_PDO_COUNT_FIELD_NUMBER: _ClassVar[int]
    STATUS_TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    CABLE_VID_FIELD_NUMBER: _ClassVar[int]
    CABLE_PID_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_VID_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_PID_FIELD_NUMBER: _ClassVar[int]
    SINK_MINIMUM_PDP_FIELD_NUMBER: _ClassVar[int]
    SINK_OPERATIONAL_PDP_FIELD_NUMBER: _ClassVar[int]
    SINK_MAXIMUM_PDP_FIELD_NUMBER: _ClassVar[int]
    OPERATING_CURRENT_FIELD_NUMBER: _ClassVar[int]
    PD_REVISION_FIELD_NUMBER: _ClassVar[int]
    PPS_CHARGING_SUPPORTED_FIELD_NUMBER: _ClassVar[int]
    HAS_BATTERY_FIELD_NUMBER: _ClassVar[int]
    DUAL_ROLE_POWER_FIELD_NUMBER: _ClassVar[int]
    HAS_EMARKER_FIELD_NUMBER: _ClassVar[int]
    OPERATING_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    REQUEST_PPSAVS_FIELD_NUMBER: _ClassVar[int]
    CABLE_XID_FIELD_NUMBER: _ClassVar[int]
    BCD_DEVICE_FIELD_NUMBER: _ClassVar[int]
    battery_vid: int
    battery_pid: int
    battery_design_capacity: int
    battery_last_full_charge_capacity: int
    battery_present_capacity: int
    battery_invalid: bool
    battery_present: bool
    battery_status: int
    cable_is_active: bool
    cable_termination_type: bool
    cable_epr_mode_capable: bool
    cable_active_phy_type: int
    cable_latency: int
    cable_max_vbus_voltage: int
    cable_max_vbus_current: int
    cable_usb_highest_speed: int
    cable_active_element: int
    cable_active_usb4: bool
    cable_active_usb2p0: bool
    cable_active_usb3p2: bool
    cable_active_usb_lanes: bool
    cable_active_optically_isolated: bool
    cable_active_usb4_asym: bool
    cable_active_usb_gen: bool
    request_epr_mode_capable: bool
    request_pdo_id: int
    request_usb_communications_capable: bool
    request_capability_mismatch: bool
    sink_capabilities: int
    sink_cap_pdo_count: int
    status_temperature: int
    cable_vid: int
    cable_pid: int
    manufacturer_vid: int
    manufacturer_pid: int
    sink_minimum_pdp: int
    sink_operational_pdp: int
    sink_maximum_pdp: int
    operating_current: int
    pd_revision: int
    pps_charging_supported: bool
    has_battery: bool
    dual_role_power: bool
    has_emarker: bool
    operating_voltage: int
    request_ppsavs: bool
    cable_xid: int
    bcd_device: int
    def __init__(self, battery_vid: _Optional[int] = ..., battery_pid: _Optional[int] = ..., battery_design_capacity: _Optional[int] = ..., battery_last_full_charge_capacity: _Optional[int] = ..., battery_present_capacity: _Optional[int] = ..., battery_invalid: bool = ..., battery_present: bool = ..., battery_status: _Optional[int] = ..., cable_is_active: bool = ..., cable_termination_type: bool = ..., cable_epr_mode_capable: bool = ..., cable_active_phy_type: _Optional[int] = ..., cable_latency: _Optional[int] = ..., cable_max_vbus_voltage: _Optional[int] = ..., cable_max_vbus_current: _Optional[int] = ..., cable_usb_highest_speed: _Optional[int] = ..., cable_active_element: _Optional[int] = ..., cable_active_usb4: bool = ..., cable_active_usb2p0: bool = ..., cable_active_usb3p2: bool = ..., cable_active_usb_lanes: bool = ..., cable_active_optically_isolated: bool = ..., cable_active_usb4_asym: bool = ..., cable_active_usb_gen: bool = ..., request_epr_mode_capable: bool = ..., request_pdo_id: _Optional[int] = ..., request_usb_communications_capable: bool = ..., request_capability_mismatch: bool = ..., sink_capabilities: _Optional[int] = ..., sink_cap_pdo_count: _Optional[int] = ..., status_temperature: _Optional[int] = ..., cable_vid: _Optional[int] = ..., cable_pid: _Optional[int] = ..., manufacturer_vid: _Optional[int] = ..., manufacturer_pid: _Optional[int] = ..., sink_minimum_pdp: _Optional[int] = ..., sink_operational_pdp: _Optional[int] = ..., sink_maximum_pdp: _Optional[int] = ..., operating_current: _Optional[int] = ..., pd_revision: _Optional[int] = ..., pps_charging_supported: bool = ..., has_battery: bool = ..., dual_role_power: bool = ..., has_emarker: bool = ..., operating_voltage: _Optional[int] = ..., request_ppsavs: bool = ..., cable_xid: _Optional[int] = ..., bcd_device: _Optional[int] = ...) -> None: ...

class DisplayConfiguration(_message.Message):
    __slots__ = ("intensity", "mode", "flip", "mode_param", "rotation", "idle_animation")
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    MODE_FIELD_NUMBER: _ClassVar[int]
    FLIP_FIELD_NUMBER: _ClassVar[int]
    MODE_PARAM_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    IDLE_ANIMATION_FIELD_NUMBER: _ClassVar[int]
    intensity: int
    mode: DisplayMode
    flip: DisplayFlip
    mode_param: int
    rotation: DisplayRotation
    idle_animation: IdleAnimationType
    def __init__(self, intensity: _Optional[int] = ..., mode: _Optional[_Union[DisplayMode, str]] = ..., flip: _Optional[_Union[DisplayFlip, str]] = ..., mode_param: _Optional[int] = ..., rotation: _Optional[_Union[DisplayRotation, str]] = ..., idle_animation: _Optional[_Union[IdleAnimationType, str]] = ...) -> None: ...

class DisplayConfig(_message.Message):
    __slots__ = ("mode", "intensity", "flip", "rotation", "idle_animation")
    MODE_FIELD_NUMBER: _ClassVar[int]
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    FLIP_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    IDLE_ANIMATION_FIELD_NUMBER: _ClassVar[int]
    mode: DisplayMode
    intensity: int
    flip: DisplayFlip
    rotation: DisplayRotation
    idle_animation: IdleAnimationType
    def __init__(self, mode: _Optional[_Union[DisplayMode, str]] = ..., intensity: _Optional[int] = ..., flip: _Optional[_Union[DisplayFlip, str]] = ..., rotation: _Optional[_Union[DisplayRotation, str]] = ..., idle_animation: _Optional[_Union[IdleAnimationType, str]] = ...) -> None: ...

class DisplaySetup(_message.Message):
    __slots__ = ("intensity", "rotation", "idle_animation")
    INTENSITY_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    IDLE_ANIMATION_FIELD_NUMBER: _ClassVar[int]
    intensity: int
    rotation: DisplayRotation
    idle_animation: IdleAnimationType
    def __init__(self, intensity: _Optional[int] = ..., rotation: _Optional[_Union[DisplayRotation, str]] = ..., idle_animation: _Optional[_Union[IdleAnimationType, str]] = ...) -> None: ...

class PortDetails(_message.Message):
    __slots__ = ("session_charge", "session_id", "manufacturer_vid", "manufacturer_pid", "battery_vid", "battery_pid", "iout_value", "vout_value", "die_temperature", "vin_value", "connected", "fc_protocol", "power_budget")
    SESSION_CHARGE_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_VID_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_PID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_VID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PID_FIELD_NUMBER: _ClassVar[int]
    IOUT_VALUE_FIELD_NUMBER: _ClassVar[int]
    VOUT_VALUE_FIELD_NUMBER: _ClassVar[int]
    DIE_TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    VIN_VALUE_FIELD_NUMBER: _ClassVar[int]
    CONNECTED_FIELD_NUMBER: _ClassVar[int]
    FC_PROTOCOL_FIELD_NUMBER: _ClassVar[int]
    POWER_BUDGET_FIELD_NUMBER: _ClassVar[int]
    session_charge: int
    session_id: int
    manufacturer_vid: int
    manufacturer_pid: int
    battery_vid: int
    battery_pid: int
    iout_value: int
    vout_value: int
    die_temperature: int
    vin_value: int
    connected: bool
    fc_protocol: FastChargingProtocol
    power_budget: int
    def __init__(self, session_charge: _Optional[int] = ..., session_id: _Optional[int] = ..., manufacturer_vid: _Optional[int] = ..., manufacturer_pid: _Optional[int] = ..., battery_vid: _Optional[int] = ..., battery_pid: _Optional[int] = ..., iout_value: _Optional[int] = ..., vout_value: _Optional[int] = ..., die_temperature: _Optional[int] = ..., vin_value: _Optional[int] = ..., connected: bool = ..., fc_protocol: _Optional[_Union[FastChargingProtocol, str]] = ..., power_budget: _Optional[int] = ...) -> None: ...

class MachineConfig(_message.Message):
    __slots__ = ("timestamp", "ports", "charging", "temperature_mode", "display")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    PORTS_FIELD_NUMBER: _ClassVar[int]
    CHARGING_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_MODE_FIELD_NUMBER: _ClassVar[int]
    DISPLAY_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    ports: _containers.RepeatedCompositeFieldContainer[MachinePortConfig]
    charging: ChargingStrategyData
    temperature_mode: TemperatureMode
    display: DisplayConfig
    def __init__(self, timestamp: _Optional[int] = ..., ports: _Optional[_Iterable[_Union[MachinePortConfig, _Mapping]]] = ..., charging: _Optional[_Union[ChargingStrategyData, _Mapping]] = ..., temperature_mode: _Optional[_Union[TemperatureMode, str]] = ..., display: _Optional[_Union[DisplayConfig, _Mapping]] = ...) -> None: ...

class MachinePortConfig(_message.Message):
    __slots__ = ("port_id", "config")
    PORT_ID_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    port_id: int
    config: PortConfigData
    def __init__(self, port_id: _Optional[int] = ..., config: _Optional[_Union[PortConfigData, _Mapping]] = ...) -> None: ...

class PortConfigData(_message.Message):
    __slots__ = ("enabled", "features", "custom_pdo_voltage_mv")
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    FEATURES_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_PDO_VOLTAGE_MV_FIELD_NUMBER: _ClassVar[int]
    enabled: bool
    features: PowerFeatures
    custom_pdo_voltage_mv: int
    def __init__(self, enabled: bool = ..., features: _Optional[_Union[PowerFeatures, _Mapping]] = ..., custom_pdo_voltage_mv: _Optional[int] = ...) -> None: ...

class PowerAllocationEntry(_message.Message):
    __slots__ = ("power_allocation",)
    POWER_ALLOCATION_FIELD_NUMBER: _ClassVar[int]
    power_allocation: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, power_allocation: _Optional[_Iterable[int]] = ...) -> None: ...

class PowerAllocationTable(_message.Message):
    __slots__ = ("version", "entries")
    VERSION_FIELD_NUMBER: _ClassVar[int]
    ENTRIES_FIELD_NUMBER: _ClassVar[int]
    version: int
    entries: _containers.RepeatedCompositeFieldContainer[PowerAllocationEntry]
    def __init__(self, version: _Optional[int] = ..., entries: _Optional[_Iterable[_Union[PowerAllocationEntry, _Mapping]]] = ...) -> None: ...

class ChargingStrategyData(_message.Message):
    __slots__ = ("strategy_type", "identifier", "static_table")
    STRATEGY_TYPE_FIELD_NUMBER: _ClassVar[int]
    IDENTIFIER_FIELD_NUMBER: _ClassVar[int]
    STATIC_TABLE_FIELD_NUMBER: _ClassVar[int]
    strategy_type: int
    identifier: int
    static_table: PowerAllocationTable
    def __init__(self, strategy_type: _Optional[int] = ..., identifier: _Optional[int] = ..., static_table: _Optional[_Union[PowerAllocationTable, _Mapping]] = ...) -> None: ...

class PortConfig(_message.Message):
    __slots__ = ("version", "features")
    VERSION_FIELD_NUMBER: _ClassVar[int]
    FEATURES_FIELD_NUMBER: _ClassVar[int]
    version: int
    features: PowerFeatures
    def __init__(self, version: _Optional[int] = ..., features: _Optional[_Union[PowerFeatures, _Mapping]] = ...) -> None: ...

class CompatibilitySettings(_message.Message):
    __slots__ = ("enable_tfcp", "enable_fcp", "enable_ufcs", "enable_hv_scp", "enable_lv_scp")
    ENABLE_TFCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_FCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_UFCS_FIELD_NUMBER: _ClassVar[int]
    ENABLE_HV_SCP_FIELD_NUMBER: _ClassVar[int]
    ENABLE_LV_SCP_FIELD_NUMBER: _ClassVar[int]
    enable_tfcp: bool
    enable_fcp: bool
    enable_ufcs: bool
    enable_hv_scp: bool
    enable_lv_scp: bool
    def __init__(self, enable_tfcp: bool = ..., enable_fcp: bool = ..., enable_ufcs: bool = ..., enable_hv_scp: bool = ..., enable_lv_scp: bool = ...) -> None: ...

class PowerHistoricalStats(_message.Message):
    __slots__ = ("current", "voltage")
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    current: int
    voltage: int
    def __init__(self, current: _Optional[int] = ..., voltage: _Optional[int] = ...) -> None: ...

class PowerStatistics(_message.Message):
    __slots__ = ("charging_protocol_version", "current", "voltage", "temperature", "battery_last_full_charge_capacity", "battery_present_capacity")
    CHARGING_PROTOCOL_VERSION_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    BATTERY_LAST_FULL_CHARGE_CAPACITY_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PRESENT_CAPACITY_FIELD_NUMBER: _ClassVar[int]
    charging_protocol_version: int
    current: int
    voltage: int
    temperature: int
    battery_last_full_charge_capacity: int
    battery_present_capacity: int
    def __init__(self, charging_protocol_version: _Optional[int] = ..., current: _Optional[int] = ..., voltage: _Optional[int] = ..., temperature: _Optional[int] = ..., battery_last_full_charge_capacity: _Optional[int] = ..., battery_present_capacity: _Optional[int] = ...) -> None: ...

class SSIDInfo(_message.Message):
    __slots__ = ("ssid", "rssi", "auth_mode", "saved")
    SSID_FIELD_NUMBER: _ClassVar[int]
    RSSI_FIELD_NUMBER: _ClassVar[int]
    AUTH_MODE_FIELD_NUMBER: _ClassVar[int]
    SAVED_FIELD_NUMBER: _ClassVar[int]
    ssid: str
    rssi: int
    auth_mode: WiFiAuthMode
    saved: bool
    def __init__(self, ssid: _Optional[str] = ..., rssi: _Optional[int] = ..., auth_mode: _Optional[_Union[WiFiAuthMode, str]] = ..., saved: bool = ...) -> None: ...

class WiFiRecord(_message.Message):
    __slots__ = ("ssid", "auth_mode")
    SSID_FIELD_NUMBER: _ClassVar[int]
    AUTH_MODE_FIELD_NUMBER: _ClassVar[int]
    ssid: str
    auth_mode: WiFiAuthMode
    def __init__(self, ssid: _Optional[str] = ..., auth_mode: _Optional[_Union[WiFiAuthMode, str]] = ...) -> None: ...

class PortStatusData(_message.Message):
    __slots__ = ("charging_minutes", "features", "details", "port_type", "custom_pdo_voltage_mv", "cable_compensation_enable", "cable_resistance", "cable_voltage_offset")
    CHARGING_MINUTES_FIELD_NUMBER: _ClassVar[int]
    FEATURES_FIELD_NUMBER: _ClassVar[int]
    DETAILS_FIELD_NUMBER: _ClassVar[int]
    PORT_TYPE_FIELD_NUMBER: _ClassVar[int]
    CUSTOM_PDO_VOLTAGE_MV_FIELD_NUMBER: _ClassVar[int]
    CABLE_COMPENSATION_ENABLE_FIELD_NUMBER: _ClassVar[int]
    CABLE_RESISTANCE_FIELD_NUMBER: _ClassVar[int]
    CABLE_VOLTAGE_OFFSET_FIELD_NUMBER: _ClassVar[int]
    charging_minutes: int
    features: PowerFeatures
    details: PortDetails
    port_type: PortType
    custom_pdo_voltage_mv: int
    cable_compensation_enable: bool
    cable_resistance: CableCompensation
    cable_voltage_offset: FixedVoltageOffset
    def __init__(self, charging_minutes: _Optional[int] = ..., features: _Optional[_Union[PowerFeatures, _Mapping]] = ..., details: _Optional[_Union[PortDetails, _Mapping]] = ..., port_type: _Optional[_Union[PortType, str]] = ..., custom_pdo_voltage_mv: _Optional[int] = ..., cable_compensation_enable: bool = ..., cable_resistance: _Optional[_Union[CableCompensation, str]] = ..., cable_voltage_offset: _Optional[_Union[FixedVoltageOffset, str]] = ...) -> None: ...

class FeatureFlag(_message.Message):
    __slots__ = ("device_power_on", "ble_on", "syslog_on")
    DEVICE_POWER_ON_FIELD_NUMBER: _ClassVar[int]
    BLE_ON_FIELD_NUMBER: _ClassVar[int]
    SYSLOG_ON_FIELD_NUMBER: _ClassVar[int]
    device_power_on: bool
    ble_on: bool
    syslog_on: bool
    def __init__(self, device_power_on: bool = ..., ble_on: bool = ..., syslog_on: bool = ...) -> None: ...

class DeviceInfo(_message.Message):
    __slots__ = ("psn", "model", "ap_version", "bp_version", "fpga_version", "zrlib_version", "ble_mac", "wifi_mac", "bssid", "ssid", "rssi", "wifi_protocol", "channel")
    PSN_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    AP_VERSION_FIELD_NUMBER: _ClassVar[int]
    BP_VERSION_FIELD_NUMBER: _ClassVar[int]
    FPGA_VERSION_FIELD_NUMBER: _ClassVar[int]
    ZRLIB_VERSION_FIELD_NUMBER: _ClassVar[int]
    BLE_MAC_FIELD_NUMBER: _ClassVar[int]
    WIFI_MAC_FIELD_NUMBER: _ClassVar[int]
    BSSID_FIELD_NUMBER: _ClassVar[int]
    SSID_FIELD_NUMBER: _ClassVar[int]
    RSSI_FIELD_NUMBER: _ClassVar[int]
    WIFI_PROTOCOL_FIELD_NUMBER: _ClassVar[int]
    CHANNEL_FIELD_NUMBER: _ClassVar[int]
    psn: str
    model: str
    ap_version: Version
    bp_version: Version
    fpga_version: Version
    zrlib_version: Version
    ble_mac: str
    wifi_mac: str
    bssid: str
    ssid: str
    rssi: int
    wifi_protocol: int
    channel: int
    def __init__(self, psn: _Optional[str] = ..., model: _Optional[str] = ..., ap_version: _Optional[_Union[Version, _Mapping]] = ..., bp_version: _Optional[_Union[Version, _Mapping]] = ..., fpga_version: _Optional[_Union[Version, _Mapping]] = ..., zrlib_version: _Optional[_Union[Version, _Mapping]] = ..., ble_mac: _Optional[str] = ..., wifi_mac: _Optional[str] = ..., bssid: _Optional[str] = ..., ssid: _Optional[str] = ..., rssi: _Optional[int] = ..., wifi_protocol: _Optional[int] = ..., channel: _Optional[int] = ...) -> None: ...

class DeviceBootInfo(_message.Message):
    __slots__ = ("esp32_version", "sw3566_version", "fpga_version", "zrlib_version", "esp_idf_version", "chip_cores", "chip_model", "chip_features", "chip_revision", "ble_mac", "ipv4", "ipv6", "uptime", "wifi_max_tx_power", "active_mcu_count", "fs_total_size", "fs_used_size", "reset_reason", "hardware_revision", "device_model", "product_family")
    ESP32_VERSION_FIELD_NUMBER: _ClassVar[int]
    SW3566_VERSION_FIELD_NUMBER: _ClassVar[int]
    FPGA_VERSION_FIELD_NUMBER: _ClassVar[int]
    ZRLIB_VERSION_FIELD_NUMBER: _ClassVar[int]
    ESP_IDF_VERSION_FIELD_NUMBER: _ClassVar[int]
    CHIP_CORES_FIELD_NUMBER: _ClassVar[int]
    CHIP_MODEL_FIELD_NUMBER: _ClassVar[int]
    CHIP_FEATURES_FIELD_NUMBER: _ClassVar[int]
    CHIP_REVISION_FIELD_NUMBER: _ClassVar[int]
    BLE_MAC_FIELD_NUMBER: _ClassVar[int]
    IPV4_FIELD_NUMBER: _ClassVar[int]
    IPV6_FIELD_NUMBER: _ClassVar[int]
    UPTIME_FIELD_NUMBER: _ClassVar[int]
    WIFI_MAX_TX_POWER_FIELD_NUMBER: _ClassVar[int]
    ACTIVE_MCU_COUNT_FIELD_NUMBER: _ClassVar[int]
    FS_TOTAL_SIZE_FIELD_NUMBER: _ClassVar[int]
    FS_USED_SIZE_FIELD_NUMBER: _ClassVar[int]
    RESET_REASON_FIELD_NUMBER: _ClassVar[int]
    HARDWARE_REVISION_FIELD_NUMBER: _ClassVar[int]
    DEVICE_MODEL_FIELD_NUMBER: _ClassVar[int]
    PRODUCT_FAMILY_FIELD_NUMBER: _ClassVar[int]
    esp32_version: Version
    sw3566_version: Version
    fpga_version: Version
    zrlib_version: Version
    esp_idf_version: Version
    chip_cores: int
    chip_model: int
    chip_features: int
    chip_revision: int
    ble_mac: bytes
    ipv4: bytes
    ipv6: bytes
    uptime: int
    wifi_max_tx_power: int
    active_mcu_count: int
    fs_total_size: int
    fs_used_size: int
    reset_reason: int
    hardware_revision: str
    device_model: str
    product_family: str
    def __init__(self, esp32_version: _Optional[_Union[Version, _Mapping]] = ..., sw3566_version: _Optional[_Union[Version, _Mapping]] = ..., fpga_version: _Optional[_Union[Version, _Mapping]] = ..., zrlib_version: _Optional[_Union[Version, _Mapping]] = ..., esp_idf_version: _Optional[_Union[Version, _Mapping]] = ..., chip_cores: _Optional[int] = ..., chip_model: _Optional[int] = ..., chip_features: _Optional[int] = ..., chip_revision: _Optional[int] = ..., ble_mac: _Optional[bytes] = ..., ipv4: _Optional[bytes] = ..., ipv6: _Optional[bytes] = ..., uptime: _Optional[int] = ..., wifi_max_tx_power: _Optional[int] = ..., active_mcu_count: _Optional[int] = ..., fs_total_size: _Optional[int] = ..., fs_used_size: _Optional[int] = ..., reset_reason: _Optional[int] = ..., hardware_revision: _Optional[str] = ..., device_model: _Optional[str] = ..., product_family: _Optional[str] = ...) -> None: ...

class DeviceStatsData(_message.Message):
    __slots__ = ("initial_boot", "free_heap_size", "free_internal_heap_size", "minimum_free_heap_size", "wifi_association_count", "wifi_disassociation_count", "wifi_rssi", "wifi_channel", "wifi_connection_time", "mqtt_connection_time", "connectivity_failure_count", "dns_resolution_failure_count", "mqtt_connection_count", "mqtt_message_tx_count", "mqtt_message_rx_count", "temperature")
    INITIAL_BOOT_FIELD_NUMBER: _ClassVar[int]
    FREE_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    FREE_INTERNAL_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    MINIMUM_FREE_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    WIFI_ASSOCIATION_COUNT_FIELD_NUMBER: _ClassVar[int]
    WIFI_DISASSOCIATION_COUNT_FIELD_NUMBER: _ClassVar[int]
    WIFI_RSSI_FIELD_NUMBER: _ClassVar[int]
    WIFI_CHANNEL_FIELD_NUMBER: _ClassVar[int]
    WIFI_CONNECTION_TIME_FIELD_NUMBER: _ClassVar[int]
    MQTT_CONNECTION_TIME_FIELD_NUMBER: _ClassVar[int]
    CONNECTIVITY_FAILURE_COUNT_FIELD_NUMBER: _ClassVar[int]
    DNS_RESOLUTION_FAILURE_COUNT_FIELD_NUMBER: _ClassVar[int]
    MQTT_CONNECTION_COUNT_FIELD_NUMBER: _ClassVar[int]
    MQTT_MESSAGE_TX_COUNT_FIELD_NUMBER: _ClassVar[int]
    MQTT_MESSAGE_RX_COUNT_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    initial_boot: bool
    free_heap_size: int
    free_internal_heap_size: int
    minimum_free_heap_size: int
    wifi_association_count: int
    wifi_disassociation_count: int
    wifi_rssi: int
    wifi_channel: int
    wifi_connection_time: int
    mqtt_connection_time: int
    connectivity_failure_count: int
    dns_resolution_failure_count: int
    mqtt_connection_count: int
    mqtt_message_tx_count: int
    mqtt_message_rx_count: int
    temperature: int
    def __init__(self, initial_boot: bool = ..., free_heap_size: _Optional[int] = ..., free_internal_heap_size: _Optional[int] = ..., minimum_free_heap_size: _Optional[int] = ..., wifi_association_count: _Optional[int] = ..., wifi_disassociation_count: _Optional[int] = ..., wifi_rssi: _Optional[int] = ..., wifi_channel: _Optional[int] = ..., wifi_connection_time: _Optional[int] = ..., mqtt_connection_time: _Optional[int] = ..., connectivity_failure_count: _Optional[int] = ..., dns_resolution_failure_count: _Optional[int] = ..., mqtt_connection_count: _Optional[int] = ..., mqtt_message_tx_count: _Optional[int] = ..., mqtt_message_rx_count: _Optional[int] = ..., temperature: _Optional[int] = ...) -> None: ...

class PortStatsData(_message.Message):
    __slots__ = ("voltage", "amperage", "temperature", "vin_value", "session_id", "session_charge", "manufacturer_vid", "manufacturer_pid", "battery_vid", "battery_pid")
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    AMPERAGE_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    VIN_VALUE_FIELD_NUMBER: _ClassVar[int]
    SESSION_ID_FIELD_NUMBER: _ClassVar[int]
    SESSION_CHARGE_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_VID_FIELD_NUMBER: _ClassVar[int]
    MANUFACTURER_PID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_VID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PID_FIELD_NUMBER: _ClassVar[int]
    voltage: int
    amperage: int
    temperature: int
    vin_value: int
    session_id: int
    session_charge: int
    manufacturer_vid: int
    manufacturer_pid: int
    battery_vid: int
    battery_pid: int
    def __init__(self, voltage: _Optional[int] = ..., amperage: _Optional[int] = ..., temperature: _Optional[int] = ..., vin_value: _Optional[int] = ..., session_id: _Optional[int] = ..., session_charge: _Optional[int] = ..., manufacturer_vid: _Optional[int] = ..., manufacturer_pid: _Optional[int] = ..., battery_vid: _Optional[int] = ..., battery_pid: _Optional[int] = ...) -> None: ...

class PortStatsEntry(_message.Message):
    __slots__ = ("port", "stats")
    PORT_FIELD_NUMBER: _ClassVar[int]
    STATS_FIELD_NUMBER: _ClassVar[int]
    port: int
    stats: _containers.RepeatedCompositeFieldContainer[PortStatsData]
    def __init__(self, port: _Optional[int] = ..., stats: _Optional[_Iterable[_Union[PortStatsData, _Mapping]]] = ...) -> None: ...

class PowerHistoricalData(_message.Message):
    __slots__ = ("data",)
    DATA_FIELD_NUMBER: _ClassVar[int]
    data: _containers.RepeatedCompositeFieldContainer[PortStatsEntry]
    def __init__(self, data: _Optional[_Iterable[_Union[PortStatsEntry, _Mapping]]] = ...) -> None: ...

class PowerAllocationData(_message.Message):
    __slots__ = ("power_budget", "remaining_power", "adc_value", "temperature", "port_allocations")
    class PortAllocation(_message.Message):
        __slots__ = ("source_cap", "usage")
        SOURCE_CAP_FIELD_NUMBER: _ClassVar[int]
        USAGE_FIELD_NUMBER: _ClassVar[int]
        source_cap: int
        usage: int
        def __init__(self, source_cap: _Optional[int] = ..., usage: _Optional[int] = ...) -> None: ...
    POWER_BUDGET_FIELD_NUMBER: _ClassVar[int]
    REMAINING_POWER_FIELD_NUMBER: _ClassVar[int]
    ADC_VALUE_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    PORT_ALLOCATIONS_FIELD_NUMBER: _ClassVar[int]
    power_budget: int
    remaining_power: int
    adc_value: int
    temperature: int
    port_allocations: _containers.RepeatedCompositeFieldContainer[PowerAllocationData.PortAllocation]
    def __init__(self, power_budget: _Optional[int] = ..., remaining_power: _Optional[int] = ..., adc_value: _Optional[int] = ..., temperature: _Optional[int] = ..., port_allocations: _Optional[_Iterable[_Union[PowerAllocationData.PortAllocation, _Mapping]]] = ...) -> None: ...

class PDPCapData(_message.Message):
    __slots__ = ("port", "data")
    PORT_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    port: int
    data: bytes
    def __init__(self, port: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class ProgrammerStart(_message.Message):
    __slots__ = ("programmer_version",)
    PROGRAMMER_VERSION_FIELD_NUMBER: _ClassVar[int]
    programmer_version: Version
    def __init__(self, programmer_version: _Optional[_Union[Version, _Mapping]] = ...) -> None: ...

class ProgrammerFinish(_message.Message):
    __slots__ = ("success", "error_reason")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    ERROR_REASON_FIELD_NUMBER: _ClassVar[int]
    success: bool
    error_reason: ProgrammerState
    def __init__(self, success: bool = ..., error_reason: _Optional[_Union[ProgrammerState, str]] = ...) -> None: ...

class UpgradeData(_message.Message):
    __slots__ = ("esp32_version", "mcu_version", "fpga_version", "status", "reason", "esp_err", "active_port_count")
    ESP32_VERSION_FIELD_NUMBER: _ClassVar[int]
    MCU_VERSION_FIELD_NUMBER: _ClassVar[int]
    FPGA_VERSION_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    REASON_FIELD_NUMBER: _ClassVar[int]
    ESP_ERR_FIELD_NUMBER: _ClassVar[int]
    ACTIVE_PORT_COUNT_FIELD_NUMBER: _ClassVar[int]
    esp32_version: Version
    mcu_version: Version
    fpga_version: Version
    status: UpgradeStatus
    reason: UpgradeFailureReason
    esp_err: int
    active_port_count: int
    def __init__(self, esp32_version: _Optional[_Union[Version, _Mapping]] = ..., mcu_version: _Optional[_Union[Version, _Mapping]] = ..., fpga_version: _Optional[_Union[Version, _Mapping]] = ..., status: _Optional[_Union[UpgradeStatus, str]] = ..., reason: _Optional[_Union[UpgradeFailureReason, str]] = ..., esp_err: _Optional[int] = ..., active_port_count: _Optional[int] = ...) -> None: ...

class UartMetricsData(_message.Message):
    __slots__ = ("reset_state_count", "resend_count", "sent_failed_count", "sent_count")
    RESET_STATE_COUNT_FIELD_NUMBER: _ClassVar[int]
    RESEND_COUNT_FIELD_NUMBER: _ClassVar[int]
    SENT_FAILED_COUNT_FIELD_NUMBER: _ClassVar[int]
    SENT_COUNT_FIELD_NUMBER: _ClassVar[int]
    reset_state_count: int
    resend_count: int
    sent_failed_count: int
    sent_count: int
    def __init__(self, reset_state_count: _Optional[int] = ..., resend_count: _Optional[int] = ..., sent_failed_count: _Optional[int] = ..., sent_count: _Optional[int] = ...) -> None: ...

class PingInfo(_message.Message):
    __slots__ = ("dummy",)
    DUMMY_FIELD_NUMBER: _ClassVar[int]
    dummy: int
    def __init__(self, dummy: _Optional[int] = ...) -> None: ...

class SW3566BootloaderInfo(_message.Message):
    __slots__ = ("infos",)
    class BootloaderInfo(_message.Message):
        __slots__ = ("version", "flash_timestamp")
        VERSION_FIELD_NUMBER: _ClassVar[int]
        FLASH_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
        version: int
        flash_timestamp: int
        def __init__(self, version: _Optional[int] = ..., flash_timestamp: _Optional[int] = ...) -> None: ...
    INFOS_FIELD_NUMBER: _ClassVar[int]
    infos: _containers.RepeatedCompositeFieldContainer[SW3566BootloaderInfo.BootloaderInfo]
    def __init__(self, infos: _Optional[_Iterable[_Union[SW3566BootloaderInfo.BootloaderInfo, _Mapping]]] = ...) -> None: ...

class OutOfMemoryAlertInfo(_message.Message):
    __slots__ = ("free_heap_size", "free_internal_heap_size", "minimum_free_heap_size", "free_heap_size_threshold")
    FREE_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    FREE_INTERNAL_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    MINIMUM_FREE_HEAP_SIZE_FIELD_NUMBER: _ClassVar[int]
    FREE_HEAP_SIZE_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    free_heap_size: int
    free_internal_heap_size: int
    minimum_free_heap_size: int
    free_heap_size_threshold: int
    def __init__(self, free_heap_size: _Optional[int] = ..., free_internal_heap_size: _Optional[int] = ..., minimum_free_heap_size: _Optional[int] = ..., free_heap_size_threshold: _Optional[int] = ...) -> None: ...

class OverTemperatureAlertInfo(_message.Message):
    __slots__ = ("temperature", "threshold_temperature")
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    THRESHOLD_TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    temperature: int
    threshold_temperature: int
    def __init__(self, temperature: _Optional[int] = ..., threshold_temperature: _Optional[int] = ...) -> None: ...

class ChargingAlertInfo(_message.Message):
    __slots__ = ("port", "pd_rx_soft_reset_count", "pd_rx_hard_reset_count", "pd_rx_error_count", "pd_rx_cable_reset_count")
    PORT_FIELD_NUMBER: _ClassVar[int]
    PD_RX_SOFT_RESET_COUNT_FIELD_NUMBER: _ClassVar[int]
    PD_RX_HARD_RESET_COUNT_FIELD_NUMBER: _ClassVar[int]
    PD_RX_ERROR_COUNT_FIELD_NUMBER: _ClassVar[int]
    PD_RX_CABLE_RESET_COUNT_FIELD_NUMBER: _ClassVar[int]
    port: int
    pd_rx_soft_reset_count: int
    pd_rx_hard_reset_count: int
    pd_rx_error_count: int
    pd_rx_cable_reset_count: int
    def __init__(self, port: _Optional[int] = ..., pd_rx_soft_reset_count: _Optional[int] = ..., pd_rx_hard_reset_count: _Optional[int] = ..., pd_rx_error_count: _Optional[int] = ..., pd_rx_cable_reset_count: _Optional[int] = ...) -> None: ...

class ACDCSelfTestInfo(_message.Message):
    __slots__ = ("status",)
    STATUS_FIELD_NUMBER: _ClassVar[int]
    status: ACDCSelfTestStatus
    def __init__(self, status: _Optional[_Union[ACDCSelfTestStatus, str]] = ...) -> None: ...

class TaskExecutionResult(_message.Message):
    __slots__ = ("task_id", "status")
    TASK_ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    task_id: int
    status: TaskExecutionStatus
    def __init__(self, task_id: _Optional[int] = ..., status: _Optional[_Union[TaskExecutionStatus, str]] = ...) -> None: ...

class CoredumpPartitionSlice(_message.Message):
    __slots__ = ("timestamp", "offset", "size", "last", "data")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    OFFSET_FIELD_NUMBER: _ClassVar[int]
    SIZE_FIELD_NUMBER: _ClassVar[int]
    LAST_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    offset: int
    size: int
    last: bool
    data: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, timestamp: _Optional[int] = ..., offset: _Optional[int] = ..., size: _Optional[int] = ..., last: bool = ..., data: _Optional[_Iterable[int]] = ...) -> None: ...

class ConfigReport(_message.Message):
    __slots__ = ("timestamp", "config")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    timestamp: int
    config: MachineConfig
    def __init__(self, timestamp: _Optional[int] = ..., config: _Optional[_Union[MachineConfig, _Mapping]] = ...) -> None: ...

class ConfigRequest(_message.Message):
    __slots__ = ("reason",)
    REASON_FIELD_NUMBER: _ClassVar[int]
    reason: str
    def __init__(self, reason: _Optional[str] = ...) -> None: ...

class FPGAConfig(_message.Message):
    __slots__ = ("adc_threshold_low", "adc_threshold_high", "action_deadzone")
    ADC_THRESHOLD_LOW_FIELD_NUMBER: _ClassVar[int]
    ADC_THRESHOLD_HIGH_FIELD_NUMBER: _ClassVar[int]
    ACTION_DEADZONE_FIELD_NUMBER: _ClassVar[int]
    adc_threshold_low: int
    adc_threshold_high: int
    action_deadzone: int
    def __init__(self, adc_threshold_low: _Optional[int] = ..., adc_threshold_high: _Optional[int] = ..., action_deadzone: _Optional[int] = ...) -> None: ...

class PowerConfig(_message.Message):
    __slots__ = ("version", "power_budget", "device_high_temp_threshold", "device_high_temp_min_power", "device_low_temp_threshold", "device_over_temp_decrement", "device_under_temp_increment", "device_power_cooldown_time", "overprovisioning_threshold", "cap_increase_min_threshold", "cap_decrease_min_threshold", "port_power_cap_increase_threshold", "port_power_cap_increase_step", "port_power_cap_decrease_threshold", "high_power_usage_threshold", "low_power_usage_threshold", "port_power_adjustment_threshold")
    VERSION_FIELD_NUMBER: _ClassVar[int]
    POWER_BUDGET_FIELD_NUMBER: _ClassVar[int]
    DEVICE_HIGH_TEMP_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    DEVICE_HIGH_TEMP_MIN_POWER_FIELD_NUMBER: _ClassVar[int]
    DEVICE_LOW_TEMP_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    DEVICE_OVER_TEMP_DECREMENT_FIELD_NUMBER: _ClassVar[int]
    DEVICE_UNDER_TEMP_INCREMENT_FIELD_NUMBER: _ClassVar[int]
    DEVICE_POWER_COOLDOWN_TIME_FIELD_NUMBER: _ClassVar[int]
    OVERPROVISIONING_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    CAP_INCREASE_MIN_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    CAP_DECREASE_MIN_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    PORT_POWER_CAP_INCREASE_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    PORT_POWER_CAP_INCREASE_STEP_FIELD_NUMBER: _ClassVar[int]
    PORT_POWER_CAP_DECREASE_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    HIGH_POWER_USAGE_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    LOW_POWER_USAGE_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    PORT_POWER_ADJUSTMENT_THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    version: int
    power_budget: int
    device_high_temp_threshold: int
    device_high_temp_min_power: int
    device_low_temp_threshold: int
    device_over_temp_decrement: int
    device_under_temp_increment: int
    device_power_cooldown_time: int
    overprovisioning_threshold: int
    cap_increase_min_threshold: int
    cap_decrease_min_threshold: int
    port_power_cap_increase_threshold: int
    port_power_cap_increase_step: int
    port_power_cap_decrease_threshold: int
    high_power_usage_threshold: int
    low_power_usage_threshold: int
    port_power_adjustment_threshold: int
    def __init__(self, version: _Optional[int] = ..., power_budget: _Optional[int] = ..., device_high_temp_threshold: _Optional[int] = ..., device_high_temp_min_power: _Optional[int] = ..., device_low_temp_threshold: _Optional[int] = ..., device_over_temp_decrement: _Optional[int] = ..., device_under_temp_increment: _Optional[int] = ..., device_power_cooldown_time: _Optional[int] = ..., overprovisioning_threshold: _Optional[int] = ..., cap_increase_min_threshold: _Optional[int] = ..., cap_decrease_min_threshold: _Optional[int] = ..., port_power_cap_increase_threshold: _Optional[int] = ..., port_power_cap_increase_step: _Optional[int] = ..., port_power_cap_decrease_threshold: _Optional[int] = ..., high_power_usage_threshold: _Optional[int] = ..., low_power_usage_threshold: _Optional[int] = ..., port_power_adjustment_threshold: _Optional[int] = ...) -> None: ...

class LicenseData(_message.Message):
    __slots__ = ("f", "b", "a")
    F_FIELD_NUMBER: _ClassVar[int]
    B_FIELD_NUMBER: _ClassVar[int]
    A_FIELD_NUMBER: _ClassVar[int]
    f: _containers.RepeatedScalarFieldContainer[int]
    b: int
    a: int
    def __init__(self, f: _Optional[_Iterable[int]] = ..., b: _Optional[int] = ..., a: _Optional[int] = ...) -> None: ...

class Task(_message.Message):
    __slots__ = ("id", "run_at", "commands")
    class Command(_message.Message):
        __slots__ = ("service", "data")
        SERVICE_FIELD_NUMBER: _ClassVar[int]
        DATA_FIELD_NUMBER: _ClassVar[int]
        service: int
        data: bytes
        def __init__(self, service: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...
    ID_FIELD_NUMBER: _ClassVar[int]
    RUN_AT_FIELD_NUMBER: _ClassVar[int]
    COMMANDS_FIELD_NUMBER: _ClassVar[int]
    id: int
    run_at: int
    commands: _containers.RepeatedCompositeFieldContainer[Task.Command]
    def __init__(self, id: _Optional[int] = ..., run_at: _Optional[int] = ..., commands: _Optional[_Iterable[_Union[Task.Command, _Mapping]]] = ...) -> None: ...
