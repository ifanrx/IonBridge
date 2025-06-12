#ifndef H_DATA_TYPES_
#define H_DATA_TYPES_

#include <cstdint>

/**
 * Fast charge protocol
 * SW356x/MinOS/inc/config_types.h
 * size: 1 byte
 */
typedef enum __attribute__((packed)) {
  FC_None = 0,    //!< 0  none
  FC_QC2,         //!< 1  Qc2
  FC_QC3,         //!< 2  Qc3
  FC_QC3P,        //!< 3  Qc3+
  FC_SFCP,        //!< 4  SFCP
  FC_AFC,         //!< 5  AFC
  FC_FCP,         //!< 6  FCP
  FC_SCP,         //!< 7  SCP
  FC_VOOC1P0,     //!< 8  VOOC1.0
  FC_VOOC4P0,     //!< 9  VOOC4.0
  FC_SVOOC2P0,    //!< 10 SupperVooc2.0
  FC_TFCP,        //!< 11 TFCP
  FC_UFCS,        //!< 12 UFCS
  FC_PE1,         //!< 13 PE1.0
  FC_PE2,         //!< 14 PE2.0
  FC_PD_Fix5V,    //!< 15 PD fix 5v
  FC_PD_FixHV,    //!< 16 PD fix high voltage
  FC_PD_SPR_AVS,  //!< 17 PD SPR AVS
  FC_PD_PPS,      //!< 18 PD PPS
  FC_PD_EPR_HV,   //!< 19 PD EPR high voltage
  FC_PD_AVS,      //!< 20 PD AVS

  FC_NOT_CHARGING = 0xff,  //!< 0xff  not charging
} FastChargingProtocol;

typedef struct __attribute__((packed)) {
  // Table 6.60 “Battery Capability Data Block (BCDB)”
  uint16_t battery_vid;
  uint16_t battery_pid;
  // 0x0000 = Battery not present
  // 0xFFFF = Unknown
  //  Battery’s design capacity in 0.1 WH
  uint16_t battery_design_capacity;
  // Battery’s last full charge capacity in 0.1 WH
  uint16_t battery_last_full_charge_capacity;
  // Table 6.47 “Battery Status Data Object (BSDO)”
  // Battery’s State of Charge (SoC) in 0.1 WH increments
  // 0xFFFF = Battery’s SOC unknown
  uint16_t battery_present_capacity;
  // Invalid Battery reference
  bool battery_invalid : 1;
  // Battery is present when set
  bool battery_present : 1;
  /*
  00b: Battery is Charging.
  01b: Battery is Discharging.
  10b: Battery is Idle.
  */
  uint8_t battery_status : 2;
  // Filled from SOP' (cable) VDM
  // Table 6.34 “ID Header VDO”
  // Table 6.42 “Passive Cable VDO”
  uint8_t cable_is_active : 1;
  // 0 : One end Active, one end passive, VCONN required ; 1 : Both ends Active,
  // VCONN required
  uint8_t cable_termination_type : 1;
  bool cable_epr_mode_capable : 1;
  // Table 6.43 “Active Cable VDO 1”
  /*
  0b = Copper
  1b = Optical
  */
  uint8_t cable_active_phy_type : 1;
  // cable_latency is also the length of the cable
  /*
    0001b – <10ns (~1m)
    0010b – 10ns to 20ns (~2m)
    0011b – 20ns to 30ns (~3m)
    0100b – 30ns to 40ns (~4m)
    0101b – 40ns to 50ns (~5m)
    0110b – 50ns to 60ns (~6m)
    0111b – 60ns to 70ns (~7m)
    1000b – > 70ns (>~7m)
  */
  uint8_t cable_latency : 4;
  /*
  00b – 20V
  01b – 30V1 (Deprecated)
  10b – 40V1 (Deprecated)
  11b – 50V
  */
  uint8_t cable_max_vbus_voltage : 2;
  /*
  01b = 3A
  10b = 5A
  */
  uint8_t cable_max_vbus_current : 2;
  /*
  000b = [USB 2.0] only, no SuperSpeed support
  001b = [USB 3.2] Gen1
  010b = [USB 3.2]/[USB4] Gen2
  011b = [USB4] Gen3
  100b = [USB4] Gen4
  */
  uint8_t cable_usb_highest_speed : 3;
  /*
  0b = Active Redriver
  1b = Active Retimer
  */
  uint8_t cable_active_element : 1;
  bool cable_active_usb4 : 1;
  bool cable_active_usb2p0 : 1;
  bool cable_active_usb3p2 : 1;
  /*
  0b = One lane
  1b = Two lanes
  */
  bool cable_active_usb_lanes : 1;
  // USB4 Asymmetric Mode Supported
  bool cable_active_optically_isolated : 1;
  bool cable_active_usb4_asym : 1;
  // 0b = Gen 1, 1b = Gen 2 or higher
  bool cable_active_usb_gen : 1;
  // PDO ID specified in the latest `Request` packet
  // This is the property of the SINK
  uint8_t request_epr_mode_capable : 1;
  // Which PDO did the SINK request for in the latest request
  uint8_t request_pdo_id : 4;
  // Table 6.23
  // Is the SINK USB capable (computer, tablet, phone etc)
  bool request_usb_communications_capable : 1;
  // Is the SINK rejecting our PD Source Capability
  bool request_capability_mismatch : 1;
  // Table 6.7 “Power Data Object”
  // Table 6.8 “Augmented Power Data Object”
  // B31…28 (if B31..B30 is not 11, B29…28 is 00)
  // All possible values: 0000 0100 1000 1100 1101 1110 1111
  uint8_t sink_capabilities : 4;
  // PDO count must >= 1, hence this count should be incremented by 1.
  // i.e. 0 -> 1 pdo, ..., 3 -> 4 or more than 4 PDO
  uint8_t sink_cap_pdo_count : 2;
  // Table 6.56 “SOP Status Data Block (SDB)”
  uint8_t status_temperature;
  uint16_t cable_vid;
  uint16_t cable_pid;
  // Table 6.61 “Get Manufacturer Info Data Block (GMIDB)”
  uint16_t manufacturer_vid;
  uint16_t manufacturer_pid;
  // Table 6.66 “Sink Capabilities Extended Data Block (SKEDB)”
  uint8_t sink_minimum_pdp;
  uint8_t sink_operational_pdp;
  uint8_t sink_maximum_pdp;
  uint8_t unused0;
  uint16_t operating_current : 10;  // 10 mA
  // 6.2.1.1.5 Specification Revision
  // 0: 1.0, 1: 2.0, 3: 3.0, 4: reserved
  uint8_t pd_revision : 2;
  // Table 6.65: sink mode: supports PPS charging
  bool pps_charging_supported : 1;
  // Table 6.65: any battery exists
  bool has_battery : 1;
  // Table 6.17 Fixed Supply PDO – Sink
  // Port is Dual-Role Power capable
  bool dual_role_power : 1;
  bool has_emarker : 1;
  uint16_t operating_voltage : 15;  // 10 mV
  bool request_ppsavs : 1;
  // Table 6.38 "Cert Stat VDO"
  uint32_t cable_xid;
  // Table 6.39 “Product VDO”
  uint16_t bcd_device;
  uint16_t unused2;
  uint32_t unused3[14];
} ClientPDStatus;

typedef struct __attribute__((packed)) {
  bool connected;
  FastChargingProtocol fc_protocol;
  uint16_t iout_value;
  uint16_t vout_value;
  uint16_t die_temperature;
  uint16_t vin_value;
  uint16_t session_id;
  uint64_t session_charge;
  uint8_t power_budget;
} PortDetails;

typedef struct __attribute__((packed)) {
  // BYTE 1
  bool EnableTfcp : 1;
  bool EnablePe : 1;
  bool EnableQc2p0 : 1;
  bool EnableQc3p0 : 1;
  bool EnableQc3plus : 1;
  bool EnableAfc : 1;
  bool EnableFcp : 1;
  bool EnableHvScp : 1;
  // BYTE 2
  bool EnableLvScp : 1;
  bool EnableSfcp : 1;
  bool EnableApple : 1;
  bool EnableSamsung : 1;
  bool EnableUfcs : 1;
  bool EnablePd : 1;
  bool EnablePdCompatMode : 1;
  bool LimitedCurrentMode : 1;
  // BYTE 3
  bool EnablePdLVPPS : 1;
  bool EnablePdEPR : 1;
  bool EnablePd5V5A : 1;
  bool EnablePdHVPPS : 1;
  uint8_t reserved : 4;
} PowerFeatures;

typedef enum __attribute__((packed)) {
  PORT_TYPE_A = 0x00,
  PORT_TYPE_C = 0x01,
  PORT_TYPE_MAX,
} PortType;

#endif
