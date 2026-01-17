#ifndef STRUCT_TO_PB_H_
#define STRUCT_TO_PB_H_

#include <cstdint>
#include <vector>

#include "pb.h"

#define PowerFeaturesFromPB(features, pb_features)                   \
  do {                                                               \
    features.EnableTfcp = pb_features.enable_tfcp;                   \
    features.EnablePe = pb_features.enable_pe;                       \
    features.EnableQc2p0 = pb_features.enable_qc2p0;                 \
    features.EnableQc3p0 = pb_features.enable_qc3p0;                 \
    features.EnableQc3plus = pb_features.enable_qc3plus;             \
    features.EnableAfc = pb_features.enable_afc;                     \
    features.EnableFcp = pb_features.enable_fcp;                     \
    features.EnableHvScp = pb_features.enable_hv_scp;                \
    features.EnableLvScp = pb_features.enable_lv_scp;                \
    features.EnableSfcp = pb_features.enable_sfcp;                   \
    features.EnableApple = pb_features.enable_apple;                 \
    features.EnableSamsung = pb_features.enable_samsung;             \
    features.EnableUfcs = pb_features.enable_ufcs;                   \
    features.EnablePd = pb_features.enable_pd;                       \
    features.EnablePdCompatMode = pb_features.enable_pd_compat_mode; \
    features.LimitedCurrentMode = pb_features.limited_current_mode;  \
    features.EnablePdLVPPS = pb_features.enable_pd_lvpps;            \
    features.EnablePdEPR = pb_features.enable_pd_epr;                \
    features.EnablePd5V5A = pb_features.enable_pd_rpi;               \
    features.EnablePdHVPPS = pb_features.enable_pd_hvpps;            \
    features.EnablePdHardResetOnRefusal =                            \
        pb_features.enable_pd_hard_reset_on_refusal;                 \
    features.EnablePdSPRAVS = pb_features.enable_pd_spr_avs;         \
  } while (0)

#define PowerFeaturesToPB(pb_features, features)                     \
  do {                                                               \
    pb_features.enable_tfcp = features.EnableTfcp;                   \
    pb_features.enable_pe = features.EnablePe;                       \
    pb_features.enable_qc2p0 = features.EnableQc2p0;                 \
    pb_features.enable_qc3p0 = features.EnableQc3p0;                 \
    pb_features.enable_qc3plus = features.EnableQc3plus;             \
    pb_features.enable_afc = features.EnableAfc;                     \
    pb_features.enable_fcp = features.EnableFcp;                     \
    pb_features.enable_hv_scp = features.EnableHvScp;                \
    pb_features.enable_lv_scp = features.EnableLvScp;                \
    pb_features.enable_sfcp = features.EnableSfcp;                   \
    pb_features.enable_apple = features.EnableApple;                 \
    pb_features.enable_samsung = features.EnableSamsung;             \
    pb_features.enable_ufcs = features.EnableUfcs;                   \
    pb_features.enable_pd = features.EnablePd;                       \
    pb_features.enable_pd_compat_mode = features.EnablePdCompatMode; \
    pb_features.limited_current_mode = features.LimitedCurrentMode;  \
    pb_features.enable_pd_lvpps = features.EnablePdLVPPS;            \
    pb_features.enable_pd_epr = features.EnablePdEPR;                \
    pb_features.enable_pd_rpi = features.EnablePd5V5A;               \
    pb_features.enable_pd_hvpps = features.EnablePdHVPPS;            \
    pb_features.enable_pd_hard_reset_on_refusal =                    \
        features.EnablePdHardResetOnRefusal;                         \
    pb_features.enable_pd_spr_avs = features.EnablePdSPRAVS;         \
  } while (0)

#define ClientPDStatusToPB(pb_status, status)                             \
  do {                                                                    \
    pb_status.battery_vid = status.battery_vid;                           \
    pb_status.battery_pid = status.battery_pid;                           \
    pb_status.battery_design_capacity = status.battery_design_capacity;   \
    pb_status.battery_last_full_charge_capacity =                         \
        status.battery_last_full_charge_capacity;                         \
    pb_status.battery_present_capacity = status.battery_present_capacity; \
    pb_status.battery_invalid = status.battery_invalid;                   \
    pb_status.battery_present = status.battery_present;                   \
    pb_status.battery_status = status.battery_status;                     \
    pb_status.cable_is_active = status.cable_is_active;                   \
    pb_status.cable_termination_type = status.cable_termination_type;     \
    pb_status.cable_epr_mode_capable = status.cable_epr_mode_capable;     \
    pb_status.cable_active_phy_type = status.cable_active_phy_type;       \
    pb_status.cable_latency = status.cable_latency;                       \
    pb_status.cable_max_vbus_voltage = status.cable_max_vbus_voltage;     \
    pb_status.cable_max_vbus_current = status.cable_max_vbus_current;     \
    pb_status.cable_usb_highest_speed = status.cable_usb_highest_speed;   \
    pb_status.cable_active_element = status.cable_active_element;         \
    pb_status.cable_active_usb4 = status.cable_active_usb4;               \
    pb_status.cable_active_usb2p0 = status.cable_active_usb2p0;           \
    pb_status.cable_active_usb3p2 = status.cable_active_usb3p2;           \
    pb_status.cable_active_usb_lanes = status.cable_active_usb_lanes;     \
    pb_status.cable_active_optically_isolated =                           \
        status.cable_active_optically_isolated;                           \
    pb_status.cable_active_usb4_asym = status.cable_active_usb4_asym;     \
    pb_status.cable_active_usb_gen = status.cable_active_usb_gen;         \
    pb_status.request_epr_mode_capable = status.request_epr_mode_capable; \
    pb_status.request_pdo_id = status.request_pdo_id;                     \
    pb_status.request_usb_communications_capable =                        \
        status.request_usb_communications_capable;                        \
    pb_status.request_capability_mismatch =                               \
        status.request_capability_mismatch;                               \
    pb_status.sink_capabilities = status.sink_capabilities;               \
    pb_status.sink_cap_pdo_count = status.sink_cap_pdo_count;             \
    pb_status.status_temperature = status.status_temperature;             \
    pb_status.cable_vid = status.cable_vid;                               \
    pb_status.cable_pid = status.cable_pid;                               \
    pb_status.manufacturer_vid = status.manufacturer_vid;                 \
    pb_status.manufacturer_pid = status.manufacturer_pid;                 \
    pb_status.sink_minimum_pdp = status.sink_minimum_pdp;                 \
    pb_status.sink_operational_pdp = status.sink_operational_pdp;         \
    pb_status.sink_maximum_pdp = status.sink_maximum_pdp;                 \
    pb_status.operating_current = status.operating_current;               \
    pb_status.pd_revision = status.pd_revision;                           \
    pb_status.pps_charging_supported = status.pps_charging_supported;     \
    pb_status.has_battery = status.has_battery;                           \
    pb_status.dual_role_power = status.dual_role_power;                   \
    pb_status.has_emarker = status.has_emarker;                           \
    pb_status.operating_voltage = status.operating_voltage;               \
    pb_status.request_ppsavs = status.request_ppsavs;                     \
    pb_status.cable_xid = status.cable_xid;                               \
    pb_status.bcd_device = status.bcd_device;                             \
  } while (0)

std::vector<uint8_t> serialize_pb(const pb_msgdesc_t *fields,
                                  const void *src_struct);
#endif /* STRUCT_TO_PB_H_ */
