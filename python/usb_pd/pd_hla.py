# https://github.com/saleae/hla-usb-pd/blob/master/HighLevelAnalyzer.py
# Feedback:
# could not figure out where to import GraphTime from, had to check our unit tests.
# unit tests or attached debugger would save many hours
# auto-rerun on save would be nice
# auto or manual console clear would be nice
# some way of displaying numbers as hex in the bubbles, besides converting to string.
# sometimes my format strings don't apply at all. reloading the analyzer fixes this.
# can't display quote character in resulting strings, they get html encoded. (nevermind, this just requires and extra set of {} in the format string to display the string as raw.)
# we have way too much data to effectively display in the data table or the graph display, we need to support efficient object display somehow.


from usb_pd.pd_msg import *

# https://www.embedded.com/usb-type-c-and-power-delivery-101-power-delivery-protocol/
# official documentation: https://www.usb.org/document-library/usb-power-delivery


# TODO / Future Features:
# add support for extended headers and data blocks
# decode all vendor data objects. (header is done)
# ID Header VDO, Cert Stat VDO, Product VDO UFP VDO 1/2, DFP VDO, Passive cable VDO, Active Cable VDO 1/2, AMA VDO, VPD VDO,Discover SVIDs Responder VDO,
# verify the CRC


rev_lookup = {
    0b00: "v1.0",
    0b01: "v2.0",
    0b10: "v3",
    0b11: "11b (RESERVED)",
}

encoding_lookup = {
    0b0000: 0b11110,
    0b0001: 0b01001,
    0b0010: 0b10100,
    0b0011: 0b10101,
    0b0100: 0b01010,
    0b0101: 0b01011,
    0b0110: 0b01110,
    0b0111: 0b01111,
    0b1000: 0b10010,
    0b1001: 0b10011,
    0b1010: 0b10110,
    0b1011: 0b10111,
    0b1100: 0b11010,
    0b1101: 0b11011,
    0b1110: 0b11100,
    0b1111: 0b11101,
}

addresses = {
    "SOP": [1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1],
    "SOP'": [1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0],
    "SOP''": [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0],
    "Hard Reset": [0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1],
    "Cable Reset": [0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0],
    "SOP'_debug": [1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0],
    "SOP''_debug": [1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1],
}

data_commands = {
    0b00001: "Source_Capabilities",
    0b00010: "Request",
    0b00011: "BIST",
    0b00100: "Sink_Capabilities",
    0b00101: "Battery_Status",
    0b00110: "Alert",
    0b00111: "Get_Country_Info",
    0b01000: "Enter_USB",
    0b01001: "EPR_Request",
    0b01010: "EPR_Mode",
    0b01011: "Source_Info",
    0b01100: "Revision",
    0b01111: "Vendor_Defined",
}

control_commands = {
    0b00001: "GoodCRC",
    0b00010: "GotoMin",
    0b00011: "Accept",
    0b00100: "Reject",
    0b00101: "Ping",
    0b00110: "PS_RDY",
    0b00111: "Get_Source_Cap",
    0b01000: "Get_Sink_Cap",
    0b01001: "DR_Swap",
    0b01010: "PR_Swap",
    0b01011: "VCONN_Swap",
    0b01100: "Wait",
    0b01101: "Soft_Reset",
    0b01110: "Data_Reset",
    0b01111: "Data_Reset_Complete",
    0b10000: "Not_Supported",
    0b10001: "Get_Source_Cap_Extended",
    0b10010: "Get_Status",
    0b10011: "FR_Swap",
    0b10100: "Get_PPS_Status",
    0b10101: "Get_Country_Codes",
    0b10110: "Get_Sink_Cap_extended",
}

extended_commands = {
    0b00001: "Source_Capabilities_Extended",
    0b00010: "Status",
    0b00011: "Get_Battery_Cap",
    0b00100: "Get_Battery_Status",
    0b00101: "Battery_Capabilities",
    0b00110: "Get_Manufacturer_Info",
    0b00111: "Manufacturer_Info",
    0b01000: "Security_Request",
    0b01001: "Security_Response",
    0b01010: "Firmware_Update_Request",
    0b01011: "Firmware_Update_Response",
    0b01100: "PPS_Status",
    0b01101: "Country_Info",
    0b01110: "Country_Codes",
    0b01111: "Sink_Capabilities_Extended",
    0b10000: "Extended_Control_Message",
    0b10001: "EPR_Source_Capabilities",
    0b10010: "EPR_Sink_Capabilities",
}

power_port_role = {0: "SNK", 1: "SRC"}

cable_plug = {0: "DFP_UFP", 1: "CBL_PLUG"}

revision = {0: "1.0", 1: "2.0", 2: "Reserved", 3: "Reserved"}

port_data_role = {0: "UFP", 1: "DFP"}


class Word:
    start_time = None
    end_time = None
    data = None

    def __init__(self, start_time, end_time, data):
        self.start_time = start_time
        self.end_time = end_time
        self.data = data


class Hla:
    # An optional list of types this analyzer produces, providing a way to customize the way frames are displayed in Logic 2.
    result_types = {
        "preamble": {"format": "Preamble"},
        "address": {"format": "Start Of Packet: {{{data.address}}}"},
        "header": {
            "format": "# of Objects: {{data.number_of_objects}} Message ID: {{data.message_id}} Command Code: {{data.msg_type}} Spec Revision: {{data.spec_revision}}"
        },
        "source_fixed_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; Dual Role Power: {{data.dual_role_power}}; USB Suspend Supported: {{data.usb_suspend_supported}}; USB Communications Supported: {{data.usb_communications_capable}} Dual Role Data: {{data.dual_role_data}}; Unchecked Extended Messages Supported: {{data.unchecked_extended_messages_supported}}; Peak Current: {{data.peak_current}}; Voltage (50mV units): {{data.voltage_50mv_units}}; Maximum Current (10mA units): {{data.maximum_current_10ma_units}}"
        },
        "source_variable_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_50mv_units: {{data.maximum_voltage_50mv_units}}; minimum_voltage_50mv_units: {{data.minimum_voltage_50mv_units}}; maximum_current_10ma_units: {{data.maximum_current_10ma_units}}"
        },
        "source_battery_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_50mv_units: {{data.maximum_voltage_50mv_units}}; minimum_voltage_50mv_units: {{data.minimum_voltage_50mv_units}}; maximum_allowable_power_250mw_units: {{data.maximum_allowable_power_250mw_units}}"
        },
        "source_programmable_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_100mv_units: {{data.maximum_voltage_100mv_units}}; minimum_voltage_100mv_units: {{data.minimum_voltage_100mv_units}}; maximum_current_50ma_units: {{data.maximum_current_50ma_units}}"
        },
        "sink_fixed_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; Dual Role Power: {{data.dual_role_power}}; USB Suspend Supported: {{data.usb_suspend_supported}}; Unconstrained Power: {{data.unconstrained_power}}; USB Communications Supported: {{data.usb_communications_capable}}; Dual Role Data: {{data.dual_role_data}}; Fast Role Swap Required Current: {{data.fast_role_swap_required_current}}; Voltage (50mV units): {{data.voltage_50mv_units}}; Operational Current (10mA units): {{data.operational_current_10ma_units}}"
        },
        "sink_variable_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_50mv_units: {{data.maximum_voltage_50mv_units}}; minimum_voltage_50mv_units: {{data.minimum_voltage_50mv_units}}; maximum_current_10ma_units: {{data.maximum_current_10ma_units}}"
        },
        "sink_battery_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_50mv_units: {{data.maximum_voltage_50mv_units}}; minimum_voltage_50mv_units: {{data.minimum_voltage_50mv_units}}; maximum_allowable_power_250mw_units: {{data.maximum_allowable_power_250mw_units}}"
        },
        "sink_programmable_supply_pdo": {
            "format": "[{{data.index}}] [{{data.raw}}] {{data.data_object_type}}; {{data.pdo_type}}; maximum_voltage_100mv_units: {{data.maximum_voltage_100mv_units}}; minimum_voltage_100mv_units: {{data.minimum_voltage_100mv_units}}; maximum_current_50ma_units: {{data.maximum_current_50ma_units}}"
        },
        "bdo": {"format": "{{data.data_object_type}} BIST Mode: {{data.bist_mode}}"},
        "fixed_supply_rdo": {
            "format": "{{data.data_object_type}} Object Position: {{data.object_position}}; Giveback Flag: {{data.giveback_flag}}; Capability Missmatch: {{data.capability_mismatch}}; usb_communications_capable: {{data.usb_communications_capable}}; no_usb_suspend: {{data.no_usb_suspend}}; unchunked_extended_messages_supported: {{data.unchunked_extended_messages_supported}}; operating_current_10ma_units: {{data.operating_current_10ma_units}}; maximum_operating_current_10ma_units: {{data.maximum_operating_current_10ma_units}}; "
        },
        "variable_supply_rdo": {
            "format": "{{data.data_object_type}} Object Position: {{data.object_position}}; Giveback Flag: {{data.giveback_flag}}; Capability Missmatch: {{data.capability_mismatch}}; usb_communications_capable: {{data.usb_communications_capable}}; no_usb_suspend: {{data.no_usb_suspend}}; unchunked_extended_messages_supported: {{data.unchunked_extended_messages_supported}}; operating_current_10ma_units: {{data.operating_current_10ma_units}}; maximum_operating_current_10ma_units: {{data.maximum_operating_current_10ma_units}}; "
        },
        "battery_rdo": {
            "format": "{{data.data_object_type}} object_position: {{data.object_position}}; giveback_flag: {{data.giveback_flag}}; capability_mismatch: {{data.capability_mismatch}}; usb_communications_capable: {{data.usb_communications_capable}}; no_usb_suspend: {{data.no_usb_suspend}}; unchunked_extended_messages_supported: {{data.unchunked_extended_messages_supported}}; operating_power_250mw_units: {{data.operating_power_250mw_units}}; maximum_operating_power_250mw_units: {{data.maximum_operating_power_250mw_units}}"
        },
        "programmable_supply_rdo": {
            "format": "{{data.data_object_type}} object_position: {{data.object_position}}; capability_mismatch: {{data.capability_mismatch}}; usb_communications_capable: {{data.usb_communications_capable}}; no_usb_suspend: {{data.no_usb_suspend}}; unchunked_extended_messages_supported: {{data.unchunked_extended_messages_supported}}; output_voltage_20mV_units: {{data.output_voltage_20mV_units}}; operating_current_50ma_unites: {{data.operating_current_50ma_unites}}"
        },
        "structured_header_vdo": {
            "format": "{{data.data_object_type}} vendor_id: {{data.vendor_id}}; vdm_type: {{data.vdm_type}}; structured_vdm_version: {{data.structured_vdm_version}}; object_position: {{data.object_position}}; command_type: {{data.command_type}}; command: {{data.command}}"
        },
        "unstructured_header_vdo": {
            "format": "{{data.data_object_type}} vendor_id: {{data.vendor_id}}; vdm_type: {{data.vdm_type}}"
        },
        "bsdo": {
            "format": "{{data.data_object_type}} invalid_battery_reference: {{data.invalid_battery_reference}}; battery_is_present: {{data.battery_is_present}}; battery_charging_status: {{data.battery_charging_status}}"
        },
        "ado": {
            "format": "{{data.data_object_type}} fixed_batteries: {{data.fixed_batteries}}; hot_swappable_batteries: {{data.hot_swappable_batteries}}; battery_status_change_event: {{data.battery_status_change_event}}; ocp_event: {{data.ocp_event}}; otp_event: {{data.otp_event}}; operating_condition_change: {{data.operating_condition_change}}; source_input_change: {{data.source_input_change}}; ovp_event: {{data.ovp_event}}"
        },
        "ccdo": {
            "format": "{{data.data_object_type}} country_code: {{data.country_code}}"
        },
        "eudo": {
            "format": "{{data.data_object_type}} usb_mode: {{data.usb_mode}}; usb4_drd: {{data.usb4_drd}}; usb3_drd: {{data.usb3_drd}}; cable_speed: {{data.cable_speed}}; cable_type: {{data.cable_type}}; cable_current: {{data.cable_current}}; pcie_support: {{data.pcie_support}}; dp_support: {{data.dp_support}}; tbt_support: {{data.tbt_support}}; host_present: {{data.host_present}}"
        },
        "object": {"format": "{{data.index}} {{data.data}}"},
        "crc": {"format": "CRC: {{data.crc}}"},
        "eop": {"format": "end of packet"},
        "error": {"format": "error: {{{data.error}}} [{{data.raw}}]"},
    }

    def __init__(self):
        self.engine = None
        self.leftover_bits = []
        self.source_capabilities_pdo_types = {}

    def get_bits(self, first_frame, num_bits):
        bits_needed = max(num_bits - len(self.leftover_bits), 0)
        bytes_to_read = int(bits_needed / 8)
        extra_bits = bits_needed % 8
        if bits_needed % 8 != 0:
            bytes_to_read += 1
        self.word_result = None
        word_start = None
        word_end = None
        # data = bytearray(num_bytes)
        raw_bits = []
        for x in range(bytes_to_read):
            frame = first_frame
            if x > 0:
                frame = yield
            if x == 0:
                word_start = frame.start_time
                if len(self.leftover_bits) > 0:
                    # adjust the start time!
                    fraction = len(self.leftover_bits) / 8 * 0.9
                    span = float(frame.end_time - frame.start_time)
                    adjustment = fraction * span
                    word_start = frame.start_time - GraphTimeDelta(second=adjustment)
            if x == bytes_to_read - 1:
                word_end = frame.end_time
                if extra_bits > 0:
                    fraction = extra_bits / 8 * 0.9
                    span = float(frame.end_time - frame.start_time)
                    adjustment = fraction * span
                    word_end = frame.end_time - GraphTimeDelta(second=adjustment)
            raw_bits.extend(self.byte_to_bits(frame.data["data"]))
        return Word(word_start, word_end, raw_bits)

    def bits_to_bytes(self, new_bits, num_bytes):
        decoded = bytearray(num_bytes)
        # leftovers is an array in time order. everything is LSB first (bytes and bits) in USB pd anyway.
        self.leftover_bits.extend(new_bits)
        # get everything into a huge array of bits
        for i in range(num_bytes):
            # convert 10 bits to 8 bits, save in decoded.
            fiver = self.leftover_bits[:5]
            nibble = self.decode5bits(fiver)
            self.leftover_bits = self.leftover_bits[5:]
            decoded[i] = nibble
            nibble = self.decode5bits(self.leftover_bits[:5])
            self.leftover_bits = self.leftover_bits[5:]
            decoded[i] |= nibble << 4
        return decoded

    def byte_to_bits(self, byte):
        bits = []
        for x in range(8):
            bits.append((byte >> x) & 0x01)
        return bits

    def decode5bits(self, bit_array):
        raw_word = 0
        for i in range(5):
            raw_word = raw_word | (bit_array[i] << i)
        new_word = -1
        for entry in encoding_lookup:
            if encoding_lookup[entry] == raw_word:
                new_word = entry
                break
        if new_word == -1:
            new_word = 0

        return new_word

    def decode_address(self, bits):
        for address in addresses:
            raw_address = addresses[address]
            fiddle_sop = []
            for word in range(4):
                for bit in range(5):
                    fiddle_sop.append(raw_address[word * 5 + 4 - bit])
            match = True
            for i in range(20):
                if bits[i] != fiddle_sop[i]:
                    match = False
            if match:
                return address
        return "Unknown SOP*"

    def decode_header(self, header, sop_type):
        extended = (header >> 15) & 0x01
        number_of_objects = (header >> 12) & 0x07
        message_id = (header >> 9) & 0x07
        spec_revision = rev_lookup[(header >> 6) & 0x03]
        msg_type = header & 0x1F
        if extended:
            if msg_type in extended_commands:
                msg_type = extended_commands[msg_type]
        elif number_of_objects == 0:
            if msg_type in control_commands:
                msg_type = control_commands[msg_type]
        else:
            if msg_type in data_commands:
                msg_type = data_commands[msg_type]

        data = {
            "extended": extended,
            "number_of_objects": number_of_objects,
            "message_id": message_id,
            "spec_revision": spec_revision,
            "msg_type": str(msg_type),
        }

        data["power_port_role"] = ""
        data["port_data_role"] = ""
        if sop_type == "SOP":
            _power_port_role = (header >> 8) & 0x01
            _port_data_role = (header >> 5) & 0x01
            data["power_port_role"] = power_port_role[_power_port_role]
            data["port_data_role"] = port_data_role[_port_data_role]
        else:
            _cable_plug = (header >> 8) & 0x01
            data["cable_plug"] = cable_plug[_cable_plug]
        return data
