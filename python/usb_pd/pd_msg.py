# https://raw.githubusercontent.com/saleae/hla-usb-pd/refs/heads/master/MessageHandling.py
pdo_type = {
    0b00: 'Fixed Supply',
    0b01: 'Battery',
    0b10: 'Variable Supply',
    0b11: 'Augmented Power Data Object'
}
augmented_pdo_type = {
    0b00: 'Programmable Power Supply',
    0b01: 'Reserved',
    0b10: 'Reserved',
    0b11: 'Reserved'
}

fixed_supply_sink_fast_role_swap_required_current = {
    0b00: 'Fast Swap not supported',
    0b01: 'Default USB Power',
    0b10: '1.5A @ 5V',
    0b11: '3.0A @ 5V'
}

bist_modes = {
    0b0000: 'Reserved',
    0b0001: 'Reserved',
    0b0010: 'Reserved',
    0b0011: 'Reserved',
    0b0100: 'Reserved',
    0b0101: 'BIST Carrier Mode',
    0b0110: 'Reserved',
    0b0111: 'Reserved',
    0b1000: 'BIST Test Data',
    0b1001: 'BIST Shared Test Mode Entry',
    0b1010: 'BIST Shared TestMode Exit',
    0b1011: 'Reserved',
    0b1100: 'Reserved',
    0b1101: 'Reserved',
    0b1110: 'Reserved',
    0b1111: 'Reserved',
}

structured_vdm_version = {
    0b00: 'Version 1.0',
    0b01: 'Version 2.0',
    0b10: 'Reserved',
    0b11: 'Reserved'
}

# object position for Enter Mode, Exit Mode, and Attention Commands. For other commands, should be 0 and is ignored.
vdo_object_position = {
    0b000: 'Reserved',
    0b001: 1,
    0b010: 2,
    0b011: 3,
    0b100: 4,
    0b101: 5,
    0b110: 6,
    0b111: 'Exit all Active Modes',
}
vdo_command_type = {
    0b00: 'REQ',
    0b01: 'ACK',
    0b10: 'NAK',
    0b11: 'BUSY',
}

battery_charging_status = {
    0b00: 'Battery is Charging',
    0b01: 'Battery is Discharging',
    0b10: 'Battery is Idle',
    0b11: 'Reserved',
}

# 5 bit command. 7-15
vdo_command= {
    0b00000: 'Reserved',
    0b00001: 'Discover Identity',
    0b00010: 'Discover SVIDs',
    0b00011: 'Discover Modes',
    0b00100: 'Enter Mode',
    0b00101: 'Exit Mode',
    0b00110: 'Attention',
    0b00111: 'Reserved',
    0b01000: 'Reserved',
    0b01001: 'Reserved',
    0b01010: 'Reserved',
    0b01011: 'Reserved',
    0b01100: 'Reserved',
    0b01101: 'Reserved',
    0b01110: 'Reserved',
    0b01111: 'Reserved',
    0b10000: 'SVID Specific Command [16]',
    0b10001: 'SVID Specific Command [17]',
    0b10010: 'SVID Specific Command [18]',
    0b10011: 'SVID Specific Command [19]',
    0b10100: 'SVID Specific Command [20]',
    0b10101: 'SVID Specific Command [21]',
    0b10110: 'SVID Specific Command [22]',
    0b10111: 'SVID Specific Command [23]',
    0b11000: 'SVID Specific Command [24]',
    0b11001: 'SVID Specific Command [25]',
    0b11010: 'SVID Specific Command [26]',
    0b11011: 'SVID Specific Command [27]',
    0b11100: 'SVID Specific Command [28]',
    0b11101: 'SVID Specific Command [29]',
    0b11110: 'SVID Specific Command [30]',
    0b11111: 'SVID Specific Command [31]'
}

usb_mode = {
    0b000: 'USB 2.0',
    0b001: 'USB 3.2',
    0b010: 'USB 4',
    0b011: 'Reserved',
    0b100: 'Reserved',
    0b101: 'Reserved',
    0b110: 'Reserved',
    0b111: 'Reserved',
}

cable_speed = {
    0b000: 'USB 2.0',
    0b001: 'USB 3.1 Gen 1',
    0b010: 'USB 3.2 Gen2 and USB 4 Gen2',
    0b011: 'USB 4 Gen 4',
    0b100: 'Reserved',
    0b101: 'Reserved',
    0b110: 'Reserved',
    0b111: 'Reserved',
}

cable_type = {
    0b00: 'Passive',
    0b01: 'Active Re-timer',
    0b10: 'Active Re-driver',
    0b11: 'Optically Isolated',
}

cable_current = {
    0b00: 'Vbus not supported',
    0b01: 'Reserved',
    0b10: '3A',
    0b11: '5A',
}

# Source_capabilities,


def decode_source_power_data_object(word):
    data = {
        'data_object_type': 'Power Data Object'
    }
    frame_type = 'object'
    _pdo_type = (word >> 30) & 0x3
    pdo_type_str = pdo_type[_pdo_type]
    if _pdo_type == 0b11:
        _augmented_pdo_type = (word >> 28) & 0x3
        pdo_type_str = augmented_pdo_type[_augmented_pdo_type]
        if _augmented_pdo_type == 0b00:
            # Programmable Power Supply Augmented Power Data Object
            pps_power_limited = (word >> 27) & 0x1
            maximum_voltage_100mv_units = (word >> 17) & 0xFF
            minimum_voltage_100mv_units = (word >> 8) & 0xFF
            maximum_current_50ma_units = word & 0x7F
            data['maximum_voltage_100mv_units'] = maximum_voltage_100mv_units
            data['minimum_voltage_100mv_units'] = minimum_voltage_100mv_units
            data['maximum_current_50ma_units'] = maximum_current_50ma_units
            frame_type = 'source_programmable_supply_pdo'
    else:
        if _pdo_type == 0b00:
            # Fixed Supply Power Data Object
            dual_role_power = (word >> 29) & 0x1
            usb_suspend_supported = (word >> 28) & 0x1
            unconstrained_power = (word >> 27) & 0x1
            usb_communications_capable = (word >> 26) & 0x1
            dual_role_data = (word >> 25) & 0x1
            unchecked_extended_messages_supported = (word >> 24) & 0x1
            peak_current = (word >> 20) & 0x3
            voltage_50mv_units = (word >> 10) & 0x3FF
            maximum_current_10ma_units = word & 0x3FF
            data['dual_role_power'] = dual_role_power
            data['usb_suspend_supported'] = usb_suspend_supported
            data['unconstrained_power'] = unconstrained_power
            data['usb_communications_capable'] = usb_communications_capable
            data['dual_role_data'] = dual_role_data
            data['unchecked_extended_messages_supported'] = unchecked_extended_messages_supported
            data['peak_current'] = peak_current
            data['voltage_50mv_units'] = voltage_50mv_units
            data['maximum_current_10ma_units'] = maximum_current_10ma_units
            frame_type = 'source_fixed_supply_pdo'

        elif _pdo_type == 0b10:
            # variable supply (non-battery) Power Data Object - source
            maximum_voltage_50mv_units = (word >> 20) & 0x3FF
            minimum_voltage_50mv_units = (word >> 10) & 0x3FF
            maximum_current_10ma_units = word & 0x3FF
            data['maximum_voltage_50mv_units'] = maximum_voltage_50mv_units
            data['minimum_voltage_50mv_units'] = minimum_voltage_50mv_units
            data['maximum_current_10ma_units'] = maximum_current_10ma_units
            frame_type = 'source_variable_supply_pdo'
        elif _pdo_type == 0b01:
            # Battery Supply Power Data Object
            maximum_voltage_50mv_units = (word >> 20) & 0x3FF
            minimum_voltage_50mv_units = (word >> 10) & 0x3FF
            maximum_allowable_power_250mw_units = word & 0x3FF
            data['maximum_voltage_50mv_units'] = maximum_voltage_50mv_units
            data['minimum_voltage_50mv_units'] = minimum_voltage_50mv_units
            data['maximum_allowable_power_250mw_units'] = maximum_allowable_power_250mw_units
            frame_type = 'source_battery_supply_pdo'

    data['pdo_type'] = pdo_type_str

    return frame_type, data

# sink_capabilities,


def decode_sink_power_data_object(word):
    data = {
        'data_object_type': 'Power Data Object'
    }
    frame_type = 'object'

    _pdo_type = (word >> 30) & 0x3
    pdo_type_str = pdo_type[_pdo_type]
    if _pdo_type == 0b11:
        _augmented_pdo_type = (word >> 28) & 0x3
        pdo_type_str = augmented_pdo_type[_augmented_pdo_type]
        if _augmented_pdo_type == 0b00:
            # Programmable Power Supply Augmented Power Data Object
            pps_power_limited = (word >> 27) & 0x1
            maximum_voltage_100mv_units = (word >> 17) & 0xFF
            minimum_voltage_100mv_units = (word >> 8) & 0xFF
            maximum_current_50ma_units = word & 0x7F
            data['maximum_voltage_100mv_units'] = maximum_voltage_100mv_units
            data['minimum_voltage_100mv_units'] = minimum_voltage_100mv_units
            data['maximum_current_50ma_units'] = maximum_current_50ma_units
            frame_type = 'sink_programmable_supply_pdo'
    else:
        if _pdo_type == 0b00:
            # Fixed Supply Power Data Object
            dual_role_power = (word >> 29) & 0x1
            usb_suspend_supported = (word >> 28) & 0x1
            unconstrained_power = (word >> 27) & 0x1
            usb_communications_capable = (word >> 26) & 0x1
            dual_role_data = (word >> 25) & 0x1
            fast_role_swap_required_current = (word >> 23) & 0x3
            voltage_50mv_units = (word >> 10) & 0x3FF
            operational_current_10ma_units = word & 0x3FF
            data['dual_role_power'] = dual_role_power
            data['usb_suspend_supported'] = usb_suspend_supported
            data['unconstrained_power'] = unconstrained_power
            data['usb_communications_capable'] = usb_communications_capable
            data['dual_role_data'] = dual_role_data
            data['fast_role_swap_required_current'] = fixed_supply_sink_fast_role_swap_required_current[fast_role_swap_required_current]
            data['voltage_50mv_units'] = voltage_50mv_units
            data['operational_current_10ma_units'] = operational_current_10ma_units
            frame_type = 'sink_fixed_supply_pdo'

        elif _pdo_type == 0b10:
            # variable supply (non-battery) Power Data Object - source
            maximum_voltage_50mv_units = (word >> 20) & 0x3FF
            minimum_voltage_50mv_units = (word >> 10) & 0x3FF
            maximum_current_10ma_units = word & 0x3FF
            data['maximum_voltage_50mv_units'] = maximum_voltage_50mv_units
            data['minimum_voltage_50mv_units'] = minimum_voltage_50mv_units
            data['maximum_current_10ma_units'] = maximum_current_10ma_units
            frame_type = 'sink_variable_supply_pdo'
        elif _pdo_type == 0b01:
            # Battery Supply Power Data Object
            maximum_voltage_50mv_units = (word >> 20) & 0x3FF
            minimum_voltage_50mv_units = (word >> 10) & 0x3FF
            maximum_allowable_power_250mw_units = word & 0x3FF
            data['maximum_voltage_50mv_units'] = maximum_voltage_50mv_units
            data['minimum_voltage_50mv_units'] = minimum_voltage_50mv_units
            data['maximum_allowable_power_250mw_units'] = maximum_allowable_power_250mw_units
            frame_type = 'sink_battery_supply_pdo'

    data['pdo_type'] = pdo_type_str

    return frame_type, data


def decode_bist_data_object(word):
    data = {
        'data_object_type': 'BIST Data Object'
    }
    frame_type = 'bdo'
    bist_mode = (word >> 28) & 0xF
    data['bist_mode'] = bist_modes[bist_mode]
    return frame_type, data

# the pdo type needs to be extracted from the source capabilities message, at the object_position taken from this (word >> 28) & 0x7


def decode_request_data_object(word, pdo_type):
    data = {
        'data_object_type': 'Request Data Object'
    }
    frame_type = 'object'
    if pdo_type == 'Fixed Supply' or pdo_type == 'Variable Supply':
        object_position = (word >> 28) & 0x7
        giveback_flag = (word >> 27) & 0x1
        capability_mismatch = (word >> 26) & 0x1
        usb_communications_capable = (word >> 25) & 0x1
        no_usb_suspend = (word >> 24) & 0x1
        unchunked_extended_messages_supported = (word >> 23) & 0x1
        operating_current_10ma_units = (word >> 10) & 0x3FF
        maximum_operating_current_10ma_units = word & 0x3FF
        data['object_position'] = object_position
        data['giveback_flag'] = giveback_flag
        data['capability_mismatch'] = capability_mismatch
        data['usb_communications_capable'] = usb_communications_capable
        data['no_usb_suspend'] = no_usb_suspend
        data['unchunked_extended_messages_supported'] = unchunked_extended_messages_supported
        data['operating_current_10ma_units'] = operating_current_10ma_units
        data['maximum_operating_current_10ma_units'] = maximum_operating_current_10ma_units
        if pdo_type == 'Fixed Supply':
            frame_type = 'fixed_supply_rdo'
        if pdo_type == 'Variable Supply':
            frame_type = 'variable_supply_rdo'
        pass
    elif pdo_type == 'Battery':
        object_position = (word >> 28) & 0x7
        giveback_flag = (word >> 27) & 0x1
        capability_mismatch = (word >> 26) & 0x1
        usb_communications_capable = (word >> 25) & 0x1
        no_usb_suspend = (word >> 24) & 0x1
        unchunked_extended_messages_supported = (word >> 23) & 0x1
        operating_power_250mw_units = (word >> 10) & 0x3FF
        maximum_operating_power_250mw_units = word & 0x3FF
        data['object_position'] = object_position
        data['giveback_flag'] = giveback_flag
        data['capability_mismatch'] = capability_mismatch
        data['usb_communications_capable'] = usb_communications_capable
        data['no_usb_suspend'] = no_usb_suspend
        data['unchunked_extended_messages_supported'] = unchunked_extended_messages_supported
        data['operating_power_250mw_units'] = operating_power_250mw_units
        data['maximum_operating_power_250mw_units'] = maximum_operating_power_250mw_units
        frame_type = 'battery_rdo'

    elif pdo_type == 'Programmable Power Supply':
        object_position = (word >> 28) & 0x7
        capability_mismatch = (word >> 26) & 0x1
        usb_communications_capable = (word >> 25) & 0x1
        no_usb_suspend = (word >> 24) & 0x1
        unchunked_extended_messages_supported = (word >> 23) & 0x1
        output_voltage_20mV_units = (word >> 9) & 0x7FF
        operating_current_50ma_unites = word & 0x7F
        data['object_position'] = object_position
        data['capability_mismatch'] = capability_mismatch
        data['usb_communications_capable'] = usb_communications_capable
        data['no_usb_suspend'] = no_usb_suspend
        data['unchunked_extended_messages_supported'] = unchunked_extended_messages_supported
        data['output_voltage_20mV_units'] = output_voltage_20mV_units
        data['operating_current_50ma_unites'] = operating_current_50ma_unites
        frame_type = 'programmable_supply_rdo'
    return frame_type, data


def decode_vendor_header_data_object(word):
    data = {
        'data_object_type': 'Vendor Data Object'
    }
    frame_type = 'header_vdo'

    vendor_id = (word >> 16) & 0xFFFF
    vdm_type = (word >> 15) & 0x1
    data['vendor_id'] = vendor_id
    data['vdm_type'] = vdm_type
    if vdm_type == 1:
        frame_type = 'structured_header_vdo'
        _structured_vdm_version = (word >> 13) & 0x3
        object_position = (word >> 8) & 0x7
        command_type = (word >> 6) & 0x3
        command = word & 0x1F
        data['structured_vdm_version'] = structured_vdm_version[_structured_vdm_version]
        data['object_position'] = vdo_object_position[object_position]
        data['command_type'] = vdo_command_type[command_type]
        data['command'] = vdo_command[command]
    else:
        frame_type = 'unstructured_header_vdo'
    return frame_type, data


def decode_battery_status_data_object(word):
    data = {
        'data_object_type': 'Battery Status Data Object'
    }
    frame_type = 'bsdo'
    battery_present_capacity = (word >> 16) & 0xFFFF
    battery_info = (word >> 8) & 0xFF
    invalid_battery_reference = battery_info & 0x1
    battery_is_present = (battery_info >> 1) & 0x1
    _battery_charging_status = (battery_info >> 2) & 0x3
    data['invalid_battery_reference'] = invalid_battery_reference
    data['battery_is_present'] = battery_is_present
    if battery_is_present == 1:
        data['battery_charging_status'] = battery_charging_status[_battery_charging_status]

    return frame_type, data


def decode_alert_data_object(word):
    data = {
        'data_object_type': 'Alert Data Object'
    }
    frame_type = 'ado'

    type_of_alert = (word >> 24) & 0x7

    fixed_batteries = (word >> 20) & 0xF
    hot_swappable_batteries = (word >> 16) & 0xF
    battery_status_change_event = (type_of_alert >> 1) & 0x1
    ocp_event = (type_of_alert >> 2) & 0x1
    otp_event = (type_of_alert >> 3) & 0x1
    operating_condition_change = (type_of_alert >> 4) & 0x1
    source_input_change = (type_of_alert >> 5) & 0x1
    ovp_event = (type_of_alert >> 6) & 0x1

    data['fixed_batteries'] = fixed_batteries
    data['hot_swappable_batteries'] = hot_swappable_batteries
    data['battery_status_change_event'] = battery_status_change_event
    data['ocp_event'] = ocp_event
    data['otp_event'] = otp_event
    data['operating_condition_change'] = operating_condition_change
    data['source_input_change'] = source_input_change
    data['ovp_event'] = ovp_event

    return frame_type, data


def decode_get_country_info_data_object(word):
    data = {
        'data_object_type': 'Country Code Data Object'
    }
    frame_type = 'ccdo'

    first_character = (word >> 24) & 0xF
    second_character = (word >> 16) & 0xF

    data['country_code'] = '{first}{second}'.format(
        first=chr(first_character), second=char(second_character))

    return frame_type, data


def decode_enter_usb_data_object(word):
    data = {
        'data_object_type': 'Enter USB Data Object'
    }
    frame_type = 'eudo'

    _usb_mode = (word >> 28) & 0x7
    usb4_drd = (word >> 26) & 0x1
    usb3_drd = (word >> 25) & 0x1
    _cable_speed = (word >> 21) & 0x7
    _cable_type = (word >> 21) & 0x7
    _cable_current = (word >> 17) & 0x3
    pcie_support = (word >> 16) & 0x1
    dp_support = (word >> 15) & 0x1
    tbt_support = (word >> 14) & 0x1
    host_present = (word >> 13) & 0x1

    data['usb_mode'] = usb_mode[_usb_mode]
    data['usb4_drd'] = usb4_drd
    data['usb3_drd'] = usb3_drd
    data['cable_speed'] = cable_speed[_cable_speed]
    data['cable_type'] = cable_type[_cable_type]
    data['cable_current'] = cable_current[_cable_current]
    data['pcie_support'] = pcie_support
    data['dp_support'] = dp_support
    data['tbt_support'] = tbt_support
    data['host_present'] = host_present

    return frame_type, data