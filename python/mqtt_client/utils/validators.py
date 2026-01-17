"""Click parameter validators for MQTT Client CLI."""

import asyncclick as click

from data_types.port import PORTS, PortCommand


def validate_port(ctx, param, value):
    """Validate port parameter is available for the current product family."""
    if value is None:
        return value

    # Handle single port
    if isinstance(value, str):
        if value not in PORTS:
            current_family = PORTS.get_family()
            available = ", ".join(PORTS)
            raise click.BadParameter(
                f"Port '{value}' is not available for product {current_family}. "
                f"Available ports: {available}"
            )
        return value

    # Handle port list
    if isinstance(value, (list, tuple)):
        invalid_ports = [p for p in value if p not in PORTS]
        if invalid_ports:
            current_family = PORTS.get_family()
            available = ", ".join(PORTS)
            raise click.BadParameter(
                f"Ports {invalid_ports} are not available for product {current_family}. "
                f"Available ports: {available}"
            )
        return value

    return value


def validate_port_with_index(ctx, param, value):
    """Validate port parameter and return (port_name, index) tuple."""
    if value is None:
        return value

    validated_port = validate_port(ctx, param, value)
    return (validated_port, PORTS.index(validated_port))


def validate_port_command(ctx, param, value):
    """Validate and convert port command name to PortCommand enum."""
    if value is None:
        return None

    try:
        # Try to find the command by name (case insensitive)
        for cmd in PortCommand:
            if cmd.name.upper() == value.upper():
                return cmd

        # If not found by name, raise error with available options
        raise ValueError(f"Invalid command '{value}'")
    except (AttributeError, ValueError):
        valid_commands = [
            cmd.name for cmd in PortCommand if cmd != PortCommand.INVALID_COMMAND
        ]
        raise click.BadParameter(
            f"Invalid command '{value}'. Valid commands: {', '.join(sorted(valid_commands))}"
        )


def validate_custom_pdo_voltage(ctx, param, value):
    """Validate custom PDO voltage is in range but not standard values."""
    standard_voltages = [5000, 9000, 15000, 20000]
    if value < 5000 or value > 20000:
        raise click.BadParameter("Voltage must be between 5000-20000 mV (5-20V)")
    if value in standard_voltages:
        raise click.BadParameter(
            f"Voltage {value}mV is a standard PDO voltage. Use a custom value."
        )
    return value
