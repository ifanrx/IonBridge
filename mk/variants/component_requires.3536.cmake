# Component dependency control - 3536 variant
# I2C + LVGL (68W/PA768S hardware)

# Interface type selection
option(ENABLE_COMPONENT_UART "Enable Component UART" OFF)
option(ENABLE_COMPONENT_I2C "Enable Component I2C" ON)

# Display driver selection (independent of interface type)
option(ENABLE_DISPLAY_DRIVER_FPGA "Enable FPGA Display Driver" OFF)
option(ENABLE_DISPLAY_DRIVER_GPU "Enable GPU Display Driver" OFF)
option(ENABLE_DISPLAY_DRIVER_LVGL "Enable LVGL Display Driver" ON)

# Programmer component (firmware update over UART for SW3566)
option(ENABLE_COMPONENT_PROGRAMMER "Enable Programmer Component" OFF)
