# Component dependency control - GPU Demo variant
# UART + GPU Display Driver

# Interface type selection
option(ENABLE_COMPONENT_UART "Enable Component UART" ON)
option(ENABLE_COMPONENT_I2C "Enable Component I2C" OFF)

# Display driver selection (independent of interface type)
option(ENABLE_DISPLAY_DRIVER_FPGA "Enable FPGA Display Driver" OFF)
option(ENABLE_DISPLAY_DRIVER_GPU "Enable GPU Display Driver" ON)
option(ENABLE_DISPLAY_DRIVER_LVGL "Enable LVGL Display Driver" OFF)

# Programmer component (firmware update over UART for SW3566)
option(ENABLE_COMPONENT_PROGRAMMER "Enable Programmer Component" ON)
