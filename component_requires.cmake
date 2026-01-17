# Define options to control component inclusion
option(ENABLE_COMPONENT_UART "Enable Component UART" ON)
option(ENABLE_COMPONENT_I2C "Enable Component I2C" OFF)

# Display driver selection (independent of interface type)
option(ENABLE_DISPLAY_DRIVER_FPGA "Enable FPGA Display Driver" ON)
option(ENABLE_DISPLAY_DRIVER_GPU "Enable GPU Display Driver" OFF)
option(ENABLE_DISPLAY_DRIVER_LVGL "Enable LVGL Display Driver" OFF)
