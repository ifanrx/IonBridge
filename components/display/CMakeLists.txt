include(${PROJECT_DIR}/component_requires.cmake)

message(STATUS "Including Component DISPLAY")

# Core source files (always compiled)
set(SRCS_FILES
  display_manager.cpp
  animation_manager.cpp
  animation_data_source.cpp
  # Animation implementations (split by domain)
  animations/base.cpp
  animations/boot.cpp
  animations/rf.cpp
  animations/charging.cpp
  animations/ota.cpp
  animations/idle.cpp
  animations/factory.cpp
)

# Base dependencies
# Note: port and esp_timer added for PowerChartAnimation (hardware-agnostic)
# Note: machine_info and wifi added for hardware-agnostic animations (IdleAnimation, DeviceInfoAnimation, etc.)
set(PRIV_REQUIRED_COMPONENTS nvs_data utils port esp_timer machine_info wifi power pb)
set(REQUIRED_COMPONENTS)

# Conditionally add driver implementations based on CMake options
# Driver selection is independent of interface type (UART/I2C)

if(ENABLE_DISPLAY_DRIVER_LVGL)
  message(STATUS "Display: Compiling LVGL driver and renderer")
  list(APPEND SRCS_FILES
    drivers/lvgl_display_driver.cpp
    lvgl_display_renderer.cpp)
  list(APPEND REQUIRED_COMPONENTS lvgl)
  list(APPEND PRIV_REQUIRED_COMPONENTS esp_lcd esp_lvgl_port bsp_generic ble)
endif()

if(ENABLE_DISPLAY_DRIVER_FPGA)
  message(STATUS "Display: Compiling FPGA driver and renderer")
  list(APPEND SRCS_FILES
    drivers/fpga_display_driver.cpp
    fpga_display_renderer.cpp)
  list(APPEND PRIV_REQUIRED_COMPONENTS rpc)
endif()

if(ENABLE_DISPLAY_DRIVER_GPU)
  message(STATUS "Display: Compiling GPU driver and renderer")
  list(APPEND SRCS_FILES
    drivers/gpu_display_driver.cpp
    gpu_display_renderer.cpp)
  list(APPEND PRIV_REQUIRED_COMPONENTS rpc storage)
endif()

if(NOT ENABLE_DISPLAY_DRIVER_LVGL AND NOT ENABLE_DISPLAY_DRIVER_FPGA AND NOT ENABLE_DISPLAY_DRIVER_GPU)
  message(STATUS "Display: No hardware driver enabled, using stub driver and renderer")
  list(APPEND SRCS_FILES stub_display_renderer.cpp)
endif()

idf_component_register(
  SRCS
  ${SRCS_FILES}
  INCLUDE_DIRS
  "include"
  REQUIRES
  ${REQUIRED_COMPONENTS}
  PRIV_REQUIRES
  ${PRIV_REQUIRED_COMPONENTS})
