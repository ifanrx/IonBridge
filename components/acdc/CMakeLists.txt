include(${PROJECT_DIR}/component_requires.cmake)
file(GLOB SRCS_FILES *.cpp)
idf_component_register(SRCS ${SRCS_FILES} INCLUDE_DIRS "include" PRIV_REQUIRES
                       esp_driver_gpio)
