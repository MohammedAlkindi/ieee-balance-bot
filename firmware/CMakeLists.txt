# ESP-IDF project: Arduino-ESP32 3.3.12 as a component, BTstack, and Bluepad32 for the gamepad.
cmake_minimum_required(VERSION 3.16)

set(ENV{BTSTACK_ROOT} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/btstack)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include($ENV{BTSTACK_ROOT}/port/esp32/components/btstack/cmake/BTstack_project.cmake)

project(balance_bot)

# Arduino Serial on the S3's native USB port (USB-Serial-JTAG). These must reach every
# component, including Arduino's own HWCDC.cpp, or Serial falls back to UART0.
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_MODE=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_CDC_ON_BOOT=1" APPEND)
