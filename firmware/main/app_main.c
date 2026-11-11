// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Ricardo Quesada
// From esp-idf-arduino-bluepad32-template@2c88185 main/main.c.
// Changed for this robot: no btstack_stdio_init() (it can install a USB-Serial-JTAG reader
// that steals bytes from Arduino Serial, which carries our telemetry and commands).
//
// Start-up order: Bluetooth comes up first, then Bluepad32 starts the Arduino task, which
// runs setup() in main.cpp on core 1. The BTstack loop below keeps running on core 0.

#include "sdkconfig.h"

#include <stddef.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>

#include <arduino_platform.h>
#include <uni.h>

int app_main(void) {
    btstack_init();                                   // BTstack on the ESP32 VHCI controller
    uni_platform_set_custom(get_arduino_platform());  // must come before uni_init()
    uni_init(0, NULL);
    btstack_run_loop_execute();  // never returns
    return 0;
}
