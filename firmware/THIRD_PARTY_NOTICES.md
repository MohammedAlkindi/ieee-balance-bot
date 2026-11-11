# Third-party code in this firmware

The repository's MIT license covers our own code (`main/main.cpp`, `main/gamepad.cpp`,
`include/`, and the sim and bridge). The code below keeps its own license.

| Path | From | Version | License |
|---|---|---|---|
| `components/bluepad32/` | [esp-idf-arduino-bluepad32-template](https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template) | 2c88185 (Bluepad32 5.0.0-beta0) | Apache-2.0 (`third_party/LICENSES/Apache-2.0.txt`), except files whose header says otherwise: `bt/uni_bt*.c` carry the BTstack license (non-commercial), the SDL-derived controller-type files are zlib, and some parsers are Apache-2.0 or MIT |
| `components/bluepad32_arduino/` | same | 2c88185 | Apache-2.0 or LGPL-2.1-or-later (both texts in `third_party/LICENSES/`) |
| `components/cmd_nvs/`, `components/cmd_system/` | same (Espressif console examples) | 2c88185 | Public domain or CC0 |
| `main/app_main.c` | same, `main/main.c` | 2c88185 | Apache-2.0 |
| `third_party/btstack/` | [BTstack](https://github.com/bluekitchen/btstack), a subset (see below) | 8a8b616 | BTstack license (`third_party/btstack/LICENSE`): **non-commercial use only** |
| `third_party/btstack/3rd-party/` | libraries bundled with BTstack | as listed in its `README.md` | Apache-2.0, BSD, MIT or public domain, per that table |
| TMC2209 driver library | `janelia-arduino/TMC2209@10.1.1`, downloaded by PlatformIO, not stored here | 10.1.1 | BSD-3-Clause |
| Arduino-ESP32 core, ESP-IDF | downloaded by PlatformIO, not stored here | 3.3.12 / 5.5.5 | LGPL-2.1 / Apache-2.0 |

## Changes we made

- `components/bluepad32_arduino/CMakeLists.txt`: picks the Arduino component name used by
  PlatformIO's hybrid build (`framework-arduinoespressif32`), else `arduino`.
- `main/app_main.c`: dropped `btstack_stdio_init()` and the `CONFIG_AUTOSTART_ARDUINO` branch.
- Nothing else in `components/` or `third_party/btstack/` was edited.

## BTstack subset

Only what BTstack's ESP32 port compiles (`port/esp32/components/btstack/CMakeLists.txt`):
`LICENSE`, `README.md`, `port/esp32/components/`, `tool/cmake/`, `src/`, `platform/embedded/`,
`platform/freertos/`, `platform/lwip/`, `chipset/zephyr/`, and from `3rd-party/`: `README.md`,
`bluedroid/decoder/`, `bluedroid/encoder/`, `hxcmod-player/`, `lc3-google/include/`,
`lc3-google/src/`, `lc3-google/LICENSE`, `lwip/dhcp-server/`, `md5/`, `qr-code-generator/`,
`yxml/`, `micro-ecc/`.

## What the BTstack license means for this team

BTstack is free for non-commercial use. Its clause 4 allows use "solely for personal benefit and
not for any commercial purpose or for monetary gain". A student club project shown at a showcase
fits. Selling the robot or its firmware, or work done for a company, would need a commercial
license from BlueKitchen (contact@bluekitchen-gmbh.com). If a prize or sponsor money is involved,
ask them first. Anyone who gives out a built `.bin` must include the BTstack license with it, plus the BSD notices
of micro-ecc and TMC2209, and must follow LGPL-2.1 for the Arduino core (this public source covers that).
