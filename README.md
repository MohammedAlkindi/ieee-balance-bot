# Balance bot (IEEE at UCSD QP, Project 28)

A two-wheeled self-balancing robot: an ESP32-S3 balances it at 500 Hz on two
NEMA 17 steppers, a Bluetooth gamepad drives it, and a Python bridge on a laptop
turns its telemetry into speech (Claude writes the line, ElevenLabs says it).

| Folder | What it is | Status |
|---|---|---|
| `sim/` | Plain-Python physics model for picking gains before hardware | Runs; default gains recover and drive in every scenario |
| `firmware/` | PlatformIO project for the ESP32-S3: Arduino-ESP32 3.3.12 inside ESP-IDF 5.5.5, plus Bluepad32 | Compiles; not yet run on a robot or paired with a pad |
| `bridge/` | Serial telemetry -> Claude -> ElevenLabs -> speaker | 6 tests pass; not yet run with real keys |

## 1. Simulate (no hardware)

```bash
cd sim
python sim.py            # recovery from tilts and shoves, plus driving with the gamepad limits
python sim.py --sweep    # which kp/kd pairs survive every scenario
python sim.py --L 0.15 --M 1.3   # your robot: centre-of-mass height above the axle (m) and body mass (kg)
```

The sim feeds the controller a simulated accelerometer and gyro through the same
complementary filter as the firmware. Its `torque` column is the peak torque each
motor must deliver; check it against your motor's pull-out torque at that speed.
Measure `L` on the printed chassis (balance it on a ruler edge) and re-run.

Driving: the stick asks for a speed, the firmware ramps the speed target with limited
acceleration and jerk, and the robot leans into that acceleration (`kff`). Without that lean
the sim lags the stick by about 0.25 m/s; with it, under 0.07 m/s, and the lean peaks near 5
degrees at the default 0.4 m/s. The sim models only the speed shaper and lean, not turning; to
check a change to `DRIVE_*` in `config.h`, copy it into `Drive` and `DRIVE_SPEED` in `sim.py`.

## 2. Wiring (ESP32-S3-DevKitC-1)

| Signal | GPIO | Goes to |
|---|---|---|
| I2C SDA / SCL | 8 / 9 | MPU-6050 SDA / SCL (3.3 V) |
| Left STEP / DIR | 4 / 5 | Left TMC2209 STEP / DIR |
| Right STEP / DIR | 6 / 7 | Right TMC2209 STEP / DIR |
| Motor EN (active low) | 15 | EN of both drivers, plus a 10 kOhm pull-up to 3.3 V |
| TMC UART TX | 17 | PDN_UART of both drivers |
| TMC UART RX | 18 | Joined to GPIO17 through a 1 kOhm resistor |
| Battery sense | 1 | Switched pack + (after the main switch) -> 100 kOhm -> GPIO1 -> 22 kOhm -> GND, 100 nF to GND |
| USB (native port) | 19 / 20 | Laptop: telemetry, tuning, flashing |

Driver addresses: left MS1 = GND, MS2 = GND (address 0); right MS1 = VIO, MS2 = GND
(address 1). Turn both VREF pots low: a driver that loses power comes back on its pins'
defaults (pot current, StealthChop, 1/8 vs 1/32 microsteps) until the firmware re-sends
its settings, and the firmware disarms the robot until both drivers read back as set up. Set the SPREAD jumper low (BTT "mute" default); the firmware selects
SpreadCycle over UART. Put a 100 uF low-ESR capacitor across VM/GND at each
driver, a stick-on heatsink on each driver, and the main switch between the pack
and those capacitors. Pins avoid the strapping pins (0, 3, 45, 46), native USB
(19, 20) and the flash/PSRAM pins (26-37).

Power: 3S LiPo -> XT60 -> 5 A blade fuse -> switch -> drivers' VM, and a 5 V buck
(an LM2596 module set to 5.0 V with a meter before you connect it, or a Pololu D36V28F5) -> a jumper -> ESP32-S3 5V pin. No Raspberry Pi on the
robot in the first build; the laptop is the host.

**USB and the buck must not both power the board.** Espressif lists USB, the 5V pin and
the 3V3 pin as mutually exclusive supplies. While the laptop is plugged in, leave the
buck jumper open: USB powers the logic and the pack still powers the motors (grounds
stay joined). Close it only when running without USB, or use a USB cable with VBUS cut.

### Free drivers instead of TMC2209s

If the drivers you can borrow (EnVision, the IEEE Project Space) are A4988s, DRV8825s, or TMC2208s
without UART, build the STEP/DIR version: `pio run -e esp32s3_stepdir -t upload`. Wire STEP, DIR and
EN as in the table, leave GPIO17/18 unconnected, and set each driver by hand:

| Driver | 1/16 microstep (`MICROSTEPS 16`) | Current |
|---|---|---|
| A4988 | MS1, MS2, MS3 all HIGH | Set Vref with the pot: I = Vref / (8 x Rsense), Rsense usually 0.1 Ohm (check the module) |
| DRV8825 | M0 LOW, M1 LOW, M2 HIGH | I = 2 x Vref; it has 1/32 too, but keep 1/16 to match `MICROSTEPS` |
| TMC2208 / TMC2209 standalone | MS1 and MS2 both HIGH (both low is 1/8) | Vref per the module's sheet |

Start at about 0.6 A and raise it only if a wheel skips steps. Keep the 100 uF cap at each driver.
Tie RESET to SLEEP on A4988 and DRV8825 boards. This build cannot tell if a driver lost power,
so check it is powered before you arm. If the microstep pins give something other than 1/16,
change `MICROSTEPS` in `config.h` to match, or every speed will be off by that factor.

## 3. Flash and bring up

```bash
cd firmware
pio run -t upload        # or: uvx --python 3.12 --from platformio pio run -t upload
pio device monitor       # JSON lines at 50 Hz
```

PlatformIO refuses to run under Git Bash/MSYS on Windows; use PowerShell or cmd. The first
build downloads ESP-IDF and builds the Arduino core from source: expect 30-45 minutes and a few
hundred MB. Later builds take about a minute.

The native USB port (the one you flash through) carries only the JSON telemetry and commands.
ESP-IDF and Bluetooth logs, and crash backtraces, go to the board's other USB connector
(marked UART) at 115200 baud. Plug both in when something crashes.

`firmware/sdkconfig.defaults` sets the ESP-IDF options. If you change it, delete
`firmware/sdkconfig.esp32s3` so the next build picks the change up.

Bring-up order, wheels off the ground (robot on a stand) until step 5:

1. Battery off, USB on. The monitor shows telemetry, or an `error`/`warn` line naming what is missing.
2. Tilt the robot forward by hand: `p` must go positive and `r` must be positive while it moves.
   If not, change `ACCEL_FWD`, `ACCEL_UP`, `GYRO_PITCH` or `GYRO_SIGN` in `include/config.h`.
   Mount the IMU on the body's centre line; its height does not need to match anything.
3. Hold it at its balance point (where it stays put when you let go of it on a stand) and type
   `trim <p>`, using the `p` it reads there. `p` then reads 0 at that point. Trims replace each other,
   so to adjust later type `trim <old trim + new p>`.
4. Battery on. The warning about a TMC2209 not answering must not appear.
5. Wheels still off the ground, hold it upright and still: it arms after 0.5 s (`{"ev":"armed"}`).
   Tip it forward a little: both wheels must spin the way that would drive the robot toward the
   lean. If one spins the wrong way, flip `RIGHT_MOTOR_REVERSED` (or swap that motor's coil pair
   with the battery off); if both do, flip both DIR senses. Then try it on the floor.
   Beyond 35 degrees it cuts the motors (`{"ev":"fell"}`); lift it upright to re-arm.

## 4. Tune (type into the serial monitor)

`kp 40` `kd 3` `kv 0.1` `ki 0.05` `kff 1` `trim -1.5` `arm` `disarm` `pair` `forget` `get`

`disarm` keeps the motors off until `arm` (or A on the pad), even if you hold the robot upright.
`pair` looks for a new pad for 20 s and `forget` unpairs every pad; both only work while disarmed.
An `arm` (or A) that cannot arm right then is dropped, not saved for later.

1. `kv 0` and `ki 0`. Raise `kp` until it catches itself and starts to oscillate, then raise `kd` until the oscillation dies.
2. Bring back a small `kv`, then `ki`, until it stops drifting across the floor.
3. To slow down it must first lean back, so the speed loop's effect looks backwards. That is expected.
4. Watch `loop_us` and `jit_us`. If the loop is late often or `jit_us` stays above about 20 us, set
   `CONTROL_HZ 250` in `config.h`: working stepper balancers run at 100-250 Hz, and the simulator's
   default gains still recover at 250 Hz.

Write the final gains back into `config.h`.

## 5. Drive with a gamepad

Bluepad32 pairs one Bluetooth LE gamepad. The S3 has no Bluetooth Classic, so PS4, PS5, Switch
and Wii pads will not connect. An Xbox Wireless controller works once its firmware is 5.x or
newer (update it in the Xbox Accessories app on a PC or Xbox). Steam and Stadia pads need their
Bluetooth firmware update.

1. Power the board and leave the robot lying down. For 20 s after boot (and after `pair`, `forget`,
   or losing the pad) it looks for pads, and it will not arm until 3 s after that window closes,
   because pairing writes to flash and that pauses the balance loop. A pad that connects closes the
   window early. Without a pad, the robot can first arm about 23 s after boot.
2. Hold the pad's pair button until its logo flashes fast. The monitor prints
   `{"ev":"pad_connected"}`, and telemetry shows `pad_ms` (age of the last report) instead of -1.
3. Arm as usual (hold it upright and still), or press A. The stick must be centred to arm.
4. Left stick up and down asks for speed, up to `DRIVE_SPEED_MAX_MPS` (0.4 m/s). Left and right turns.
5. B is the stop button: motors off at once, and they stay off until A or `arm`.

If the pad goes quiet for 0.3 s or disconnects, the targets fall back to zero and the robot keeps
balancing in place (`{"ev":"pad_lost"}` on a disconnect). A pad lost while armed cannot reconnect
until the robot is disarmed (tip it past 35 degrees, or type `disarm`), so B is gone until then. Telemetry adds `vt` (speed target, m/s)
and `yaw` (turn rate target, rad/s). To check directions with the wheels off the ground, watch telemetry
rather than the wheels (the balance loop fights you on a stand: to drive forward it first spins the
wheels backward to lean forward). Stick up must make `vt` go positive, and stick right must make
`yaw` positive and the left wheel spin faster than the right. If pushing the stick
still stops the robot after 0.3 s, watch `pad_ms` while holding the stick still; if it climbs past
300, raise `GAMEPAD_TIMEOUT_MS`. Measure `TRACK_WIDTH_M` (wheel centre to wheel centre).

## 6. Voice bridge

```bash
cd bridge
uv sync
uv run pytest -q
uv run python bridge.py --fake --no-voice        # synthetic robot, text only (needs ANTHROPIC_API_KEY)
uv run python bridge.py --make-clips             # one-time: render the instant reaction clips (needs ELEVENLABS_API_KEY)
uv run python bridge.py --port COM5              # real robot (/dev/ttyACM0 on Linux/macOS)
```

Close the serial monitor first; only one program can hold the port. Type a
question into the bridge's terminal and press Enter to ask the robot.

## Safety

- Never plug or unplug a motor, or touch a driver's pins, while the pack is connected. Hot-plugging a stepper destroys TMC2209s.
- Check the pack's polarity with a meter before every first connection.
- On a bench supply, set `MOTOR_RMS_MA` to 400 first: the Makerspace supplies give 1 A at 12 V, two drivers at 0.9 A pull them into current limit, and a bench supply cannot absorb the energy motors push back when they slow down. Run full current only on the pack.
- Charge LiPos on a balance charger at 1C or less, in a LiPo bag, never unattended. Never use a swollen pack.
- The firmware warns at 10.8 V and cuts the motors at 10.2 V under load; also plug a LiPo alarm into the balance lead.
- The voice bridge only reads telemetry. Nothing it does can drive the motors.
- The gamepad can. Keep a hand near the robot and the pad's B button in the first drive tests,
  and test new drive limits with the wheels off the ground.
- The firmware includes BTstack, which is free for non-commercial use only; see
  `firmware/THIRD_PARTY_NOTICES.md` before using it for anything paid.
- Keep API keys in environment variables, never in the repo, and set a monthly spend limit in the Anthropic and ElevenLabs consoles.
