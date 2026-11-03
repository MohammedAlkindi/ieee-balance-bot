# Balance bot (IEEE at UCSD QP, Project 28)

A two-wheeled self-balancing robot: an ESP32-S3 balances it at 500 Hz on two
NEMA 17 steppers, and a Python bridge on a laptop turns its telemetry into
speech (Claude writes the line, ElevenLabs says it).

| Folder | What it is | Status |
|---|---|---|
| `sim/` | Plain-Python physics model for picking gains before hardware | Runs; default gains recover in every scenario |
| `firmware/` | PlatformIO project for the ESP32-S3 (Arduino-ESP32 3.x) | Compiles; not yet run on a robot |
| `bridge/` | Serial telemetry -> Claude -> ElevenLabs -> speaker | 6 tests pass; not yet run with real keys |

## 1. Simulate (no hardware)

```bash
cd sim
python sim.py            # recovery from tilts and shoves with the default gains
python sim.py --sweep    # which kp/kd pairs survive every scenario
python sim.py --L 0.15 --M 1.3   # your robot: centre-of-mass height above the axle (m) and body mass (kg)
```

The sim feeds the controller a simulated accelerometer and gyro through the same
complementary filter as the firmware. Its `torque` column is the peak torque each
motor must deliver; check it against your motor's pull-out torque at that speed.
Measure `L` on the printed chassis (balance it on a ruler edge) and re-run.

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

## 3. Flash and bring up

```bash
cd firmware
pio run -t upload        # or: uvx --from platformio pio run -t upload
pio device monitor       # JSON lines at 50 Hz
```

PlatformIO refuses to run under Git Bash/MSYS on Windows; use PowerShell or cmd.

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

`kp 40` `kd 3` `kv 0.1` `ki 0.05` `trim -1.5` `arm` `disarm` `get`

1. `kv 0` and `ki 0`. Raise `kp` until it catches itself and starts to oscillate, then raise `kd` until the oscillation dies.
2. Bring back a small `kv`, then `ki`, until it stops drifting across the floor.
3. To slow down it must first lean back, so the speed loop's effect looks backwards. That is expected.
4. Watch `loop_us` and `jit_us`. If the loop is late often or `jit_us` stays above about 20 us, set
   `CONTROL_HZ 250` in `config.h`: working stepper balancers run at 100-250 Hz, and the simulator's
   default gains still recover at 250 Hz.

Write the final gains back into `config.h`.

## 5. Voice bridge

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
- Keep API keys in environment variables, never in the repo, and set a monthly spend limit in the Anthropic and ElevenLabs consoles.
