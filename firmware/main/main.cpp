// Phase 1 balance firmware for an ESP32-S3 two-wheeled robot.
//
// Core 1: a 500 Hz hardware-timer interrupt wakes the control task, which reads
//         the IMU, runs a complementary filter and the cascaded balance law, and
//         sets both wheel step rates. A 40 kHz timer interrupt turns those rates
//         into STEP pulses (phase accumulator), so speed changes apply at once.
// Core 0: telemetry task. Prints one JSON line every 20 ms plus event lines,
//         re-sends the driver config after a power cycle, and reads tuning
//         commands typed on the same USB serial port:
//           kp 40 | kd 3 | kv 0.1 | ki 0.05 | kff 1 | trim -1.5 | arm | disarm | pair | forget | get
//         It also checks both drivers at 10 Hz and disarms if one resets.
// Core 0: gamepad task (gamepad.cpp, Bluepad32). The left stick asks for a speed and a turn rate.
//         The control task shapes the speed with limited acceleration and jerk, leans
//         into that acceleration, and splits the turn across the two wheels. If the
//         pad goes quiet for GAMEPAD_TIMEOUT_MS the targets fall back to zero and the
//         robot keeps balancing in place.
// The control task never prints and never waits on Serial or the gamepad.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <soc/gpio_reg.h>

#include "config.h"
#if DRIVER_UART
#include <TMC2209.h>
#endif
#include "pad.h"

// Float versions of Arduino's degrees()/radians(), which promote to double;
// the S3 FPU is single precision, so doubles would be emulated in the 500 Hz loop.
static inline float toDeg(float rad) { return rad * 57.2957795f; }
static inline float toRad(float deg) { return deg * 0.0174532925f; }

// ---------------------------------------------------------------- shared state

struct Gains {
  volatile float kp = KP_DEFAULT, kd = KD_DEFAULT, kv = KV_DEFAULT, ki = KI_DEFAULT, kff = KFF_DEFAULT;
  volatile float trim_rad = 0.0f;  // pitch offset so "upright" reads as zero
};
static Gains gains;

struct Snapshot {
  uint32_t t_ms;
  float pitch_deg, rate_dps, speed_mps, ref_deg, accel_mps2, vbat, target_mps, turn_rps;
  uint32_t loop_us, jitter_us;
  int32_t pad_ms;  // age of the last gamepad report; -1 = no pad
  bool armed;
};
static Snapshot snap;
static portMUX_TYPE snap_mux = portMUX_INITIALIZER_UNLOCKED;

// Latest stick position from the gamepad, as fractions of full stick (-1..1).
struct DriveCmd {
  float speed = 0, turn = 0;
  int64_t stamp_us = -1;  // esp_timer time of the last report; -1 = never
};
static DriveCmd drive_cmd;
static portMUX_TYPE drive_mux = portMUX_INITIALIZER_UNLOCKED;

enum EventType : uint8_t {
  EV_ARMED, EV_FELL, EV_RECOVERED, EV_LOWBAT, EV_CUTOFF, EV_IMU_LOST, EV_DRIVER_FAULT, EV_PAD_ON, EV_PAD_OFF
};
struct Event {
  EventType type;
  float a, b;
};
static QueueHandle_t events;

static volatile bool arm_request = false, disarm_request = false, pad_connected = false;
static volatile bool robot_armed = false;  // copy of the control task's state for other tasks
// Set by the gamepad task while it looks for pads and for PAIR_SETTLE_MS after. Pairing writes
// the bond to flash, which pauses the control task, so the robot does not arm during it.
volatile bool pad_pairing = false;
// Set by the telemetry task's driver check. A TMC2209 that lost motor power comes back
// with default registers (StealthChop, and 1/8 vs 1/32 microsteps set by its address
// pins), so the robot must not run on it until it has been configured again.
// Plain STEP/DIR drivers (DRIVER_UART 0) cannot be checked, so they count as ready.
static volatile bool drivers_ok = (DRIVER_UART == 0), driver_fault = false;
static TaskHandle_t control_handle;
#if DRIVER_UART
static TMC2209 tmc_l, tmc_r;
#endif

// ---------------------------------------------------------------- step generator

// Each tick adds the wheel's increment to a 32-bit phase; when the phase wraps,
// that wheel takes one step. increment = steps_per_s * 2^32 / STEP_TICK_HZ.
static volatile uint32_t inc_l = 0, inc_r = 0;
static uint32_t phase_l = 0, phase_r = 0;

static void IRAM_ATTR onStepTick() {
  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << PIN_STEP_L) | (1UL << PIN_STEP_R));  // end last pulse
  uint32_t high = 0, before = phase_l;
  phase_l += inc_l;
  if (phase_l < before) high |= 1UL << PIN_STEP_L;
  before = phase_r;
  phase_r += inc_r;
  if (phase_r < before) high |= 1UL << PIN_STEP_R;
  if (high) REG_WRITE(GPIO_OUT_W1TS_REG, high);  // pulse stays high for one tick (25 us)
}

static uint32_t stepIncrement(float speed_mps) {
  return (uint32_t)(fabsf(speed_mps) * STEPS_PER_M * (4294967296.0f / STEP_TICK_HZ));
}

// Signed ground speeds for each wheel, m/s (+ = forward).
static void setWheelSpeeds(float left_mps, float right_mps) {
  digitalWrite(PIN_DIR_L, left_mps >= 0 ? HIGH : LOW);
  digitalWrite(PIN_DIR_R, ((right_mps >= 0) != (bool)RIGHT_MOTOR_REVERSED) ? HIGH : LOW);
  inc_l = stepIncrement(left_mps);
  inc_r = stepIncrement(right_mps);
}

static void motorsOff() {
  inc_l = 0;
  inc_r = 0;
  digitalWrite(PIN_MOTOR_EN, HIGH);
}

static void motorsOn() { digitalWrite(PIN_MOTOR_EN, LOW); }

static void setupDrivers() {
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, HIGH);  // off until armed
  for (int pin : {PIN_STEP_L, PIN_DIR_L, PIN_STEP_R, PIN_DIR_R}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  hw_timer_t *step_timer = timerBegin(1000000);  // 1 MHz timebase
  timerAttachInterrupt(step_timer, &onStepTick);
  timerAlarm(step_timer, 1000000 / STEP_TICK_HZ, true, 0);
}

#if DRIVER_UART
// Driver registers reset whenever the battery is off, so this runs at boot and
// again whenever a driver answers but reports that it is not set up.
static void configureDriver(TMC2209 &d, TMC2209::SerialAddress addr) {
  d.setup(Serial1, 115200, addr, PIN_TMC_RX, PIN_TMC_TX);
  d.setReplyDelay(2);  // two drivers share the line
  d.setMicrostepsPerStep(MICROSTEPS);
  d.setRMSCurrent(MOTOR_RMS_MA, TMC_RSENSE_OHM, 0.5f);
  d.disableStealthChop();  // SpreadCycle: StealthChop can stall on fast reversals
  d.enable();              // EN pin still gates the outputs
}
#endif

// ---------------------------------------------------------------- IMU (MPU-6050)

static float gyro_bias = 0.0f;

static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t mpuInit() {
  mpuWrite(0x6B, 0x01);  // PWR_MGMT_1: wake, PLL on gyro X
  delay(50);
  mpuWrite(0x1A, 0x02);  // CONFIG: DLPF ~94 Hz accel / 98 Hz gyro, 1 kHz internal rate
  mpuWrite(0x19, 0x01);  // SMPLRT_DIV: 1 kHz / (1 + 1) = 500 Hz
  mpuWrite(0x1B, 0x08);  // GYRO_CONFIG: +-500 deg/s (65.5 LSB per deg/s)
  mpuWrite(0x1C, 0x08);  // ACCEL_CONFIG: +-4 g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I: 0x68 on a genuine MPU-6050
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

// Reads accel (g) and gyro (deg/s). Returns false on an I2C error.
static bool mpuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  ax = raw[0] / 8192.0f;
  ay = raw[1] / 8192.0f;
  az = raw[2] / 8192.0f;
  gx = raw[4] / 65.5f;  // raw[3] is temperature
  gy = raw[5] / 65.5f;
  gz = raw[6] / 65.5f;
  return true;
}

static void calibrateGyro() {
  float sum = 0, ax, ay, az, gx, gy, gz;
  int n = 0;
  for (int i = 0; i < 500; i++) {  // ~1 s; keep the robot still
    if (mpuRead(ax, ay, az, gx, gy, gz)) {
      sum += GYRO_PITCH;
      n++;
    }
    delay(2);
  }
  gyro_bias = n ? sum / n : 0.0f;
}

// ---------------------------------------------------------------- control task

static void IRAM_ATTR onControlTick() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(control_handle, &woken);
  portYIELD_FROM_ISR(woken);
}

static void sendEvent(EventType type, float a = 0, float b = 0) {
  Event e{type, a, b};
  xQueueSend(events, &e, 0);  // drop rather than block the loop
}

// Turns the stick into a speed target the robot can follow: the target closes on
// the stick at DRIVE_GAIN, with its acceleration capped at DRIVE_ACCEL and that
// acceleration changing no faster than DRIVE_JERK. Same as Shaper in sim/sim.py.
struct SpeedShaper {
  float v = 0, a = 0;
  void reset() { v = a = 0; }
  void step(float want, float dt) {
    float a_want = constrain((want - v) * DRIVE_GAIN, -DRIVE_ACCEL_MPS2, DRIVE_ACCEL_MPS2);
    a += constrain(a_want - a, -DRIVE_JERK_MPS3 * dt, DRIVE_JERK_MPS3 * dt);
    v += a * dt;
  }
};

// ---------------------------------------------------------------- gamepad hooks (pad.h)

void padReport(float speed, float turn, int64_t stamp_us) {
  DriveCmd c{constrain(speed, -1.0f, 1.0f), constrain(turn, -1.0f, 1.0f), stamp_us};
  portENTER_CRITICAL(&drive_mux);
  drive_cmd = c;
  portEXIT_CRITICAL(&drive_mux);
}

void padArm() {
  if (!robot_armed) arm_request = true;
}
void padDisarm() { disarm_request = true; }
bool robotArmed() { return robot_armed; }

void padConnection(bool connected) {
  if (connected == pad_connected) return;
  pad_connected = connected;
  if (!connected) padReport(0, 0, -1);  // the pad keeps its last stick values; forget them
  sendEvent(connected ? EV_PAD_ON : EV_PAD_OFF);
}

// Stick deadband, then rescale so motion starts from zero just past it.
static float applyDeadband(float x) {
  if (fabsf(x) < STICK_DEADBAND) return 0.0f;
  float s = (fabsf(x) - STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
  return copysignf(constrain(s, 0.0f, 1.0f), x);
}

static void controlTask(void *) {
  const float dt = 1.0f / CONTROL_HZ;
  const float alpha = COMP_FILTER_TAU_S / (COMP_FILTER_TAU_S + dt);
  const float cutoff = toRad(FALL_CUTOFF_DEG), arm_window = toRad(ARM_WINDOW_DEG);
  const float ref_max = toRad(THETA_REF_MAX_DEG);

  float pitch = 0, integ = 0, ref = 0, accel_cmd = 0, speed = 0;
  bool armed = false, disturbed = false, lowbat_sent = false;
  bool hold_off = false;  // after a disarm command, stay off until an arm command
  float peak = 0, vbat = 0;
  SpeedShaper target;
  float turn = 0;  // yaw rate target, rad/s (+ = clockwise seen from above)
  int64_t upright_since = -1, disturb_start = 0, inside_since = -1;
  int64_t last_us = esp_timer_get_time();
  uint32_t jitter_max = 0, loop_us = 0;
  int tick = 0, bad_reads = 0;

  float ax, ay, az, gx, gy, gz;
  if (mpuRead(ax, ay, az, gx, gy, gz)) pitch = atan2f(ACCEL_FWD, ACCEL_UP);

  auto disarm = [&](EventType why, float a) {
    arm_request = false;
    motorsOff();
    speed = 0;
    target.reset();
    turn = 0;
    armed = false;
    sendEvent(why, a);
  };

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    loop_us = (uint32_t)(now - last_us);
    last_us = now;
    uint32_t jit = (uint32_t)abs((int32_t)loop_us - 1000000 / CONTROL_HZ);
    if (jit > jitter_max) jitter_max = jit;

    if (!mpuRead(ax, ay, az, gx, gy, gz)) {
      if (++bad_reads > 3 && armed) disarm(EV_IMU_LOST, bad_reads);
      continue;
    }
    bad_reads = 0;
    float rate = toRad((GYRO_PITCH - gyro_bias) * GYRO_SIGN);
    // Forward wheel acceleration reads as a backward tilt; add back last tick's command.
    float acc_pitch = atan2f(ACCEL_FWD + ACCEL_COMP * accel_cmd / 9.81f, ACCEL_UP);
    pitch = alpha * (pitch + rate * dt) + (1 - alpha) * acc_pitch;
    float theta = pitch - gains.trim_rad;

    if (disarm_request) {
      disarm_request = false;
      arm_request = false;
      hold_off = true;
      if (armed) motorsOff();
      speed = 0;
      target.reset();
      turn = 0;
      armed = false;
    }
    if (driver_fault) {
      driver_fault = false;
      if (armed) disarm(EV_DRIVER_FAULT, 0);
    }

    // Stick targets. A report older than GAMEPAD_TIMEOUT_MS counts as a centred stick,
    // so a lost pad or a stalled Bluetooth task brings the robot to a stop in place.
    DriveCmd cmd;
    portENTER_CRITICAL(&drive_mux);
    cmd = drive_cmd;
    portEXIT_CRITICAL(&drive_mux);
    bool fresh = cmd.stamp_us >= 0 && now - cmd.stamp_us < GAMEPAD_TIMEOUT_MS * 1000LL;
    float want_v = fresh ? applyDeadband(cmd.speed) * DRIVE_SPEED_MAX_MPS : 0.0f;
    float want_turn = fresh ? applyDeadband(cmd.turn) * TURN_RATE_MAX_RPS : 0.0f;

    if (armed && fabsf(theta) > cutoff) {
      disarm(EV_FELL, toDeg(theta));
    } else if (armed) {
      target.step(want_v, dt);
      turn += constrain(want_turn - turn, -TURN_ACCEL_RPS2 * dt, TURN_ACCEL_RPS2 * dt);
      // outer loop: lean toward the speed target, plus the lean that target's
      // acceleration needs (a robot speeding up at a leans forward about a / g)
      float err_v = target.v - speed;
      integ = constrain(integ + err_v * dt, -2.0f, 2.0f);
      ref = constrain(gains.kv * err_v + gains.ki * integ + gains.kff * target.a / 9.81f, -ref_max, ref_max);
      // inner loop: wheel acceleration from pitch error and pitch rate
      accel_cmd = constrain(gains.kp * (theta - ref) + gains.kd * rate, -A_MAX_MPS2, A_MAX_MPS2);
      speed = constrain(speed + accel_cmd * dt, -V_MAX_MPS, V_MAX_MPS);
      // turning: one wheel faster, the other slower; the mean speed balances the robot
      float half_diff = turn * TRACK_WIDTH_M * 0.5f;
      setWheelSpeeds(speed + half_diff, speed - half_diff);

      // disturbance -> recovery bookkeeping for the voice layer; a lean the driver
      // asked for is not a disturbance, so none starts while the targets are moving
      bool driving = fabsf(target.v) > 0.02f || fabsf(target.a) > 0.02f || fabsf(turn) > 0.05f;
      float a = fabsf(toDeg(theta));
      if (!disturbed && !driving && a > DISTURB_DEG) {
        disturbed = true;
        peak = a;
        disturb_start = now;
        inside_since = -1;
      } else if (disturbed) {
        peak = max(peak, a);
        if (a < RECOVER_DEG) {
          if (inside_since < 0) inside_since = now;
          if (now - inside_since > 100000) {
            sendEvent(EV_RECOVERED, peak, (now - disturb_start) / 1000.0f);
            disturbed = false;
          }
        } else {
          inside_since = -1;
        }
      }
    } else {
      // arm once the robot is held upright and still for ARM_HOLD_MS
      bool upright = fabsf(theta) < arm_window && fabsf(rate) < toRad(20);
      if (!upright) upright_since = -1;
      else if (upright_since < 0) upright_since = now;
      bool held = upright_since >= 0 && now - upright_since > ARM_HOLD_MS * 1000LL;
      bool battery_ok = vbat < 5.0f || vbat > VBAT_CUTOFF_V;  // < 5 V: USB only, no pack fitted
      // the last reported stick position, however old: a pushed stick whose reports
      // stopped still counts as pushed, so it cannot arm into a sudden drive later
      bool stick_centred = !pad_connected || cmd.stamp_us < 0 ||
                           (applyDeadband(cmd.speed) == 0.0f && applyDeadband(cmd.turn) == 0.0f);
      // an arm request is one-shot: it arms now or is dropped, and it needs a still robot too
      bool request = arm_request;
      arm_request = false;
      bool can_arm = upright && battery_ok && drivers_ok && stick_centred && !pad_pairing;
      if (((held && !hold_off) || request) && can_arm) {
        hold_off = false;
        integ = ref = speed = 0;
        target.reset();
        turn = 0;
        disturbed = false;
        setWheelSpeeds(0, 0);
        motorsOn();
        armed = true;
        sendEvent(EV_ARMED);
      }
      accel_cmd = 0;
    }

    if (++tick % (CONTROL_HZ / 10) == 0) {  // battery at 10 Hz
      vbat = analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_DIVIDER;
      if (vbat > 5.0f && vbat < VBAT_CUTOFF_V && armed) disarm(EV_CUTOFF, vbat);
      if (!lowbat_sent && vbat > 5.0f && vbat < VBAT_WARN_V) {
        lowbat_sent = true;
        sendEvent(EV_LOWBAT, vbat);
      }
    }

    portENTER_CRITICAL(&snap_mux);
    int32_t pad_ms = -1;
    if (pad_connected && cmd.stamp_us >= 0) pad_ms = (int32_t)min<int64_t>((now - cmd.stamp_us) / 1000, 99999);
    robot_armed = armed;
    snap = {(uint32_t)(now / 1000), toDeg(theta), toDeg(rate), speed, toDeg(ref),
            accel_cmd, vbat, target.v, turn, loop_us, jitter_max, pad_ms, armed};
    portEXIT_CRITICAL(&snap_mux);
    if (tick % (CONTROL_HZ / TELEMETRY_HZ) == 0) jitter_max = 0;  // per telemetry window
  }
}

// ---------------------------------------------------------------- telemetry task

static void handleCommand(String line) {
  line.trim();
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  float val = sp < 0 ? 0 : line.substring(sp + 1).toFloat();
  if (cmd == "kp") gains.kp = val;
  else if (cmd == "kd") gains.kd = val;
  else if (cmd == "kv") gains.kv = val;
  else if (cmd == "ki") gains.ki = val;
  else if (cmd == "kff") gains.kff = val;
  else if (cmd == "trim") gains.trim_rad = toRad(val);
  else if (cmd == "arm") padArm();
  else if (cmd == "disarm") disarm_request = true;
  else if (cmd == "forget") gamepadForgetKeys();  // unpair every pad (only while disarmed)
  else if (cmd == "pair") gamepadPair();          // look for a new pad for PAIR_WINDOW_S
  else if (cmd != "get") return;
  Serial.printf("{\"gains\":{\"kp\":%.3f,\"kd\":%.3f,\"kv\":%.3f,\"ki\":%.3f,\"kff\":%.2f,\"trim\":%.2f}}\n",
                gains.kp, gains.kd, gains.kv, gains.ki, gains.kff, toDeg(gains.trim_rad));
}

static void telemetryTask(void *) {
  String line;
  TickType_t wake = xTaskGetTickCount();
  int n = 0;
  for (;;) {
    Snapshot s;
    portENTER_CRITICAL(&snap_mux);
    s = snap;
    portEXIT_CRITICAL(&snap_mux);
    Serial.printf("{\"t\":%lu,\"p\":%.2f,\"r\":%.1f,\"v\":%.3f,\"ref\":%.2f,\"a\":%.2f,\"armed\":%d,"
                  "\"vbat\":%.2f,\"loop_us\":%lu,\"jit_us\":%lu,\"vt\":%.3f,\"yaw\":%.2f,\"pad_ms\":%ld}\n",
                  (unsigned long)s.t_ms, s.pitch_deg, s.rate_dps, s.speed_mps, s.ref_deg, s.accel_mps2,
                  s.armed, s.vbat, (unsigned long)s.loop_us, (unsigned long)s.jitter_us, s.target_mps,
                  s.turn_rps, (long)s.pad_ms);

    Event e;
    while (xQueueReceive(events, &e, 0) == pdTRUE) {
      switch (e.type) {
        case EV_ARMED: Serial.print("{\"ev\":\"armed\"}\n"); break;
        case EV_FELL: Serial.printf("{\"ev\":\"fell\",\"p\":%.1f}\n", e.a); break;
        case EV_RECOVERED: Serial.printf("{\"ev\":\"recovered\",\"peak\":%.1f,\"ms\":%.0f}\n", e.a, e.b); break;
        case EV_LOWBAT: Serial.printf("{\"ev\":\"lowbat\",\"vbat\":%.2f}\n", e.a); break;
        case EV_CUTOFF: Serial.printf("{\"ev\":\"battery_cutoff\",\"vbat\":%.2f}\n", e.a); break;
        case EV_IMU_LOST: Serial.print("{\"ev\":\"imu_lost\"}\n"); break;
        case EV_DRIVER_FAULT: Serial.print("{\"ev\":\"driver_fault\"}\n"); break;
        case EV_PAD_ON: Serial.print("{\"ev\":\"pad_connected\"}\n"); break;
        case EV_PAD_OFF: Serial.print("{\"ev\":\"pad_lost\"}\n"); break;
      }
    }

#if DRIVER_UART
    // 10 Hz driver check. Any driver that is unpowered or has reset to defaults
    // disarms the robot; one that answers but lost its settings is configured again,
    // and the robot can re-arm only after both read back as set up.
    // No pack (USB only) means unpowered drivers: skip the poll, whose unanswered reads would
    // hold this task busy and starve Bluetooth on this core. After a failed poll, poll at 1 Hz.
    int poll_every = drivers_ok ? TELEMETRY_HZ / 10 : TELEMETRY_HZ;
    if (++n % poll_every == 0 && s.vbat > 5.0f) {
      bool l_ok = tmc_l.isSetupAndCommunicating(), r_ok = tmc_r.isSetupAndCommunicating();
      if (!(l_ok && r_ok)) {
        if (s.armed) driver_fault = true;
        if (!l_ok && tmc_l.isCommunicatingButNotSetup()) configureDriver(tmc_l, TMC2209::SERIAL_ADDRESS_0);
        if (!r_ok && tmc_r.isCommunicatingButNotSetup()) configureDriver(tmc_r, TMC2209::SERIAL_ADDRESS_1);
      }
      drivers_ok = l_ok && r_ok;
    }
#endif

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        handleCommand(line);
        line = "";
      } else if (line.length() < 40) {
        line += c;
      }
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TELEMETRY_HZ));
  }
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(921600);     // native USB CDC; the baud rate is ignored
  Serial.setTxTimeoutMs(0);  // with no host attached, drop output instead of stalling
  delay(500);
  analogReadResolution(12);
  setupDrivers();  // EN high first, so the motors stay off through the rest of boot

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
  Wire.setTimeOut(3);  // ms; a hung bus must not stall the loop for long
  uint8_t who = mpuInit();
  while (who == 0x00 || who == 0xFF) {
    Serial.print("{\"error\":\"IMU not found; check SDA/SCL wiring, 3.3 V and the I2C address\"}\n");
    delay(1000);
    who = mpuInit();
  }
  if (who != 0x68)
    Serial.printf("{\"warn\":\"WHO_AM_I is 0x%02X, not 0x68: probably a clone MPU-6050; check the pitch reading before arming\"}\n", who);
  calibrateGyro();

#if DRIVER_UART
  configureDriver(tmc_l, TMC2209::SERIAL_ADDRESS_0);
  configureDriver(tmc_r, TMC2209::SERIAL_ADDRESS_1);
  drivers_ok = tmc_l.isSetupAndCommunicating() && tmc_r.isSetupAndCommunicating();
  if (!drivers_ok)
    Serial.print("{\"warn\":\"a TMC2209 is not answering on UART; check PDN_UART wiring, MS1/MS2 address pins and battery power\"}\n");
#endif

  events = xQueueCreate(8, sizeof(Event));
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, configMAX_PRIORITIES - 2, &control_handle, 1);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 2, nullptr, 0);

  hw_timer_t *control_timer = timerBegin(1000000);
  timerAttachInterrupt(control_timer, &onControlTick);
  timerAlarm(control_timer, 1000000 / CONTROL_HZ, true, 0);

  gamepadSetup();  // its own task on core 0
}

void loop() { vTaskDelete(nullptr); }  // all work happens in the tasks and timer interrupts
