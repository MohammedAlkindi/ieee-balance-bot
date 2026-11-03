// Phase 1 balance firmware for an ESP32-S3 two-wheeled robot.
//
// Core 1: a 500 Hz hardware-timer interrupt wakes the control task, which reads
//         the IMU, runs a complementary filter and the cascaded balance law, and
//         sets both wheel step rates. A 40 kHz timer interrupt turns those rates
//         into STEP pulses (phase accumulator), so speed changes apply at once.
// Core 0: telemetry task. Prints one JSON line every 20 ms plus event lines,
//         re-sends the driver config after a power cycle, and reads tuning
//         commands typed on the same USB serial port:
//           kp 30 | kd 2 | kv 0.1 | ki 0.05 | trim -1.5 | arm | disarm | get
// The control task never prints and never waits on Serial.
#include <Arduino.h>
#include <TMC2209.h>
#include <Wire.h>
#include <math.h>
#include <soc/gpio_reg.h>

#include "config.h"

// Float versions of Arduino's degrees()/radians(), which promote to double;
// the S3 FPU is single precision, so doubles would be emulated in the 500 Hz loop.
static inline float toDeg(float rad) { return rad * 57.2957795f; }
static inline float toRad(float deg) { return deg * 0.0174532925f; }

// ---------------------------------------------------------------- shared state

struct Gains {
  volatile float kp = KP_DEFAULT, kd = KD_DEFAULT, kv = KV_DEFAULT, ki = KI_DEFAULT;
  volatile float trim_rad = 0.0f;  // pitch offset so "upright" reads as zero
};
static Gains gains;

struct Snapshot {
  uint32_t t_ms;
  float pitch_deg, rate_dps, speed_mps, ref_deg, accel_mps2, vbat;
  uint32_t loop_us, jitter_us;
  bool armed;
};
static Snapshot snap;
static portMUX_TYPE snap_mux = portMUX_INITIALIZER_UNLOCKED;

enum EventType : uint8_t { EV_ARMED, EV_FELL, EV_RECOVERED, EV_LOWBAT, EV_CUTOFF, EV_IMU_LOST };
struct Event {
  EventType type;
  float a, b;
};
static QueueHandle_t events;

static volatile bool arm_request = false, disarm_request = false;
static TaskHandle_t control_handle;
static TMC2209 tmc_l, tmc_r;

// ---------------------------------------------------------------- step generator

// Each tick adds the wheel's increment to a 32-bit phase; when the phase wraps,
// that wheel takes one step. increment = steps_per_s * 2^32 / STEP_TICK_HZ.
static volatile uint32_t inc_l = 0, inc_r = 0;
static uint32_t phase_l = 0, phase_r = 0;

static void ARDUINO_ISR_ATTR onStepTick() {
  REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << PIN_STEP_L) | (1UL << PIN_STEP_R));  // end last pulse
  uint32_t high = 0, before = phase_l;
  phase_l += inc_l;
  if (phase_l < before) high |= 1UL << PIN_STEP_L;
  before = phase_r;
  phase_r += inc_r;
  if (phase_r < before) high |= 1UL << PIN_STEP_R;
  if (high) REG_WRITE(GPIO_OUT_W1TS_REG, high);  // pulse stays high for one tick (25 us)
}

static void setWheelSpeed(float speed_mps) {
  float sps = speed_mps * STEPS_PER_M;  // signed steps per second
  bool fwd = sps >= 0;
  digitalWrite(PIN_DIR_L, fwd ? HIGH : LOW);
  digitalWrite(PIN_DIR_R, (fwd != (bool)RIGHT_MOTOR_REVERSED) ? HIGH : LOW);
  uint32_t inc = (uint32_t)(fabsf(sps) * (4294967296.0f / STEP_TICK_HZ));
  inc_l = inc;
  inc_r = inc;
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

static void ARDUINO_ISR_ATTR onControlTick() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(control_handle, &woken);
  portYIELD_FROM_ISR(woken);
}

static void sendEvent(EventType type, float a = 0, float b = 0) {
  Event e{type, a, b};
  xQueueSend(events, &e, 0);  // drop rather than block the loop
}

static void controlTask(void *) {
  const float dt = 1.0f / CONTROL_HZ;
  const float alpha = COMP_FILTER_TAU_S / (COMP_FILTER_TAU_S + dt);
  const float cutoff = toRad(FALL_CUTOFF_DEG), arm_window = toRad(ARM_WINDOW_DEG);
  const float ref_max = toRad(THETA_REF_MAX_DEG);

  float pitch = 0, integ = 0, ref = 0, accel_cmd = 0, speed = 0;
  bool armed = false, disturbed = false, lowbat_sent = false;
  float peak = 0, vbat = 0;
  int64_t upright_since = -1, disturb_start = 0, inside_since = -1;
  int64_t last_us = esp_timer_get_time();
  uint32_t jitter_max = 0, loop_us = 0;
  int tick = 0, bad_reads = 0;

  float ax, ay, az, gx, gy, gz;
  if (mpuRead(ax, ay, az, gx, gy, gz)) pitch = atan2f(ACCEL_FWD, ACCEL_UP);

  auto disarm = [&](EventType why, float a) {
    motorsOff();
    speed = 0;
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
      if (armed) motorsOff();
      speed = 0;
      armed = false;
    }

    if (armed && fabsf(theta) > cutoff) {
      disarm(EV_FELL, toDeg(theta));
    } else if (armed) {
      // outer loop: lean against wheel speed so the robot stays in place
      float err_v = 0.0f - speed;
      integ = constrain(integ + err_v * dt, -2.0f, 2.0f);
      ref = constrain(gains.kv * err_v + gains.ki * integ, -ref_max, ref_max);
      // inner loop: wheel acceleration from pitch error and pitch rate
      accel_cmd = constrain(gains.kp * (theta - ref) + gains.kd * rate, -A_MAX_MPS2, A_MAX_MPS2);
      speed = constrain(speed + accel_cmd * dt, -V_MAX_MPS, V_MAX_MPS);
      setWheelSpeed(speed);

      // disturbance -> recovery bookkeeping for the voice layer
      float a = fabsf(toDeg(theta));
      if (!disturbed && a > DISTURB_DEG) {
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
      if ((held || arm_request) && fabsf(theta) < arm_window && battery_ok) {
        arm_request = false;
        integ = ref = speed = 0;
        disturbed = false;
        setWheelSpeed(0);
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
    snap = {(uint32_t)(now / 1000), toDeg(theta), toDeg(rate), speed, toDeg(ref),
            accel_cmd, vbat, loop_us, jitter_max, armed};
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
  else if (cmd == "trim") gains.trim_rad = toRad(val);
  else if (cmd == "arm") arm_request = true;
  else if (cmd == "disarm") disarm_request = true;
  else if (cmd != "get") return;
  Serial.printf("{\"gains\":{\"kp\":%.3f,\"kd\":%.3f,\"kv\":%.3f,\"ki\":%.3f,\"trim\":%.2f}}\n",
                gains.kp, gains.kd, gains.kv, gains.ki, toDeg(gains.trim_rad));
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
                  "\"vbat\":%.2f,\"loop_us\":%lu,\"jit_us\":%lu}\n",
                  (unsigned long)s.t_ms, s.pitch_deg, s.rate_dps, s.speed_mps, s.ref_deg, s.accel_mps2,
                  s.armed, s.vbat, (unsigned long)s.loop_us, (unsigned long)s.jitter_us);

    Event e;
    while (xQueueReceive(events, &e, 0) == pdTRUE) {
      switch (e.type) {
        case EV_ARMED: Serial.println("{\"ev\":\"armed\"}"); break;
        case EV_FELL: Serial.printf("{\"ev\":\"fell\",\"p\":%.1f}\n", e.a); break;
        case EV_RECOVERED: Serial.printf("{\"ev\":\"recovered\",\"peak\":%.1f,\"ms\":%.0f}\n", e.a, e.b); break;
        case EV_LOWBAT: Serial.printf("{\"ev\":\"lowbat\",\"vbat\":%.2f}\n", e.a); break;
        case EV_CUTOFF: Serial.printf("{\"ev\":\"battery_cutoff\",\"vbat\":%.2f}\n", e.a); break;
        case EV_IMU_LOST: Serial.println("{\"ev\":\"imu_lost\"}"); break;
      }
    }

    // Once a second, and only while disarmed, re-send the driver config if a
    // driver has power-cycled (battery switched on after USB).
    if (++n % TELEMETRY_HZ == 0 && !s.armed) {
      if (tmc_l.isCommunicatingButNotSetup()) configureDriver(tmc_l, TMC2209::SERIAL_ADDRESS_0);
      if (tmc_r.isCommunicatingButNotSetup()) configureDriver(tmc_r, TMC2209::SERIAL_ADDRESS_1);
    }

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
    Serial.println("{\"error\":\"IMU not found; check SDA/SCL wiring, 3.3 V and the I2C address\"}");
    delay(1000);
    who = mpuInit();
  }
  if (who != 0x68)
    Serial.printf("{\"warn\":\"WHO_AM_I is 0x%02X, not 0x68: probably a clone MPU-6050; check the pitch reading before arming\"}\n", who);
  calibrateGyro();

  configureDriver(tmc_l, TMC2209::SERIAL_ADDRESS_0);
  configureDriver(tmc_r, TMC2209::SERIAL_ADDRESS_1);
  if (!tmc_l.isSetupAndCommunicating() || !tmc_r.isSetupAndCommunicating())
    Serial.println("{\"warn\":\"a TMC2209 is not answering on UART; check PDN_UART wiring, MS1/MS2 address pins and battery power\"}");

  events = xQueueCreate(8, sizeof(Event));
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, configMAX_PRIORITIES - 2, &control_handle, 1);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 2, nullptr, 0);

  hw_timer_t *control_timer = timerBegin(1000000);
  timerAttachInterrupt(control_timer, &onControlTick);
  timerAlarm(control_timer, 1000000 / CONTROL_HZ, true, 0);
}

void loop() { vTaskDelete(nullptr); }  // all work happens in the tasks and timer interrupts
