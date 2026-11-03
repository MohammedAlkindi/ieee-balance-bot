// Pins, mechanics and starting gains. Change these, not main.cpp.
// Pin map targets an ESP32-S3-DevKitC-1 (N8R2 / N16R8). It avoids strapping
// pins (0, 3, 45, 46), native USB (19, 20) and octal PSRAM/flash pins (26-37).
#pragma once

// ---- IMU (MPU-6050 on I2C) ----
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define I2C_HZ 400000
#define MPU_ADDR 0x68  // 0x69 if AD0 is pulled high

// Which IMU axis points forward/up depends on how the board is mounted.
// Pitch = atan2(ACCEL_FWD, ACCEL_UP); pitch rate = GYRO_PITCH * GYRO_SIGN.
// Tilt the robot forward by hand: "p" in the telemetry must go positive.
#define ACCEL_FWD ax
#define ACCEL_UP az
#define GYRO_PITCH gy
#define GYRO_SIGN (-1.0f)

// ---- Steppers (TMC2209 in STEP/DIR mode, configured over one UART) ----
// Fit an external 10k pull-up from EN to 3.3 V so the drivers stay off while the S3 boots or flashes.
#define PIN_STEP_L 4
#define PIN_DIR_L 5
#define PIN_STEP_R 6
#define PIN_DIR_R 7
#define PIN_MOTOR_EN 15        // both drivers' EN, active low
#define PIN_TMC_TX 17          // to PDN_UART of both drivers
#define PIN_TMC_RX 18          // joined to TX through a 1 kOhm resistor
#define TMC_ADDR_L 0           // left driver MS1 = GND, MS2 = GND
#define TMC_ADDR_R 1           // right driver MS1 = VIO, MS2 = GND
#define MOTOR_RMS_MA 900       // 0.9-1.2 A for a 1.5-1.7 A NEMA 17; heatsink + airflow on the drivers
#define TMC_RSENSE_OHM 0.11f   // sense resistor on common TMC2209 modules; check yours
#define MICROSTEPS 16           // finer steps = smaller pitch wobble
#define STEP_TICK_HZ 40000      // step-generator interrupt rate; max step rate is half of this
#define RIGHT_MOTOR_REVERSED 1 // the motors face each other, so one spins backward

// ---- Mechanics ----
#define WHEEL_DIAMETER_M 0.090f // 85-95 mm printed wheel with a rubber O-ring or TPU tire
#define FULL_STEPS_PER_REV 200
#define STEPS_PER_M (FULL_STEPS_PER_REV * MICROSTEPS / (3.14159265f * WHEEL_DIAMETER_M))
#define V_MAX_MPS 1.0f         // wheel speed limit
#define A_MAX_MPS2 12.0f       // wheel acceleration limit (raise if motors keep up)

// ---- Battery sense: 3S pack -> 100k / 22k divider -> GPIO1 (ADC1) ----
#define PIN_VBAT 1
#define VBAT_DIVIDER ((100.0f + 22.0f) / 22.0f)
#define VBAT_WARN_V 10.8f      // 3.6 V per cell: finish the run
#define VBAT_CUTOFF_V 10.2f    // 3.4 V per cell under load: motors off

// ---- Control (units: rad, rad/s, m/s, m/s^2). Tune live over serial. ----
#define CONTROL_HZ 500
#define TELEMETRY_HZ 50
#define COMP_FILTER_TAU_S 0.5f // complementary filter time constant
#define KP_DEFAULT 40.0f       // wheel accel per rad of pitch error (sim: 30-50 all recover)
#define KD_DEFAULT 3.0f        // wheel accel per rad/s of pitch rate
#define KV_DEFAULT 0.10f       // target lean (rad) per m/s of speed error
#define KI_DEFAULT 0.05f       // target lean per (m/s * s) of speed error
#define THETA_REF_MAX_DEG 8.0f // the speed loop may lean the robot this far
#define FALL_CUTOFF_DEG 35.0f  // beyond this the motors switch off
#define ARM_WINDOW_DEG 2.0f    // hold upright within this for ARM_HOLD_MS to arm
#define ARM_HOLD_MS 500
#define DISTURB_DEG 6.0f       // a lean past this counts as a disturbance
#define RECOVER_DEG 2.0f       // back inside this for 100 ms = recovered

static_assert(V_MAX_MPS * STEPS_PER_M < STEP_TICK_HZ / 2, "top speed needs more steps/s than the step generator can make");
