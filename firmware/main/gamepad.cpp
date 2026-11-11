// Gamepad drive through Bluepad32. The ESP32-S3 only has Bluetooth LE, so only BLE pads
// connect (an Xbox Wireless controller on firmware 5.x or newer; not PS4, PS5 or Switch).
//
// One task on core 0 makes every Bluepad32 call, so the balance loop on core 1 never waits
// on Bluetooth. Pairing writes the bond to flash, and while flash is written code in flash
// pauses on both cores, including the control task. So the robot only looks for pads in a
// pairing window (PAIR_WINDOW_S after boot, after "pair" or "forget", and after the pad is
// lost), only while disarmed, and it cannot arm until PAIR_SETTLE_MS after the window closes.
// The window closes early once a pad connects. A connected pad keeps working when armed.
#include <Arduino.h>
#include <Bluepad32.h>

#include "config.h"
#include "pad.h"

static ControllerPtr pad = nullptr;  // the one pad that drives; any other is sent away
static volatile bool forget_request = false, pair_request = false;

static void onConnected(ControllerPtr c) {
  if (pad == nullptr) {
    pad = c;
    padConnection(true);
  } else {
    c->disconnect();
  }
}

static void onDisconnected(ControllerPtr c) {
  if (c != pad) return;
  pad = nullptr;
  padConnection(false);
}

// Bluepad32 sticks read about -512..511; up is negative on Y, right is positive on X.
static float stick(int32_t v) { return constrain(v / 512.0f, -1.0f, 1.0f); }

static void gamepadTask(void *) {
  const int64_t window_us = PAIR_WINDOW_S * 1000000LL, settle_us = PAIR_SETTLE_MS * 1000LL;
  bool scanning = true, had_pad = false, a_was = false, b_was = false;
  int64_t window_end = esp_timer_get_time() + window_us;  // scanning starts in gamepadSetup()
  int64_t scan_stopped = -1;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    BP32.update();  // runs onConnected/onDisconnected in this task
    int64_t now = esp_timer_get_time();
    bool armed = robotArmed(), connected = pad != nullptr;

    if (connected) had_pad = true;
    if (forget_request && !armed) {
      BP32.forgetBluetoothKeys();  // writes flash: never while armed
      pair_request = true;
    }
    forget_request = false;
    // reopen the window on request, or once disarmed after the pad was lost
    if ((pair_request || (had_pad && !connected)) && !armed) {
      window_end = now + window_us;
      had_pad = false;
    }
    pair_request = false;

    bool want_scan = !armed && !connected && now < window_end;
    if (want_scan != scanning) {
      scanning = want_scan;
      BP32.enableNewBluetoothConnections(scanning);
      if (!scanning) scan_stopped = now;
    }
    // a connection that started before the scan stopped can still be pairing; wait it out
    pad_pairing = scanning || (scan_stopped >= 0 && now - scan_stopped < settle_us);

    // hasData() is true only when a new report arrived since the last update(), so the
    // report time doubles as the dead-man: the balance loop ignores reports that are too old.
    if (pad && pad->isConnected() && pad->isGamepad() && pad->hasData()) {
      padReport(-stick(pad->axisY()), stick(pad->axisX()), now);
      bool a = pad->a(), b = pad->b();
      if (b && !b_was) padDisarm();  // B: stop button
      if (a && !a_was && !b) padArm();
      a_was = a;
      b_was = b;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

void gamepadSetup() {
  // setup() runs once Bluetooth is up. Scanning starts here; pads already paired reconnect.
  pad_pairing = true;
  BP32.setup(&onConnected, &onDisconnected, true);
  BP32.enableVirtualDevice(false);  // no-ops when already off, so no flash write
  BP32.enableBLEService(false);
  xTaskCreatePinnedToCore(gamepadTask, "gamepad", 4096, nullptr, 1, nullptr, 0);
}

void gamepadForgetKeys() { forget_request = true; }
void gamepadPair() { pair_request = true; }
