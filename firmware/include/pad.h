// Link between the gamepad code (gamepad.cpp) and the balance firmware (main.cpp).
#pragma once
#include <stdint.h>

// main.cpp: safe to call from any task.
// Stick fractions -1..1 (+ = forward, + = turn right) and the esp_timer time the report arrived.
void padReport(float speed, float turn, int64_t stamp_us);
void padArm();                       // same as typing "arm": arms if upright and the stick is centred
void padDisarm();                    // same as typing "disarm": motors off until the next arm
void padConnection(bool connected);  // connect/lose events; losing the pad centres the stick
bool robotArmed();
extern volatile bool pad_pairing;  // set by gamepad.cpp; the robot will not arm while it is true

// gamepad.cpp
void gamepadSetup();       // once, from setup(), after the balance tasks exist
void gamepadForgetKeys();  // drop every stored pairing and open a pairing window (ignored while armed)
void gamepadPair();        // open a pairing window (only while disarmed)
