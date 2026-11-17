// Straight-drive PWM sweep for the MSP432 / TI-RSLK MAX.
// Starts automatically after one second. Switch motor power off to stop.
// Right: 0..255. Left: 0.98 x right, rounded to a whole PWM value.
// Holds each step for PWM_STEP_MS, then stays at the maximum until stopped.
#include <Arduino.h>

const uint8_t LEFT_SLEEP = 31, LEFT_DIR = 29, LEFT_PWM = 40;
const uint8_t RIGHT_SLEEP = 11, RIGHT_DIR = 30, RIGHT_PWM = 39;
const uint8_t STATUS_LED = 41;
const float LEFT_TRIM = 0.98f;
const uint16_t MAX_PWM = 255;
const uint32_t PWM_STEP_MS = 100;  // +1 PWM every 0.1 s: 0 to 255 in 25.5 s.
const uint32_t START_DELAY_MS = 1000;

uint32_t startMs = 0, lastStepMs = 0;
uint16_t commandPwm = 0;
bool motorsRunning = false;

void applyPwm() {
  // Round instead of truncating: at command 255, left = 250, right = 255.
  const int leftPwm = (int)(LEFT_TRIM * commandPwm + 0.5f);
  analogWrite(LEFT_PWM, leftPwm);
  analogWrite(RIGHT_PWM, commandPwm);
}

void setup() {
  pinMode(LEFT_SLEEP, OUTPUT);
  pinMode(RIGHT_SLEEP, OUTPUT);
  digitalWrite(LEFT_SLEEP, LOW);
  digitalWrite(RIGHT_SLEEP, LOW);

  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  digitalWrite(LEFT_DIR, LOW);  // LOW = forward on the stock RSLK.
  digitalWrite(RIGHT_DIR, LOW);

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  startMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();
  if (!motorsRunning) {
    const uint32_t elapsedMs = nowMs - startMs;
    if (elapsedMs < START_DELAY_MS) {
      digitalWrite(STATUS_LED, (elapsedMs / 250) % 2);
      return;
    }

    commandPwm = 0;
    applyPwm();
    digitalWrite(LEFT_SLEEP, HIGH);
    digitalWrite(RIGHT_SLEEP, HIGH);
    digitalWrite(STATUS_LED, HIGH);
    lastStepMs = nowMs;
    motorsRunning = true;
    return;
  }

  if (commandPwm < MAX_PWM && nowMs - lastStepMs >= PWM_STEP_MS) {
    // Advance only one step so a delayed loop never skips a PWM value.
    commandPwm+=8;
    lastStepMs = nowMs;
    applyPwm();
  }
  // At 255, PWM hardware holds the final commands until motor power is off.
}
