// ECE3 slow starting point: follow one black line and stop at the end bar.
// S1: press and release to start. S2: stop. Reset the board for another run.
// See README.md before the first run. Gains and thresholds need track testing.
#include <Arduino.h>
#include <ECE3.h>

const uint8_t LEFT_SLEEP = 31, LEFT_DIR = 29, LEFT_PWM = 40;
const uint8_t RIGHT_SLEEP = 11, RIGHT_DIR = 30, RIGHT_PWM = 39;
const uint8_t START_BUTTON = 73, STOP_BUTTON = 74;
const uint8_t STATUS_LED = 41;

// First parameters to tune. PWM uses 0..255, not percent.
const int BASE_PWM = 45;
const float KP = 10.0f;  // CHANGED from 5.0: stronger turning on curves (try 8-12)
const float KD = 35.0f;  // Error difference per nominal 6 ms, NOT per second.
                         // Raise to 35-50 if the car wobbles after raising KP.
const float CENTER_OFFSET = 0.48509f; // From the supplied centered sample.
const float STEERING_SIGN = 1.0f;
// ADDED for sharp turns:
const float MAX_TURN = 70.0f;     // Turn limit. Above BASE_PWM lets the inner wheel spin BACKWARD (pivot).
const float CURVE_SLOW = 3.0f;    // PWM removed from base speed per unit of |error| (slow down in curves).
const float MIN_BASE_PWM = 25.0f; // Never slow below this in a curve.    // Change only if verified steering is reversed.
const uint32_t SAMPLE_US = 6000;
const uint32_t LOST_LINE_MS = 150;   // Short dropout allowance, not gap navigation.
const uint32_t MAX_RUN_MS = 120000;  // Backstop; increase if the slow course needs it.
const int MIN_LINE_PEAK = 220;       // Normalized 0..1000. Initial guess.
const int MIN_LINE_SUM = 300;
const int BAR_DARK_LEVEL = 700;
const uint8_t BAR_SENSOR_COUNT = 7;   // CHANGED from 6: curves were triggering it
const uint8_t BAR_CONFIRM_READS = 3;  // CHANGED from 2: bar must be seen 3 reads in a row
const bool STOP_AT_BAR = true;        // ADDED: set false to disable stopping entirely (testing only)
const uint8_t CLEAR_READS_TO_ARM = 10;

// S1..S8 in the spreadsheet = raw indices 0..7 in the calibration sketch.
// The supplied starter identifies that order as RIGHT to LEFT.
const int SENSOR_MIN[8] = {772, 657, 612, 612, 567, 612, 635, 797};
const int SENSOR_MAX[8] = {2500, 2469, 2500, 1934, 2027, 1911, 2500, 2500};
const int POSITION[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

enum RunState { WAITING, START_DELAY, TRACKING, FINISHED, FAULT };
RunState state = WAITING;
uint16_t raw[8];
uint32_t buttonChangedMs = 0, startMs = 0, lastSeenMs = 0, lastSampleUs = 0;
bool lastButtonDown = false, startPressSeen = false;
bool finishArmed = false, derivativeReady = false;
uint8_t clearReads = 0, barReads = 0;
float previousError = 0.0f, filteredDelta = 0.0f, lastTurn = 0.0f;

float limitFloat(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_DIR, LOW);   // back to forward
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_SLEEP, LOW);
  digitalWrite(RIGHT_SLEEP, LOW);
}

// CHANGED: one wheel command; negative = run that wheel backward.
void setWheel(uint8_t dirPin, uint8_t pwmPin, float cmd) {
  digitalWrite(dirPin, cmd < 0 ? HIGH : LOW);   // LOW = forward on stock RSLK
  int pwm = (int)(cmd < 0 ? -cmd : cmd);
  if (pwm > 255) pwm = 255;
  analogWrite(pwmPin, pwm);
}

// CHANGED: takes a base speed, and allows the inner wheel to reverse on sharp turns.
void drive(float turn, float base) {
  // Positive error = line to the left: slow the left wheel, speed up the right.
  turn = limitFloat(turn, -MAX_TURN, MAX_TURN);
  setWheel(LEFT_DIR, LEFT_PWM, base - turn);
  setWheel(RIGHT_DIR, RIGHT_PWM, base + turn);
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
}

void stopRun(bool success) {
  stopMotors();
  state = success ? FINISHED : FAULT;
}

void setup() {
  pinMode(LEFT_SLEEP, OUTPUT);
  pinMode(RIGHT_SLEEP, OUTPUT);
  digitalWrite(LEFT_SLEEP, LOW);
  digitalWrite(RIGHT_SLEEP, LOW);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  // Stock RSLK motor driver: LOW means forward on both sides.
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  stopMotors();
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  ECE3_Init();
  stopMotors();
  state = START_DELAY;   // skip the button wait
  startMs = millis();    // start the delay timer
}

void loop() {
  uint32_t nowMs = millis();

  // LED: slow blink ready/countdown, on running/finished, fast blink stopped/fault.
  if (state == FAULT) {
    digitalWrite(STATUS_LED, (nowMs / 150) % 2);
    return;
  }
  if (state == FINISHED) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }
  if (state == WAITING) {
    digitalWrite(STATUS_LED, (nowMs / 500) % 2);
    const bool down = digitalRead(START_BUTTON) == LOW;
    if (down != lastButtonDown) {
      lastButtonDown = down;
      buttonChangedMs = nowMs;
    }
    if (nowMs - buttonChangedMs >= 30) {
      if (down) startPressSeen = true;
      else if (startPressSeen) {
        state = START_DELAY;
        startMs = nowMs;
      }
    }
    return;
  }
  if (state == START_DELAY) {
    digitalWrite(STATUS_LED, (nowMs / 500) % 2);
    if (nowMs - startMs < 1000) return;
    state = TRACKING;
    startMs = lastSeenMs = nowMs;
    lastSampleUs = micros();
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastSampleUs;
  if (elapsedUs < SAMPLE_US) return;  // No delay() and no catch-up bursts.
  lastSampleUs = nowUs;
  ECE3_read_IR(raw);
  nowMs = millis();
  

  digitalWrite(STATUS_LED, HIGH);

  int sum = 0, peak = 0, weighted = 0;
  uint8_t darkSensors = 0;
  bool edgeDark[8] = {false};
  for (uint8_t i = 0; i < 8; ++i) {
    // Signed, wide arithmetic prevents underflow and early integer truncation.
    int value = (int32_t(raw[i]) - SENSOR_MIN[i]) * 1000L /
                (SENSOR_MAX[i] - SENSOR_MIN[i]);
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;
    sum += value;
    weighted += POSITION[i] * value;
    if (value > peak) peak = value;
    if (value >= BAR_DARK_LEVEL) { ++darkSensors; edgeDark[i] = true; }
  }

  const bool lineSeen = peak >= MIN_LINE_PEAK && sum >= MIN_LINE_SUM;
  // CHANGED: a real crossbar covers BOTH outermost sensors at once.
  // A sharp curve comes in from one side, so it fails this check.
  const bool crosspiece = darkSensors >= BAR_SENSOR_COUNT && edgeDark[0] && edgeDark[7];
  if (crosspiece) {
    clearReads = 0;
    derivativeReady = false;
    filteredDelta = 0.0f;
    // A lone all-dark glitch must not produce a steering spike or a finish.
    if (finishArmed && STOP_AT_BAR) {
      if (++barReads >= BAR_CONFIRM_READS) {
        stopRun(true);
        return;
      }
    }
    lastSeenMs = nowMs;
    drive(lastTurn, BASE_PWM);
    return;
  }
  barReads = 0; // Confirmation must be consecutive.

  if (!lineSeen) {
    clearReads = 0;
    derivativeReady = false;
    filteredDelta = 0.0f;
    // CHANGED: was limitFloat(lastTurn, -15, 15), which weakened the turn
    // exactly when the car slid off a curve. Now: pivot hard toward the side
    // the line was last seen, at low speed, until the line comes back.
    drive(lastTurn >= 0 ? MAX_TURN : -MAX_TURN, MIN_BASE_PWM);
    return;
  }

  lastSeenMs = nowMs;
  // Ignore a starting bar. Arm the finish only after ten valid narrow readings.
  if (!finishArmed && ++clearReads >= CLEAR_READS_TO_ARM) finishArmed = true;
  const float error = float(weighted) / float(sum) - CENTER_OFFSET;
  float delta = 0.0f;
  if (derivativeReady) {
    delta = (error - previousError) * (float(SAMPLE_US) / float(elapsedUs));
  }
  filteredDelta += 0.35f * (delta - filteredDelta);
  previousError = error;
  derivativeReady = true;
  lastTurn = STEERING_SIGN * (KP * error + KD * filteredDelta);
  lastTurn = limitFloat(lastTurn, -MAX_TURN, MAX_TURN);   // CHANGED: was +/-BASE_PWM

  // ADDED: slow down the bigger the error, so the car can make tight curves.
  float base = BASE_PWM - CURVE_SLOW * fabs(error);
  if (base < MIN_BASE_PWM) base = MIN_BASE_PWM;
  drive(lastTurn, base);
}
