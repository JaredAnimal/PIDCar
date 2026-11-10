// ECE3 line follower (PID): turn at the first confirmed bar and stop at the second.
// Starts automatically after a one-second countdown. Reset for another run.
// Gains and thresholds need track testing.
#include <Arduino.h>
#include <ECE3.h>
#include <math.h>

const uint8_t LEFT_SLEEP = 31, LEFT_DIR = 29, LEFT_PWM = 40;
const uint8_t RIGHT_SLEEP = 11, RIGHT_DIR = 30, RIGHT_PWM = 39;
const uint8_t START_BUTTON = 73, STOP_BUTTON = 74;
const uint8_t STATUS_LED = 41;

// First parameters to tune. PWM uses 0..255, not percent.
const float BASE_PWM = 45.0f;
const float STEERING_ERROR_PERCENT = 150.0f; // CHANGED from 50: 50 gave KP ~3 in sensor units, too weak to stay centered.
const float KD_TO_KP_RATIO = 7.5f;          // Professor's starting range is 5..10.
// ADDED (PID): integral term. It builds up while the car stays on one side of
// the line (a long or sharp curve), adding extra turn the longer the curve lasts.
const float KI_TO_KP_RATIO = 3.0f;          // KI = 3 x KP, per second. Try 1..6.
const float INTEGRAL_MAX_TURN = 25.0f;      // Most PWM the I term can ever add.
const float INTEGRAL_DEADBAND = 80.0f;      // CHANGED from 300: now also removes a steady offset on straights.
const float CENTER_OFFSET = 0.48509f; // From the supplied centered sample.
const float STEERING_SIGN = 1.0f;
// ADDED: motor trim. If the car drifts to one side on a straight, lower the
// stronger side slightly (e.g. 0.95). 1.0 = no change.
const float LEFT_TRIM = 1.0f;
const float RIGHT_TRIM = 1.0f;
// ADDED: centering check. true = motors OFF, prints error over Serial (9600).
// Put the car exactly centered on the line: error should read about 0.
const bool CENTER_CHECK = false;
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
const uint8_t BAR_SENSOR_COUNT = 8;   // All eight sensors must see black.
const uint8_t BAR_CONFIRM_READS = 2;  // Reject a one-sample all-black glitch.
const bool STOP_AT_BAR = true;        // ADDED: set false to disable stopping entirely (testing only)

// ADDED: 180-degree turnaround at the FIRST bar, stop at the SECOND bar.
const int TURN_PWM = 60;              // Spin speed for the turnaround.
const long TURN_COUNTS = 360;         // Encoder counts per wheel for 180 deg. INITIAL GUESS: tune on the floor.
const uint32_t TURN_TIMEOUT_MS = 2000;// Backstop if the encoders don't count.
const long REARM_COUNTS = 300;        // Distance to drive after turning before the next bar can count.
const uint8_t CLEAR_READS_TO_ARM = 10;

// S1..S8 in the spreadsheet = raw indices 0..7 in the calibration sketch.
// The supplied starter identifies that order as RIGHT to LEFT.
const int SENSOR_MIN[8] = {772, 657, 612, 612, 567, 612, 635, 797};
const int SENSOR_MAX[8] = {2500, 2469, 2500, 1934, 2027, 1911, 2500, 2500};
const int POSITION[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

// Scale PD to a maximum absolute error of 2000 for the professor's gain rule.
// POSITION is ordered, so its endpoints bound every possible centroid.
// Curve slowing and recovery continue to use the original centroid units.
const float MAX_CENTROID_ERROR =
    fmaxf(fabsf(float(POSITION[0]) - CENTER_OFFSET),
          fabsf(float(POSITION[7]) - CENTER_OFFSET));
const float PD_ERROR_SCALE = 2000.0f / MAX_CENTROID_ERROR;
const float KP = (STEERING_ERROR_PERCENT / 100.0f) * BASE_PWM / 2000.0f;
const float KD = KD_TO_KP_RATIO * KP; // Error change per nominal 6 ms.
const float KI = KI_TO_KP_RATIO * KP; // ADDED (PID): per second of accumulated error.
const float INTEGRAL_LIMIT = INTEGRAL_MAX_TURN / KI; // Anti-windup clamp.

enum RunState { WAITING, START_DELAY, TRACKING, TURNING, FINISHED, FAULT };
RunState state = WAITING;
uint16_t raw[8];
uint32_t buttonChangedMs = 0, startMs = 0, lastSeenMs = 0, lastSampleUs = 0;
bool lastButtonDown = false, startPressSeen = false;
bool finishArmed = false, derivativeReady = false;
uint8_t clearReads = 0, barReads = 0;
float previousError = 0.0f, filteredDelta = 0.0f, lastTurn = 0.0f;
uint8_t barsSeen = 0;                 // ADDED: 0 = going out, 1 = coming back
uint32_t turnStartMs = 0;
float integral = 0.0f;                // ADDED (PID): accumulated error x seconds

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
  digitalWrite(dirPin, cmd < 0.0f ? HIGH : LOW); // LOW = forward on stock RSLK
  int pwm = (int)fabsf(cmd);
  if (pwm > 255) pwm = 255;
  analogWrite(pwmPin, pwm);
}

// CHANGED: takes a base speed, and allows the inner wheel to reverse on sharp turns.
void drive(float turn, float base) {
  // Positive error = line to the left: slow the left wheel, speed up the right.
  turn = limitFloat(turn, -MAX_TURN, MAX_TURN);
  setWheel(LEFT_DIR, LEFT_PWM, (base - turn) * LEFT_TRIM);    // CHANGED: trim
  setWheel(RIGHT_DIR, RIGHT_PWM, (base + turn) * RIGHT_TRIM);
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
}

// ADDED: encoder helpers (ECE3 library)
long encoderAverage() {
  long l = getEncoderCount_left(), r = getEncoderCount_right();
  if (l < 0) l = -l;
  if (r < 0) r = -r;
  return (l + r) / 2;
}
void resetEncoders() {
  resetEncoderCount_left();
  resetEncoderCount_right();
}

// ADDED: start spinning in place (left wheel back, right wheel forward).
void startTurn(uint32_t nowMs) {
  resetEncoders();
  setWheel(LEFT_DIR, LEFT_PWM, -TURN_PWM);
  setWheel(RIGHT_DIR, RIGHT_PWM, TURN_PWM);
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
  turnStartMs = nowMs;
  state = TURNING;
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
  Serial.begin(9600);    // ADDED: for CENTER_CHECK
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

  // ADDED: turnaround in progress. Non-blocking: check encoders each loop.
  if (state == TURNING) {
    digitalWrite(STATUS_LED, (nowMs / 100) % 2);
    if (encoderAverage() >= TURN_COUNTS || nowMs - turnStartMs >= TURN_TIMEOUT_MS) {
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, 0);
      digitalWrite(LEFT_DIR, LOW);     // both forward again
      digitalWrite(RIGHT_DIR, LOW);
      delay(100);                      // brief settle so the car doesn't lurch
      resetEncoders();                 // measure distance driven since the turn
      finishArmed = false;
      clearReads = barReads = 0;
      derivativeReady = false;
      filteredDelta = 0.0f;
      integral = 0.0f;                 // ADDED (PID)
      lastTurn = 0.0f;
      lastSampleUs = micros();
      state = TRACKING;
    }
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
  for (uint8_t i = 0; i < 8; ++i) {
    // Signed, wide arithmetic prevents underflow and early integer truncation.
    int value = (int32_t(raw[i]) - SENSOR_MIN[i]) * 1000L /
                (SENSOR_MAX[i] - SENSOR_MIN[i]);
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;
    sum += value;
    weighted += POSITION[i] * value;
    if (value > peak) peak = value;
    if (value >= BAR_DARK_LEVEL) ++darkSensors;
  }

  const bool lineSeen = peak >= MIN_LINE_PEAK && sum >= MIN_LINE_SUM;
  // Require all eight normalized sensors to see black at once.
  const bool crosspiece = darkSensors >= BAR_SENSOR_COUNT;
  if (crosspiece) {
    clearReads = 0;
    derivativeReady = false;
    filteredDelta = 0.0f;
    integral = 0.0f;                   // ADDED (PID)
    // A lone all-dark glitch must not produce a steering spike or a finish.
    if (finishArmed && STOP_AT_BAR) {
      if (++barReads >= BAR_CONFIRM_READS) {
        // CHANGED: first bar = turn around, second bar = stop.
        if (barsSeen == 0) {
          barsSeen = 1;
          startTurn(nowMs);
        } else {
          stopRun(true);
        }
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
    drive(lastTurn >= 0.0f ? MAX_TURN : -MAX_TURN, MIN_BASE_PWM);
    return;
  }

  lastSeenMs = nowMs;
  // Ignore a starting bar. Arm the finish only after ten valid narrow readings.
  // CHANGED: after the turnaround, also require REARM_COUNTS of driving,
  // so the car can't re-trigger on the bar it just turned on.
  if (!finishArmed && ++clearReads >= CLEAR_READS_TO_ARM &&
      (barsSeen == 0 || encoderAverage() >= REARM_COUNTS)) finishArmed = true;
  const float error = float(weighted) / float(sum) - CENTER_OFFSET;

  // ADDED: centering check mode. No driving; print the error 5x per second.
  if (CENTER_CHECK) {
    stopMotors();
    static uint32_t lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 200) {
      lastPrintMs = nowMs;
      Serial.print("error = ");
      Serial.println(error, 3);
    }
    return;
  }
  float delta = 0.0f;
  if (derivativeReady) {
    delta = (error - previousError) * (float(SAMPLE_US) / float(elapsedUs));
  }
  filteredDelta += 0.35f * (delta - filteredDelta);
  previousError = error;
  derivativeReady = true;
  const float pdError = error * PD_ERROR_SCALE;
  const float pdDelta = filteredDelta * PD_ERROR_SCALE;

  // ADDED (PID): integral term with three safety rules.
  // 1. Only accumulate when clearly off-center (in a curve, not on a straight).
  // 2. Reset to zero the moment the error changes side, so it never fights
  //    the car on the way out of a curve (no overshoot).
  // 3. Clamp it, and stop adding while steering is already maxed (anti-windup).
  const float dt = float(elapsedUs) * 1e-6f;
  if ((pdError > 0.0f && integral < 0.0f) || (pdError < 0.0f && integral > 0.0f)) {
    integral = 0.0f;
  }
  const bool saturated = fabsf(lastTurn) >= MAX_TURN &&
                         ((lastTurn > 0.0f) == (STEERING_SIGN * pdError > 0.0f));
  if (fabsf(pdError) > INTEGRAL_DEADBAND && !saturated) {
    integral += pdError * dt;
  }
  integral = limitFloat(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  lastTurn = STEERING_SIGN * (KP * pdError + KI * integral + KD * pdDelta);
  lastTurn = limitFloat(lastTurn, -MAX_TURN, MAX_TURN);   // CHANGED: was +/-BASE_PWM

  // ADDED: slow down the bigger the error, so the car can make tight curves.
  float base = BASE_PWM - CURVE_SLOW * fabsf(error);
  if (base < MIN_BASE_PWM) base = MIN_BASE_PWM;
  drive(lastTurn, base);
}
