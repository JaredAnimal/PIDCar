// U track: filtered PD, sensor-triggered braking, and fading right-pivot help.
// Automatic start after one second. Stop manually with motor power.
#include <Arduino.h>
#include <ECE3.h>
#include <math.h>

const float CENTER_OFFSET = -0.0565749235f;
const float STEERING_SIGN = 1.0f;
const float LEFT_TRIM = 0.98f; // Left motor is faster; apply once after steering.

const float STRAIGHT_PWM = 255.0f; // Maximum reached after steady tracking.
const float UNSETTLED_PWM = 160.0f;
const float SIGNAL_LOSS_PWM = 80.0f;
const float START_PWM = 100.0f;
const float ACCEL_STEP_PWM = 6.0f; // Per nominal 6 ms sample; previous trial used 4.
const float MAX_STRAIGHT_TURN = 40.0f;
const uint8_t STABLE_READS_FOR_FAST = 12; //DECREASE to 8 from 12

const float BEND_PWM = 80.0f; // Curve-only trial; straight settings stay as tested.
// Hold the steering reference at 70 while increasing forward command to 80.
// Do not automatically increase steering aggressiveness with curve speed.
const float BEND_TUNING_PWM = 70.0f;
const float BEND_COMMAND_SCALE = BEND_TUNING_PWM / 45.0f;
const float BEND_P_GAIN_SCALE = 0.90f; // 10% less P for observed right overshoot.
const float PIVOT_HELP_SCALE = 0.90f;  // Soften the pivot floor as well as P.
const float RIGHT_TURN_ERROR = 4.0f; // Less sensitive than the original 1.5.
const int BEND_SENSOR_LEVEL = 650; // Stronger right-side signal than original 500.
const uint8_t BEND_CONFIRM_READS = 2;
const int EDGE_BOOST_START = 400; // Normalized darkness where extra weight begins.
const float REVERSE_TURN = 70.0f * BEND_COMMAND_SCALE * PIVOT_HELP_SCALE;
const float PIVOT_RELEASE_ERROR = 0.8f; // Release help earlier; previous trial used 0.5.
const float PIVOT_FULL_ERROR = 3.5f;    // Full help with the line far to the right.
const float MAX_BEND_TURN = 110.0f * BEND_COMMAND_SCALE;
const uint8_t STRAIGHT_READS_AFTER_BEND = 12;//DECRESE TO 12 from 20

const float STRAIGHT_ERROR_PERCENT = 25.0f;
const float BEND_ERROR_PERCENT = 50.0f; // Reference proportional tuning at 45 PWM.
const float STRAIGHT_KD_TO_KP_RATIO = 7.5f;
const float BEND_REFERENCE_KD_TO_KP_RATIO = 7.5f;
const float BEND_D_GAIN_SCALE = 1.10f; // Retain damping while reducing right-turn force.
const float DERIVATIVE_ALPHA = 0.35f;
const uint32_t SAMPLE_US = 6000, START_DELAY_MS = 1000;
const uint8_t LEFT_SLEEP = 31, LEFT_DIR = 29, LEFT_PWM = 40;
const uint8_t RIGHT_SLEEP = 11, RIGHT_DIR = 30, RIGHT_PWM = 39;
const uint8_t STATUS_LED = 41;

// Raw indices run from right to left.
const int SENSOR_MIN[8] = {771, 759, 681, 726, 521, 612, 657, 703};
const int SENSOR_MAX[8] = {2500, 2500, 2500, 2091, 1909, 2500, 2500, 2500};
const int ALIGN_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
const int TURN_WEIGHTS[8] = {-35, -12, -3, -1, 1, 3, 12, 35};
// Use the alignment scale for both errors so the edge boost adds authority.
const float MAX_ERROR = fmaxf(fabsf(float(ALIGN_WEIGHTS[0]) - CENTER_OFFSET),
                             fabsf(float(ALIGN_WEIGHTS[7]) - CENTER_OFFSET));
const float ERROR_SCALE = 2000.0f / MAX_ERROR;
const float STRAIGHT_KP = (STRAIGHT_ERROR_PERCENT / 100.0f) * STRAIGHT_PWM / 2000.0f;
const float STRAIGHT_KD = STRAIGHT_KD_TO_KP_RATIO * STRAIGHT_KP;
const float BEND_REFERENCE_KP = (BEND_ERROR_PERCENT / 100.0f) * STRAIGHT_PWM / 2000.0f;
const float BEND_KP = BEND_REFERENCE_KP * BEND_COMMAND_SCALE * BEND_P_GAIN_SCALE;
// D has its own scale so bend damping can be tuned independently of speed/P.
// User observed right overshoot before left correction; hold D for this trial.
const float BEND_KD = BEND_REFERENCE_KD_TO_KP_RATIO * BEND_REFERENCE_KP * BEND_D_GAIN_SCALE;

uint16_t raw[8];
uint32_t startMs = 0, lastSampleUs = 0;
uint8_t leftDirection = LOW, rightDirection = LOW;
uint8_t stableReads = 0, bendReads = 0;
bool bendSeen = false, bendDone = false, derivativeReady = false;
bool edgeWeightsArmed = false;
bool bendCandidateSeen = false;
uint32_t lastBendCandidateMs = 0;
float previousError = 0.0f, filteredDelta = 0.0f, lastTurn = 0.0f;
float basePwm = START_PWM;

float limitFloat(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

void updateStatusLed(uint32_t nowMs, bool validLine) {
  // Controller state only: "done" does not prove a physical U-turn occurred.
  // Slow blink = last turn finished, detection still enabled.
  // Fast blink = confirmed turn active.
  // Double flash = candidate, held visible for 800 ms after the last candidate.
  // Solid = valid line, detection available; off = no valid line before a turn.
  bool on;
  if (bendDone) {
    on = (nowMs % 1000) < 500;
  } else if (bendSeen) {
    on = (nowMs % 200) < 100;
  } else if (bendCandidateSeen &&
             uint32_t(nowMs - lastBendCandidateMs) < 800) {
    const uint32_t phase = uint32_t(nowMs - lastBendCandidateMs);
    on = phase < 100 || (phase >= 200 && phase < 300);
  } else {
    on = validLine;
  }
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
}

float bendProportionalTurn(float proportionalTurn, float alignmentError) {
  // Fade the minimum right-pivot request smoothly using the regular centroid.
  // This is independent of the sharpRight detection threshold, so moving
  // between two sensor channels cannot switch the pivot help on/off abruptly.
  const float rightError = -STEERING_SIGN * alignmentError;
  float assistance = limitFloat((rightError - PIVOT_RELEASE_ERROR) /
      (PIVOT_FULL_ERROR - PIVOT_RELEASE_ERROR), 0.0f, 1.0f);
  assistance = assistance * assistance * (3.0f - 2.0f * assistance);
  const float assistedTurn = fminf(proportionalTurn, -REVERSE_TURN);
  return proportionalTurn + assistance * (assistedTurn - proportionalTurn);
}

void setWheel(uint8_t dirPin, uint8_t pwmPin, uint8_t &direction, float command) {
  const uint8_t nextDirection = command < 0.0f ? HIGH : LOW;
  if (nextDirection != direction) {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, nextDirection);
    direction = nextDirection;
  }
  const float magnitude = limitFloat(fabsf(command), 0.0f, 255.0f);
  analogWrite(pwmPin, (int)(magnitude + 0.5f));
}

void drive(float turn, float base) {
  // Keep the requested steering differential inside the PWM range.
  base = fminf(base, 255.0f - fabsf(turn));
  setWheel(LEFT_DIR, LEFT_PWM, leftDirection, (base - turn) * LEFT_TRIM);
  setWheel(RIGHT_DIR, RIGHT_PWM, rightDirection, base + turn);
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
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
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  pinMode(STATUS_LED, OUTPUT);
  ECE3_Init();
  startMs = millis();
  lastSampleUs = micros();
}

void loop() {
  const uint32_t sinceStartMs = millis() - startMs;
  if (sinceStartMs < START_DELAY_MS) {
    digitalWrite(STATUS_LED, LOW); // Off during the one-second startup delay.
    return;
  }
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastSampleUs;
  if (elapsedUs < SAMPLE_US) return;
  lastSampleUs = nowUs;

  bool slow = bendSeen && !bendDone;
  ECE3_read_IR(raw);
  int sum = 0, weighted = 0, turnWeighted = 0, peak = 0, rightShoulderPeak = 0;
  uint8_t darkSensors = 0, lineSensors = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    int value = (int32_t(raw[i]) - SENSOR_MIN[i]) * 1000L /
                (SENSOR_MAX[i] - SENSOR_MIN[i]);
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;
    sum += value;
    weighted += ALIGN_WEIGHTS[i] * value;
    // Ramp the extra edge weight above the noise guard. At darkness 1000
    // each channel reaches its TURN_WEIGHTS value.
    const int edgeStrength = value > EDGE_BOOST_START ?
        (value - EDGE_BOOST_START) * 1000 / (1000 - EDGE_BOOST_START) : 0;
    turnWeighted += ALIGN_WEIGHTS[i] * value +
                    (TURN_WEIGHTS[i] - ALIGN_WEIGHTS[i]) * edgeStrength;
    if (value > peak) peak = value;
    if (i <= 2 && value > rightShoulderPeak) rightShoulderPeak = value;
    if (value >= 700) ++darkSensors;
    if (value >= 220) ++lineSensors;
  }

  const bool validLine = peak >= 220 && sum >= 300 && darkSensors < 8;
  if (validLine) {
    const float alignmentError = float(weighted) / float(sum) - CENTER_OFFSET;
    const float error = edgeWeightsArmed ?
        float(turnWeighted) / float(sum) - CENTER_OFFSET : alignmentError;
    const bool sharpRight = STEERING_SIGN * alignmentError <= -RIGHT_TURN_ERROR &&
                            rightShoulderPeak >= BEND_SENSOR_LEVEL;
    const bool bendCandidate = sharpRight; // Allow subsequent turns after settling.
    // Brake on the first candidate. Confirm a second valid right-side
    // pattern before allowing the right wheel to reverse.
    if (bendCandidate) {
      bendDone = false;
      slow = true;
      bendCandidateSeen = true;
      lastBendCandidateMs = sinceStartMs;
    }
    if (bendCandidate && !bendSeen) {
      if (++bendReads >= BEND_CONFIRM_READS) {
        bendSeen = true;
        stableReads = 0;
      }
    } else {
      bendReads = 0;
    }

    float delta = 0.0f;
    if (derivativeReady) {
      delta = (error - previousError) * (float(SAMPLE_US) / float(elapsedUs));
    }
    filteredDelta += DERIVATIVE_ALPHA * (delta - filteredDelta);
    previousError = error;
    derivativeReady = true;
    const bool inBend = slow && bendSeen;
    const float kp = inBend ? BEND_KP : STRAIGHT_KP;
    const float kd = inBend ? BEND_KD : STRAIGHT_KD;
    float proportionalTurn = STEERING_SIGN * kp * error * ERROR_SCALE;
    if (inBend) {
      proportionalTurn = bendProportionalTurn(proportionalTurn, alignmentError);
    }
    // Add D AFTER the pivot help, so damping can ease the right turn while
    // the line is still on the right. Do not force a minimum on the final PD
    // output: that previously overrode damping and encouraged overshoot.
    lastTurn = proportionalTurn + STEERING_SIGN * kd * filteredDelta * ERROR_SCALE;

    const bool settled = fabsf(alignmentError) <= 0.5f &&
                         fabsf(filteredDelta) <= 0.10f &&
                         fabsf(lastTurn) <= 10.0f && lineSensors <= 4;
    if (!settled) stableReads = 0;
    else if (stableReads < 255) ++stableReads;
    if (inBend && stableReads >= STRAIGHT_READS_AFTER_BEND) {
      bendDone = true;
      bendSeen = false; // Require fresh confirmation for the next turn.
      bendReads = 0;
      bendCandidateSeen = false;
      slow = false;
      lastTurn = STEERING_SIGN * (STRAIGHT_KP * error +
                                 STRAIGHT_KD * filteredDelta) * ERROR_SCALE;
    }
    if (!edgeWeightsArmed && stableReads >= STABLE_READS_FOR_FAST) {
      edgeWeightsArmed = true;
      // Changing the weight map must not create a derivative kick.
      derivativeReady = false;
      filteredDelta = 0.0f;
    }
  } else {
    stableReads = bendReads = 0;
    derivativeReady = false;
    filteredDelta = 0.0f;
    // A dropout slows the car and releases stale straight-line steering.
    lastTurn = slow && bendSeen ? -REVERSE_TURN : 0.8f * lastTurn;
  }

  float targetPwm = stableReads >= STABLE_READS_FOR_FAST ? STRAIGHT_PWM :
                   fminf(STRAIGHT_PWM, UNSETTLED_PWM);
  if (slow) targetPwm = BEND_PWM;
  if (!validLine) targetPwm = fminf(targetPwm, SIGNAL_LOSS_PWM);
  basePwm = fminf(targetPwm, basePwm + ACCEL_STEP_PWM);
  // Both wheels stay forward outside the confirmed bend.
  const float turnLimit = slow && bendSeen ? MAX_BEND_TURN :
                          fminf(MAX_STRAIGHT_TURN, basePwm);
  lastTurn = limitFloat(lastTurn, -turnLimit, turnLimit);
  updateStatusLed(sinceStartMs, validLine);
  drive(lastTurn, basePwm);
}
