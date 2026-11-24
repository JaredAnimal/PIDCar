// ECE3 line follower - ZONE STEERING (aggressive, abrupt corrections).
// The line position picks one of 4 zones; each zone applies a FIXED turn and
// speed immediately. Small fast flicks on straights, hard turns in curves.
// Turns around at the first T-bar, stops at the second.
// Starts automatically after a 2 second countdown. Reset for another run.
#include <Arduino.h>
#include <ECE3.h>
#include <math.h>

// ---------------- Pins ----------------
const uint8_t LEFT_SLEEP = 31, LEFT_DIR = 29, LEFT_PWM = 40;
const uint8_t RIGHT_SLEEP = 11, RIGHT_DIR = 30, RIGHT_PWM = 39;
const uint8_t STATUS_LED = 41;

// ---------------- Zones (main tuning) ----------------
// |error| limits, sensor units (max ~7):
const float ZONE0 = 0.3f;             // below: centered -> go straight
const float ZONE1 = 1.5f;             // below: slightly off -> small flick
const float ZONE2 = 3.0f;             // below: curve -> medium turn; above: hard turn
// Turn in each zone (PWM added to the outer wheel, taken from the inner one):
const float ZONE_TURN1 = 30.0f;       // small flick (straights)
const float ZONE_TURN2 = 65.0f;       // medium turn (curves)
const float ZONE_TURN3 = 110.0f;      // hard turn (sharp curves)
// Forward speed in each zone (PWM 0..255):
const float ZONE_SPEED0 = 130.0f;     // centered
const float ZONE_SPEED1 = 120.0f;     // small flick
const float ZONE_SPEED2 = 90.0f;      // curve
const float ZONE_SPEED3 = 55.0f;      // sharp curve

// Line lost: pivot in place toward the side the line was last seen on.
const float RECOVERY_TURN = 90.0f;
const float SIDE_MEMORY_ERROR = 1.0f; // |error| needed to remember the side

// ---------------- Car setup ----------------
const float STEERING_SIGN = 1.0f;     // -1 if the car steers away from the line
const uint32_t SAMPLE_US = 6000;      // Sensor read / steering update every 6 ms
const uint32_t START_DELAY_MS = 2000;

// ---------------- Sensors / calibration ----------------
// Order RIGHT to LEFT, same as the calibration sketch.
const int SENSOR_MIN[8] = {771, 759, 681, 726, 521, 612, 657, 703};
const int SENSOR_MAX[8] = {2500, 2500, 2500, 2091, 1909, 2500, 2500, 2500};
const int POSITION[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
const float CENTER_OFFSET = -0.0565749235f;
const int MIN_LINE_PEAK = 220;        // normalized 0..1000
const int MIN_LINE_SUM = 300;

// ---------------- T-bar ----------------
const int BAR_DARK_LEVEL = 600;
const uint8_t BAR_SENSOR_COUNT = 8;   // all eight dark = bar
const uint8_t BAR_CONFIRM_READS = 2;
const uint8_t CLEAR_READS_TO_ARM = 10;
const long REARM_COUNTS = 300;        // distance after the turnaround before the next bar counts
const uint32_t BAR_BACKUP_MS = 80;    // line lost within this time after a bar-like reading = bar

// ---------------- Turnaround ----------------
const int TURN_PWM = 255;
const int TURN_FINISH_PWM = 80;
const long TURN_SLOWDOWN_COUNTS = 90;
const long TURN_COUNTS = 360;         // tune on the floor for exactly 180 degrees
const uint32_t TURN_TIMEOUT_MS = 1500;

// ---------------- State ----------------
enum RunState { START_DELAY, TRACKING, TURNING, FINISHED };
RunState state = START_DELAY;
uint16_t raw[8];
uint32_t startMs = 0, lastSampleUs = 0, turnStartMs = 0, wideSeenMs = 0;
bool finishArmed = false;
uint8_t clearReads = 0, barReads = 0, barsSeen = 0;
int lastSide = 0;
float lastTurn = 0.0f;

// ---------------- Motors ----------------
void setWheel(uint8_t dirPin, uint8_t pwmPin, float cmd) {
  digitalWrite(dirPin, cmd < 0.0f ? HIGH : LOW);   // LOW = forward
  int pwm = (int)fabsf(cmd);
  if (pwm > 255) pwm = 255;
  analogWrite(pwmPin, pwm);
}

void drive(float turn, float speed) {
  // Positive turn: slow the left wheel, speed up the right.
  setWheel(LEFT_DIR, LEFT_PWM, speed - turn);
  setWheel(RIGHT_DIR, RIGHT_PWM, speed + turn);
  digitalWrite(LEFT_SLEEP, HIGH);
  digitalWrite(RIGHT_SLEEP, HIGH);
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
  digitalWrite(LEFT_SLEEP, LOW);
  digitalWrite(RIGHT_SLEEP, LOW);
}

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

// First bar -> spin 180 degrees. Second bar -> stop.
void handleBar(uint32_t nowMs) {
  if (barsSeen == 0) {
    barsSeen = 1;
    resetEncoders();
    setWheel(LEFT_DIR, LEFT_PWM, -TURN_PWM);
    setWheel(RIGHT_DIR, RIGHT_PWM, TURN_PWM);
    digitalWrite(LEFT_SLEEP, HIGH);
    digitalWrite(RIGHT_SLEEP, HIGH);
    turnStartMs = nowMs;
    state = TURNING;
  } else {
    stopMotors();
    state = FINISHED;
  }
}

int normalize(uint8_t i) {
  int v = (int32_t(raw[i]) - SENSOR_MIN[i]) * 1000L / (SENSOR_MAX[i] - SENSOR_MIN[i]);
  if (v < 0) v = 0;
  if (v > 1000) v = 1000;
  return v;
}

// ---------------- Setup ----------------
void setup() {
  pinMode(LEFT_SLEEP, OUTPUT);
  pinMode(RIGHT_SLEEP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  stopMotors();
  ECE3_Init();
  startMs = millis();
}

// ---------------- Loop ----------------
void loop() {
  uint32_t nowMs = millis();

  if (state == FINISHED) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  if (state == START_DELAY) {
    digitalWrite(STATUS_LED, (nowMs / 500) % 2);    // slow blink
    if (nowMs - startMs < START_DELAY_MS) return;
    state = TRACKING;
    lastSampleUs = micros();
    return;
  }

  if (state == TURNING) {
    digitalWrite(STATUS_LED, (nowMs / 100) % 2);    // fast blink
    const long turned = encoderAverage();
    if (turned >= TURN_COUNTS - TURN_SLOWDOWN_COUNTS) {   // slow down near the end
      setWheel(LEFT_DIR, LEFT_PWM, -TURN_FINISH_PWM);
      setWheel(RIGHT_DIR, RIGHT_PWM, TURN_FINISH_PWM);
    }
    if (turned >= TURN_COUNTS || nowMs - turnStartMs >= TURN_TIMEOUT_MS) {
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, 0);
      digitalWrite(LEFT_DIR, LOW);
      digitalWrite(RIGHT_DIR, LOW);
      delay(50);
      resetEncoders();
      finishArmed = false;
      clearReads = barReads = 0;
      lastSide = 0;
      lastTurn = 0.0f;
      lastSampleUs = micros();
      state = TRACKING;
    }
    return;
  }

  // ---- TRACKING: one update every SAMPLE_US ----
  const uint32_t nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_US) return;
  lastSampleUs = nowUs;
  ECE3_read_IR(raw);
  nowMs = millis();
  digitalWrite(STATUS_LED, HIGH);

  int sum = 0, peak = 0, weighted = 0;
  uint8_t darkSensors = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    const int v = normalize(i);
    sum += v;
    weighted += POSITION[i] * v;
    if (v > peak) peak = v;
    if (v >= BAR_DARK_LEVEL) ++darkSensors;
  }

  // Bar-like reading: most sensors dark INCLUDING both outer ones.
  if (darkSensors >= 6 && normalize(0) >= BAR_DARK_LEVEL && normalize(7) >= BAR_DARK_LEVEL) {
    wideSeenMs = nowMs;
  }

  // ---- T-bar ----
  if (darkSensors >= BAR_SENSOR_COUNT) {
    clearReads = 0;
    if (finishArmed && ++barReads >= BAR_CONFIRM_READS) {
      handleBar(nowMs);
      return;
    }
    drive(lastTurn, ZONE_SPEED2);
    return;
  }
  barReads = 0;

  // ---- Line lost ----
  if (!(peak >= MIN_LINE_PEAK && sum >= MIN_LINE_SUM)) {
    clearReads = 0;
    // Backup: line ended right after a bar-like reading = we passed the T-bar.
    if (finishArmed && nowMs - wideSeenMs < BAR_BACKUP_MS) {
      handleBar(nowMs);
      return;
    }
    // Pivot in place toward the side the line was last seen on.
    const float side = (lastSide != 0) ? lastSide : (lastTurn >= 0.0f ? 1.0f : -1.0f);
    drive(STEERING_SIGN * side * RECOVERY_TURN, 0.0f);
    return;
  }

  // Arm the bar check after 10 clean readings (and some distance after the turnaround).
  if (!finishArmed && ++clearReads >= CLEAR_READS_TO_ARM &&
      (barsSeen == 0 || encoderAverage() >= REARM_COUNTS)) finishArmed = true;

  // ---- Line position ----
  const float error = float(weighted) / float(sum) - CENTER_OFFSET;
  if (error > SIDE_MEMORY_ERROR) lastSide = 1;
  else if (error < -SIDE_MEMORY_ERROR) lastSide = -1;

  // ---- ZONE STEERING: fixed turn + speed, applied immediately ----
  const float a = fabsf(error);
  float turn, speed;
  if (a < ZONE0)      { turn = 0.0f;       speed = ZONE_SPEED0; }
  else if (a < ZONE1) { turn = ZONE_TURN1; speed = ZONE_SPEED1; }
  else if (a < ZONE2) { turn = ZONE_TURN2; speed = ZONE_SPEED2; }
  else                { turn = ZONE_TURN3; speed = ZONE_SPEED3; }

  lastTurn = STEERING_SIGN * (error > 0.0f ? turn : -turn);
  drive(lastTurn, speed);
}
