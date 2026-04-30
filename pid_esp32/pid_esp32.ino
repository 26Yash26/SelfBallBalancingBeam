#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

// ── Pins ─────────────────────────────────────────────────────
#define SDA0      8
#define SCL0      9
#define SDA1      6
#define SCL1      7
#define SERVO_PIN 3

// ═════════════════════════════════════════════════════════════
//  HYPERPARAMETERS — tune these
// ═════════════════════════════════════════════════════════════

const int   SERVO_MIN_DEG  = 75;    // maximum tilt one way
const int   SERVO_MAX_DEG  = 120;   // maximum tilt other way
const int   NEUTRAL_DEG    = 97;    // bar level position — adjust until bar is flat
const int   SERVO_SPEED    = 2;     // degrees per cycle — higher = faster response
const int   NO_BALL_SLACK  = 50;    // mm — increase if ball gets ignored
const int   MIN_DIST_MM    = 20;    // mm — ignore readings below this
const float DEADBAND_MM    = 25.0f; // mm — within this error, just level the bar

float Kp = 0.06f;
float Ki = 0.0004f;
float Kd = 0.45f;

// ═════════════════════════════════════════════════════════════

const float L_MM        = 420.0f;
const float SETPOINT_MM = L_MM / 2.0f;  // 210 mm

const unsigned long DT_MS = 30;

// ── PID state ────────────────────────────────────────────────
float integral          = 0.0f;
float prevError         = 0.0f;
bool  firstLoop         = true;
int   currentServoAngle = NEUTRAL_DEG;

// ── Objects ──────────────────────────────────────────────────
TwoWire I2C_left  = TwoWire(0);
TwoWire I2C_right = TwoWire(1);
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofRight;
Servo myServo;

unsigned long lastTime = 0;

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
int clampInt(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void servoMoveTo(int target) {
  target = clampInt(target, SERVO_MIN_DEG, SERVO_MAX_DEG);
  if (target == currentServoAngle) return;

  int step = (target > currentServoAngle) ? SERVO_SPEED : -SERVO_SPEED;
  currentServoAngle += step;

  if (step > 0 && currentServoAngle > target) currentServoAngle = target;
  if (step < 0 && currentServoAngle < target) currentServoAngle = target;

  currentServoAngle = clampInt(currentServoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  myServo.write(currentServoAngle);
}

float readMM(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);
  if (m.RangeStatus == 4)                return -1.0f;  // out of range
  if (m.RangeMilliMeter < MIN_DIST_MM)   return -1.0f;  // too close, noise
  return (float)m.RangeMilliMeter;
}

// Right sensor is primary (more accurate).
// Left sensor is fallback only — used when ball is near left end.
float fusePosition(float dL, float dR) {
  bool leftOk  = (dL > 0 && dL < 150.0f);   // left only trusted below 150mm
  bool rightOk = (dR > 0 && dR < 200.0f);   // right trusted up to 300mm

  if (rightOk && leftOk) {
    // Both reliable — average them
    return (dL + (L_MM - dR)) / 2.0f;
  }
  if (rightOk) return L_MM - dR;   // right only (primary)
  if (leftOk)  return dL;          // left only (ball very close to left end)
  return -1.0f;                     // no ball
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Ball Balancer Boot ===");

  // ── Servo ────────────────────────────────────────────────
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(NEUTRAL_DEG);
  currentServoAngle = NEUTRAL_DEG;
  delay(600);
  Serial.print("Servo: OK  (limits: ");
  Serial.print(SERVO_MIN_DEG);
  Serial.print("° – ");
  Serial.print(SERVO_MAX_DEG);
  Serial.println("°)");

  // ── Left sensor ──────────────────────────────────────────
  I2C_left.begin(SDA0, SCL0, 400000);
  Serial.print("Left  sensor (fallback)... ");
  if (!tofLeft.begin(0x29, false, &I2C_left)) {
    Serial.println("FAILED. Check SDA->GPIO8, SCL->GPIO9.");
    while (1);
  }
  Serial.println("OK");

  // ── Right sensor ─────────────────────────────────────────
  I2C_right.begin(SDA1, SCL1, 400000);
  Serial.print("Right sensor (primary)... ");
  if (!tofRight.begin(0x29, false, &I2C_right)) {
    Serial.println("FAILED. Check SDA->GPIO6, SCL->GPIO7.");
    while (1);
  }
  Serial.println("OK");

  Serial.println("--------------------------------------------------------");
  Serial.println("Place ball on bar.");
  Serial.println("L(mm)\tR(mm)\tpos\terr\tPID\ttarget\tservo");
  Serial.println("--------------------------------------------------------");

  lastTime = millis();
}

// ─────────────────────────────────────────────────────────────
//  Main loop
// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastTime < DT_MS) return;
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  float dL  = readMM(tofLeft);
  float dR  = readMM(tofRight);
  float pos = fusePosition(dL, dR);

  // ── No ball → return to neutral, reset PID ───────────────
  if (pos < 0) {
    integral  = 0;
    prevError = 0;
    firstLoop = true;
    servoMoveTo(NEUTRAL_DEG);
    Serial.print("L:"); Serial.print(dL, 0);
    Serial.print("mm\tR:"); Serial.print(dR, 0);
    Serial.println("mm\t-- no ball --");
    return;
  }

  // ── PID ──────────────────────────────────────────────────
  float error      = SETPOINT_MM - pos;
  float derivative = firstLoop ? 0.0f : (error - prevError) / dt;
  firstLoop        = false;

  integral += error * dt;
  integral  = clampf(integral, -500.0f, 500.0f);

  float pidOut = Kp * error
               + Ki * integral
               + Kd * derivative;

  // ── Deadband + servo command ──────────────────────────────
  int targetAngle;
  if (fabs(error) < DEADBAND_MM) {
    targetAngle = NEUTRAL_DEG;   // ball close enough → level bar
    integral    = 0;             // prevent integrator windup at rest
  } else {
    targetAngle = clampInt(NEUTRAL_DEG - (int)pidOut, SERVO_MIN_DEG, SERVO_MAX_DEG);
  }

  servoMoveTo(targetAngle);
  prevError = error;

  // ── Telemetry ─────────────────────────────────────────────
  Serial.print("L:"); Serial.print(dL, 0);
  Serial.print("mm\tR:"); Serial.print(dR, 0);
  Serial.print("mm\tpos:"); Serial.print(pos, 1);
  Serial.print("mm\terr:"); Serial.print(error, 1);
  Serial.print("\tPID:"); Serial.print(pidOut, 2);
  Serial.print("\ttarget:"); Serial.print(targetAngle);
  Serial.print("\tservo:"); Serial.println(currentServoAngle);
}