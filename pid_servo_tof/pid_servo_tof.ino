/*
 * ================================================================
 *  PING-PONG BALL BALANCER  —  Single PID with MG995 Servo
 *  Arduino Uno R3
 * ================================================================
 *
 *  PID:  error = (L/2) − tof_distance  =  210 − dist_mm
 *        setpoint = 0  (ball at bar centre)
 *        output   = angle offset added to servo neutral (90°)
 *
 *  WIRING:
 *    D3          → Servo signal wire (orange/yellow)
 *    A4 (SDA)    → VL53L0X SDA
 *    A5 (SCL)    → VL53L0X SCL
 *    5V          → VL53L0X VIN
 *    GND         → VL53L0X GND + Servo GND (black)
 *    External 5V → Servo VCC (red)  ← NOT Arduino 5V
 *
 *  LIBRARIES:
 *    Servo           (built-in)
 *    Adafruit VL53L0X
 *    Wire            (built-in)
 * ================================================================
 */

#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL53L0X.h"

// ── Pin ──────────────────────────────────────────────────────
#define SERVO_PIN 3

// ── Physical constants ───────────────────────────────────────
const float L_MM          = 420.0f;       // bar sensing length (mm)
const float SETPOINT_MM   = L_MM / 2.0f;  // 210 mm — ball target
const int   NEUTRAL_DEG   = 90;           // servo angle when bar is level
                                           // ↑ adjust if bar isn't perfectly
                                           //   horizontal at 90°
const int   MAX_TILT_DEG  = 20;           // max offset from neutral (±20°)
const unsigned long DT_MS = 30;           // loop period — ~33 Hz

// ── PID gains ────────────────────────────────────────────────
// Tune Kp first, then Kd, then Ki last (usually stays near 0).
float Kp = 0.06f;
float Ki = 0.0004f;
float Kd = 0.45f;

// ── PID state ────────────────────────────────────────────────
float integral    = 0.0f;
float prevError   = 0.0f;
bool  firstLoop   = true;

// ── Objects ──────────────────────────────────────────────────
Servo             myServo;
Adafruit_VL53L0X  tof;

// ── Timing ───────────────────────────────────────────────────
unsigned long lastTime = 0;

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

int clampInt(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Returns distance in mm, or -1 if ball is out of range
float tofDistMM() {
  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? (float)m.RangeMilliMeter : -1.0f;
}

// Write servo, always clamped to 0–180
void servoWrite(int deg) {
  myServo.write(clampInt(deg, 0, 180));
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // ── Servo ────────────────────────────────────────────────
  myServo.attach(SERVO_PIN);
  servoWrite(NEUTRAL_DEG);   // level bar before anything else
  delay(600);                // let servo reach neutral
  Serial.println("Servo ready.");

  // ── I2C + VL53L0X ───────────────────────────────────────
  Wire.begin();
  Wire.setClock(400000);
  Serial.print("Initialising VL53L0X... ");
  if (!tof.begin()) {
    Serial.println("FAILED. Check SDA->A4, SCL->A5, VIN->5V.");
    while (1);
  }
  Serial.println("OK");

  Serial.println("==============================================");
  Serial.println("  Balancer running. Place ball on bar.       ");
  Serial.println("  dist(mm)  error  PID_out  servoAngle       ");
  Serial.println("==============================================");

  lastTime = millis();
}

// ─────────────────────────────────────────────────────────────
//  Main loop  (~33 Hz)
// ─────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastTime < DT_MS) return;
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  float dist = tofDistMM();

  // ── No ball → hold level, reset integrator ───────────────
  if (dist < 0) {
    integral  = 0;
    prevError = 0;
    firstLoop = true;
    servoWrite(NEUTRAL_DEG);
    Serial.println("-- no ball --");
    return;
  }

  // ── PID ──────────────────────────────────────────────────
  float error = SETPOINT_MM - dist;   // positive: ball too close → tilt away
                                       // negative: ball too far   → tilt toward

  // Skip derivative on first reading (prevError is meaningless)
  float derivative = firstLoop ? 0.0f : (error - prevError) / dt;
  firstLoop = false;

  integral += error * dt;
  integral  = clampf(integral, -500.0f, 500.0f);  // anti-windup

  float pidOut = Kp * error
               + Ki * integral
               + Kd * derivative;

  // Clamp PID output to safe tilt range
  pidOut = clampf(pidOut, -(float)MAX_TILT_DEG, (float)MAX_TILT_DEG);

  // Map to servo angle: positive error → tilt bar so ball rolls toward centre
  // If ball rolls the WRONG way, change (NEUTRAL_DEG + pidOut) to (NEUTRAL_DEG - pidOut)
  int servoAngle = NEUTRAL_DEG + (int)pidOut;

  servoWrite(servoAngle);
  prevError = error;

  // ── Telemetry ────────────────────────────────────────────
  Serial.print(dist, 0);       Serial.print('\t');
  Serial.print(error, 1);      Serial.print('\t');
  Serial.print(pidOut, 2);     Serial.print('\t');
  Serial.println(servoAngle);
}

/*
 * ================================================================
 *  TUNING GUIDE
 * ================================================================
 *
 *  1. CHECK DIRECTION FIRST
 *     Place ball near one end. The bar should tilt to push ball
 *     toward centre. If it tilts the wrong way, change:
 *       int servoAngle = NEUTRAL_DEG + (int)pidOut;
 *     to:
 *       int servoAngle = NEUTRAL_DEG - (int)pidOut;
 *
 *  2. SET NEUTRAL_DEG
 *     With Kp/Ki/Kd all at 0, the servo sits at NEUTRAL_DEG.
 *     Adjust until the bar is physically level.
 *
 *  3. TUNE Kp FIRST  (Ki=0, Kd=0)
 *     Raise Kp until ball oscillates back and forth around centre.
 *     Back it off ~30% from that point.
 *
 *  4. ADD Kd to kill overshoot and oscillation.
 *     Typical range: 5× to 10× Kp.
 *
 *  5. ADD Ki last, only if ball drifts to one side.
 *     Keep it tiny (0.0001 to 0.001). Too much = slow oscillation.
 *
 *  TYPICAL STARTING POINT (420 mm bar, MG995, VL53L0X):
 *    Kp = 0.06,  Ki = 0.0004,  Kd = 0.45
 *    NEUTRAL_DEG = 90,  MAX_TILT_DEG = 20
 * ================================================================
 */