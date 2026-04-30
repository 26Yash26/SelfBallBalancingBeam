#include <Wire.h>
#include <AS5600.h>

#define STEP_PIN 4
#define DIR_PIN  3

#define STEPS_PER_REV    200      // Full step, 1.8° motor
#define DEGREES_PER_STEP 1.8f
#define STEP_DELAY_US    800      // Speed — lower = faster

AS5600 encoder;

// ─── Read a stable angle from AS5600 ──────────────────────
float readAngleDeg() {
  // Average 5 readings to reduce noise
  long sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += encoder.readAngle();
    delay(2);
  }
  return (sum / 5.0f) * 360.0f / 4096.0f;
}

// ─── Step motor by exact number of degrees ────────────────
// Positive = clockwise, Negative = counter-clockwise
void rotateByDegrees(float targetDeg) {
  bool clockwise = targetDeg >= 0;
  float absDeg   = abs(targetDeg);
  int   steps    = round(absDeg / DEGREES_PER_STEP);

  Serial.print("Steps to move: ");
  Serial.println(steps);

  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
  delayMicroseconds(5);  // A4988 direction setup time

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  Wire.begin();
  Wire.setClock(400000);

  encoder.begin(-1);  // DIR pin hardwired to GND
  encoder.setDirection(AS5600_CLOCK_WISE);

  if (!encoder.isConnected()) {
    Serial.println("ERROR: AS5600 not found! Check SDA->A4, SCL->A5.");
    while (1);
  }
  Serial.println("AS5600 ready.");
  Serial.println("─────────────────────────────────");
  Serial.println("Enter degrees to rotate (+CW / -CCW):");
}

// ═══════════════════════════════════════════════════════════
void loop() {
  if (!Serial.available()) return;

  float inputDeg = Serial.parseFloat();

  // Flush leftover newline characters
  while (Serial.available()) Serial.read();

  if (inputDeg == 0) {
    Serial.println("Got 0 or invalid input. Try again.");
    return;
  }

  // Read initial angle
  float angleBefore = readAngleDeg();

  Serial.println("─────────────────────────────────");
  Serial.print("Requested rotation : ");
  Serial.print(inputDeg, 1);
  Serial.println("°");
  Serial.print("Angle before       : ");
  Serial.print(angleBefore, 2);
  Serial.println("°");

  // Rotate
  rotateByDegrees(inputDeg);

  // Short settle time for encoder to catch up
  delay(150);

  // Read final angle
  float angleAfter = readAngleDeg();

  // Actual delta — handle wrap-around (e.g. 355° → 5°)
  float actualDelta = angleAfter - angleBefore;
  if (actualDelta >  180.0f) actualDelta -= 360.0f;
  if (actualDelta < -180.0f) actualDelta += 360.0f;

  Serial.print("Angle after        : ");
  Serial.print(angleAfter, 2);
  Serial.println("°");
  Serial.print("Actual rotation    : ");
  Serial.print(actualDelta, 2);
  Serial.println("°");
  Serial.print("Error              : ");
  Serial.print(fmod((actualDelta - inputDeg),360), 2);
  Serial.println("°");
  Serial.println("─────────────────────────────────");
  Serial.println("Enter next angle:");
}