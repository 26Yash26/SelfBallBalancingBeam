#include <Servo.h>

Servo myServo;

// ─── HYPERPARAMETER ───────────────────────────
const int STEP_DELAY_MS = 45;  // ms per degree. Lower = faster, Higher = slower
//   Suggested range: 5 (fast) to 30 (very slow)
// ──────────────────────────────────────────────

const int SERVO_PIN = 3;
int currentAngle =95;  // Must match physical resting position

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
  delay(500);
  Serial.println("Servo ready. Enter angle (0-180):");
}

void sweepTo(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);

  if (targetAngle == currentAngle) {
    Serial.println("Already at that angle.");
    return;
  }

  int step = (targetAngle > currentAngle) ? 1 : -1;

  Serial.print("Moving from ");
  Serial.print(currentAngle);
  Serial.print(" to ");
  Serial.print(targetAngle);
  Serial.println("...");

  while (currentAngle != targetAngle) {
    currentAngle += step;
    myServo.write(currentAngle);
    delay(STEP_DELAY_MS);
  }

  Serial.print("Reached ");
  Serial.println(currentAngle);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Strip \r, spaces, etc.

    // Ignore empty lines
    if (input.length() == 0) return;

    // Make sure it's actually a number
    bool isNumber = true;
    for (int i = 0; i < input.length(); i++) {
      if (!isDigit(input[i])) { isNumber = false; break; }
    }

    if (!isNumber) {
      Serial.println("Invalid input. Enter a number between 0 and 180.");
      return;
    }

    int target = input.toInt();

    if (target >= 0 && target <= 180) {
      sweepTo(target);
    } else {
      Serial.println("Out of range. Enter a value between 0 and 180.");
    }
  }
}