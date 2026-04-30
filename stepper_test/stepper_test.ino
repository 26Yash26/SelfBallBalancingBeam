#define STEP_PIN 4
#define DIR_PIN  3

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Stepper test starting...");
}

void stepMotor(int steps, bool direction, int speedDelay) {
  digitalWrite(DIR_PIN, direction);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(speedDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(speedDelay);
  }
}

void loop() {
  Serial.println("Rotating clockwise 200 steps...");
  stepMotor(200, HIGH, 800);   // 200 steps = 1 full revolution (1.8° motor)
  delay(1000);

  Serial.println("Rotating counter-clockwise 200 steps...");
  stepMotor(200, LOW, 800);
  delay(1000);
}