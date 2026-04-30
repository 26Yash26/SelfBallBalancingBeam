#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox;

void setup() {
  Serial.begin(9600);        // ← Change to 9600
  Wire.begin();
  if (!lox.begin()) {
    Serial.println("Failed to init VL53L0X!");
    while (1);
  }
  Serial.println("VL53L0X ready.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    Serial.print("Status: ");
    Serial.print(measure.RangeStatus);
    Serial.print("  |  Distance (mm): ");
    Serial.println(measure.RangeMilliMeter - 30);
  } else {
    Serial.println("Out of range");
  }
  delay(100);
}