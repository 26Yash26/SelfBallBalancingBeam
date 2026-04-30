#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  encoder.begin(4);  // DIR pin on D4... 
                     // but we're using D4 for STEP!
                     // use -1 if DIR pin is wired to GND
  encoder.begin(-1);
  encoder.setDirection(AS5600_CLOCK_WISE);
  
  if (!encoder.isConnected()) {
    Serial.println("AS5600 not found! Check wiring.");
    while(1);
  }
  Serial.println("AS5600 ready.");
}

void loop() {
  // Raw angle: 0-4095 (12-bit, full 360°)
  int raw = encoder.readAngle();
  
  // Converted to degrees
  float degrees = encoder.readAngle() * 360.0 / 4096.0;
  
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("  |  Angle: ");
  Serial.print(degrees, 1);
  Serial.println("°");
  
  delay(100);
}