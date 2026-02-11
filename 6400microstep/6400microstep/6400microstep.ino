#include <AccelStepper.h>

// Define the stepper motor and its pins
#define stepPin 3
#define dirPin 2
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

unsigned long lastReport = 0;
const unsigned long REPORT_INTERVAL_MS = 500;

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.flush();
  while (!Serial) { ; }

  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);

  float rpm = 3.0;
  float stepsPerRevolution = 6400.0;
  float stepsPerSecond = rpm * stepsPerRevolution / 60.0;
  stepper.setSpeed(stepsPerSecond);

  Serial.println("Stepper started – binary position packets @ ~2 Hz");
}

void loop() {
  stepper.runSpeed();

  unsigned long now = millis();
  if (now - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = now;

    long pos = stepper.currentPosition();

    // Binary packet:  AA 50 [int32 LE] 55
    Serial.write(0x67);
    
    Serial.write(0x50);           // packet type = Position

    Serial.write((byte)(pos >>  0));  // LSB
    Serial.write((byte)(pos >>  8));
    Serial.write((byte)(pos >> 16));
    Serial.write((byte)(pos >> 24));  // MSB

    Serial.write(0x55);           // end marker
  }
}