#include <Arduino.h>
#include <RPC.h>
#include <AccelStepper.h>

// === Pin Definitions ===
const uint8_t stepPin = 3;
const uint8_t dirPin = 2;
const uint8_t _dirPin = 4;          // inverted dir for your specific driver/safety logic
const int positionSetPin = A0;
const int positionReadPin = A1;
const int rotaryKnobReadPin1 = A2;
const int rotaryKnobReadPin2 = A3;

AccelStepper stepper(1, stepPin, dirPin);

// Motor State Variables
bool motorControlPin = false;

// === RPC Function: Called by M7 to update Motor ===
void updateMotorState(float stepsPerSecond, int controlPinInt, int isBackward) {
  motorControlPin = (controlPinInt > 0);

  if (motorControlPin) {
    // Manual direction control as required by your hardware
    if (isBackward > 0) {
      digitalWrite(dirPin, HIGH);     // Backward
      digitalWrite(_dirPin, LOW);
    } else {
      digitalWrite(dirPin, LOW);      // Forward
      digitalWrite(_dirPin, HIGH);
    }

    // Use absolute (positive) speed only — let AccelStepper handle pulsing only
    stepper.setSpeed(stepsPerSecond);   // always positive here
  } 
  else {
    stepper.setSpeed(0);
    motorControlPin = false;   // safety
  }
}

void setup() {
  RPC.begin();
  RPC.bind("updateMotorState", updateMotorState);

  // Pin Modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  
  // Buttons - pull-ups for active-LOW where applicable
  pinMode(53, INPUT_PULLUP); // Scram
  pinMode(52, INPUT_PULLUP); // Power
  pinMode(51, INPUT_PULLUP); // Magnet
  pinMode(46, INPUT_PULLUP); // Speed Fast/Slow
  pinMode(50, INPUT);        // Forward (active-HIGH)
  pinMode(49, INPUT);        // Backward
  pinMode(48, INPUT);        // Min Pos
  pinMode(47, INPUT);        // Max Pos

  pinMode(positionSetPin, INPUT);
  pinMode(positionReadPin, INPUT);
  pinMode(rotaryKnobReadPin1, INPUT);
  pinMode(rotaryKnobReadPin2, INPUT);

  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);  // unused with runSpeed(), but harmless
  stepper.setMinPulseWidth(5);
}

void loop() {
  // 1. Highest priority: keep motor stepping smoothly
  if (motorControlPin) {
    stepper.runSpeed();
  }

  // 2. Gather Data and Push to M7 (now 20 Hz to avoid jitter)
  static unsigned long lastSensorPush = 0;
  if (millis() - lastSensorPush >= 50) {   // <--- changed
    lastSensorPush = millis();

    int posSet = analogRead(positionSetPin);
    int posRead = analogRead(positionReadPin);
    int knob1 = analogRead(rotaryKnobReadPin1);
    int knob2 = analogRead(rotaryKnobReadPin2);

    int digitalPacket1 = 0;
    if (digitalRead(53) == LOW)  digitalPacket1 |= (1 << 0); // Scram
    if (digitalRead(52) == LOW)  digitalPacket1 |= (1 << 1); // Power
    if (digitalRead(51) == LOW)  digitalPacket1 |= (1 << 2); // Magnet
    if (digitalRead(50) == HIGH) digitalPacket1 |= (1 << 3); // Forward
    if (digitalRead(49) == HIGH) digitalPacket1 |= (1 << 4); // Backward
    if (digitalRead(48) == HIGH) digitalPacket1 |= (1 << 5); // Min Pos
    if (digitalRead(47) == HIGH) digitalPacket1 |= (1 << 6); // Max Pos

    int digitalPacket2 = 0;
    if (digitalRead(46) == LOW) digitalPacket2 |= (1 << 0); // Speed Fast/Slow

    RPC.call("updateSensors", posSet, posRead, knob1, knob2, digitalPacket1, digitalPacket2);
  }
}