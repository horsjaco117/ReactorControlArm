#include <Arduino.h>
#include <string.h>        // for memset
#include <AccelStepper.h>

// === Pin Definitions ===
const uint8_t stepPin = 3;
const uint8_t dirPin = 2;
const uint8_t _dirPin = 4;
const int positionSetPin = A0;
const int positionReadPin = A1;
const int rotaryKnobReadPin1 = A2;
const int rotaryKnobReadPin2 = A3;

AccelStepper stepper(1, stepPin, dirPin);

// Motor State
bool motorControlPin = false;

// === Shared Memory Struct (MUST match M7 exactly) ===
typedef struct {
  // Motor command (from M7)
  float targetSpeed;
  bool  controlPin;
  bool  isBackward;
  bool  newMotorCmd;

  // Sensor data (from M4)
  int   posSet;
  int   posRead;
  int   knob1;
  int   knob2;
  int   digitalPacket1;
  int   digitalPacket2;
  bool  newSensorData;
} SharedData;

SharedData *shared = (SharedData *)0x38000000;   // SRAM4 - correct address

void applyMotorCommand() {
  if (shared->newMotorCmd) {
    motorControlPin = shared->controlPin;

    if (motorControlPin) {
      if (shared->isBackward) {
        digitalWrite(dirPin, HIGH);
        digitalWrite(_dirPin, LOW);
      } else {
        digitalWrite(dirPin, LOW);
        digitalWrite(_dirPin, HIGH);
      }
      stepper.setSpeed(shared->targetSpeed);
    } else {
      stepper.setSpeed(0);
    }
    shared->newMotorCmd = false;   // acknowledge
  }
}

void setup() {
  // Initialize shared memory
  memset(shared, 0, sizeof(SharedData));

  // Pin Modes
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);

  pinMode(53, INPUT_PULLUP); // Scram
  pinMode(52, INPUT_PULLUP); // Power
  pinMode(51, INPUT_PULLUP); // Magnet
  pinMode(46, INPUT_PULLUP); // Speed Fast/Slow

  pinMode(50, INPUT); // Forward
  pinMode(49, INPUT); // Backward
  pinMode(48, INPUT); // Min Pos
  pinMode(47, INPUT); // Max Pos

  pinMode(positionSetPin, INPUT);
  pinMode(positionReadPin, INPUT);
  pinMode(rotaryKnobReadPin1, INPUT);
  pinMode(rotaryKnobReadPin2, INPUT);

  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);
  stepper.setMinPulseWidth(5);
}

void loop() {
  // 1. Highest priority: motor stepping
  if (motorControlPin) {
    stepper.runSpeed();
  }

  // 2. Apply any new motor command
  applyMotorCommand();

  // 3. Read sensors and push to shared memory (100 Hz)
  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate >= 10) {
    lastSensorUpdate = millis();

    shared->posSet = analogRead(positionSetPin);
    shared->posRead = analogRead(positionReadPin);
    shared->knob1 = analogRead(rotaryKnobReadPin1);
    shared->knob2 = analogRead(rotaryKnobReadPin2);

    int pkt1 = 0;
    if (digitalRead(53) == LOW)  pkt1 |= (1 << 0);
    if (digitalRead(52) == LOW)  pkt1 |= (1 << 1);
    if (digitalRead(51) == LOW)  pkt1 |= (1 << 2);
    if (digitalRead(50) == HIGH) pkt1 |= (1 << 3);
    if (digitalRead(49) == HIGH) pkt1 |= (1 << 4);
    if (digitalRead(48) == HIGH) pkt1 |= (1 << 5);
    if (digitalRead(47) == HIGH) pkt1 |= (1 << 6);
    shared->digitalPacket1 = pkt1;

    int pkt2 = 0;
    if (digitalRead(46) == LOW) pkt2 |= (1 << 0);
    shared->digitalPacket2 = pkt2;

    shared->newSensorData = true;   // signal M7
  }
}