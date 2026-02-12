#include <AccelStepper.h>

// ────────────────────────────────────────────────
// Configuration – adjust these values as needed
// ────────────────────────────────────────────────
const uint8_t CONTROL_PIN       = 4;      // Digital input pin (HIGH = run, LOW = stop)
const uint8_t stepPin           = 3;
const uint8_t dirPin            = 2;
const uint8_t motorInterfaceType = 1;

const float TARGET_RPM          = 3.0;
const float STEPS_PER_REVOLUTION = 6400.0;

// For reporting position
unsigned long lastReport         = 0;
const unsigned long REPORT_INTERVAL_MS = 2;

// ────────────────────────────────────────────────
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.flush();
  while (!Serial) { ; }

  // Configure the control input pin
  pinMode(CONTROL_PIN, INPUT);           // Use INPUT_PULLUP if your signal is open-drain / active-low

  // Stepper configuration
  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);

  float stepsPerSecond = TARGET_RPM * STEPS_PER_REVOLUTION / 60.0;
  stepper.setSpeed(stepsPerSecond);

  Serial.println("Stepper ready – runs only when control pin is HIGH");
  Serial.print  ("Control pin: ");
  Serial.println(CONTROL_PIN);
}

void loop() {
  // Read the control signal once per loop iteration
  bool shouldRun = (digitalRead(CONTROL_PIN) == HIGH);

  if (shouldRun) {
    stepper.runSpeed();   // Only generate steps when requested
  }
  // When shouldRun == false → runSpeed() is skipped → motor stops smoothly
  // (no abrupt halt; it simply stops sending pulses)

  // ────────────────────────────────
  // Position reporting (unchanged)
  // ────────────────────────────────
  unsigned long now = millis();
  if (now - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = now;

    long pos = stepper.currentPosition();

    // Binary packet:  0x67  0x50  [int32 LE]  0x55
    Serial.write(0x67);
    Serial.write(0x50);                   // packet type = Position

    Serial.write(static_cast<byte>(pos >>  0));
    Serial.write(static_cast<byte>(pos >>  8));
    Serial.write(static_cast<byte>(pos >> 16));
    Serial.write(static_cast<byte>(pos >> 24));

    Serial.write(0x55);                   // end marker
  }
}