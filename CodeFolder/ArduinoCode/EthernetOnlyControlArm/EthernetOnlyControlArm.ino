#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>
#include <mbed.h>

using namespace std::chrono_literals;

// Ethernet Configuration
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(134, 50, 51, 22);
IPAddress piIP(134, 50, 51, 24);
EthernetServer server(80);
EthernetServer commandServer(5005);

// Periodic push to Pi settings
const uint16_t piPort = 6006;
const unsigned long pushIntervalMs = 20;
unsigned long lastPushTime = 0;

// Link status tracking (non-blocking)
bool ethernetLinkUp = false;
unsigned long lastLinkCheck = 0;

// SPI CS pin (kept from your Ethernet code)
const int CS_Pin = 10;

// ======================== ALL VARIABLES (cleaned & consistent) ========================
// Analog input pins
int positionSetPin = A0;
int positionReadPin = A1;
int rotaryKnobReadPin1 = A2;
int rotaryKnobReadPin2 = A3;

// Digital input pins
// Packet 1
int scramPin = 53;
int powerPin = 52;
int electromagnetPin = 51;
int forwardPin = 50;
int backwardPin = 49;
int rodPositionMinPin = 48;
int rodPositionMaxPin = 47;

// Packet 2
int speedPin = 46;

// Latched (toggled) states
bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool forwardToggledState = false;
bool backwardToggledState = false;
bool posMinToggledState = false;
bool posMaxToggledState = false;
bool speedToggledState = false;

// Button debouncing
bool lastScramButtonReading = HIGH;
bool lastPowerButtonReading = HIGH;
bool lastMagnetButtonReading = HIGH;
bool lastForwardButtonReading = HIGH;
bool lastBackwardButtonReading = HIGH;
bool lastPosMinButtonReading = HIGH;
bool lastPosMaxButtonReading = HIGH;
bool lastSpeedButtonReading = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Stepper control
volatile bool controlPin = false;   // volatile because it's used in ISR (ticker)

// Stepper hardware
const uint8_t stepPin = 6;
const uint8_t dirPin = 5;
const uint8_t _dirPin = 7;
const uint8_t motorInterfaceType = 1;
const float stepsPerRevolution = 6400.0;

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Hardware timer for stepper (non-blocking)
mbed::Ticker stepperTicker;

// ======================== MOTOR UPDATE (runs in background) ========================
void updateMotor() {
  if (controlPin) {
    stepper.runSpeed();
  }
}

void setup() {
  // ======================== PIN SETUP ========================
  // Digital inputs (Packet 1)
  pinMode(scramPin, INPUT);
  pinMode(powerPin, INPUT);
  pinMode(electromagnetPin, INPUT);
  pinMode(forwardPin, INPUT);
  pinMode(backwardPin, INPUT);
  pinMode(rodPositionMinPin, INPUT);
  pinMode(rodPositionMaxPin, INPUT);
  // Digital input (Packet 2)
  pinMode(speedPin, INPUT);

  // Stepper outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);

  // Analog inputs
  pinMode(positionSetPin, INPUT);
  pinMode(positionReadPin, INPUT);
  pinMode(rotaryKnobReadPin1, INPUT);
  pinMode(rotaryKnobReadPin2, INPUT);

  // SPI CS (kept from your Ethernet code)
  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin, HIGH);

  // ======================== STEPPER CONFIG ========================
  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);
  stepper.setMinPulseWidth(5);

  // ======================== ETHERNET SETUP ========================
  Serial.begin(115200);                    // USB debug only
  Serial.println("Ethernet WebServerExample");

  Ethernet.begin(mac, ip);

  SPI1.begin();                          // kept from your original Ethernet code

  // Optional SPI tuning (kept exactly as you had it)
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Hardware checks
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) { delay(1); }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  commandServer.begin();
  Serial.print("Command server listening on port 5005 for Pi to Arduino");

  // Fast, non-blocking Ethernet timeouts
  Ethernet.setRetransmissionTimeout(50);
  Ethernet.setRetransmissionCount(2);

  ethernetLinkUp = (Ethernet.linkStatus() == LinkON);

  // ======================== HARDWARE TIMER FOR STEPPER ========================
  // Runs updateMotor() every 50 µs – completely non-blocking
// Option 1: Explicit chrono literal (most reliable on GIGA)
stepperTicker.attach(&updateMotor, std::chrono::microseconds(50));

// Option 2: Using the us suffix with proper namespace (if you add using)
using namespace std::chrono_literals;
stepperTicker.attach(&updateMotor, 50us);
}

void loop() {
  // ======================== READ ALL INPUTS ========================
  bool currentScramReading = digitalRead(scramPin);
  bool currentPowerReading = digitalRead(powerPin);
  bool currentMagnetReading = digitalRead(electromagnetPin);
  bool currentForwardReading = digitalRead(forwardPin);
  bool currentBackwardReading = digitalRead(backwardPin);
  bool currentPosMinReading = digitalRead(rodPositionMinPin);
  bool currentPosMaxReading = digitalRead(rodPositionMaxPin);
  bool currentSpeedReading = digitalRead(speedPin);

  // ======================== LATCHED BUTTON LOGIC (debounced) ========================
  if (currentScramReading == LOW && lastScramButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      scramToggledState = !scramToggledState;
      lastDebounceTime = millis();
    }
  }
  lastScramButtonReading = currentScramReading;

  if (currentPowerReading == LOW && lastPowerButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      powerToggledState = !powerToggledState;
      lastDebounceTime = millis();
    }
  }
  lastPowerButtonReading = currentPowerReading;

  if (currentMagnetReading == LOW && lastMagnetButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      magnetToggledState = !magnetToggledState;
      lastDebounceTime = millis();
    }
  }
  lastMagnetButtonReading = currentMagnetReading;

  if (currentForwardReading == LOW && lastForwardButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      forwardToggledState = !forwardToggledState;
      lastDebounceTime = millis();
    }
  }
  lastForwardButtonReading = currentForwardReading;

  if (currentBackwardReading == LOW && lastBackwardButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      backwardToggledState = !backwardToggledState;
      lastDebounceTime = millis();
    }
  }
  lastBackwardButtonReading = currentBackwardReading;

  if (currentPosMinReading == LOW && lastPosMinButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      posMinToggledState = !posMinToggledState;
      lastDebounceTime = millis();
    }
  }
  lastPosMinButtonReading = currentPosMinReading;

  if (currentPosMaxReading == LOW && lastPosMaxButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      posMaxToggledState = !posMaxToggledState;
      lastDebounceTime = millis();
    }
  }
  lastPosMaxButtonReading = currentPosMaxReading;

  if (currentSpeedReading == LOW && lastSpeedButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      speedToggledState = !speedToggledState;
      lastDebounceTime = millis();
    }
  }
  lastSpeedButtonReading = currentSpeedReading;

  // ======================== BUILD DATA PACKETS ========================
  uint8_t packet1 = 0;
  uint8_t packet2 = 0;
  uint16_t positionSet = analogRead(positionSetPin);
  uint16_t positionRead = analogRead(positionReadPin);
  uint16_t rotaryKnob1Read = analogRead(rotaryKnobReadPin1);
  uint16_t rotaryKnob2Read = analogRead(rotaryKnobReadPin2);

  // Speed hysteresis (same as your original logic)
  static int lastPositionSet = -1;
  const int hysteresis = 4;
  if (abs((int)positionSet - lastPositionSet) >= hysteresis) {
    lastPositionSet = positionSet;
    float targetRPM = positionSet / 50.0;
    float stepsPerSecond = targetRPM * stepsPerRevolution / 25.0;
    stepper.setSpeed(stepsPerSecond);
  }

  // Active (unlatched) states
  bool scramActive = (digitalRead(scramPin) == LOW);
  bool powerActive = (digitalRead(powerPin) == LOW);
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);
  bool forwardActive = (digitalRead(forwardPin) == HIGH);
  bool backwardActive = (digitalRead(backwardPin) == HIGH);
  bool maxPositionActive = (digitalRead(rodPositionMaxPin) == HIGH);
  bool minPositionActive = (digitalRead(rodPositionMinPin) == HIGH);

  // Direction control for stepper
  if (forwardActive) {
    digitalWrite(dirPin, LOW);
    digitalWrite(_dirPin, HIGH);
    controlPin = true;
  } else if (backwardActive) {
    digitalWrite(dirPin, HIGH);
    digitalWrite(_dirPin, LOW);
    controlPin = true;
  } else {
    controlPin = false;
  }

  bool fast_Slow = (digitalRead(speedPin) == LOW);

  // Packet 1 (latched + unlatched bits – exactly as you defined)
  if (scramToggledState)   packet1 |= (1 << 0);
  if (powerToggledState)   packet1 |= (1 << 1);
  if (magnetToggledState)  packet1 |= (1 << 2);
  if (forwardActive)       packet1 |= (1 << 3);
  if (backwardActive)      packet1 |= (1 << 4);
  if (minPositionActive)   packet1 |= (1 << 5);
  if (maxPositionActive)   packet1 |= (1 << 6);
  if (speedToggledState)   packet1 |= (1 << 7);

  // Packet 2
  if (controlPin) packet2 |= (1 << 0);

  // ======================== NON-BLOCKING ETHERNET LINK CHECK ========================
  unsigned long now = millis();
  if (now - lastLinkCheck >= 500) {
    lastLinkCheck = now;
    ethernetLinkUp = (Ethernet.linkStatus() == LinkON);
  }

  // ======================== HTTP JSON WEB SERVER (GET requests) ========================
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);

        if (c == '\n' && currentLineIsBlank) {
          // === JSON RESPONSE ===
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Connection: close");
          client.println("Access-Control-Allow-Origin: *");
          client.println();

          StaticJsonDocument<384> doc;
          doc["scram"]          = scramToggledState;
          doc["power"]          = powerToggledState;
          doc["magnet"]         = magnetToggledState;
          doc["forward"]        = forwardActive;
          doc["backward"]       = backwardActive;
          doc["pos_min"]        = minPositionActive;
          doc["pos_max"]        = maxPositionActive;
          doc["fast_slow"]      = fast_Slow;
          doc["control_active"] = controlPin;

          doc["position_set"]   = positionSet;
          doc["position_read"]  = positionRead;
          doc["knob1"]          = rotaryKnob1Read;
          doc["knob2"]          = rotaryKnob2Read;

          doc["packet1"]        = packet1;
          doc["packet2"]        = packet2;
          doc["timestamp"]      = millis();

          serializeJson(doc, client);
          client.println();
          break;
        }
        if (c == '\n') {
          currentLineIsBlank = true;
        } else if (c != '\r') {
          currentLineIsBlank = false;
        }
      }
    }
    delay(1);
    client.stop();
    Serial.println("client disconnected");
  }

// ==================== RECEIVE COMMANDS FROM PI (port 5005) ====================
  EthernetClient cmdClient = commandServer.available();
  if (cmdClient) {
    Serial.println("Pi connected for command");

    // Read the small JSON payload (safe for your heartbeat message)
    String jsonStr = "";
    unsigned long timeout = millis() + 500;   // 500 ms safety timeout

    while (cmdClient.connected() && millis() < timeout) {
      if (cmdClient.available()) {
        char c = cmdClient.read();
        jsonStr += c;
      }
    }

    cmdClient.stop();   // Close immediately (same style as your push)

    // Log what we received
    Serial.print("Received from Pi: ");
    Serial.println(jsonStr);

    // Optional: parse it (works with your current Pi payload)
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (!err) {
      Serial.println("→ JSON parsed successfully");
      // Example: you can react to commands here later
      // if (doc.containsKey("pi_state")) {
      //   Serial.print("Pi state: "); Serial.println(doc["pi_state"].as<const char*>());
      // }
    } else {
      Serial.print("→ JSON parse error: ");
      Serial.println(err.c_str());
    }
  }
 // ======================== PERIODIC JSON PUSH TO PI ========================
if (ethernetLinkUp && (now - lastPushTime >= pushIntervalMs)) {
  lastPushTime = now;

  EthernetClient outgoing;
  if (outgoing.connect(piIP, piPort)) {
    StaticJsonDocument<384> doc;

    doc["scram"]          = scramToggledState;
    doc["power"]          = powerToggledState;
    doc["magnet"]         = magnetToggledState;
    doc["forward"]        = forwardActive;
    doc["backward"]       = backwardActive;
    doc["pos_min"]        = minPositionActive;
    doc["pos_max"]        = maxPositionActive;
    doc["fast_slow"]      = fast_Slow;
    doc["control_active"] = controlPin;
    doc["position_set"]   = positionSet;
    doc["position_read"]  = positionRead;
    doc["knob1"]          = rotaryKnob1Read;
    doc["knob2"]          = rotaryKnob2Read;
    doc["packet1"]        = packet1;
    doc["packet2"]        = packet2;
    doc["timestamp"]      = millis();

    serializeJson(doc, outgoing);   // valid JSON (compact, single line)
    outgoing.print('\n');           // clean newline only (no \r)
    outgoing.flush();               // ← forces the buffer out BEFORE close
    outgoing.stop();

    Serial.println("Pushed JSON to Pi");
  } else {
    ethernetLinkUp = false;
  
  }
}
}