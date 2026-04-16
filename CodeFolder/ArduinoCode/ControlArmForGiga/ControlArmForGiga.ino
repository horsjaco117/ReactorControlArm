#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>
#include <mbed.h>

using namespace std::chrono_literals;

// ======================== ETHERNET CONFIG ========================
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(134, 50, 51, 22);
IPAddress piIP(134, 50, 51, 24);
EthernetServer server(80);
EthernetServer commandServer(5005);

const uint16_t piPort = 6006;
const unsigned long pushIntervalMs = 20;
unsigned long lastPushTime = 0;

bool ethernetLinkUp = false;
unsigned long lastLinkCheck = 0;
const int CS_Pin = 10;

// ======================== SERIAL CONFIG ========================
unsigned long lastTxTime = 0; 
const unsigned long txIntervalMs = 200;

// ======================== HARDWARE PINS ========================
// Analog input pins
int positionSetPin = A0;
int positionReadPin = A1;
int rotaryKnobReadPin1 = A2;
int rotaryKnobReadPin2 = A3;

// Digital input pins - Packet 1
int scramPin = 53;
int powerPin = 52;
int electromagnetPin = 51;
int forwardPin = 50;
int backwardPin = 49;
int rodPositionMinPin = 48;
int rodPositionMaxPin = 47;

// Digital input pins - Packet 2
int speedPin = 46;

// Stepper Hardware (Using pins from Serial sketch)
const uint8_t stepPin = 6;
const uint8_t dirPin = 5;
const uint8_t _dirPin = 7;
const uint8_t motorInterfaceType = 1;
const float stepsPerRevolution = 6400.0;

// ======================== STATE VARIABLES ========================
// Latched (toggled) states
bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool forwardToggledState = false;
bool backwardToggledState = false;
bool posMinToggledState = false;
bool posMaxToggledState = false;
bool speedToggledState = false;

// Button debouncing states
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

// ======================== MOTOR SETUP ========================
volatile bool controlPin = false; 
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
mbed::Ticker stepperTicker;

// Hardware timer for stepper (non-blocking)
void updateMotor() {
  if (controlPin) {
    stepper.runSpeed();
  }
}

void setup() {
  // --- Pin Setup ---
  pinMode(scramPin, INPUT);
  pinMode(powerPin, INPUT);
  pinMode(electromagnetPin, INPUT);
  pinMode(forwardPin, INPUT);
  pinMode(backwardPin, INPUT);
  pinMode(rodPositionMinPin, INPUT);
  pinMode(rodPositionMaxPin, INPUT);
  pinMode(speedPin, INPUT);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(5, OUTPUT); // Kept from Serial sketch

  pinMode(positionSetPin, INPUT);
  pinMode(positionReadPin, INPUT);
  pinMode(rotaryKnobReadPin1, INPUT);
  pinMode(rotaryKnobReadPin2, INPUT);

  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin, HIGH);

  // --- Stepper Config ---
  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);
  stepper.setMinPulseWidth(5);

  // --- Serial Setup ---
  Serial.begin(115200);   // USB
  Serial1.begin(115200);  // Hardware TX/RX
  Serial.setTimeout(10);  // Prevents USB string reading from lagging the Ethernet loop

  // --- Ethernet Setup ---
  Ethernet.begin(mac, ip);
  SPI1.begin(); 
  
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
    while (true) { delay(1); }
  }

  server.begin();
  commandServer.begin();

  Ethernet.setRetransmissionTimeout(50);
  Ethernet.setRetransmissionCount(2);
  ethernetLinkUp = (Ethernet.linkStatus() == LinkON);

  // --- Attach Ticker ---
  stepperTicker.attach(&updateMotor, 50us);
}

void loop() {
  // ======================== 1. READ ALL PHYSICAL INPUTS ========================
  bool currentScramReading = digitalRead(scramPin);
  bool currentPowerReading = digitalRead(powerPin);
  bool currentMagnetReading = digitalRead(electromagnetPin);
  bool currentForwardReading = digitalRead(forwardPin);
  bool currentBackwardReading = digitalRead(backwardPin);
  bool currentPosMinReading = digitalRead(rodPositionMinPin);
  bool currentPosMaxReading = digitalRead(rodPositionMaxPin);
  bool currentSpeedReading = digitalRead(speedPin);

  uint16_t positionSet = analogRead(positionSetPin);
  uint16_t positionRead = analogRead(positionReadPin);
  uint16_t rotaryKnob1Read = analogRead(rotaryKnobReadPin1);
  uint16_t rotaryKnob2Read = analogRead(rotaryKnobReadPin2);

  // Active (unlatched) logic
  bool scramActive = (currentScramReading == LOW);
  bool powerActive = (currentPowerReading == LOW);
  bool magnetActive = (currentMagnetReading == LOW);
  bool forwardActive = (currentForwardReading == HIGH);
  bool backwardActive = (currentBackwardReading == HIGH);
  bool maxPositionActive = (currentPosMaxReading == HIGH);
  bool minPositionActive = (currentPosMinReading == HIGH);
  bool fast_Slow = (currentSpeedReading == LOW);

  // ======================== 2. DEBOUNCE LATCHED BUTTONS ========================
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

  // ==================== 3. INCOMING USB SERIAL COMMANDS ====================
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim(); 

    if (command.length() > 0) {
      if (command.equalsIgnoreCase("scram") || command.equalsIgnoreCase("s")) scramToggledState = !scramToggledState;
      else if (command.equalsIgnoreCase("power") || command.equalsIgnoreCase("p")) powerToggledState = !powerToggledState;
      else if (command.equalsIgnoreCase("magnet") || command.equalsIgnoreCase("m")) magnetToggledState = !magnetToggledState;
      else if (command.equalsIgnoreCase("forward") || command.equalsIgnoreCase("f")) forwardToggledState = !forwardToggledState;
      else if (command.equalsIgnoreCase("backward") || command.equalsIgnoreCase("b")) backwardToggledState = !backwardToggledState;
      else if (command.equalsIgnoreCase("min") || command.equalsIgnoreCase("minpos")) posMinToggledState = !posMinToggledState;
      else if (command.equalsIgnoreCase("max") || command.equalsIgnoreCase("maxpos")) posMaxToggledState = !posMaxToggledState;
      else if (command.equalsIgnoreCase("speed") || command.equalsIgnoreCase("sp")) speedToggledState = !speedToggledState;
      else if (command.startsWith("help")) {
        Serial.println("Commands: scram(s), power(p), magnet(m), forward(f), backward(b), min, max, speed(sp)");
      }
    }
  }

  // ==================== 4. INCOMING HARDWARE SERIAL (0x24 0xFF VERIFICATION) ====================
  static enum { IDLE, SAW_24, SAW_FF } serialState = IDLE;

  // while (Serial1.available() > 0) {
  //   uint8_t incoming = Serial1.read();
  //   switch (serialState) {
  //     case IDLE:
  //       if (incoming == 0x24) serialState = SAW_24;
  //       break;
  //     case SAW_24:
  //       if (incoming == 0xFF) serialState = SAW_FF;
  //       else serialState = IDLE;
  //       break;
  //     case SAW_FF:
  //       uint8_t flipMask = incoming;
  //       if (flipMask & (1 << 0)) scramToggledState    = !scramToggledState;
  //       if (flipMask & (1 << 1)) powerToggledState    = !powerToggledState;
  //       if (flipMask & (1 << 2)) magnetToggledState   = !magnetToggledState;
  //       if (flipMask & (1 << 3)) forwardToggledState  = !forwardToggledState;
  //       if (flipMask & (1 << 4)) backwardToggledState = !backwardToggledState;
  //       if (flipMask & (1 << 5)) posMinToggledState   = !posMinToggledState;
  //       if (flipMask & (1 << 6)) posMaxToggledState   = !posMaxToggledState;
  //       if (flipMask & (1 << 7)) speedToggledState    = !speedToggledState;
  //       serialState = IDLE; 
  //       break;
  //   }
  // }

  // ======================== 5. MOTOR MATH & DIRECTION ========================
  static int lastPositionSet = -1;
  const int hysteresis = 4;

  if (abs((int)positionSet - lastPositionSet) >= hysteresis) {
    lastPositionSet = positionSet;
    float targetRPM = positionSet / 50.0;
    float stepsPerSecond = targetRPM * stepsPerRevolution / 50.0; 
    stepper.setSpeed(stepsPerSecond);
  }

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

  // ======================== 6. BUILD DATA PACKETS ========================
  uint8_t packet1 = 0;
  uint8_t packet2 = 0;

  if (forwardActive) packet1 |= (1 << 3); 
  if (backwardActive) packet1 |= (1 << 4); 
  if (minPositionActive) packet1 |= (1 << 5);
  if (maxPositionActive) packet1 |= (1 << 6); 

  if (scramToggledState)  packet1 |= (1 << 0);
  if (powerToggledState)  packet1 |= (1 << 1); 
  if (magnetToggledState) packet1 |= (1 << 2); 
  if (speedToggledState)  packet1 |= (1 << 7);

  if (controlPin) packet2 |= (1 << 0); 

  // ======================== 7. PERIODIC SERIAL PUSH ========================
  unsigned long now = millis();
  if (now - lastTxTime >= txIntervalMs) {
    lastTxTime = now;

    // Hardware TX
    Serial1.write(packet1);
    Serial1.write(packet2);
    Serial1.write(highByte(positionSet));
    Serial1.write(lowByte(positionSet));
    Serial1.write(highByte(positionRead));
    Serial1.write(lowByte(positionRead));
    Serial1.write(highByte(rotaryKnob1Read));
    Serial1.write(lowByte(rotaryKnob1Read));
    Serial1.write(highByte(rotaryKnob2Read));
    Serial1.write(lowByte(rotaryKnob2Read));
    Serial1.write(0b00100100);

    // USB Serial Debug
    Serial.write(packet1);
    Serial.write(packet2);
    Serial.write(highByte(positionSet));
    Serial.write(lowByte(positionSet));
    Serial.write(highByte(positionRead));
    Serial.write(lowByte(positionRead));
    Serial.write(highByte(rotaryKnob1Read));
    Serial.write(lowByte(rotaryKnob1Read));
    Serial.write(highByte(rotaryKnob2Read));
    Serial.write(lowByte(rotaryKnob2Read));
    Serial.write(0b00100100);
  }

  // ======================== 8. ETHERNET NON-BLOCKING TASKS ========================
  if (now - lastLinkCheck >= 500) {
    lastLinkCheck = now;
    ethernetLinkUp = (Ethernet.linkStatus() == LinkON);
  }

  // HTTP JSON Server (Port 80)
  EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && currentLineIsBlank) {
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
        if (c == '\n') currentLineIsBlank = true;
        else if (c != '\r') currentLineIsBlank = false;
      }
    }
    delay(1);
    client.stop();
  }

  // Pi Command Server (Port 5005)
  EthernetClient cmdClient = commandServer.available();
  if (cmdClient) {
    String jsonStr = "";
    unsigned long timeout = millis() + 500; 
    while (cmdClient.connected() && millis() < timeout) {
      if (cmdClient.available()) jsonStr += (char)cmdClient.read();
    }
    cmdClient.stop(); 
    
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (!err) {
      // Parse commands here if needed later
    }
  }

  // Periodic Push to Pi (Port 6006)
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

      serializeJson(doc, outgoing); 
      outgoing.print('\n'); 
      outgoing.flush(); 
      outgoing.stop();
    } else {
      ethernetLinkUp = false;
    }
  }
}