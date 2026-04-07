#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>
#include <stm32h7xx_hal.h>   // For cache maintenance

// === Shared Memory Struct (MUST be identical on M4) ===
typedef struct {
  // Motor commands: M7 → M4
  float targetSpeed;
  bool  controlPin;
  bool  isBackward;
  bool  newMotorCmd;

  // Sensor data: M4 → M7
  int   posSet;
  int   posRead;
  int   knob1;
  int   knob2;
  int   digitalPacket1;
  int   digitalPacket2;
  bool  newSensorData;
} SharedData;

SharedData *shared = (SharedData *)0x38000000;   // SRAM4

// === Ethernet Setup ===
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(134, 50, 51, 21);
IPAddress piIP(134, 50, 51, 24);
EthernetServer server(80);
const uint16_t piPort = 6006;
unsigned long lastPushTime = 0;
const unsigned long pushIntervalMs = 1000;

// === Logic Variables ===
unsigned long lastTxTime = 0;
const unsigned long txIntervalMs = 200;

bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool speedToggledState = false;

uint8_t lastDigitalPacket1 = 0;
uint8_t lastDigitalPacket2 = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int lastPositionSet = -1;
const int hysteresis = 4;
float targetStepsPerSecond = 0.0f;

enum { IDLE, SAW_24, SAW_FF } serialState = IDLE;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  LL_RCC_ForceCM4Boot();
  delay(500);

  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet hardware missing!");
    while (true) delay(1);
  }
  
  server.begin();
  Serial.print("M7 Server is at ");
  Serial.println(Ethernet.localIP());

  // Initialize shared memory
  SCB_InvalidateDCache_by_Addr((uint32_t*)shared, sizeof(SharedData));
  memset(shared, 0, sizeof(SharedData));
}

void loop() {
  unsigned long now = millis();

  // === 1. Read fresh sensor data from shared memory ===
  int posSetLocal = 0, posReadLocal = 0, knob1Local = 0, knob2Local = 0;
  bool rawScram = false, rawPower = false, rawMagnet = false;
  bool rawForward = false, rawBackward = false;
  bool rawPosMin = false, rawPosMax = false;
  bool rawSpeed = false;

  if (shared->newSensorData) {
    SCB_InvalidateDCache_by_Addr((uint32_t*)shared, sizeof(SharedData));  // Critical for M7 cache

    posSetLocal   = shared->posSet;
    posReadLocal  = shared->posRead;
    knob1Local    = shared->knob1;
    knob2Local    = shared->knob2;

    int pkt1 = shared->digitalPacket1;
    int pkt2 = shared->digitalPacket2;

    rawScram    = (pkt1 & (1 << 0));
    rawPower    = (pkt1 & (1 << 1));
    rawMagnet   = (pkt1 & (1 << 2));
    rawForward  = (pkt1 & (1 << 3));
    rawBackward = (pkt1 & (1 << 4));
    rawPosMin   = (pkt1 & (1 << 5));
    rawPosMax   = (pkt1 & (1 << 6));
    rawSpeed    = (pkt2 & (1 << 0));

    // Update speed from knob
    if (abs(posSetLocal - lastPositionSet) >= hysteresis) {
      lastPositionSet = posSetLocal;
      float targetRPM = posSetLocal / 50.0;
      targetStepsPerSecond = targetRPM * (6400.0 / 50.0);
    }

    lastDigitalPacket1 = pkt1;
    lastDigitalPacket2 = pkt2;

    shared->newSensorData = false;
  }

  // === 2. Debounce toggles ===
  if (now - lastDebounceTime > debounceDelay) {
    if (rawScram && !(lastDigitalPacket1 & (1 << 0))) scramToggledState = !scramToggledState;
    if (rawPower && !(lastDigitalPacket1 & (1 << 1))) powerToggledState = !powerToggledState;
    if (rawMagnet && !(lastDigitalPacket1 & (1 << 2))) magnetToggledState = !magnetToggledState;
    if (rawSpeed && !(lastDigitalPacket2 & (1 << 0))) speedToggledState = !speedToggledState;
    
    lastDebounceTime = now;
  }

  // === 3. Motor command to shared memory (only on change) ===
  bool controlPin = rawForward || rawBackward;

  static float lastSentSpeed = 0.0f;
  static bool lastSentControl = false;
  static bool lastSentBackward = false;

  if (abs(targetStepsPerSecond - lastSentSpeed) > 0.1f ||
      controlPin != lastSentControl ||
      rawBackward != lastSentBackward) {

    SCB_CleanDCache_by_Addr((uint32_t*)shared, sizeof(SharedData));  // Ensure M4 sees update

    shared->targetSpeed = targetStepsPerSecond;
    shared->controlPin  = controlPin;
    shared->isBackward  = rawBackward;
    shared->newMotorCmd = true;

    lastSentSpeed    = targetStepsPerSecond;
    lastSentControl  = controlPin;
    lastSentBackward = rawBackward;
  }

  // === 4. Ethernet Web Server ===
  EthernetClient client = server.available();
  if (client) {
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n' && currentLineIsBlank) {
          StaticJsonDocument<384> doc;
          doc["scram"]          = scramToggledState;
          doc["power"]          = powerToggledState;
          doc["magnet"]         = magnetToggledState;
          doc["forward"]        = rawForward;
          doc["backward"]       = rawBackward;
          doc["pos_min"]        = rawPosMin;
          doc["pos_max"]        = rawPosMax;
          doc["fast_slow"]      = rawSpeed;
          doc["control_active"] = controlPin;
          doc["position_set"]   = posSetLocal;
          doc["position_read"]  = posReadLocal;
          doc["knob1"]          = knob1Local;
          doc["knob2"]          = knob2Local;

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Connection: close");  
          client.println("Access-Control-Allow-Origin: *");
          client.println();
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

  // === 5. Periodic Push to Raspberry Pi ===
  if (now - lastPushTime >= pushIntervalMs) {
    lastPushTime = now;
    EthernetClient outgoing;
    if (outgoing.connect(piIP, piPort)) {
      StaticJsonDocument<384> doc;
      doc["scram"]          = scramToggledState;
      doc["power"]          = powerToggledState;
      doc["forward"]        = rawForward;
      doc["backward"]       = rawBackward;
      doc["position_set"]   = posSetLocal;
      doc["position_read"]  = posReadLocal;
      
      serializeJson(doc, outgoing);
      outgoing.println();
      outgoing.stop();
    }
  }

  // === 6. Serial1 TX out ===
  if (now - lastTxTime >= txIntervalMs) {
    lastTxTime = now;
    
    uint8_t packet1Out = 0;
    if (scramToggledState) packet1Out |= (1 << 0);
    if (powerToggledState) packet1Out |= (1 << 1);
    if (magnetToggledState) packet1Out |= (1 << 2);
    if (rawForward) packet1Out |= (1 << 3);
    if (rawBackward) packet1Out |= (1 << 4);
    if (rawPosMin) packet1Out |= (1 << 5);
    if (rawPosMax) packet1Out |= (1 << 6);
    if (speedToggledState) packet1Out |= (1 << 7);

    Serial1.write(packet1Out);
    Serial1.write(controlPin ? 1 : 0); 
    Serial1.write(highByte(posSetLocal));
    Serial1.write(lowByte(posSetLocal));
    Serial1.write(highByte(posReadLocal));
    Serial1.write(lowByte(posReadLocal));
    Serial1.write(highByte(knob1Local));
    Serial1.write(lowByte(knob1Local));
    Serial1.write(highByte(knob2Local));
    Serial1.write(lowByte(knob2Local));
    Serial1.write(0b00100100);
  }
}