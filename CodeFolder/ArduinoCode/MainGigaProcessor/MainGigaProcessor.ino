#include <Arduino.h>
#include <RPC.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>

// === Sensor Global Variables (Updated by M4) ===
int m4_posSet = 0;
int m4_posRead = 0;
int m4_knob1 = 0;
int m4_knob2 = 0;
int m4_packet1 = 0;
int m4_packet2 = 0;

// === RPC Function: Called by M4 to push sensor data ===
void updateSensors(int pSet, int pRead, int k1, int k2, int pack1, int pack2) {
  m4_posSet = pSet;
  m4_posRead = pRead;
  m4_knob1 = k1;
  m4_knob2 = k2;
  m4_packet1 = pack1;
  m4_packet2 = pack2;
}

// === Ethernet Setup ===
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(134, 50, 51, 21);
IPAddress piIP(134, 50, 51, 24);
EthernetServer server(80);
const uint16_t piPort = 6006;
const unsigned long pushIntervalMs = 1000;
unsigned long lastPushTime = 0;

// === Serial & Logic Variables ===
unsigned long lastTxTime = 0;
const unsigned long txIntervalMs = 200;

bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool forwardToggledState = false;
bool backwardToggledState = false;
bool posMinToggledState = false;
bool posMaxToggledState = false;
bool speedToggledState = false;

uint8_t lastDigitalPacket1 = 0;
uint8_t lastDigitalPacket2 = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int lastPositionSet = -1;
const int hysteresis = 4;
float targetStepsPerSecond = 0;

enum { IDLE, SAW_24, SAW_FF } serialState = IDLE;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  RPC.begin();
  RPC.bind("updateSensors", updateSensors); 
  
  LL_RCC_ForceCM4Boot();
  delay(500); 

  Ethernet.begin(mac, ip);
  // SPI.beginTransaction removed — Ethernet library handles it

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet hardware missing!");
    while (true) delay(1);
  }
  
  server.begin();
  Serial.print("M7 Server is at ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  unsigned long now = millis();   // <<< MOVED TO TOP (fixes scoping)

  // 1. Read Raw Logic from the M4's globals
  bool rawScram    = (m4_packet1 & (1 << 0));
  bool rawPower    = (m4_packet1 & (1 << 1));
  bool rawMagnet   = (m4_packet1 & (1 << 2));
  bool rawForward  = (m4_packet1 & (1 << 3));
  bool rawBackward = (m4_packet1 & (1 << 4));
  bool rawPosMin   = (m4_packet1 & (1 << 5));
  bool rawPosMax   = (m4_packet1 & (1 << 6));
  bool rawSpeed    = (m4_packet2 & (1 << 0));



  // 2. Process Debouncing and Toggles
  if (now - lastDebounceTime > debounceDelay) {
    if (rawScram && !(lastDigitalPacket1 & (1 << 0))) scramToggledState = !scramToggledState;
    if (rawPower && !(lastDigitalPacket1 & (1 << 1))) powerToggledState = !powerToggledState;
    if (rawMagnet && !(lastDigitalPacket1 & (1 << 2))) magnetToggledState = !magnetToggledState;
    if (rawSpeed && !(lastDigitalPacket2 & (1 << 0))) speedToggledState = !speedToggledState;
    // Only toggle buttons (scram/power/magnet/speed) are handled here.
    // Forward/backward are momentary. Pos limits are switches.
    
    lastDebounceTime = now;
    lastDigitalPacket1 = m4_packet1;
    lastDigitalPacket2 = m4_packet2;
  }

  // 3. Handle Serial1 Command Handshake (0x24 -> 0xFF -> Mask)
  while (Serial1.available() > 0) {
    uint8_t incoming = Serial1.read();
    switch (serialState) {
      case IDLE:
        if (incoming == 0x24) serialState = SAW_24;
        break;
      case SAW_24:
        if (incoming == 0xFF) serialState = SAW_FF;
        else serialState = IDLE;
        break;
      case SAW_FF:
        uint8_t flipMask = incoming;
        if (flipMask & (1 << 0)) scramToggledState   = !scramToggledState;
        if (flipMask & (1 << 1)) powerToggledState   = !powerToggledState;
        if (flipMask & (1 << 2)) magnetToggledState  = !magnetToggledState;
        if (flipMask & (1 << 3)) forwardToggledState = !forwardToggledState;
        if (flipMask & (1 << 4)) backwardToggledState= !backwardToggledState;
        if (flipMask & (1 << 5)) posMinToggledState  = !posMinToggledState;
        if (flipMask & (1 << 6)) posMaxToggledState  = !posMaxToggledState;
        if (flipMask & (1 << 7)) speedToggledState   = !speedToggledState;
        serialState = IDLE;
        break;
    }
  }

  // 4. Calculate Motor Speed & Direction
  if (abs(m4_posSet - lastPositionSet) >= hysteresis) {
    lastPositionSet = m4_posSet;
    float targetRPM = m4_posSet / 50.0;
    targetStepsPerSecond = targetRPM * (6400.0 / 50.0);
  }

  // 5. Send Motor commands to M4 (throttled + change-detect)  <<< CRITICAL FIX
  static unsigned long lastMotorCmd = 0;
  static float lastTargetSPS = 0.0;
  static bool lastControlPin = false;
  static bool lastIsBackward = false;

  bool controlPin = rawForward || rawBackward;
  
  if ((now - lastMotorCmd >= 20) || 
      (abs(targetStepsPerSecond - lastTargetSPS) > 0.1) ||
      (controlPin != lastControlPin) || (rawBackward != lastIsBackward)) {
    
    RPC.call("updateMotorState", targetStepsPerSecond, (int)controlPin, (int)rawBackward);
    
    lastMotorCmd = now;
    lastTargetSPS = targetStepsPerSecond;
    lastControlPin = controlPin;
    lastIsBackward = rawBackward;
  }

// 6. Handle Ethernet Server (Web Requests)
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
          doc["position_set"]   = m4_posSet;
          doc["position_read"]  = m4_posRead;
          doc["knob1"]          = m4_knob1;
          doc["knob2"]          = m4_knob2;

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

  // 7. Periodic Push to Raspberry Pi
  //unsigned long now = millis();
  if (now - lastPushTime >= pushIntervalMs) {
    lastPushTime = now;
    EthernetClient outgoing;
    if (outgoing.connect(piIP, piPort)) {
      StaticJsonDocument<384> doc;
      doc["scram"]          = scramToggledState;
      doc["power"]          = powerToggledState;
      doc["forward"]        = rawForward;
      doc["backward"]       = rawBackward;
      doc["position_set"]   = m4_posSet;
      doc["position_read"]  = m4_posRead;
      
      serializeJson(doc, outgoing);
      outgoing.println();
      outgoing.stop();
    }
  }
  // 8. Serial1 TX out
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
    Serial1.write(highByte(m4_posSet));
    Serial1.write(lowByte(m4_posSet));
    Serial1.write(highByte(m4_posRead));
    Serial1.write(lowByte(m4_posRead));
    Serial1.write(highByte(m4_knob1));
    Serial1.write(lowByte(m4_knob1));
    Serial1.write(highByte(m4_knob2));
    Serial1.write(lowByte(m4_knob2));
    Serial1.write(0b00100100);
  }
}