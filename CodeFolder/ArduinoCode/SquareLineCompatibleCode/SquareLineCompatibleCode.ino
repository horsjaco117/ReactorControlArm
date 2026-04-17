#include "lvgl.h"
#include "ui.h"

#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"

#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>

using namespace std::chrono_literals;

// ======================== DISPLAY & TOUCH ========================
Arduino_H7_Video          Display(800, 480, GigaDisplayShield); 
Arduino_GigaDisplayTouch  Touch;

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
int rodPositionMinPin = 48;
int rodPositionMaxPin = 47;

// Stepper Hardware
const uint8_t stepPin = 6;
const uint8_t dirPin = 5;
const uint8_t _dirPin = 7;
const uint8_t motorInterfaceType = 1;
const float stepsPerRevolution = 6400.0;

// ======================== STATE VARIABLES ========================
bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool forwardActive = false;        
bool backwardActive = false;       
bool speedToggledState = false;

bool minPositionActive = false;    
bool maxPositionActive = false;    
bool fast_Slow = false;            

// ======================== MOTOR SETUP ========================
volatile bool controlPin = false; 
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  Serial.begin(115200);
  delay(3000);

  // --- LVGL + Display + Touch ---
  Display.begin();
  Touch.begin();
  ui_init();                    

  // --- Pin Setup ---
  pinMode(rodPositionMinPin, INPUT);
  pinMode(rodPositionMaxPin, INPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin, HIGH);

  // --- Stepper Config ---
  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);
  stepper.setMinPulseWidth(5);

  // --- Serial Setup ---
  Serial1.begin(115200);
  Serial.setTimeout(10);

  // --- Ethernet Setup ---
  Ethernet.init(CS_Pin); // CRITICAL: Tell library to use pin 10
  Ethernet.begin(mac, ip);
  SPI1.begin(); 

  // Removed locking SPI.beginTransaction calls to prevent bus lockups

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Continuing without network...");
    // Removed the infinite while(true) trap here
  }

  server.begin();
  commandServer.begin();

  Ethernet.setRetransmissionTimeout(50);
  Ethernet.setRetransmissionCount(2);
  ethernetLinkUp = (Ethernet.linkStatus() == LinkON);
}

void loop() {
  // ======================== MOTOR EXECUTION ========================
  // Run this as fast as possible in the loop, completely replacing the interrupt
  if (controlPin) {
    stepper.runSpeed();
  }

  // ======================== LVGL ENGINE (Non-blocking) ========================
  static unsigned long lastLvglTime = 0;
  if (millis() - lastLvglTime >= 5) {
    lastLvglTime = millis();
    lv_timer_handler();
  }

  // ======================== READ VALUES FROM SQUARELINE UI ========================
  uint16_t positionSet      = lv_arc_get_value(ui_PositionSetArc);
  uint16_t rotaryKnob1Read  = lv_arc_get_value(ui_Rotaryknob1read);
  uint16_t rotaryKnob2Read  = lv_arc_get_value(ui_RotaryKnob2Read);

  uint16_t positionRead = positionSet;

  lv_label_set_text_fmt(ui_PositionReadLabel, "%d", positionRead);
  lv_bar_set_value(ui_PositionReadBar, positionRead, LV_ANIM_OFF);

  scramToggledState   = lv_obj_has_state(ui_ScramButton,       LV_STATE_CHECKED);
  powerToggledState   = lv_obj_has_state(ui_PowerButton,       LV_STATE_CHECKED);
  magnetToggledState  = lv_obj_has_state(ui_ElectromagnetButton, LV_STATE_CHECKED);
  speedToggledState   = lv_obj_has_state(ui_SpeedSelectButton, LV_STATE_CHECKED);
  fast_Slow           = speedToggledState;  

  forwardActive  = lv_obj_has_state(ui_ForwardButton,  LV_STATE_PRESSED);
  backwardActive = lv_obj_has_state(ui_BackwardButton, LV_STATE_PRESSED);

  minPositionActive = (digitalRead(rodPositionMinPin) == HIGH);
  maxPositionActive = (digitalRead(rodPositionMaxPin) == HIGH);

  if (minPositionActive) {
    lv_obj_add_state(ui_PositionMinCheckbox, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_PositionMinCheckbox, LV_STATE_CHECKED);
  }
  if (maxPositionActive) {
    lv_obj_add_state(ui_PositionMaxCheckbox, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(ui_PositionMaxCheckbox, LV_STATE_CHECKED);
  }

  // ======================== MOTOR MATH & DIRECTION ========================
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

  // ======================== BUILD DATA PACKETS ========================
  uint8_t packet1 = 0;
  uint8_t packet2 = 0;

  if (forwardActive)  packet1 |= (1 << 3); 
  if (backwardActive) packet1 |= (1 << 4); 
  if (minPositionActive) packet1 |= (1 << 5);
  if (maxPositionActive) packet1 |= (1 << 6); 

  if (scramToggledState)  packet1 |= (1 << 0);
  if (powerToggledState)  packet1 |= (1 << 1); 
  if (magnetToggledState) packet1 |= (1 << 2); 
  if (speedToggledState)  packet1 |= (1 << 7);

  if (controlPin) packet2 |= (1 << 0); 

  // ======================== INCOMING USB SERIAL COMMANDS ========================
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim(); 

    if (command.length() > 0) {
      if (command.equalsIgnoreCase("scram") || command.equalsIgnoreCase("s")) {
        scramToggledState = !scramToggledState;
        if (scramToggledState) lv_obj_add_state(ui_ScramButton, LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_ScramButton, LV_STATE_CHECKED);
      }
      else if (command.equalsIgnoreCase("power") || command.equalsIgnoreCase("p")) {
        powerToggledState = !powerToggledState; 
        if (powerToggledState) lv_obj_add_state(ui_PowerButton, LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_PowerButton, LV_STATE_CHECKED);
      }
      else if (command.equalsIgnoreCase("magnet") || command.equalsIgnoreCase("m")) {
        magnetToggledState = !magnetToggledState; 
        if (magnetToggledState) lv_obj_add_state(ui_ElectromagnetButton, LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_ElectromagnetButton, LV_STATE_CHECKED);
      }
      else if (command.equalsIgnoreCase("speed") || command.equalsIgnoreCase("sp")) {
        speedToggledState = !speedToggledState; 
        if (speedToggledState) lv_obj_add_state(ui_SpeedSelectButton, LV_STATE_CHECKED);
        else lv_obj_clear_state(ui_SpeedSelectButton, LV_STATE_CHECKED);
      }
      else if (command.startsWith("help")) {
        Serial.println("Commands: scram(s), power(p), magnet(m), speed(sp)");
      }
    }
  }

  // ======================== INCOMING HARDWARE SERIAL (0x24 0xFF) ========================
  static enum { IDLE, SAW_24, SAW_FF } serialState = IDLE;
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
      case SAW_FF: {
        uint8_t flipMask = incoming;
        if (flipMask & (1 << 0)) { scramToggledState = !scramToggledState; if (scramToggledState) lv_obj_add_state(ui_ScramButton, LV_STATE_CHECKED); else lv_obj_clear_state(ui_ScramButton, LV_STATE_CHECKED); }
        if (flipMask & (1 << 1)) { powerToggledState = !powerToggledState; if (powerToggledState) lv_obj_add_state(ui_PowerButton, LV_STATE_CHECKED); else lv_obj_clear_state(ui_PowerButton, LV_STATE_CHECKED); }
        if (flipMask & (1 << 2)) { magnetToggledState = !magnetToggledState; if (magnetToggledState) lv_obj_add_state(ui_ElectromagnetButton, LV_STATE_CHECKED); else lv_obj_clear_state(ui_ElectromagnetButton, LV_STATE_CHECKED); }
        if (flipMask & (1 << 7)) { speedToggledState = !speedToggledState; if (speedToggledState) lv_obj_add_state(ui_SpeedSelectButton, LV_STATE_CHECKED); else lv_obj_clear_state(ui_SpeedSelectButton, LV_STATE_CHECKED); }
        serialState = IDLE; 
        break;
      }
    }
  }

  // ======================== PERIODIC SERIAL PUSH ========================
  unsigned long now = millis();
  if (now - lastTxTime >= txIntervalMs) {
    lastTxTime = now;

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

  // ======================== ETHERNET NON-BLOCKING TASKS ========================
  if (now - lastLinkCheck >= 500) {
    lastLinkCheck = now;
    ethernetLinkUp = (Ethernet.linkStatus() == LinkON);
  }

  // HTTP JSON Server (Port 80) - Made non-blocking
  EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.available()) {
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
    client.stop();
  }

  // Pi Command Server (Port 5005) - Made non-blocking
  EthernetClient cmdClient = commandServer.available();
  if (cmdClient) {
    String jsonStr = "";
    while (cmdClient.available()) {
      jsonStr += (char)cmdClient.read();
    }
    cmdClient.stop(); 
    
    if (jsonStr.length() > 0) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, jsonStr);
      if (!err) {
        // Parse commands here if needed later
      }
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