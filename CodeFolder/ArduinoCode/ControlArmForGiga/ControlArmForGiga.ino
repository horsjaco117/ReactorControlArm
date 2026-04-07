#include <AccelStepper.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>

//Ethernet Communication
//MAC address
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// IP address
IPAddress ip(134, 50, 51, 21);
IPAddress piIP(134, 50, 51, 24);
EthernetServer server(80);

// === NEW: Settings for periodic push to Pi ===
const uint16_t piPort = 6006;                    // Port your Pi is listening on
const unsigned long pushIntervalMs = 1000;       // Send JSON to Pi every 1 second (adjust as needed)
unsigned long lastPushTime = 0;

//All Variables
//Analog input variables---------------------------------------------------------
int positionSetPin = A0;      //The position set voltage wire needs to go to A0
int positionReadPin = A1;     //The position read voltage wire needs to go to A1
int rotaryKnobReadPin1 = A2;  //The rotaryknob1 voltage wire needs to go to A2
int rotaryKnobReadPin2 = A3;  //The rotaryknob2 voltage wire needs to go to A3

//Digital input variables -------------------------------------------------------

//packet 1
int scramPin = 53;          //Wire the SCRAM button to this pin
int powerPin = 52;          //Wire the software power button to this pin
int electromagnetPin = 51;  //Wire the button for the electromagnet to this pin
int forwardPin = 50;        //Wire the part of switch to move forward to this pin
int backwardPin = 49;       //Wire the part of switch to move backward to this pin
int rodPositionMinPin = 48; //Wire the min rod position to this pin
int rodPositionMaxPin = 47; //Wire the max rod position to this pin

//packet 2
int speedPin = 46;          //Wire the switch that determines speed to this pin

//Test variables--------------------------------------------------------------------
//Sets default for the variable tracking button states
bool scramToggledState = false;
bool powerToggledState = false;
bool magnetToggledState = false;
bool forwardToggledState = false;
bool backwardToggledState = false;
bool posMinToggledState = false;
bool posMaxToggledState = false;
bool speedToggledState = false;

//Other set of variables to track button states
bool lastScramButtonReading = HIGH;
bool lastPowerButtonReading = HIGH; 
bool lastMagnetButtonReading = HIGH;
bool lastForwardButtonReading = HIGH;
bool lastBackwardButtonReading = HIGH;
bool lastPosMinButtonReading = HIGH;
bool lastPosMaxButtonReading = HIGH;
bool lastSpeedButtonReading = HIGH;

//Software Debounce
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

//End of test variables ^-----------------------

//PWM input
bool controlPin = false; //Controls were tied to pin 7 but now run off of a variable

//Digital Output Variables--------------------------------------

//PWM motor outputs
const uint8_t stepPin = 3;            //Outputs the PWM signal
const uint8_t dirPin = 2;             //Ties to the positive direction pin of the stepper driver
const uint8_t _dirPin = 4;            //Ties to the negative direction pin of the stepper driver
const uint8_t motorInterfaceType = 1; //Proprietary motor type for the header file

//Variables for movement adjustments on the stepper
const float stepsPerRevolution = 6400.0;    //This is tied to the stepper driver switch settings
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//Serial variables HYSTERESIS
unsigned long lastTxTime = 0;             //Two transmission variable for more stable sending
const unsigned long txIntervalMs = 200;

void setup() {//Digital Input Setup
  //Digital input setup--------------------------------------------------------------------------

  //Packet 1
  pinMode(53, INPUT);              //Scram pin
  pinMode(52, INPUT);              //Power pin
  pinMode(51, INPUT);              //Electromagnet pin
  pinMode(50, INPUT);              //Tells Arm to move forward
  pinMode(49, INPUT);              //Tells arm to move backward
  pinMode(48, INPUT);              //Tells the controller max position has been reached
  pinMode(47, INPUT);              //Tells the controller the min position has been reached

  //Packet 2
  pinMode(46, INPUT);              //Tells the controller desired speed option

  //Digital input for PWM and stepper
  pinMode(3, OUTPUT);             //Connects to +Dir of Stepper Driver
  pinMode(4, OUTPUT);             //Connects to +Pul of Stepper Driver
  pinMode(5,OUTPUT);              //Connects to _Dir of Stepper Driver

  
  //Analog input setup---------------------------------------------------------------------------------
  pinMode(A0,INPUT);                //Analog position for the control rod
  pinMode(A1, INPUT);               //Analog voltage indicating the position of the rod
  pinMode(A2, INPUT);               //Analog voltage indicating position of the rotary knob
  pinMode(A3, INPUT);               //Analog voltage indicating position of the second rotary knob

  pinMode(10,OUTPUT);

  // Stepper configuration (moved here - only needs to run once)
  stepper.setMaxSpeed(12800.0);     //Max Speed
  stepper.setAcceleration(6400.0);  //Acceleration
  stepper.setMinPulseWidth(5);      //Pulse width config for the Timers

  //Serial setup------------------------------------------
  Serial1.begin(9600);       //Tx Pin        
  Serial.begin(9600);       // USB debugging

  Serial.println("Ethernet WebServerExample");

  //Ethernet Setup
  Ethernet.begin(mac, ip);

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));  // optional tuning
// But more importantly, use shorter transactions if possible

 // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

}

void loop() {

  //Boolean Assignments for the digital input pins
  bool currentScramReading = digitalRead(scramPin);
  bool currentPowerReading = digitalRead(powerPin);
  bool currentMagnetReading = digitalRead(electromagnetPin);
  bool currentForwardReading = digitalRead(forwardPin);
  bool currentBackwardReading = digitalRead(backwardPin);
  bool currentPosMinReading = digitalRead(rodPositionMinPin);
  bool currentPosMaxReading = digitalRead(rodPositionMaxPin);
  bool currentSpeedReading = digitalRead(speedPin);

  //This is for the Serial
  static unsigned long lastSyncTime = 0;
  
  //Logic for toggling buttons as latched
  if (currentScramReading == LOW && lastScramButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      scramToggledState = !scramToggledState;
      lastDebounceTime = millis();
    }
  }

  lastScramButtonReading = currentScramReading;

  // 2. Toggle Logic for Power
  if (currentPowerReading == LOW && lastPowerButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      powerToggledState = !powerToggledState; 
      lastDebounceTime = millis();
    }
  }
  // This update MUST be outside the IF block to track the button release
  lastPowerButtonReading = currentPowerReading; 

  // 3. Toggle Logic for Magnet
  if (currentMagnetReading == LOW && lastMagnetButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      magnetToggledState = !magnetToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastMagnetButtonReading = currentMagnetReading;

  // 4. Toggle Logic for Forward Logic
  if (currentForwardReading == LOW && lastForwardButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      forwardToggledState = !forwardToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastForwardButtonReading = currentForwardReading; 

    // 5. Toggle Logic for Backward Logic
  if (currentBackwardReading == LOW && lastBackwardButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      backwardToggledState = !backwardToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastBackwardButtonReading = currentBackwardReading; 

    // 6. Toggle Logic for Minimum Position
  if (currentPosMinReading == LOW && lastPosMinButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      posMinToggledState = !posMinToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastPosMinButtonReading = currentPosMinReading; 

    // 7. Toggle logic for maximum Position
    if (currentPosMaxReading == LOW && lastPosMaxButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      posMaxToggledState = !posMaxToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastPosMaxButtonReading = currentPosMaxReading; 

    // 8. Toggle logic for High or low speed
    if (currentSpeedReading == LOW && lastSpeedButtonReading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      speedToggledState = !speedToggledState; 
      lastDebounceTime = millis();
    }
  }
  lastSpeedButtonReading = currentSpeedReading; 

   // ==================== SERIAL READ WITH 0x24 FF VERIFICATION ====================
  // Only flips the toggled states when it receives the exact sequence: 0x24 followed by 0xFF,
  // then the flipMask byte.
  // This adds strong verification before any change is applied.
  static enum { IDLE, SAW_24, SAW_FF } serialState = IDLE;

  while (Serial1.available() > 0) {
    uint8_t incoming = Serial1.read();

    switch (serialState) {
      case IDLE:
      //First handshake byte
        if (incoming == 0x24) {
          serialState = SAW_24;
        }
        break;

      case SAW_24:
      //Second Handshake byte
        if (incoming == 0xFF) {
          serialState = SAW_FF;        // Ready for the flipMask
        } else {
          serialState = IDLE;          // Wrong second byte → reset
        }
        break;

      case SAW_FF:
        // This byte is the flipMask — apply the flips
        uint8_t flipMask = incoming;

        //Same as previous latching sequence but it changes with the digital serial is received
        if (flipMask & (1 << 0)) scramToggledState   = !scramToggledState;
        if (flipMask & (1 << 1)) powerToggledState   = !powerToggledState;
        if (flipMask & (1 << 2)) magnetToggledState  = !magnetToggledState;
        if (flipMask & (1 << 3)) forwardToggledState = !forwardToggledState;
        if (flipMask & (1 << 4)) backwardToggledState = !backwardToggledState;
        if (flipMask & (1 << 5)) posMinToggledState  = !posMinToggledState;
        if (flipMask & (1 << 6)) posMaxToggledState  = !posMaxToggledState;
        if (flipMask & (1 << 7)) speedToggledState   = !speedToggledState;

        serialState = IDLE;   // Reset for next command
        break;
    }
  }
  // =====================================================================
  // 4. Stepper Motor Update
  // Keep runSpeed() outside of an IF if possible, or ensure controlPin is reliable
  if(controlPin) {
    stepper.runSpeed();
  }

  //Define variable again---------------------------------
  uint8_t packet1 = 0;              //Packet that sends the digital bytes (See packet setup below) 8 bit data
  uint8_t packet2 = 0;              //I ran out of space so this has the speed indicator
  uint16_t positionSet = 0;         //Packet that sends where to set the position voltage set as 16 bits the controller only sends 10 bits
  uint16_t positionRead = 0;        //Packet that sends where the position voltage is. Set as 16 bits only sends 10 bits
  uint16_t rotaryKnob1Read = 0;     //Packet that sends the 1st rotary knob voltage. Set as 16 bits only sends 10 bits
  uint16_t rotaryKnob2Read = 0;     //Packet that sends the 2nd rotary knob voltage. Set as 16 bits only sends 10 bits

  //Analog Inputs-----------------------------------------
  positionSet = analogRead(positionSetPin);           //Assigns the digital value of the position set to variable. Variable is what sends through serial
  positionRead = analogRead(positionReadPin);         //Assigns the digital value of the Position read to variable. Variable is what sends through serial
  rotaryKnob1Read = analogRead(rotaryKnobReadPin1);   //Assigns the digital value of the rotaryKnob1 read to variable. Variable is what sends through serial
  rotaryKnob2Read = analogRead(rotaryKnobReadPin2);   //Assigns the digital value of the rotaryKnob2 read to variable. Variable is what sends through serial

  // Update speed only when the setpoint changes (with small hysteresis)
  static int lastPositionSet = -1;
  const int hysteresis = 4;   //hysteresis accounts for the LSB switching all the time

  if (abs((int)positionSet - lastPositionSet) >= hysteresis) {    //Provides smooth stepping and RPM control
    lastPositionSet = positionSet;

    float targetRPM = positionSet / 50;
    float stepsPerSecond = targetRPM * stepsPerRevolution / 50.0;   //Target RPM in charge of changing speed
    stepper.setSpeed(stepsPerSecond);
  }

  //First digital packet(And other Serial Handling)----------------------------------
  bool scramActive = (digitalRead(scramPin) == LOW);                 //This section of code tells pins whether to activate at 5v(high) or .7V(low)
  bool powerActive = (digitalRead(powerPin) == LOW);                 //The variables are assigned as boolean type 
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);        //See Packet setup for which bit is what function
  bool forwardActive = (digitalRead(forwardPin) == HIGH);
  bool backwardActive = (digitalRead(backwardPin) == HIGH);
  bool maxPositionActive = (digitalRead(rodPositionMaxPin) == HIGH);
  bool minPositionActive = (digitalRead(rodPositionMinPin) == HIGH);

    //Direction Control test code-----------------------
  if (forwardActive) {
    digitalWrite(dirPin, LOW);      // Forward = dir LOW (0)
    digitalWrite(_dirPin, HIGH);
    controlPin = true;
  }
  else if (backwardActive) {
    digitalWrite(dirPin, HIGH);     // Backward = dir HIGH (1)
    digitalWrite(_dirPin, LOW);
    controlPin = true;
  }
  else{
    controlPin = false;             //Ensures the motor stops with no input
  }

  //Second digital packet----------------------------------
  bool fast_Slow = (digitalRead(speedPin) == LOW);
  //bool exampleFunction = (digtialRead(examplePin) == Low);

  //Packet1 Setup---------------------------------------------
  
  //Unlatched variables
  // if (scramActive) packet1 |= (1 << 0);         //1st bit activates SCRAM condition
  // if (powerActive) packet1 |= (1 << 1);         //2nd bit activates power
  // if (magnetActive) packet1 |= (1 << 2);        //3rd bit activates the electromagnet
  if (forwardActive) packet1 |= (1 << 3);       //4th bit activates the forward action of the stepper
  if (backwardActive) packet1 |= (1 << 4);      //5th bit activates the backward action of the stepper
  if (minPositionActive) packet1 |= (1 << 5);   //6th bit activates the pause movement at max range
  if (maxPositionActive) packet1 |= (1 << 6);   //7th bit activates the pause movement at min range

  //Latched Serial
  if (scramToggledState)  packet1 |= (1 << 0);
  if (powerToggledState)  packet1 |= (1 << 1); // Uses the toggled state
  if (magnetToggledState) packet1 |= (1 << 2); // Uses the toggled state
  //if (forwardToggledState) packet1 |= (1 << 3);
  //if (backwardToggledState) packet1 |= (1 << 4);
  //if (posMinToggledState) packet1 |= (1<< 5);
  //if (posMaxToggledState) packet1 |= (1 << 6);
  if (speedToggledState) packet1 |= (1<< 7);


  //Packet2 Setup
  if (controlPin) packet2 |= (1<< 0);            //8th bit is a debug variable right now to see if PWM should output
  //if (fast_Slow) packet2 |= (1 << 0);
  //if (bitExample) packet1 |= (1 << bit#);

  //Test serial code 
  unsigned long now = millis();
  if (now - lastTxTime >= txIntervalMs) {
    lastTxTime = now;

    //Serial1.flush();                  // Wait for any previous transmission to complete
    //Serial.println(positionSet);

    //Serial1.write(0b00100100);                //HEX 24 only for triggering on the oscope
    //TX and RX comms
    Serial1.write(packet1);                   //1st set of digital inputs
    Serial1.write(packet2);                   //2nd set of digital inputs
    Serial1.write(highByte(positionSet));     //Sets position of the control rod
    Serial1.write(lowByte(positionSet));
    Serial1.write(highByte(positionRead));    //Reads the position of the control rod
    Serial1.write(lowByte(positionRead));
    Serial1.write(highByte(rotaryKnob1Read)); //For the knob on the control panel
    Serial1.write(lowByte(rotaryKnob1Read));
    Serial1.write(highByte(rotaryKnob2Read)); //For the other knob of the control panel
    Serial1.write(lowByte(rotaryKnob2Read));
    Serial1.write(0b00100100);


    //Test code for serial USB comms
    //TX and RX comms
    // Serial.println(packet1);                   //1st set of digital inputs
    // Serial.println(packet2);                   //2nd set of digital inputs
    // Serial.println(highByte(positionSet));     //Sets position of the control rod
    // Serial.println(lowByte(positionSet));
    // Serial.println(highByte(positionRead));    //Reads the position of the control rod
    // Serial.println(lowByte(positionRead));
    // Serial.println(highByte(rotaryKnob1Read)); //For the knob on the control panel
    // Serial.println(lowByte(rotaryKnob1Read));
    // Serial.println(highByte(rotaryKnob2Read)); //For the other knob of the control panel
    // Serial.println(lowByte(rotaryKnob2Read));
    // Serial.println(24);
  }

  // ==================== ETHERNET JSON WEB SERVER RESPONSE ====================
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      stepper.runSpeed(); //This ensure the Stepper motor runs while connected to etherenet
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // === JSON RESPONSE INSTEAD OF HTML TABLE ===
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: application/json");
          client.println("Connection: close");  
          client.println("Access-Control-Allow-Origin: *"); // Allows cross-origin requests (useful for testing)
          client.println();

          // Build JSON document with all relevant data
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

          // Send the JSON directly over Ethernet
          serializeJson(doc, client);
          client.println();

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
  if (now - lastPushTime >= pushIntervalMs) {
    lastPushTime = now;

    EthernetClient outgoing;
    if (outgoing.connect(piIP, piPort)) {
      StaticJsonDocument<384> doc;

      //JSON stuff and organizing through SPI to Ethernet

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
      outgoing.println();        // Makes it easy for Pi to read line-by-line
      outgoing.stop();           // Important: close after sending

      Serial.println("Pushed JSON to Pi");
    } else {
      //Serial.println("Failed to connect to Pi for push");
    }
}
}