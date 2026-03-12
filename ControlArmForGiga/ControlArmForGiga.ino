#include <AccelStepper.h>

//Analog input variables---------------------------------------------------------
int positionSetPin = A0;      //The position set voltage wire needs to go to A0
int positionReadPin = A1;     //The position read voltage wire needs to go to A1
int rotaryKnobReadPin1 = A2;  //The rotaryknob1 voltage wire needs to go to A2
int rotaryKnobReadPin2 = A3;  //The rotaryknob2 voltage wire needs to go to A3

//Digital input variables -------------------------------------------------------

//packet 1
int scramPin = 22;          //Wire the SCRAM button to this pin
int powerPin = 23;          //Wire the software power button to this pin
int electromagnetPin = 24;  //Wire the button for the electromagnet to this pin
int forwardPin = 25;        //Wire the part of switch to move forward to this pin
int backwardPin = 26;       //Wire the part of switch to move backward to this pin
int rodPositionMinPin = 27; //Wire the min rod position to this pin
int rodPositionMaxPin = 28; //Wire the max rod position to this pin

//packet 2
int speedPin = 29;          //Wire the switch that determines speed to this pin

//PWM input
bool controlPin = false; //Controls were tied to pin 7 but now run off of a variable

//Digital Output Variables--------------------------------------

//PWM motor outputs
const uint8_t stepPin = 6;            //Outputs the PWM signal
const uint8_t dirPin = 5;             //Ties to the positive direction pin of the stepper driver
const uint8_t _dirPin = 4;            //Ties to the negative direction pin of the stepper driver
const uint8_t motorInterfaceType = 1; //Proprietary motor type for the header file



//Variables for movement adjustments on the stepper
const float stepsPerRevolution = 6400.0;    //This is tied to the stepper driver switch settings
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//Serial variables
unsigned long lastTxTime = 0;             //Two transmission variable for more stable sending
const unsigned long txIntervalMs = 200;

void setup() {
  //Digital input setup--------------------------------------------------------------------------

  //Packet 1
  pinMode(D22, INPUT);              //Scram pin
  pinMode(D23, INPUT);              //Power pin
  pinMode(D24, INPUT);              //Electromagnet pin
  pinMode(D25, INPUT);              //Tells Arm to move forward
  pinMode(D26, INPUT);              //Tells arm to move backward
  pinMode(D27, INPUT);              //Tells the controller max position has been reached
  pinMode(D28, INPUT);              //Tells the controller the min position has been reached

  //Packet 2
  pinMode(D29, INPUT);              //Tells the controller desired speed option

  //Digital input for PWM and stepper
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6,OUTPUT);

  
  //Analog input setup---------------------------------------------------------------------------------
  pinMode(A0,INPUT);                //Analog position for the control rod
  pinMode(A1, INPUT);               //Analog voltage indicating the position of the rod
  pinMode(A2, INPUT);               //Analog voltage indicating position of the rotary knob
  pinMode(A3, INPUT);               //Analog voltage indicating position of the second rotary knob

  // Stepper configuration (moved here - only needs to run once)
  stepper.setMaxSpeed(12800.0);
  stepper.setAcceleration(6400.0);
  stepper.setMinPulseWidth(5);

  //Serial setup------------------------------------------
 // Serial.begin(9600);               // USB debugging
  Serial1.begin(115200, SERIAL_8N1);  // TX1 output 1, 9600 baude, 8 bit packets, no parity bit
}

void loop() {
  //Start by checking if the motor should be running
  // This must be called frequently for smooth stepping
  if(controlPin) {
    stepper.runSpeed();
  }

  //Define variable again---------------------------------
  uint8_t packet1 = 0;              //Packet that sends the digital bytes (See packet setup below) 8 bit data
  uint8_t packet2 = 0;
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

  //First digital packet----------------------------------
  bool scramActive = (digitalRead(scramPin) == LOW);                 //This section of code tells pins whether to activate at 5v(high) or .7V(low)
  bool powerActive = (digitalRead(powerPin) == LOW);                 //The variables are assigned as boolean type 
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);        //See Packet setup for which bit is what function
  bool forwardActive = (digitalRead(forwardPin) == LOW);
  bool backwardActive = (digitalRead(backwardPin) == LOW);
  bool maxPositionActive = (digitalRead(rodPositionMaxPin) == LOW);
  bool minPositionActive = (digitalRead(rodPositionMinPin) == LOW);

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
  if (scramActive) packet1 |= (1 << 0);         //1st bit activates SCRAM condition
  if (powerActive) packet1 |= (1 << 1);         //2nd bit activates power
  if (magnetActive) packet1 |= (1 << 2);        //3rd bit activates the electromagnet
  if (forwardActive) packet1 |= (1 << 3);       //4th bit activates the forward action of the stepper
  if (backwardActive) packet1 |= (1 << 4);      //5th bit activates the backward action of the stepper
  if (maxPositionActive) packet1 |= (1 << 5);   //6th bit activates the pause movement at max range
  if (minPositionActive) packet1 |= (1 << 6);   //7th bit activates the pause movement at min range
  if (controlPin) packet1 |= (1<< 7);            //8th bit is a debug variable right now to see if PWM should output

  //Packet2 Setup
  if (fast_Slow) packet2 |= (1 << 0);
  //if (bitExample) packet1 |= (1 << bit#);

  //Test serial code 
  unsigned long now = millis();
  if (now - lastTxTime >= txIntervalMs) {
    lastTxTime = now;

    //Serial1.flush();                  // Wait for any previous transmission to complete
    Serial.println(positionSet);

    Serial1.write(0b00100100);                //HEX 24 only for triggering on the oscope
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
  }
}