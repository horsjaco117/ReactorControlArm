//Analog input variables
int positionSetPin = A0;      //The position set voltage wire needs to go to A0
int positionReadPin = A1;     //The position read voltage wire needs to go to A1
int rotaryKnobReadPin1 = A2;  //The rotaryknob1 voltage wire needs to go to A2
int rotaryKnobReadPin2 = A3;  //The rotaryknob2 voltage wire needs to go to A3

//Digital input variables
int scramPin = 22;          //Wire the SCRAM button to this pin
int powerPin = 23;          //Wire the software power button to this pin
int electromagnetPin = 24;  //Wire the button for the electromagnet to this pin
int forwardPin = 25;        //Wire the part of switch to move forward to this pin
int backwardPin = 26;       //Wire the part of switch to move backward to this pin
int rodPositionMinPin = 27; //Wire the min rod position to this pin
int rodPositionMaxPin = 28; //Wire the max rod position to this pin

//Serial Variables to send
int analogIn1 = 0;          //Temporary analog in position variable. Can be discarded
int digitalInputs1 = 0;     //DO NOT DELETE. This sends the current readings for all the digital inputs. 

void setup() {
  //Digital input setup------------------------------------
  pinMode(D22, INPUT);              //Scram pin
  pinMode(D23, INPUT);              //Power pin
  pinMode(D24, INPUT);              //Electromagnet pin
  pinMode(D25, INPUT);              //Tells Arm to move forward
  pinMode(D26, INPUT);              //Tells arm to move backward
  pinMode(D27, INPUT);              //Tells the controller max position has been reached
  pinMode(D28, INPUT);              //Tells the controller the min position has been reached

  //Analog input setup-----------------------------------
  pinMode(A0,INPUT);                //Analog position for the control rod
  pinMode(A1, INPUT);               //Analog voltage indicating the position of the rod
  pinMode(A2, INPUT);               //Analog voltage indicating position of the rotary knob
  pinMode(A3, INPUT);               //Analog voltage indicating position of the second rotary knob
  //Serial setup------------------------------------------
  Serial.begin(9600);               // USB debugging
  Serial1.begin(9600, SERIAL_8N1);  // TX1 output 1, 9600 baude, 8 bit packets, no parity bit
}

void loop() {
  //Define variable again---------------------------------
  uint8_t packet1 = 0;              //Packet that sends the digital bytes (See packet setup below) 8 bit data
  uint16_t positionSet = 0;         //Packet that sends where to set the position voltage set as 16 bits the controller only sends 10 bits
  uint16_t positionRead = 0;        //Packet that sends where the position voltage is. Set as 16 bits only sends 10 bits
  uint16_t rotaryKnob1Read = 0;     //Packet that sends the 1st rotary knob voltage. Set as 16 bits only sends 10 bits
  uint16_t rotaryKnob2Read = 0;     //Packet that sends the 2nd rotary knob voltage. Set as 16 bits only sends 10 bits
  //Analog Inputs-----------------------------------------
  positionSet = analogRead(positionSetPin);           //Assigns the digital value of the position set to variable. Variable is what sends through serial
  positionRead = analogRead(positionReadPin);         //Assigns the digital value of the Position read to variable. Variable is what sends through serial
  rotaryKnob1Read = analogRead(rotaryKnobReadPin1);   //Assigns the digital value of the rotaryKnob1 read to variable. Variable is what sends through serial
  rotaryKnob2Read = analogRead(rotaryKnobReadPin2);   //Assigns the digital value of the rotaryKnob2 read to variable. Variable is what sends through serial

  //First digital packet----------------------------------
  bool scramActive = (digitalRead(scramPin) == LOW);                 //This section of code tells pins whether to activate at 5v(high) or .7V(low)
  bool powerActive = (digitalRead(powerPin) == LOW);                 //The variables are assigned as boolean type 
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);        //See Packet setup for which bit is what function
  bool forwardActive = (digitalRead(forwardPin) == LOW);
  bool backwardActive = (digitalRead(backwardPin) == LOW);
  bool maxPositionActive = (digitalRead(rodPositionMaxPin) == LOW);
  bool minPositionActive = (digitalRead(rodPositionMinPin) == LOW);

    //Packet Setup---------------------------------------------
  if (scramActive) packet1 |= (1 << 0);         //1st bit 
  if (powerActive) packet1 |= (1 << 1); 
  if (magnetActive) packet1 |= (1 << 2);
  if (forwardActive) packet1 |= (1 << 3);
  if (backwardActive) packet1 |= (1 << 4);
  if (maxPositionActive) packet1 |= (1 << 5);
  if (minPositionActive) packet1 |= (1 << 6);
  //if (bit7) packet1 |= (1 << 7);

  Serial1.flush();                  // Wait for any previous transmission to complete
  Serial.println(positionSet);

  Serial1.write(packet1);
  delay(1);
  Serial1.write(highByte(positionSet));
  delay(1);
  Serial1.write(lowByte(positionSet));
  delay(1);
  Serial1.write(highByte(positionRead));
  delay(1);
  Serial1.write(lowByte(positionRead));
  delay(1);
  Serial1.write(highByte(rotaryKnob1Read));
  delay(1);
  Serial1.write(lowByte(rotaryKnob1Read));
  delay(1);
  Serial1.write(highByte(rotaryKnob2Read));
  delay(1);
  Serial1.write(lowByte(rotaryKnob2Read));


  delay(100);                       // Slow down for easier scope/analyzer viewing
}