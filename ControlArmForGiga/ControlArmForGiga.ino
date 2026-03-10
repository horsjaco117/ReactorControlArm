//Analog input variables
int positionSetPin = A0;
int positionReadPin = A1;
int rotaryKnobReadPin1 = A2;
int rotaryKnobReadPin2 = A3;

//Digital input variables
int scramPin = 22;
int powerPin = 23;
int electromagnetPin = 24;
int forwardPin = 25;
int backwardPin = 26;
int rodPositionMinPin = 27;
int rodPositionMaxPin = 28;

//Serial Variables to send
int analogIn1 = 0;
int digitalInputs1 = 0;

void setup() {
  //Digital input setup------------------------------------
  pinMode(D22, INPUT);              //Scram pin
  pinMode(D23, INPUT);              //Power pin
  pinMode(D24, INPUT);              //Electromagnet pin
  pinMode(D25, INPUT);
  pinMode(D26, INPUT);
  pinMode(D27, INPUT);
  pinMode(D28, INPUT);

  //Analog input setup-----------------------------------
  pinMode(A0,INPUT);                //Analog position for the control rod
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  //Serial setup------------------------------------------
  Serial.begin(9600);               // USB debugging
  Serial1.begin(9600, SERIAL_8N1);  // TX1 output 1, 9600 baude, 8 bit packets, no parity bit
}

void loop() {
  //Define variable again---------------------------------
  uint8_t packet1 = 0;
  uint16_t positionSet = 0;
  uint16_t positionRead = 0;
  uint16_t rotaryKnob1Read = 0;
  uint16_t rotaryKnob2Read = 0;
  //Analog Inputs-----------------------------------------
  positionSet = analogRead(positionSetPin);
  positionRead = analogRead(positionReadPin);
  rotaryKnob1Read = analogRead(rotaryKnobReadPin1);
  rotaryKnob2Read = analogRead(rotaryKnobReadPin2);

  //First digital packet----------------------------------
  bool scramActive = (digitalRead(scramPin) == LOW);
  bool powerActive = (digitalRead(powerPin) == LOW);
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);
  bool forwardActive = (digitalRead(forwardPin) == LOW);
  bool backwardActive = (digitalRead(backwardPin) == LOW);
  bool maxPositionActive = (digitalRead(rodPositionMaxPin) == LOW);
  bool minPositionActive = (digitalRead(rodPositionMinPin) == LOW);


  if (scramActive) packet1 |= (1 << 0);
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