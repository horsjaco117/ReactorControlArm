//Analog input variables
int positionSetPin = A0;
int positionReadPin = A1;
int rotaryKnobPin1 = A2;
int rotaryKnobPin2 = A3;

//Digital input variables
int scramPin = 22;
int powerPin = 23;
int electromagnetPin = 24;

//Serial Variables to send
int analogIn1 = 0;
int digitalInputs1 = 0;

void setup() {
  //Digital input setup------------------------------------
  pinMode(D22, INPUT);              //Scram pin
  pinMode(D23, INPUT);              //Power pin
  pinMode(D24, INPUT);              //Electromagnet pin
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
  uint8_t positionSet;
  uint8_t positionRead;
  uint8_t rotaryKnob1Read;
  uint8_t rotaryKnob2Read;
  //Analog Inputs-----------------------------------------
  positionSet= analogRead(positionSetPin);
  positionRead = analogRead(positionReadPin);
  rotaryKnob1Read = analogRead(rotaryKnobPin1);
  rotaryKnob2Read = analogRead(rotaryKnobPin2);

  //First digital packet----------------------------------
  bool scramActive = (digitalRead(scramPin) == LOW);
  bool powerActive = (digitalRead(powerPin) == LOW);
  bool magnetActive = (digitalRead(electromagnetPin) == LOW);


  if (scramActive) packet1 |= (1 << 0);
  if (powerActive) packet1 |= (1 << 1); 
  if (magnetActive) packet1 |= (1<< 2);
  //6 more digital indicators available

  Serial1.flush();                  // Wait for any previous transmission to complete
  Serial.println(positionSet);

  Serial1.write(packet1);
  delay(1);
  Serial1.write(positionSet);
  delay(1);
  Serial1.write(positionRead);
  delay(1);
  Serial1.write(rotaryKnobPin1);
  delay(1);
  Serial1.write(rotaryKnobPin2);
  delay(1);

  delay(100);                       // Slow down for easier scope/analyzer viewing
}