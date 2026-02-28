
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.


 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

#include <Stepper.h>
#include <AccelStepper.h>

const int PIN_STEP = 2;
const int PIN_DIR = 3;
const int PIN_EN = 4;


AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor
const int ArmBottomLimit = 22;
const int AmrTopLimit = 23;

const int PositionPotentiometer = A0;
const int CH2Pot = A1;
const int CH3Pot = A2;
const int PowerBit = 24;
const int ScramBit = 25;


const unsigned long SendPeriod = 20;



// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {

stepper.setMaxSpeed(200);
stepper.setAcceleration(100);
stepper.moveTo(200);

  // set the speed at 60 rpm:
 // myStepper.setSpeed(60);
  // initialize the serial port:
  Serial.begin(9600);
  Serial1.begin(9600);
  while (!Serial);
  Serial.println("Serial communication started.");
  Serial1.println("COMMS Started");
}

void loop() {


if (stepper.distanceToGo() == 0) {
  stepper.moveTo(-stepper.currentPosition());
}

stepper.run();
  // step one revolution  in one direction:
  Serial.println("clockwise");
  Serial1.println("**!!");

  myStepper.step(stepsPerRevolution);
  delay(500);

  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  Serial1.println("HI")
  myStepper.step(-stepsPerRevolution);
  delay(500);

  char incomingByte = Serial.read();
  Serial.println(incomingByte);
  myStepper.step(stepsPerRevolution);

  
}

