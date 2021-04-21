#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <Servo.h>
#include "timer.h"
#include "decideRobotState.h"

// Constants defining unused pins
const int GREEN_LED = 2;
const int RED_LED = 6;

const int IR_LEFT_SENSOR = A0;
const int IR_RIGHT_SENSOR = A2;

// LED connected to pin 13 of the zumo robot.
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1500 // 1500 microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     300 // 0 is stopped, 400 is full speed
#define TURN_SPEED        300
#define FORWARD_SPEED     300
#define REVERSE_DURATION  300 // ms
#define TURN_DURATION     300 // ms

// Defining integrated classes for the zumo robot.
ZumoBuzzer buzzer; // buzzer on pin 3
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// System states
const int S_FREE_DRIVE = 0;
const int S_TURN_LEFT = 1;
const int S_TURN_RIGHT = 2;
const int S_EVADE_OBJECT = 3;
const int S_EVADE_OBJECT_TURN_LEFT = 4;
const int S_EVADE_OBJECT_TURN_RIGHT = 5;

// Global state variable which tells starting sytem state.
int currentState = S_FREE_DRIVE;

// Defining variable which uses the Timer-class.
Timer myTimer;

// Defining variable which uses the robotState-class.
RobotState zumoRobot;

// Global variable to decide which state robot needs to be in.
int newState;

// Create a servo-objekt
Servo myServo;

void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  Serial.begin(115200);
  
  zumoRobot.init();
  // Configure pins
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(LED, HIGH);
  
  zumoRobot.initObjectSensor();

  //myServo.attach(11);            // Use pin 11 to control the servo
  
  waitForButtonAndCountDown();
}


void loop()
{
  
  switch(currentState)
  {
    // State where robot drives freely
    case S_FREE_DRIVE:
    
      // Checks if callibration mode is finished.
      if (myTimer.hasExpired())
      {
        motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
        turnLedOn(GREEN_LED);
        turnLedOff(RED_LED);
        //setPositionServo(180);
        stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.

        zumoRobot.getDistance();
        zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
        newState = zumoRobot.checkWhichStateNeeded();
        changeStateTo(newState);
       
      }

    break;

    case S_TURN_LEFT:
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      
      // if rightmost sensor detects line, reverse and turn to the left
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION);
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      delay(TURN_DURATION);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      
      //setPositionServo(30); // Sets servo degree at 30.
      turnLedOff(RED_LED); // Turns of red LED
      blinkLED(GREEN_LED); // Green LED blinks

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;
    
    case S_TURN_RIGHT:
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION);
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(TURN_DURATION);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      //setPositionServo(150); // Sets servo degree at 150.
      turnLedOff(RED_LED); // Turns off red LED.
      blinkLED(GREEN_LED); // Green LED blinks.

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;

    case S_EVADE_OBJECT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION);
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(TURN_DURATION);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); // Turns green LED off.
      turnLedOn(RED_LED); // Turns red LED on.
      
      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;

    case S_EVADE_OBJECT_TURN_LEFT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
     
      turnLedOff(GREEN_LED); // Turns off green LED.
      blinkLED(RED_LED); // Red LED blinks.

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;

    case S_EVADE_OBJECT_TURN_RIGHT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); // Turns of green LED.
      blinkLED(RED_LED); // Red LED blinks.

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);

      
    break;
  }
  
}

// ------------------FUNCTIONS-------------------------------

void waitForButtonAndCountDown()
{
  button.waitForButton(); // Starts calibration for the robot if button is pressed.
  myTimer.start(5000); // Starts a 5 second timer.

  for(int i = 0; i<2; i++)
  {
  buzzer.playNote(NOTE_F(2), 200, 15);
  delay(1000);
  }

  buzzer.playNote(NOTE_C(3), 200, 15);
  delay(350);
  buzzer.playNote(NOTE_C(3), 200, 15);
  delay(350);
  buzzer.playNote(NOTE_C(3), 200, 15);

}

// Function for changing system-state.
void changeStateTo(int newState)
{
 
   //Serial.print("State changed from ");
   (printState(currentState));
   //Serial.print(" to ");
   currentState = newState;
   (printState(newState));
  
}


//Prints out system-state.
void printState(int state)
{/*
  switch(state)
  {
    case 0:
      Serial.println("S_FREE_DRIVE");
    break;
    case 1:
      Serial.println("S_TURN_LEFT");
    break;
    case 2:
      Serial.println("S_TURN_RIGHT");
    break;
    case 3:
      Serial.println("S_EVADE_OBJECT");
    break;
  }*/
}

// Function to turn on LED.
void turnLedOn(int ledPin)
{
  digitalWrite(ledPin, HIGH);
}

// Function to turn off LED.
void turnLedOff(int ledPin)
{
  digitalWrite(ledPin, LOW);
}

// Function which sets zumo robots speed at 0.
void stopZumoRobot()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
}



void blinkLED(int ledPin)
{
  turnLedOn(ledPin);
  delay(250);
  turnLedOff(ledPin);
}
