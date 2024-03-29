// Including libraries
#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include "timer.h"
#include "decideRobotState.h"

// Constants defining unused pins
const int GREEN_LED = 2;
const int RED_LED = 6;

const int IR_LEFT_SENSOR = A0;
const int IR_RIGHT_SENSOR = A2;

// LED connected to pin 13 of the zumo robot.
#define LED 13
  
// Defining motorspeeds for robot
#define REVERSE_SPEED     400 // 0 is stopped, 400 is full speed.
#define TURN_SPEED        400
#define FORWARD_SPEED     400
#define REVERSE_DURATION  350 // ms
#define TURN_DURATION     350 // ms

#define FREE_DRIVE_SPEED 325 // Speed during state S_FREE_DRIVE.
#define TURN_SPEED_90_DEG 275 // Speed when ir-sensors detects far object.
#define TURN_DURATION_90_DEG 125 // ms

#define TURN_SPEED_CLOSE_OBJECT 200 // Speed when ir-sensors detect close object.
#define TURN_DURATION_CLOSE_OBJECT 300 // ms
#define REVERSE_DURATION_CLOSE_OBJECT 600 // ms


// Defining integrated classes for the zumo robot.
ZumoBuzzer buzzer; // buzzer on pin 3
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// System states
const int S_FREE_DRIVE = 0;
const int S_TURN_LEFT = 1;
const int S_TURN_RIGHT = 2;
const int S_ATTACK_OBJECT = 3;
const int S_EVADE_OBJECT_TURN_LEFT = 4;
const int S_EVADE_OBJECT_TURN_RIGHT = 5;
const int S_EVADE_CLOSE_OBJECT_TURN_LEFT = 6;
const int S_EVADE_CLOSE_OBJECT_TURN_RIGHT = 7;

// Global state variable for the starting sytem state.
int currentState = S_FREE_DRIVE;

// Global variable to hold which state robot needs to be in.
int newState;

// Defining object which uses the Timer-class.
Timer myTimer;

// Defining object which uses the RobotState-class.
RobotState zumoRobot;

void setup()
{

  Serial.begin(115200);

  // Configure pins
  zumoRobot.init(); // Method from RobotState-class. 
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(LED, HIGH);
  
  zumoRobot.initObjectSensor();
  
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
        motors.setSpeeds(FREE_DRIVE_SPEED, FREE_DRIVE_SPEED);
        turnLedOn(GREEN_LED); // Turns green LED on
        turnLedOff(RED_LED); // Turns red LED off
       
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
      
      turnLedOff(RED_LED); 
      blinkLED(GREEN_LED); // Green LED blinks

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;
    
    case S_TURN_RIGHT:
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.

      // if leftmost sensor detects line, reverse and turn to the right
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION);
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(TURN_DURATION);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      
      turnLedOff(RED_LED); 
      blinkLED(GREEN_LED); 

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;

    case S_ATTACK_OBJECT:
      motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
      
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); 
      turnLedOn(RED_LED); 
      
      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;
    
    case S_EVADE_OBJECT_TURN_LEFT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION);
      motors.setSpeeds(-TURN_SPEED_90_DEG, TURN_SPEED_90_DEG);
      delay(TURN_DURATION_90_DEG);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      
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
      delay(REVERSE_DURATION);
      motors.setSpeeds(TURN_SPEED_90_DEG, -TURN_SPEED_90_DEG);
      delay(TURN_DURATION_90_DEG);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); // Turns of green LED.
      blinkLED(RED_LED); // Red LED blinks.

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);

    break;

    case S_EVADE_CLOSE_OBJECT_TURN_LEFT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION_CLOSE_OBJECT);
      motors.setSpeeds(-TURN_SPEED_CLOSE_OBJECT, TURN_SPEED_CLOSE_OBJECT);
      delay(TURN_DURATION_CLOSE_OBJECT);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); // Turns of green LED.
      blinkLED(RED_LED); // Red LED blinks.

      zumoRobot.getDistance();
      zumoRobot.getDistanceLeftRight(IR_LEFT_SENSOR, IR_RIGHT_SENSOR);
      newState = zumoRobot.checkWhichStateNeeded();
      changeStateTo(newState);
      
    break;

    case S_EVADE_CLOSE_OBJECT_TURN_RIGHT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      delay(REVERSE_DURATION_CLOSE_OBJECT);
      motors.setSpeeds(TURN_SPEED_CLOSE_OBJECT, -TURN_SPEED_CLOSE_OBJECT);
      delay(TURN_DURATION_CLOSE_OBJECT);
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

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
  playStartingMelody(); // Play starting melody
}

// Function for changing system-state.
void changeStateTo(int changedState)
{
 
   Serial.print("State changed from ");
   (printState(newState));
   Serial.print(" to ");
   currentState = changedState;
   (printState(newState));
  
}


//Prints out system-state in serial monitor.
void printState(int state)
{
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
      Serial.println("S_ATTACK_OBJECT");
    break;
    case 4:
      Serial.println("S_EVADE_OBJECT_TURN_LEFT");
    case 5:
      Serial.println("S_EVADE_OBJECT_TURN_RIGHT");
    break;
    case 6:
      Serial.println("S_EVADE_CLOSE_OBJECT_TURN_LEFT");
    break;
    case 7:
      Serial.println("S_EVADE_CLOSE_OBJECT_TURN_RIGHT");
    break;
  }
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

// Function to blink LED
void blinkLED(int ledPin)
{
  turnLedOn(ledPin);
  delay(250);
  turnLedOff(ledPin);
}

// Function for the melody played at calibration mode
void playStartingMelody()
{
  for(int i = 0; i<4; i++)
  {
  buzzer.playNote(NOTE_C(4), 200, 15);
  delay(500);
  }
  
  buzzer.playNote(NOTE_A(2), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_A(3), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_A(4), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_B(2), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_B(3), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_B(4), 200, 15);
  delay(375);
  buzzer.playNote(NOTE_C(2), 200, 15);
  delay(250);
  buzzer.playNote(NOTE_C(3), 200, 15);
  delay(250);
  buzzer.playNote(NOTE_C(2), 200, 15);
  delay(250);
}
