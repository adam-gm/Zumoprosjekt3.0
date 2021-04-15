#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <Servo.h>
#include "timer.h"

// Constants defining unused pins
const int GREEN_LED = 2;
const int RED_LED = 6;

const int TRIG_PIN = A1;           //connects to the trigger pin on the distance sensor
const int ECHO_PIN = A3;           //connects to the echo pin on the distance sensor

// LED connected to pin 13 of the zumo robot.
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  500 // 1500 microseconds
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     150 // 0 is stopped, 400 is full speed
#define TURN_SPEED        150
#define FORWARD_SPEED     150
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

// Defining integrated classes for the zumo robot.
ZumoBuzzer buzzer; // buzzer on pin 3
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

// System states
const int S_FREE_DRIVE = 0;
const int S_TURN_LEFT = 1;
const int S_TURN_RIGHT = 2;
const int S_EVADE_OBJECT = 3;
const int S_EVADE_OBJECT_TURN_LEFT = 4;
const int S_EVADE_OBJECT_TURN_RIGHT = 5;

// Global state variable which tells starting sytem state.
int currentState = S_FREE_DRIVE;

// Declaring variable to hold distance between the robot and an object.
float distance;

// Variable to hold maximum distance from object
float chosenDistanceObject = 5.0;

// Defining variable which uses the Timer-class.
Timer myTimer;

// Create a servo-objekt
Servo myServo;
 
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void setup()
{
  // uncomment if necessary to correct motor directions
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  Serial.begin(200000);
  
  // Configure pins
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(LED, HIGH);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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
        
        sensors.read(sensor_values);
        distance = getDistance();
        
       if(checkIfTurnRight() == true)
       {
        changeStateTo(S_TURN_RIGHT);
       }
        
       else if(checkIfTurnLeft() == true)
       {
        changeStateTo(S_TURN_LEFT);
       }

       // Checks zumo robots distance to an other object.
       else if(distance <= chosenDistanceObject)
       {
        if((sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
        {
          changeStateTo(S_EVADE_OBJECT);
        }
       }

       else if((distance <= chosenDistanceObject) and (sensor_values[5] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_LEFT);
       }

       else if ((distance <= chosenDistanceObject) and (sensor_values[0] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_RIGHT);
       }
       
       else
       {
        
       }
       
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
      
      sensors.read(sensor_values); // Checks which color-surface the robot is driving on.
      distance = getDistance();
      
      if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
      {
       changeStateTo(S_TURN_RIGHT);
      }
      
      else if(checkIfFreeDrive() == true)
      {
       changeStateTo(S_FREE_DRIVE);
      }

      // Checks zumo robots distance to an other object.
      else if(distance <= chosenDistanceObject)
       {
        if((sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
        {
          //blinkLED(RED_LED); // Makes the red LED blink
          changeStateTo(S_EVADE_OBJECT);
        }
       }

       else if((distance <= chosenDistanceObject) and (sensor_values[5] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_LEFT);
       }

       else if ((distance <= chosenDistanceObject) and (sensor_values[0] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_RIGHT);
       }

       else
       {
        
       }
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
      
      sensors.read(sensor_values); // Checks which color-surface the robot is driving on.
      distance = getDistance();
      
      if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
      {
        changeStateTo(S_TURN_LEFT);
      }

      else if(checkIfFreeDrive() == true)
      {
       changeStateTo(S_FREE_DRIVE);
      }

      // Checks zumo robots distance to an other object.
      else if(distance <= chosenDistanceObject)
       {
        if((sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
        {
          changeStateTo(S_EVADE_OBJECT);
        }
       }

       else if((distance <= chosenDistanceObject) and (sensor_values[5] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_LEFT);
       }

       else if ((distance <= chosenDistanceObject) and (sensor_values[0] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_RIGHT);
       }
       
       else
       {
        
       }
    break;

    case S_EVADE_OBJECT:
      motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
      // Checks zumo robots distance to an other object.
      distance = getDistance();
      // Checks which color-surface the robot is driving on.
      sensors.read(sensor_values); 
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      turnLedOff(GREEN_LED); // Turns green LED off.
      turnLedOn(RED_LED); // Turns red LED on.
      
      if(checkIfFreeDrive() == true)
        {
          changeStateTo(S_FREE_DRIVE);
        }
       
      else if((distance <= chosenDistanceObject) and (sensor_values[5] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_LEFT);
       }

      else if ((distance <= chosenDistanceObject) and (sensor_values[0] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_RIGHT);
       }

      else if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_RIGHT);
       }
        
      else if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_LEFT);
       }

      else
      {
        
      }
      
    break;

    case S_EVADE_OBJECT_TURN_LEFT:
      motors.setSpeeds(0, 0);
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      // Checks zumo robots distance to an other object.
      distance = getDistance();
      // Checks which color-surface the robot is driving on.
      sensors.read(sensor_values); 
     
      turnLedOff(GREEN_LED); // Turns off green LED.
      blinkLED(RED_LED); // Red LED blinks.

      if(checkIfFreeDrive() == true)
        {
         changeStateTo(S_FREE_DRIVE);
       }

      else if((sensor_values[0] > QTR_THRESHOLD) and (sensor_values[5] > QTR_THRESHOLD))  
       {
        if(distance <= chosenDistanceObject)
        {
          //blinkLED(RED_LED); // Makes the blue LED blink
          changeStateTo(S_EVADE_OBJECT);
        }
       }

      else if ((distance <= chosenDistanceObject) and (sensor_values[0] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_RIGHT);
       }

      else if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_RIGHT);
       }
        
      else if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_LEFT);
       }

      else
      {
        
      }
    break;

    case S_EVADE_OBJECT_TURN_RIGHT:
      motors.setSpeeds(0, 0);
      stopZumoRobot(); // Opportunity to stop the robot if the button is pressed.
      // Checks zumo robots distance to an other object.
      distance = getDistance();
      // Checks which color-surface the robot is driving on.
      sensors.read(sensor_values); 

      turnLedOff(GREEN_LED); // Turns of green LED.
      blinkLED(RED_LED); // Red LED blinks.

      if(checkIfFreeDrive() == true)
        {
          changeStateTo(S_FREE_DRIVE);
        }

      else if((sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
       {
        if(distance <= chosenDistanceObject)
        {
          changeStateTo(S_EVADE_OBJECT);
        }
       }

      else if((distance <= chosenDistanceObject) and (sensor_values[5] < QTR_THRESHOLD))
       {
        changeStateTo(S_EVADE_OBJECT_TURN_LEFT);
       }
       
      else if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_RIGHT);
       }
        
      else if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_LEFT);
       }

      else
      {
        
      }
    break;
  }

  /*
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
   

  sensors.read(sensor_values);
  
  if (sensor_values[0] > QTR_THRESHOLD)
  {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (sensor_values[5] > QTR_THRESHOLD)
  {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  }
  */
  
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
  
  /*
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
  */
}

// Function for changing system-state.
void changeStateTo(int newState)
{
 
   Serial.print("State changed from ");
   (printState(currentState));
   Serial.print(" to ");
   currentState = newState;
   (printState(newState));
  
}


//Prints out system-state.
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
      Serial.println("S_EVADE_OBJECT");
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

// Function to change servo-angle.
void setPositionServo(int degree)
{
  if(degree >= 0 and degree <= 180)
  {
    myServo.write(degree);
  }

  else if (degree < 0)
  {
    myServo.write(0);
  }

  else
  {
    myServo.write(180);
  }
}

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  echoTime = pulseIn(ECHO_PIN, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime/148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound) (in cm)

  return calculatedDistance;              //send back the distance that was calculated
}

void blinkLED(int ledPin)
{
  turnLedOn(ledPin);
  delay(250);
  turnLedOff(ledPin);
}

// Functions to decide which state the robot should be in

bool checkIfFreeDrive()
{
  int variable;
  if(distance > chosenDistanceObject)
     {
       if ((sensor_values[0] > QTR_THRESHOLD) and (sensor_values[5] > QTR_THRESHOLD))
         {
           variable = true;
         }

       else
         {
           variable = false;
         }
     }

    return variable;
}

bool checkIfTurnRight()
{
  int variable;
  if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject ))
   {
     variable = true;  
        //changeStateTo(S_TURN_RIGHT);
   }
  else
  {
    variable = false;
  }
}

bool checkIfTurnLeft()
{
  int variable;
 if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
   {
    variable = true;
    changeStateTo(S_TURN_LEFT);
   }
 else
 {
  variable = false;
 }
}

bool checkIfAvoidObject()
{
  int variable;
  if(distance <= chosenDistanceObject)
    {
     if((sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
      {
       variable = true;
       changeStateTo(S_EVADE_OBJECT);
      }
     else
     {
      variable = false;
     }
    }
}

bool checkIfAvoidObjectTurnRight()
{
  int variable;
  if((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
       {
        changeStateTo(S_TURN_RIGHT);
       }
}

bool checkIfAvoidObjectTurnLeft()
{
  int variable;
  if((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
    {
     variable = true;
     changeStateTo(S_TURN_LEFT);
    }
   else
   {
    variable = false;
   }
}
