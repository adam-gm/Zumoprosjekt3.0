#include "decideRobotState.h"

void RobotState::initObjectSensor()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  sensors = new ZumoReflectanceSensorArray(QTR_NO_EMITTER_PIN);

}

void RobotState::getDistanceLeftRight(int irLeftSensorPin, int irRightSensorPin)
{
  irDistanceLeft = analogRead(irLeftSensorPin); // This value measures distance by how high voltage get returned. The closer the object, the higher voltage returns.
  irDistanceRight = analogRead(irRightSensorPin); // This value measures distance by how high voltage get returned. The closer the object, the higher voltage returns.
}

void RobotState::getDistance()
{
  float echoTime;
  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  echoTime = pulseIn(ECHO_PIN, HIGH);      //use the pulsein command to see how long it takes for the
  //pulse to bounce back to the sensor

  distance = echoTime / 148.0; //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound) (in cm)

}

int RobotState::checkWhichStateNeeded()
{
  sensors->read(sensor_values);

  if ((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
  {
    robotState = 1; // S_TURN_LEFT
  }

  else if ((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject ))
  {
    robotState = 2; // S_TURN_RIGHT;
  }

  else if (irDistanceLeft >= 400)
  {
    robotState = 6; // S_EVADE_CLOSE_OBJECT_TURN_LEFT
  }

  else if(irDistanceRight >= 400)
  {
    robotState = 7; // S_EVADE_CLOSE_OBJECT_TURN_RIGHT
  }
  
  else if (irDistanceLeft >= 125 and irDistanceLeft < 400)
  {
    robotState = 4; // S_EVADE_OBJECT_TURN_LEFT
  }

  else if (irDistanceRight >= 125 and irDistanceRight < 400)
  {
    robotState = 5; // S_EVADE_OBJECT_TURN_RIGHT
  }
  
  else if ((distance <= chosenDistanceObject) and (sensor_values[5] > QTR_THRESHOLD) and (sensor_values[0] > QTR_THRESHOLD))
  {
    robotState = 3; //S_EVADE_OBJECT

  }
  
  else if ((distance > chosenDistanceObject) and (sensor_values[0] > QTR_THRESHOLD) and (sensor_values[5] > QTR_THRESHOLD))
  {
    robotState = 0; // S_FREE_DRIVE
  }

  else if ((sensor_values[5] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
  {
    robotState = 4; // S_EVADE_OBJECT_TURN_LEFT
  }

  else if ((sensor_values[0] < QTR_THRESHOLD) and (distance > chosenDistanceObject))
  {
    robotState = 5; // S_EVADE_OBJECT_TURN_RIGHT
  }

  
  Serial.print("State: ");
  Serial.println(robotState);
  return robotState;
}

void RobotState::init()
{
  //variable to store the time it takes for a ping to bounce off an object
  chosenDistanceObject = 15;
}
