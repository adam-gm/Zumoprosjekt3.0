#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H
#include <arduino.h>
#include <ZumoReflectanceSensorArray.h>

//Class declaration of the robot state class

class RobotState 
{
  public:
    int checkWhichStateNeeded();
    void getDistance(); 
    void getDistanceLeftRight(int irLeftSensorPin, int irRightSensorPin);
    void init();
    void initObjectSensor(); 
    const int TRIG_PIN = A1; // Pin connected to A1
    const int ECHO_PIN = A3; // Pin connected to A3
    const int IR_LEFT_SENSOR = A0; // Pin connected to A0
    const int IR_RIGHT_SENSOR = A2; // Pin connected to A2.
    const int QTR_THRESHOLD = 1500; // Threshold for reflectancesensors
  private:
    float chosenDistanceObject; // Initializing the chosenDistance for objectsensor on the front.
    float distance; // Calculated distance for the objectsensor on front
    float irDistanceLeft; // Variable to hold distance to object from left sensor.
    float irDistanceRight; // Variable to hold distance to object from right sensor.
    unsigned int sensor_values[6];
    ZumoReflectanceSensorArray* sensors;//(QTR_NO_EMITTER_PIN); // Array to hold the reflectancesensors
    int robotState = 0; // Variable to hold the needed state.
};

#endif
