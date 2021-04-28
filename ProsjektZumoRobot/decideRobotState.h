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
    const int TRIG_PIN = A1;
    const int ECHO_PIN = A3;    
    const int IR_LEFT_SENSOR = A0;
    const int IR_RIGHT_SENSOR = A2;
    const int QTR_THRESHOLD = 1500;//1500;
  private:
    float chosenDistanceObject;
    float distance;
    float irDistanceLeft;
    float irDistanceRight;
    unsigned int sensor_values[6];
    ZumoReflectanceSensorArray* sensors;//(QTR_NO_EMITTER_PIN);
    int robotState = 0;
};

#endif
