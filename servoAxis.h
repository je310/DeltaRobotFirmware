#ifndef SERVO_AXIS_H
#define SERVO_AXIS_H

#include "mbed.h"

#include "BufferedSerial.h"

class ServoAxis
{
public:

    ServoAxis(PinName pin,float maxAngle_, float minAngle, float middleUs_, float usPerDeg_, float degOffset_);
    int setAngle(float angle);

private:
    PwmOut* pwm;
    float middleUs;
    float usPerDeg;
    float degOffset;

    float maxAngle; //roughtly
    float minAngle;

};





#endif // SERVO_AXIS_H