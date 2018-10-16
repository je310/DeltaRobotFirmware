#ifndef SERVO_AXIS_H
#define SERVO_AXIS_H

#include "mbed.h"


class ServoAxis
{
public:

    ServoAxis(PinName pin,float maxAngle_, float minAngle, float middleUs_, float usPerDeg_);
    void setAngle(float angle);

private:
    PwmOut* pwm;
    float middleUs;
    float usPerDeg;

    float maxAngle; //roughtly
    float minAngle;

};





#endif // SERVO_AXIS_H