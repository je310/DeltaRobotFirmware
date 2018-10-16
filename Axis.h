#ifndef AXIS_H
#define AXIS_H

#include "odrive.h"
#include "mbed.h"
#include "calibration.h"
#include "comms.h"
#include <string>
#include <sstream>

enum axisName {
    AX_A,
    AX_B,
    AX_C,
};

class Axis
{
public:

    Axis(ODrive* od, int ax, DigitalIn* homeSwitch, calVals calibration_,axisName identity);
    void   homeAxis();
    void    findIndex();
    void   goAngle(float angle);
    void  goAngleSpeed(float angle, float speed);
    void runState(int requestedState);
    int readState();
    DigitalIn* homeSwitch_;
    void setMaxVel(float stepsPerSec);
    int setParams(float stepsPerSec, float vel_gain, float encoder_bandwidth, float pos_gain, float vel_int);
    ODrive* odrive;
    void idle();
    int test();
    float readParam(string in);
    float readBattery();
    float writeParam(string in, float val);
    axisName name;

private:
    
    int axNum;
    int homeOffset;
    float currentSetPos;
    float currentSetVel;

    float maxAngle; //roughtly
    float gearRatio_;
    float pulse_per_rev;
    float pulse_per_rad;
    float rotation_offset; // defines the rotational ofset from flat, to be calibrated.


};





#endif // COMMS_H