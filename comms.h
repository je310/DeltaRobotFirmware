#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
struct OdriveCMD{
    float position;
    float velocity;
    float torque;
    float maxCurrent;
    int32_t mode;
};
struct ServoCMD{
    float angle;
    float atTime;
};

struct rosTime{
    int32_t seconds;
    int32_t nSeconds;
};

struct toRobot{
    OdriveCMD motor1;
    OdriveCMD motor2;
    OdriveCMD motor3;
    ServoCMD servo1;
    ServoCMD servo2;
    rosTime time;
    float timeOffset;
    int32_t isPing;
    int32_t count;
};

struct OdriveINFO{
    float speed;
    int32_t position;
    float busVoltage;
    float setCurrent;
    float measuredCurrent;
};

struct vectorThree{
    float x;
    float y;
    float z;
};

struct imuINFO{
    vectorThree accel;
    vectorThree gyro;
};

struct gunINFO{
    bool buttons[32];
};

struct fromRobot{
    OdriveINFO motor1;
    OdriveINFO motor2;
    OdriveINFO motor3;
    imuINFO imu;
    gunINFO gun;
    rosTime time;
    rosTime pingTime;
    int32_t isPing;
    int32_t count;
};


#endif // COMMS_H
