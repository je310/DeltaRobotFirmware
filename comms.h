#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>


//lower case starts for the enumeration, upper case for the struct definition.
enum messageType{
    angleSpaceCmd = 0,
    cartSpaceCmd= 1,
    pingOut= 2,
    locationOut = 3,
    pingIn= 4,
    locationIn=5,
    userIn = 6,
    imuData = 7,
    imuDataChunk = 8
};

struct vectorThree{
    float x;
    float y;
    float z;
};
struct vectorFour{
    float x;
    float y;
    float z;
    float w;
};
struct rosTime{
    int32_t seconds;
    int32_t nSeconds;
};

struct AngleSpaceCmd{
    int32_t type;
    float A;
    float B;
    float C;
    float yaw;
    float pitch;
};

struct CartSpaceCmd{
    int32_t type;
    vectorThree pos;
    vectorThree vel;
    float yaw;
    float pitch;
};

struct PingOut{
    int32_t type;
    rosTime time;
    float timeOffset;
};

struct PingIn{
    int32_t type;
    rosTime responseTime;
    rosTime sentTime;

};

struct LocationOut{
    int32_t type;
    vectorThree pos;
    vectorFour quat;
    rosTime time;
};

struct LocationIn{
    int32_t type;
    vectorThree pos;
    vectorFour quat;
    rosTime time;
    rosTime refTime;
};

struct UserIn{
    int32_t type;
    int32_t buttons;
};

struct ImuData{
    int32_t type;
    vectorThree accel;
    vectorThree gyro;
    rosTime time;
};

struct ImuDataChunk{
    int32_t type;
    ImuData array[20];
};

//struct OdriveCMD{
//    float position;
//    float velocity;
//    float torque;
//    float maxCurrent;
//    int32_t mode;
//};
//struct ServoCMD{
//    float angle;
//    float atTime;
//};



//struct toRobot{
//    OdriveCMD motor1;
//    OdriveCMD motor2;
//    OdriveCMD motor3;
//    ServoCMD servo1;
//    ServoCMD servo2;
//    rosTime time;
//    float timeOffset;
//    int32_t isPing;
//    int32_t count;
//};

//struct OdriveINFO{
//    float speed;
//    int32_t position;
//    float busVoltage;
//    float setCurrent;
//    float measuredCurrent;
//};



//struct imuINFO{
//    vectorThree accel;
//    vectorThree gyro;
//};

//struct gunINFO{
//    bool buttons[32];
//};

//struct fromRobot{
//    OdriveINFO motor1;
//    OdriveINFO motor2;
//    OdriveINFO motor3;
//    imuINFO imu;
//    gunINFO gun;
//    rosTime time;
//    rosTime pingTime;
//    int32_t isPing;
//    int32_t count;
//};


#endif // COMMS_H
