#ifndef CALIBRATION_H
#define CALIBRATION_H
//this is a struct containing all measured values that effect the movement values. To be  calibrated. 

struct calVals{
    float e ;    // end effector
    float f;     // base
    float re;
    float rf;
    float Aoffset;
    float Boffset;
    float Coffset;
    float gearRatio;
    //other things related to servo offsets for 4/5th axis also. 
};

#endif