#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "odrive.h"
#include "Axis.h"
#include <math.h>
#include "calibration.h"
#include "comms.h"
#include "DeltaKinematics.h"




 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;

class Kinematics
{
public:
    Kinematics(Axis* A_, Axis* B_, Axis* C_,calVals calibration_);
    int goToPos(float x, float y, float z);
    void goToAngles(float a, float b, float c);
    void goToPosVel(float x, float y, float z, float xv, float yv, float zv);
    void activateMotors();
    void homeMotors();
    void findIndex();
    void goIdle();
    void updateCalibration(calVals calibration_);
    int setSafeParams();
    int setFastParams();
    DeltaKinematics<float>* DK;


private:
    float e ;    // end effector
    float f;     // base
    float re;
    float rf;
    int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
    void delta_calcInverseDy(float x0, float y0, float z0, float vx0, float vy0, float vz0, float &theta1, float &theta2, float &theta3,float &vtheta1, float &vtheta2, float &vtheta3);
    int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
    int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
    Axis* A;
    Axis* B;
    Axis* C;


};









#endif // KINEMATICS_H