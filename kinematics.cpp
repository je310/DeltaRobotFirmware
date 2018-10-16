
#include "kinematics.h"
//this implementation has been taken from http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
// robot geometry
// (look at pics above for explanation)
extern BufferedSerial buffered_pc;
Kinematics::Kinematics(Axis* A_, Axis* B_, Axis* C_,calVals calibration_)
{
    e = calibration_.e;
    f = calibration_.f;
    re = calibration_.re;
    rf = calibration_.rf;
    A = A_;
    B = B_;
    C = C_;
    DeltaKinematics<float>::DeltaGeometricDim dim;
    dim.sb = f;
    dim.sp = e;
    dim.L = rf;
    dim.l = re;
    dim.h = 60;
    dim.max_neg_angle = -20;
    dim.min_parallelogram_angle = 30;
    
    DK = new DeltaKinematics<float>(dim);

}


// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int Kinematics::delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0)
{
    float t = (f-e)*tan30/2;
    float dtr = pi/(float)180.0;

    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;

    float y1 = -(t + rf*cos(theta1));
    float z1 = -rf*sin(theta1);

    float y2 = (t + rf*cos(theta2))*sin30;
    float x2 = y2*tan60;
    float z2 = -rf*sin(theta2);

    float y3 = (t + rf*cos(theta3))*sin30;
    float x3 = -y3*tan60;
    float z3 = -rf*sin(theta3);

    float dnm = (y2-y1)*x3-(y3-y1)*x2;

    float w1 = y1*y1 + z1*z1;
    float w2 = x2*x2 + y2*y2 + z2*z2;
    float w3 = x3*x3 + y3*y3 + z3*z3;

    // x = (a1*z + b1)/dnm
    float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

    // y = (a2*z + b2)/dnm;
    float a2 = -(z2-z1)*x3+(z3-z1)*x2;
    float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

    // a*z^2 + b*z + c = 0
    float a = a1*a1 + a2*a2 + dnm*dnm;
    float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

    // discriminant
    float d = b*b - (float)4.0*a*c;
    if (d < 0) return -1; // non-existing point

    z0 = -(float)0.5*(b+sqrt(d))/a;
    x0 = (a1*z0 + b1)/dnm;
    y0 = (a2*z0 + b2)/dnm;
    return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int Kinematics::delta_calcAngleYZ(float x0, float y0, float z0, float &theta)
{
    float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
    y0 -= 0.5 * 0.57735    * e;    // shift center to edge
    // z = a + b*y
    float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
    float b = (y1-y0)/z0;
    // discriminant
    float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
    if (d < 0) return -1; // non-existing point
    float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
    float zj = a + b*yj;
    theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
    return 0;
}

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int Kinematics::delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3)
{
    theta1 = theta2 = theta3 = 0;
    int status = delta_calcAngleYZ(x0, y0, z0, theta1);
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
    return status;
}

void Kinematics::activateMotors()
{
    //int error = 0;
    //C->setMaxVel(15001); // for safety of the linkages, override to go fast!

//    Thread::wait(100);
//        C->setSafePerams(15004, 0.00025, 4000, 250,0.001);
//    Thread::wait(100);
//    B->setSafePerams(15004, 0.00025, 4000, 250,0.001);
//    Thread::wait(100);
//    A->setSafePerams(15004, 0.00025, 4000, 250,0.001);
//    Thread::wait(100);

    //buffered_pc.printf("there were %d errors in setting perameters",error);

    int requested_state;
    requested_state = ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL;
    A->runState(requested_state);
    B->runState(requested_state);
    C->runState(requested_state);
}

int Kinematics::setSafeParams(){
        int error = 0;
        error += C->setParams(15004, 0.00025, 4000, 250,0.001);
    Thread::wait(1);
    error +=B->setParams(15004, 0.00025, 4000, 250,0.001);
    Thread::wait(1);
    error +=A->setParams(15004, 0.00025, 4000, 250,0.001);
    Thread::wait(1);
     buffered_pc.printf("there were %d errors in setting perameters",error);
     return error;
    }
    
    int Kinematics::setFastParams(){
        int error = 0;
        error += C->setParams(40000, 0.00032, 4000, 300,0.001);
    Thread::wait(1);
    error +=B->setParams(40000, 0.00032, 4000, 300,0.001);
    Thread::wait(1);
    error +=A->setParams(40000, 0.00032, 4000, 300,0.001);
    Thread::wait(1);
     buffered_pc.printf("there were %d errors in setting perameters",error);
     return error;
    }

void Kinematics::findIndex(){
    Thread::wait(100);
    A->findIndex();
    Thread::wait(100);
    B->findIndex();
    Thread::wait(100);
    C->findIndex();
}

void Kinematics::homeMotors()
{
    Thread::wait(1000);
    A->homeAxis();
    Thread::wait(1000);
    B->homeAxis();
    Thread::wait(1000);
    C->homeAxis();
}

void Kinematics::goIdle(){
    Thread::wait(100);
    A->idle();
    Thread::wait(100);
    B->idle();
    Thread::wait(100);
    C->idle();
    Thread::wait(100);
}

int Kinematics::goToPos(float x, float y, float z)
{
    //int error = delta_calcInverse(x,y,z,a,b,c);
    DeltaKinematics<float>::DeltaVector DV = {x,y,z,0,0,0};
    int  error = DK->CalculateIpk(&DV, 1);
    //buffered_pc.printf("angles: a:%f , b:%f , c%f\r\n",DV.phi1,DV.phi2,DV.phi3);
    DV.phi1 = (DV.phi1/180)*pi;
    DV.phi2 = (DV.phi2/180)*pi;
    DV.phi3 = (DV.phi3/180)*pi;
    if(error == 0 ) {
        A->goAngle(DV.phi1);
        B->goAngle(DV.phi2);
        C->goAngle(DV.phi3);
    }
    return error;
}

void Kinematics::goToAngles(float a, float b, float c)
{
    A->goAngle(a);
    B->goAngle(b);
    C->goAngle(c);
}