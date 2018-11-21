
#include "kinematics.h"
extern BufferedSerial buffered_pc;
Kinematics::Kinematics(Axis* A_, Axis* B_, Axis* C_, ServoAxis* yawAx_,ServoAxis* pitchAx_ ,calVals calibration_)
{
    e = calibration_.e;
    f = calibration_.f;
    re = calibration_.re;
    rf = calibration_.rf;
    A = A_;
    B = B_;
    C = C_;
    pitchAx = pitchAx_;
    yawAx = yawAx_;
    DeltaKinematics<float>::DeltaGeometricDim dim;
    dim.sb = f;
    dim.sp = e;
    dim.L = rf;
    dim.l = re;
    dim.h = 60;
    dim.max_neg_angle = -20;
    dim.min_parallelogram_angle = 30;
    
    DK = new DeltaKinematics<float>(dim);

// produced automatically from the delta gun ros package. 
GunMarkerToBaseCentreM<<0.988361,-0.151568,-0.0130129,0.0511989,0.151498,0.988438,-0.00624483,-0.015117,0.013809,0.00420072,0.999896,-0.0495781,0,0,0,1;
GunMarkerToBaseCentreInvM<<0.988361,0.151498,0.013809,-0.0476282,-0.151568,0.988438,0.00420072,0.0229106,-0.0130129,-0.00624483,0.999896,0.0501448,0,0,0,1;
HeadCentreToPitchM<<1,0,0,0.01921,0,1,0,-0.013051,0,0,1,-0.008051,0,0,0,1;
HeadCentreToPitchInvM<<1,0,0,-0.01921,0,1,0,0.013051,0,0,1,0.008051,0,0,0,1;
PitchToYawM<<1,0,0,0.0081,0,1,0,0,0,0,1,0,0,0,0,1;
PitchToYawInvM<<1,0,0,-0.0081,0,1,0,0,0,0,1,0,0,0,0,1;
imuToOriginM<<1,0,0,0.085571,0,1,0,0.0048,0,0,1,-1.11022e-16,0,0,0,1;
imuToOriginInvM<<1,0,0,-0.085571,0,1,0,-0.0048,0,0,1,1.11022e-16,0,0,0,1;
boardMarkerToCentreM<<0.967938,-0.0839857,0.236732,0.0163319,0.250649,0.261201,-0.932175,-0.0518183,0.0164548,0.961624,0.273877,-0.182812,0,0,0,1;
boardMarkerToCentreInvM<<0.967938,0.250649,0.0164548,0.000188067,-0.0839857,0.261201,0.961624,0.190703,0.236732,-0.932175,0.273877,-0.00210193,0,0,0,1;
yawToTipM<<1,0,0,0.1094,0,1,0,0,0,0,1,0.024,0,0,0,1;
yawToTipInvM<<1,0,0,-0.1094,0,1,0,0,0,0,1,-0.024,0,0,0,1;

    GunMarkerToBaseCentre = GunMarkerToBaseCentreM;
    GunMarkerToBaseCentreInv = GunMarkerToBaseCentreInvM;
    HeadCentreToPitch = HeadCentreToPitchM;
    HeadCentreToPitchInv = HeadCentreToPitchInvM;
    PitchToYaw = PitchToYawM;
    PitchToYawInv = PitchToYawInvM;
    imuToOrigin = imuToOriginM;
    imuToOriginInv = imuToOriginInvM;
    boardMarkerToCentre = boardMarkerToCentreM;
    boardMarkerToCentreInv = boardMarkerToCentreInvM;
    yawToTip = yawToTipM;
    yawToTipInv = yawToTipInvM;


}

//taken from the ros source code and changed to use eigen. Has  more pleasing result. 
void Kinematics::getEulerYPREigen(Eigen::Matrix3f mat, float& yaw, float& pitch, float& roll)
{
     unsigned int solution_number = 1;
    Eigen::Vector3f m_el[3];
   m_el[0] =  mat.row(0);
   m_el[1] =  mat.row(1);
   m_el[2] =  mat.row(2);
    struct Euler
    {
        float yaw;
        float pitch;
        float roll;
    };

    Euler euler_out;
    Euler euler_out2; //second solution
    //get the pointer to the raw data

    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity
    if (fabs(m_el[2].x()) >= 1)
    {
        euler_out.yaw = 0;
        euler_out2.yaw = 0;

        // From difference of angles formula
        if (m_el[2].x() < 0)  //gimbal locked down
        {
          float delta = atan2(m_el[0].y(),m_el[0].z());
            euler_out.pitch = M_PI / 2.0;
            euler_out2.pitch = M_PI / 2.0;
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
        else // gimbal locked up
        {
          float delta = atan2(-m_el[0].y(),-m_el[0].z());
            euler_out.pitch = -M_PI / 2.0;
            euler_out2.pitch = -M_PI / 2.0;
            euler_out.roll = delta;
            euler_out2.roll = delta;
        }
    }
    else
    {
        euler_out.pitch = - asin(m_el[2].x());
        euler_out2.pitch = M_PI - euler_out.pitch;

        euler_out.roll = atan2(m_el[2].y()/cos(euler_out.pitch),
            m_el[2].z()/cos(euler_out.pitch));
        euler_out2.roll = atan2(m_el[2].y()/cos(euler_out2.pitch),
            m_el[2].z()/cos(euler_out2.pitch));

        euler_out.yaw = atan2(m_el[1].x()/cos(euler_out.pitch),
            m_el[0].x()/cos(euler_out.pitch));
        euler_out2.yaw = atan2(m_el[1].x()/cos(euler_out2.pitch),
            m_el[0].x()/cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
        yaw = euler_out.yaw;
        pitch = euler_out.pitch;
        roll = euler_out.roll;
    }
    else
    {
        yaw = euler_out2.yaw;
        pitch = euler_out2.pitch;
        roll = euler_out2.roll;
    }
}

// this function will calculate the actuation neccessary  to get to a world position. 
// current pos is the position of the IMU
int Kinematics::goToWorldPos(Eigen::Affine3f currentPos, Eigen::Affine3f targetPos,Eigen::Vector3f angRates,float ffGain){

    int error = 0;
         Eigen::Affine3f origin = currentPos* imuToOrigin;
    Eigen::Affine3f originToTarget = origin.inverse((Eigen::TransformTraits)1) * targetPos;
    float r , p , y;
    getEulerYPREigen(originToTarget.rotation(), y, p, r);
    //make separate rotation matrix
    float yawAng =  y;
    float pitchAng = p;

    Eigen::Affine3f yaw = Eigen::Translation3f(0,0,0) * Eigen::AngleAxisf(yawAng, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f pitch = Eigen::Translation3f(0,0,0) * Eigen::AngleAxisf(pitchAng, Eigen::Vector3f::UnitY());

    

    Eigen::Affine3f kinOut = origin.inverse((Eigen::TransformTraits)1)
                            * targetPos
                            * yawToTipInv
                            * yaw.inverse((Eigen::TransformTraits)1)
                            * PitchToYawInv
                            * pitch.inverse((Eigen::TransformTraits)1)
                            * HeadCentreToPitchInv;
    // Eigen::Matrix4f originM = currentPos.matrix() * imuToOriginM;
    // Eigen::Matrix4f targetM = targetPos.matrix();
    // Eigen::Matrix4f kinOutMat = origin.inverse()
    //                             * targetM
    //                             * yawToTipInvM
    //                             * yaw.matrix().inverse()
    //                             * PitchToYawInvM
    //                             * pitch.matrix().inverse()
    //                             * HeadCentreToPitchInvM;
    // kinOut = kinOutMat;
    Eigen::Vector3f kinTrans = kinOut.translation();

    

    Eigen::Vector3f centre(0.170,0,0);
    float range = 0.090;
    float dif = (kinTrans - centre).norm();
    if(dif < range){

        goToPos(kinTrans[0]*1000, kinTrans[1]*1000, kinTrans[2]*1000);
        error+=yawAx->setAngle(180*(yawAng - ffGain * angRates[2])/3.14);
        error+=pitchAx->setAngle(180*(pitchAng - ffGain * angRates[1])/3.14);
        //buffered_pc.printf("kin x y z %f %f %f",kinTrans[0]*1000,kinTrans[1]*1000,kinTrans[2]*1000);
    }
    else{
        error += 1;
        // here we instead clip the result to the allowable kinematics.
        Eigen::Vector3f centreToTarget;
        centreToTarget = kinTrans - centre;
        kinTrans = centreToTarget.normalized()*range + centre;
        goToPos(kinTrans[0]*1000, kinTrans[1]*1000, kinTrans[2]*1000);
        //buffered_pc.printf("kin x y z %f %f %f",kinTrans[0]*1000,kinTrans[1]*1000,kinTrans[2]*1000);
        yawAx->setAngle(180*(yawAng - ffGain * angRates[2])/3.14);
        pitchAx->setAngle(180*(pitchAng - ffGain * angRates[1])/3.14);
    }
    return error;



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
    // here we are going to swap around the vector for the benefit of using ros cooridinates as input and output.
    // for the delta gun project x is forwards (extending all arms equally), and Z is 'up' towards only one arm. 
    // y is the right hand rule taking these two. 
    DeltaKinematics<float>::DeltaVector DV = {-y,-z,-x,0,0,0};
    int  error = DK->CalculateIpk(&DV, 1);

    DV.phi1 = (DV.phi1/180)*pi;
    DV.phi2 = (DV.phi2/180)*pi;
    DV.phi3 = (DV.phi3/180)*pi;
    //buffered_pc.printf("a b c error %f %f %f %d" , DV.phi1,DV.phi2,DV.phi3, error);
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