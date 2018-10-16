
#include "Axis.h"

extern BufferedSerial buffered_pc;

Axis::Axis(ODrive* od, int ax, DigitalIn* homeSwitch, calVals calibration_,axisName identity)
{
    odrive = od;
    axNum = ax;
    homeSwitch_ = homeSwitch;
    name = identity;
    switch(identity) {
        case AX_A:
            rotation_offset = calibration_.Aoffset;
            break;
        case AX_B:
            rotation_offset = calibration_.Boffset;
            break;
        case AX_C:
            rotation_offset = calibration_.Coffset;
            break;
    }

    gearRatio_ = calibration_.gearRatio; // 89/24
    pulse_per_rev = 8192 * gearRatio_;
    pulse_per_rad = pulse_per_rev / (2* 3.14159265359);

}

void  Axis::homeAxis()
{
    int count  = 0;
    bool homed = false;
    homeOffset = 0;
    int homeCount = 0;
    while(!homed) {
        odrive->SetPosition(axNum,homeOffset);
        Thread::wait(1);
        for(int i = 0 ; i < 99; i++) {
            if(*homeSwitch_ == false) {
                homeCount++;
            } else {
                homeCount = 0;
            }
            if(homeCount > 50) {
                homed = true;
            }
        }
        homeCount= 0;
        homeOffset++;
    }
    homeOffset--;
    float homeBounce = 0.5;
    goAngle(homeBounce);
    currentSetPos = 0.1;
    currentSetVel = 0;
}

int Axis::readState(){
        std::stringstream ss;
    ss<< "r axis" << axNum << ".current_state" << '\n';
    //buffered_pc.printf(ss.str().c_str());
    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    return odrive->readInt();
    }

void Axis::findIndex()
{


    int requested_state;
    requested_state = ODrive::AXIS_STATE_ENCODER_INDEX_SEARCH;
    runState(requested_state);
    int foundIndex = 0;
    while(!foundIndex) {
        Thread::wait(10);
        int state = readState();
        if( state == ODrive::AXIS_STATE_IDLE) foundIndex = 1;
    }
    int requestedState;
    requestedState = ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL;
    runState(requestedState);
}

void Axis::idle()
{
    int requested_state;
    requested_state = ODrive::AXIS_STATE_IDLE;
    runState(requested_state);
}

void  Axis::goAngle(float angle)
{
    odrive->SetPosition(axNum,homeOffset - angle*pulse_per_rad - rotation_offset*pulse_per_rad);
}

void  Axis::goAngleSpeed(float angle, float speed)
{
    odrive->SetPosition(axNum,homeOffset - angle*pulse_per_rad - rotation_offset*pulse_per_rad, -speed*pulse_per_rad);
}

void Axis::runState(int requestedState)
{
    odrive->run_state(axNum, requestedState, false); // don't wait
}

int Axis::setParams(float stepsPerSec, float vel_gain, float encoder_bandwidth, float pos_gain, float vel_int)
{
    int error = 0;
    error += writeParam(".controller.config.vel_limit", stepsPerSec);
    error +=writeParam(".controller.config.vel_gain", vel_gain);
    error +=writeParam(".controller.config.pos_gain", pos_gain);
    error +=writeParam(".encoder.config.bandwidth", encoder_bandwidth);
    error +=writeParam(".controller.config.vel_integrator_gain", vel_int);
    return error;

//    std::stringstream ss;
//    ss<< "w axis" << axNum << ".controller.config.vel_limit " << stepsPerSec << '\n';
//    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
//    ss.clear();
//    ss<< "w axis" << axNum << ".controller.config.vel_gain " << vel_gain << '\n';
//    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
//    ss.clear();
//    ss<< "w axis" << axNum << ".controller.config.pos_gain " << encoder_bandwidth << '\n';
//    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
//    ss.clear();
//    ss<< "w axis" << axNum << ".encoder.config.bandwidth " << pos_gain << '\n';
//    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
//    ss.clear();
//    ss<< "w axis" << axNum << ".controller.config.vel_integrator_gain " << vel_int << '\n';
//    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());

}

void Axis::setMaxVel(float stepsPerSec)
{
    std::stringstream ss;
    ss<< "w axis" << axNum << ".controller.config.vel_limit " << stepsPerSec << '\n';
    buffered_pc.printf(ss.str().c_str());
    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
}
float Axis::readParam(string in)
{
    Thread::wait(10);
    std::stringstream ss;
    ss <<"r axis" << axNum << in << '\n';
    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    return odrive->readFloat();
}
float Axis::readBattery()
{
    std::stringstream ss;
    ss <<"r vbus_voltage"<< '\n';
    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    return odrive->readFloat();
}


float Axis::writeParam(string in, float val)
{
    Thread::wait(10);
    std::stringstream ss;
    ss<< "w axis" << axNum << in<< " " << val << '\n';
    odrive->serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    float read = readParam(in);
    if(read == val) return 0;
    else return 1;
}

int Axis::test()
{
    int wait = 1;
    int waitTime = 10;
    if(wait) Thread::wait(waitTime);
    int error = 0;
    float battery = readBattery();
    if(wait) Thread::wait(waitTime);
    battery = readBattery();
    if(wait) Thread::wait(waitTime);
    //buffered_pc.printf("Battery: %f \r\n",battery);
    if (battery <1.0) error++;
    float param = readParam(".controller.config.vel_limit");
    if(wait) Thread::wait(waitTime);
    //buffered_pc.printf("param: %f\r\n",param);
    writeParam(".controller.config.vel_limit", param+1.0);
    if(wait) Thread::wait(waitTime);
    float param2 = readParam(".controller.config.vel_limit");
    if(wait) Thread::wait(waitTime);
    //buffered_pc.printf("param2: %f\r\n",param2);
    if(param2 != param + 1) error ++;
    writeParam(".controller.config.vel_limit", param);
    if(wait) Thread::wait(waitTime);
    float param3 = readParam(".controller.config.vel_limit");
    if(wait) Thread::wait(waitTime);
    //buffered_pc.printf("param2: %f\r\n",param3);
    if(param3 != param) error ++;
    if (error!=0) {
        buffered_pc.printf("There have been %d errors on axis %d\r\n",error, name);
    } else {
        //buffered_pc.printf("there were no errors on axis %d  :) \r\n", name);
    }
    return error;

}