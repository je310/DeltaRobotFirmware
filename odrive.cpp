#include "odrive.h"
extern BufferedSerial buffered_pc;
static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;


ODrive::ODrive(BufferedSerial& serial)
    : serial_(serial) {}

void ODrive::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODrive::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    std::stringstream ss;
    ss << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
}

void ODrive::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODrive::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    std::stringstream ss;
    ss  << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
}

float ODrive::readFloat() {
    return atof(readString().c_str());
}

int32_t ODrive::readInt() {
    return atoi(readString().c_str());
}

void ODrive::FinishedSending(){
    
}

bool ODrive::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    std::stringstream ss;
    ss   << "w axis" << axis << ".requested_state " << requested_state << '\n';
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    if (wait) {
        do {
            Thread::wait(100);
            std::stringstream ss2;
            ss2  << "r axis" << axis << ".current_state\n";
            serial_.write((const uint8_t *)ss2.str().c_str(),ss2.str().length());
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

std::string ODrive::readString() {
    Timer timer;
    std::string str = "";
    static const unsigned long timeout = 1000;
    timer.reset();
    timer.start();
    for (;;) {
        while (!serial_.readable ()) {
            if (timer.read_ms() >= timeout) {
                return str;
            }
        }
        char c = serial_.getc ();
        if (c == '\n')
            break;
        str += c;
    }
    timer.stop();
    //buffered_pc.printf(str.c_str());
    return str;
}

float ODrive::readBattery(){
        std::stringstream ss;
   ss   <<"r vbus_voltage"<< '\n';
    serial_.write((const uint8_t *)ss.str().c_str(),ss.str().length());
    return readFloat();
    
    }