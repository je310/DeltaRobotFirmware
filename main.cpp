#include "mbed.h"
#include "odrive.h"
#include "lwip-interface/EthernetInterface.h"
#include "comms.h"
#include <string>
#include "calibration.h"
#include "Axis.h"
#include "servoAxis.h"
#include "kinematics.h"
#include "BufferedSerial.h"
#include    "syncTime.h"
#include "MPU6050.h"
#include "ESKF.h"
#include <Core.h>
#include <Geometry.h>
#define EXTPIN1 PB_5  //pwm spi1_mosii
#define EXTPIN2 PB_15 // PWM spi2 mosi
#define EXTPIN3 PB_13   //PWM spi2sclk
#define EXTPIN4 PB_12   //
#define EXTPIN5 PA_15   //pwm
#define EXTPIN6 PC_7    //RX6 PWM
#define ACCEL_INT PA_5    //RX6 PWM
DigitalOut led1(LED1);
DigitalOut homeGND(PF_5);
DigitalIn homeSwitchA(PA_3);
DigitalIn homeSwitchB(PC_0);
DigitalIn homeSwitchC(PC_3);
DigitalOut accelPower(PA_6);
DigitalIn trigger(PF_3);
BufferedSerial buffered_pc(SERIAL_TX, SERIAL_RX,1024);
calVals calibration;
MPU6050 mpu;
I2C i2c(PB_9, PB_8);

AngleSpaceCmd ABCSpaceCommand;
int newABCSpaceCommand = 0;
CartSpaceCmd XYZSpaceCommand;
int newXYZSpaceCommand = 0;

LocationOut mocapLocation;
int newMocapLocation = 0;
LocationIn locationReturnMsg;

int newLocationReturnMsg =0;

using std::string;
const int BROADCAST_PORT_T = 58080;
const int BROADCAST_PORT_R = 58081;
EthernetInterface eth;

SyncTime timeTracker(0,0);
float batteryV = -1;
UDPSocket socket;
int isCon = 0;
volatile int imuToSend = 0;
    ImuDataChunk chunk;


Mutex imudataMutex;
ImuData imuDataBuffer;
void transmit()
{
    while(isCon != 0) {
        Thread::yield();
    }
    string out_buffer = "very important data";
    SocketAddress transmit("10.0.0.160", BROADCAST_PORT_T);
    chunk.type = imuDataChunk;
    // fromRobot msg;
    // msg.motor1.busVoltage = 69;
    // buffered_pc.printf("starting send loop");
    while (true) {
      //  msg.motor1.busVoltage += 1;
        //int ret = socket.sendto(transmit, &msg, sizeof(msg));
        //printf("sendto return: %d\n", ret);
        
        // imudataMutex.lock();
        if(imuToSend == 1){
            static rosTime last;
            imuToSend = 0;
            
            if(last.seconds != imuDataBuffer.time.seconds || last.nSeconds != imuDataBuffer.time.nSeconds){
                static int count = -1;
                count++;
                last = imuDataBuffer.time;
                // memcpy(&chunk.array[count%20], &imuDataBuffer,sizeof(ImuData));
                // //chunk.array[count%20] = imuDataBuffer;
                // if(count % 20 == 19){
                //     int ret = socket.sendto(transmit, &chunk, sizeof(chunk));
                // }
                int ret = socket.sendto(transmit, &imuDataBuffer, sizeof(imuDataBuffer));
            }
            
        }
        //imudataMutex.unlock();
        if(newLocationReturnMsg ==1){
            static rosTime last;
            newLocationReturnMsg = 0;
            if(last.seconds != locationReturnMsg.time.seconds || last.nSeconds != locationReturnMsg.time.nSeconds){
                last = locationReturnMsg.time;
                int ret = socket.sendto(transmit, &locationReturnMsg, sizeof(locationReturnMsg));

            }
        }
        
        Thread::yield();
    }
}
PingIn pingMsg;
void receive()
{
    eth.connect();

    //socket.open(&eth);
    
    //socket.set_blocking(false);
    

 
    buffered_pc.printf("Ethernet returned %d \n\r", isCon);
    Thread::wait(100);
    if(isCon !=0) eth.disconnect();
    while(isCon != 0) {
        Thread::yield();
    }
    socket.open(&eth);
    buffered_pc.printf("Ethernet is connected at %s \n\r", eth.get_ip_address());
    Thread::wait(100);
    SocketAddress receive;
    //UDPSocket socket(&eth);
    int bind = socket.bind(BROADCAST_PORT_R);
    SocketAddress transmit("10.0.0.160", BROADCAST_PORT_T);
    //printf("bind return: %d", bind);

    char buffer[256];
    
    
        buffered_pc.printf("starting receive loop");
        rosTime now;
        Timer debugTimer;
        
    while (true) {
        //printf("\nWait for packet...\n");
        int n = socket.recvfrom(&receive, buffer, sizeof(buffer));
        if(n > 0 ){
            //buffered_pc.printf("Ho\n\r");
            debugTimer.reset();
        debugTimer.start();
                int32_t typeInt = (int32_t)buffer[0];
                now = timeTracker.getTime();
        switch(typeInt){
        case angleSpaceCmd:
            memcpy(&ABCSpaceCommand, buffer, n);
            newABCSpaceCommand = 1;


            break;

        case cartSpaceCmd:
            memcpy(&XYZSpaceCommand, buffer, n);
            newXYZSpaceCommand = 1;
            break;

        case pingOut:{
                PingOut inmsg;
                memcpy(&inmsg, buffer, n);

                
                if(now.seconds < 10000){
                    buffered_pc.printf("Hard resetting time to  %ds and %dns \n\r",inmsg.time.seconds,inmsg.time.nSeconds);
                    timeTracker.hardReset(inmsg.time.seconds,inmsg.time.nSeconds);
                }

                pingMsg.type = pingIn;
                pingMsg.sentTime.seconds = inmsg.time.seconds;
                pingMsg.sentTime.nSeconds = inmsg.time.nSeconds;
                
                pingMsg.responseTime.seconds = now.seconds;
                pingMsg.responseTime.nSeconds = now.nSeconds;
                debugTimer.stop();
                int ret = socket.sendto(transmit, &pingMsg, sizeof(pingMsg));
                //
                
                //buffered_pc.printf("sectionTime %d \r\n", debugTimer.read_high_resolution_us());
                
                //buffered_pc.printf("current time is %ds and %dns \r\n", now.seconds, now.nSeconds);
            
                if(inmsg.timeOffset !=0.0f){
                    //buffered_pc.printf("Applying offset %fs \n\r",inmsg.timeOffset);

                    timeTracker.updateTime(inmsg.timeOffset);            
                }
            }
            break;

        case locationOut:{
            memcpy(&mocapLocation, buffer, n);
            newMocapLocation = 1;
            
            locationReturnMsg.type = locationIn;
            locationReturnMsg.pos = mocapLocation.pos;
            locationReturnMsg.quat = mocapLocation.quat;
            locationReturnMsg.refTime = mocapLocation.time;
            locationReturnMsg.time = now;
            //int ret = socket.sendto(transmit, &locationReturnMsg, sizeof(locationReturnMsg));
            //newLocationReturnMsg = 1;
        }
            break;

        case pingIn:
        
            buffered_pc.printf("This is an outgoing type !!\n\r");
        
            break;

        case locationIn:
                buffered_pc.printf("This is an outgoing type !!\n\r");
            break;

        case userIn:
                buffered_pc.printf("This is an outgoing type !!\n\r");
            break;

        default: ;

        }
        
        //buffer[n] = '\0';
        //buffered_pc.printf("Count:%i, Motor1Vel:%f, Motor1Tor: %f \r\n",inmsg.count, inmsg.motor1.velocity,inmsg.motor1.torque);
        }
        //debugTimer.reset();
        Thread::yield();
        //Thread::wait(1);
    }
}

int getCountPos(BufferedSerial ser, int axis)
{

}

Eigen::Affine3f posAngToEigen(Eigen::Vector3f pos, float yawin, float pitchin){
    float pi = M_PI;
    Eigen::Affine3f mat = Eigen::Translation3f(pos) * Eigen::AngleAxisf(pi*yawin/180, Eigen::Vector3f::UnitZ())
                        * Eigen::AngleAxisf(pi*pitchin/180, Eigen::Vector3f::UnitY());

    return mat;

}

Eigen::Affine3f transformToEigen(Eigen::Vector3f pos, Eigen::Quaternionf quat){
    Eigen::Affine3f out;
    out.setIdentity();
    Eigen::Vector3f translate(pos[0],pos[1],pos[2]);
    out.translate(translate);
    out.rotate(quat);
    return out;
}

void runOdrive()
{
    //start servos on endEffector
    ServoAxis pitch(EXTPIN1,35, -35, 1500, 1200.0/120.0, 6.0);
    ServoAxis yaw(EXTPIN2,50, -50, 1500, -1200.0/120.0, 2.0);  
//    while(1) {
//        pitch.setAngle(30);
//        yaw.setAngle(30);
//        Thread::wait(300);
//        pitch.setAngle(-30);
//        yaw.setAngle(-30);
//        Thread::wait(300);
//        }
        
    
    BufferedSerial ODSerial0(PC_12,PD_2);
    ODSerial0.baud(921600);
    BufferedSerial ODSerial1(PD_5,PD_6);
    ODSerial1.baud(921600);
    ODrive OD0(ODSerial0);
    ODrive OD1(ODSerial1);
    Axis A(&OD1, 1, &homeSwitchA,calibration,AX_A);
    Axis B(&OD1, 0, &homeSwitchB,calibration,AX_B);
    Axis C(&OD0, 0, &homeSwitchC,calibration,AX_C);
    int error = 0;
    error += A.test();
    error +=B.test();
    error +=C.test();
    buffered_pc.printf("there were %d errors in the read/write\r\n",error);
    Kinematics kin(&A, &B, &C,&yaw, &pitch, calibration); // the Kinematics class contains everything
    buffered_pc.printf("setting motors to idle\r\n");
    kin.goIdle();
    kin.goIdle();
    error += kin.setSafeParams();
    while(error) {}
        pitch.setAngle(0);
            yaw.setAngle(0);
//    kin.goIdle();//for some reason we need to do it multiple times!
    while(*A.homeSwitch_||*B.homeSwitch_ || *C.homeSwitch_) {
       Thread::wait(5); 
       batteryV = OD1.readBattery();
       } // wait till user
    buffered_pc.printf("finding index\r\n");
    kin.findIndex();
    buffered_pc.printf("activating motors\r\n");
    kin.activateMotors();
    buffered_pc.printf("homing motors\r\n");
    kin.homeMotors();
    error += kin.goToPos(120,0,0);


    //kin.goToAngles(pi/4,pi/4,pi/4);
    Thread::wait(500);
    //error += kin.setFastParams();
    while(error) {}
    Thread::wait(1000);

    Thread::wait(100);
    float inc = pi /150;
    float i = 0;
    float k = -100;
    int up = 0;
    float min = 80;
    float max =220;
    float mid = min+max;
    mid/= 2;
    float span = mid - min;
    float radius = 40;
    int loopCounter = 0;
    while(true) {
        loopCounter++;
        int count = 0;
        for(int j = 0 ; j < 100; j++) {

            if(trigger == false) {
                count++;
            }
        }
        if(count>90) {
           
        }
        if(newABCSpaceCommand == 1){
            newABCSpaceCommand = 0;
            kin.goToAngles(ABCSpaceCommand.A,ABCSpaceCommand.B,ABCSpaceCommand.C);
            pitch.setAngle(ABCSpaceCommand.pitch);
            yaw.setAngle(ABCSpaceCommand.yaw);
        }
        if(newXYZSpaceCommand == 1){
            newXYZSpaceCommand = 0;
            buffered_pc.printf("going to %f %f %f\r\n",XYZSpaceCommand.pos.x,XYZSpaceCommand.pos.y,XYZSpaceCommand.pos.z);
            error += kin.goToPos(XYZSpaceCommand.pos.x,XYZSpaceCommand.pos.y,XYZSpaceCommand.pos.z);
            if(error) buffered_pc.printf("There was an error moving ");
            pitch.setAngle(XYZSpaceCommand.pitch);
            yaw.setAngle(XYZSpaceCommand.yaw);
        }
        if(newMocapLocation){
            newMocapLocation = 0;
            buffered_pc.printf("mocap!\r\n");
            Eigen::Affine3f here = transformToEigen(Eigen::Vector3f(mocapLocation.pos.x,mocapLocation.pos.y,mocapLocation.pos.z),Eigen::Quaternionf(mocapLocation.quat.w,mocapLocation.quat.x,mocapLocation.quat.y,mocapLocation.quat.z));
            Eigen::Affine3f target = Eigen::Translation3f(0,0,1) * Eigen::Quaternionf(1,0,0,0);
            kin.goToWorldPos(here,target);
        }
        Thread::wait(1);
        if(loopCounter % 2000 == 0) {
            batteryV = OD1.readBattery();
        }
    }

}

volatile int newData = 0;
rosTime timeMes;
float accelDeltaT =1;
void accelInterrupt(){

            newData = 1;
            timeMes = timeTracker.getTime();

}

void accelThread()
{
    InterruptIn intPin(ACCEL_INT);

    accelPower = 1;
    uint8_t whoami = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    buffered_pc.printf("I AM 0x%x\n\r", whoami);
    buffered_pc.printf("I SHOULD BE 0x68\n\r");
    mpu.resetMPU6050(); // Reset registers to default in preparation for device calibration
    //mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    mpu.initMPU6050();
    buffered_pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    intPin.rise(&accelInterrupt);
    Timer t;
    t.start();
    float sum = 0;
    uint32_t sumCount = 0;
    int dataCount = -1 ;
    //rotation mat for the accel, to correct for mounting, result should be ros convention. 
    Eigen::Matrix<float, 3,3> rotMat;
    rotMat <<   -1, 0, 0
                ,0, 0,-1
                ,0,-1, 0;
    ImuData dataOut; 
    Eigen::Vector3f accel;
    Eigen::Vector3f gyro;
    SocketAddress transmit("10.0.0.160", BROADCAST_PORT_T);
    ImuDataChunk chunk;
    chunk.type = imuDataChunk;
    while(1) {
        //Thread::wait(200);

        if(newData){
            newData = 0;
            imuDataBuffer.time.seconds = timeMes.seconds;
            imuDataBuffer.time.nSeconds = timeMes.nSeconds;
            dataCount ++;

            mpu.readAccelData(accelCount);  // Read the x/y/z adc values
            mpu.getAres();
            mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu.getGres();
            
            accel <<  (float)accelCount[0]*aRes,(float)accelCount[1]*aRes,(float)accelCount[2]*aRes;

            accel = (rotMat * accel).eval();
            gyro <<  (float)gyroCount[0]*gRes,(float)gyroCount[1]*gRes,(float)gyroCount[2]*gRes;
            gyro = (rotMat * gyro).eval();

                        // Now we'll calculate the accleration value into actual g's

            imuDataBuffer.accel.x  = accel[0];  // get actual g value, this depends on scale being set
            imuDataBuffer.accel.y = accel[1];
            imuDataBuffer.accel.z = accel[2];

            imuDataBuffer.gyro.x = gyro[0]; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            imuDataBuffer.gyro.y = gyro[1]; // - gyroBias[1];
            imuDataBuffer.gyro.z = gyro[2]; // - gyroBias[2];

            imuDataBuffer.type = imuData;
            //int ret = socket.sendto(transmit, &imuDataBuffer, sizeof(imuDataBuffer));
            // chunk.array[dataCount] = imuDataBuffer;
            // if(dataCount == 19){
            //     dataCount = 0;
            //     int ret = socket.sendto(transmit, &chunk, sizeof(chunk));
            // }
            //imuToSend = 1;
            
            
            // if(dataCount % 2000 ==0){
            //     buffered_pc.printf("Ax:%f \t Ay:%f\t Az:%f\t Gx:%f\t Gy:%f\t Gz:%f\t at time %d\ts %d\tns %fhz\r\n",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],timeMes.seconds,timeMes.nSeconds, 1.0/accelDeltaT);
            // }
        }

        Thread::yield();

    }
}

void batteryThread()
{
    while(1) {
        Thread::wait(5000);
        buffered_pc.printf("batteryVoltage = %f\n\r", batteryV);
    }


}

void ESKFThread(){
ESKF ourESKF;
    Vector3f acc, gyro,pos;
    Quaternionf quat(0.25,0.25,0.25,0.25);
    acc = Vector3f::Random();
    gyro = Vector3f::Random();
    pos = Vector3f::Random();
    Timer eigenTimer;
    eigenTimer.reset();
    eigenTimer.start();
    //for(int i = 0; i < 1000; i ++){
        ourESKF.predictionUpdate(acc,gyro, 0.001);
        eigenTimer.stop();
    buffered_pc.printf("the eigen took %f seconds\r\n",eigenTimer.read());
    eigenTimer.reset();
    eigenTimer.start();
   // }
   // for(int i = 0; i < 100; i ++){
        ourESKF.observeErrorState(pos,quat);
   // }
    eigenTimer.stop();
    buffered_pc.printf("the eigen took %f seconds\r\n",eigenTimer.read());

}




int main()
{
    
    // testing eigen run time. 
    buffered_pc.baud(115200);
    buffered_pc.printf("hello\r\n");
    


    calibration.e = 58.095;
    calibration.f = 60.722;
    calibration.re = 150;
    calibration.rf = 137.0;
    calibration.Aoffset = 0.43 - 0.111701 + 0.0142;
    calibration.Boffset = 0.43 - 0.111701 + 0.0142;
    calibration.Coffset = 0.43 - 0.111701 + 0.0142;
    calibration.gearRatio = 89.0/24.0;
    
    i2c.frequency(400000);
    Thread transmitter;
    Thread receiver(osPriorityNormal, 32 * 1024);
    Thread odriveThread(osPriorityNormal, 16 * 1024);;
    Thread accel(osPriorityNormal, 32 * 1024);;
    Thread timeUpdate;
    Thread printBattery;
    Thread ESKFT(osPriorityNormal, 32 * 1024) /* 32K stack */;
    //ESKFT.start(ESKFThread);

    //set switches up
    homeGND = false;
    homeSwitchA.mode(PullUp);
    homeSwitchB.mode(PullUp);
    homeSwitchC.mode(PullUp);
    trigger.mode(PullUp);



    //buffered_pc.printf("Controller IP Address is %s\r\n", eth.get_ip_address());
    Thread::wait(5000);

    
    receiver.start(receive);
    Thread::wait(5000);
    transmitter.start(transmit);
    odriveThread.start(runOdrive);
    accel.start(accelThread);
    printBattery.start(batteryThread);

    while (true) {
        //led1 = !led1;
        Thread::wait(500);
    }
}
