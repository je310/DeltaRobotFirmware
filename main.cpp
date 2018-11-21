
//includes. 
#include "mbed.h"
#include "odrive.h"
#include "EthernetInterface.h"
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
#include "lTime.h"

//pin definiitions
#define EXTPIN1 PB_5  //pwm spi1_mosii
#define EXTPIN2 PB_15 // PWM spi2 mosi
#define EXTPIN3 PB_13   //PWM spi2sclk
#define EXTPIN4 PB_12   //
#define EXTPIN5 PA_15   //pwm
#define EXTPIN6 PC_7    //RX6 PWM
#define ACCEL_INT PA_5    //RX6 PWM

#define EIGEN_NO_MALLOC
#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())
#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

#define GRAVITY     9.812  // London g value.

using namespace Eigen;
using std::string;


//threads and debugging for threads 
Timer debugTimer;

Thread transmitterT(osPriorityNormal, 2 * 1024,NULL, "transmitterThread");
Thread receiverT(osPriorityNormal, 32 * 1024,NULL, "receiverThread");
Thread odriveThread(osPriorityBelowNormal, 16 * 1024,NULL, "OdriveThread");
Thread accelT(osPriorityAboveNormal, 32 * 1024,NULL, "AccelThread");

Thread printBattery(osPriorityNormal, 2 * 1024,NULL, "printBatteryThread");
Thread ESKFT(osPriorityNormal, 64 * 1024,NULL, "ESKFThread") /* 32K stack */;


//io declarations
DigitalOut led1(LED1);
DigitalOut UVLed(EXTPIN4);
DigitalOut homeGND(PF_5);
DigitalIn homeSwitchA(PA_3);
DigitalIn homeSwitchB(PC_0);
DigitalIn homeSwitchC(PC_3);
DigitalOut accelPower(PA_6);
DigitalIn trigger(PF_3);
BufferedSerial buffered_pc(SERIAL_TX, SERIAL_RX,1024);
I2C i2c(PB_9, PB_8);


//data latches. 
AngleSpaceCmd ABCSpaceCommand;
int newABCSpaceCommand = 0;
CartSpaceCmd XYZSpaceCommand;
int newXYZSpaceCommand = 0;

LocationOut mocapLocation;
int newMocapLocation = 0;
LocationIn locationReturnMsg;

int newLocationReturnMsg =0;
PingIn pingMsg;


//network globals
const int BROADCAST_PORT_T = 58080;
const int BROADCAST_PORT_R = 58081;
EthernetInterface eth;
SyncTime timeTracker(0,0);
UDPSocket socket;
int isCon = 0;


//eskf globals
ESKF* eskfPTR;
int updatedESKF = 0;

//odrive globals
calVals calibration;
float batteryV = -1;

Eigen::Vector3f targetPos(0,0,0);
Eigen::Quaternionf targetRot(1,0,0,0);

//imu globals
MPU6050 mpu;
Mutex imudataMutex;
ImuData imuDataBuffer;
volatile int imuToSend = 0;
ImuDataChunk chunk;
Eigen::Vector3f accel;
Eigen::Vector3f gyro;
volatile int newData = 0;
rosTime timeMes;
rosTime lastIMUTime;
int accelPending = 0;


typedef struct{
    Vector3f accel;
    Vector3f gyro;
    rosTime stamp;
} imuMail;
Mail<imuMail, 128> mail_box;

Mail<ImuData, 128> mail_box_transmit_imu;


void transmit()
{
    while(isCon != 0) {
        Thread::wait(1);
    }
    string out_buffer = "very important data";
    SocketAddress transmit("10.0.0.51", BROADCAST_PORT_T);
    chunk.type = imuDataChunk;
    // fromRobot msg;
    // msg.motor1.busVoltage = 69;
    // buffered_pc.printf("starting send loop");
    while (true) {
      //  msg.motor1.busVoltage += 1;
        //int ret = socket.sendto(transmit, &msg, sizeof(msg));
        //printf("sendto return: %d\n", ret);
        
        // imudataMutex.lock();
        // if(imuToSend == 1){
        //     static rosTime last;
        //     imuToSend = 0;
            
        //     if(last.seconds != imuDataBuffer.time.seconds || last.nSeconds != imuDataBuffer.time.nSeconds){
        //         static int count = -1;
        //         count++;
        //         last = imuDataBuffer.time;
        //         // memcpy(&chunk.array[count%20], &imuDataBuffer,sizeof(ImuData));
        //         // //chunk.array[count%20] = imuDataBuffer;
        //         // if(count % 20 == 19){
        //         //     int ret = socket.sendto(transmit, &chunk, sizeof(chunk));
        //         // }
        //         int ret = socket.sendto(transmit, &imuDataBuffer, sizeof(imuDataBuffer));
        //     }
            
        // }
        while(!mail_box_transmit_imu.empty()){
                        osEvent evt = mail_box_transmit_imu.get();
            if(evt.status == osEventMail){
            ImuData *mail = (ImuData*)evt.value.p;
            int ret = socket.sendto(transmit, mail, sizeof(imuDataBuffer));

            mail_box_transmit_imu.free(mail);
            Thread::yield();
        }
        }
        // //imudataMutex.unlock();
        // if(newLocationReturnMsg ==1){
        //     static rosTime last;
        //     newLocationReturnMsg = 0;
        //     if(last.seconds != locationReturnMsg.time.seconds || last.nSeconds != locationReturnMsg.time.nSeconds){
        //         last = locationReturnMsg.time;
        //         int ret = socket.sendto(transmit, &locationReturnMsg, sizeof(locationReturnMsg));

        //     }
        // }
        
        Thread::signal_wait(0x1);
    }
}

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
    SocketAddress transmit("10.0.0.51", BROADCAST_PORT_T);
    //printf("bind return: %d", bind);

    char buffer[256];
    
    
        buffered_pc.printf("starting receive loop");
        rosTime now;
        
        
    while (true) {
        //printf("\nWait for packet...\n");
        int n = socket.recvfrom(&receive, buffer, sizeof(buffer));
        if(n > 0 ){
            //buffered_pc.printf("Ho\n\r");
                int32_t typeInt = (int32_t)buffer[0];
                now = timeTracker.getTime();
        switch(typeInt){
        case angleSpaceCmd:
            memcpy(&ABCSpaceCommand, buffer, n);
            newABCSpaceCommand = 1;
            odriveThread.signal_set(0x1);

            break;

        case cartSpaceCmd:
            memcpy(&XYZSpaceCommand, buffer, n);
            newXYZSpaceCommand = 1;
            odriveThread.signal_set(0x1);
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
            ESKFT.signal_set(0x1);
            locationReturnMsg.type = locationIn;
            locationReturnMsg.pos = mocapLocation.pos;
            locationReturnMsg.quat = mocapLocation.quat;
            locationReturnMsg.refTime = mocapLocation.time;
            locationReturnMsg.time = now;
            //int ret = socket.sendto(transmit, &locationReturnMsg, sizeof(locationReturnMsg));
            //newLocationReturnMsg = 1;
        }
            break;

        case lineCmd:{
            LineCmd inmsg;
            memcpy(&inmsg, buffer, n);
            //for now only use the first point and orientation. 
            targetPos << inmsg.posStart.x , inmsg.posStart.y, inmsg.posStart.z;
            targetRot= Eigen::Quaternionf(inmsg.quat.w , inmsg.quat.x , inmsg.quat.y , inmsg.quat.z);

        }
        break;

        case pingIn:
        
            buffered_pc.printf("This is an outgoing type !! Type: %d  \n\r",typeInt );
        
            break;

        case locationIn:
                buffered_pc.printf("This is an outgoing type !! Type: %d  \n\r",typeInt );
            break;

        case userIn:
                buffered_pc.printf("This is an outgoing type !! Type: %d  \n\r",typeInt );
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
    ServoAxis pitch(EXTPIN1,35, -15, 1500, (1200.0*1.0425)/120.0, 6.0);
    ServoAxis yaw(EXTPIN2,60, -60, 1500, -(1200.0*1.0425)/120.0, 2.0);  
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
    error = 1;
    while(error > 0){
        error = kin.setSafeParams();
        Thread::wait(100);
        buffered_pc.printf("there were %d errors in the read/write\r\n",error);
        Thread::wait(100);
    }

        pitch.setAngle(0);
            yaw.setAngle(0);
//    kin.goIdle();//for some reason we need to do it multiple times!
            int even = 1 ;
    while(*A.homeSwitch_||*B.homeSwitch_ || *C.homeSwitch_) {
       Thread::wait(1000); 
        even = even * -1.0;
            yaw.setAngle(40*even);
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
    error = 1;
    while(error > 0){
        error = kin.setFastParams();
        Thread::wait(100);
        buffered_pc.printf("there were %d errors in the read/write\r\n",error);
        Thread::wait(100);
    }
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
             i += inc;
            if(up == 1)k+=0.2;
            else k-=0.2;
            if(k <= min) up = 0;
            if(k >= max) up = 1;
            radius =0.6* (span - abs(abs(k) - mid));
            //buffered_pc.printf("x,y,z = %f %f %f",radius*sin(i),radius*cos(i),k);
            int error = kin.goToPos(40.0*sin(i)/1000.0,40.0*cos(i)/1000.0,k/1000.0);
            pitch.setAngle(25.0*sin(i));
            yaw.setAngle(25.0*cos(i));
            //int error = kin.goToPos(0,0,k);
            if(i > 2*pi) i = 0;
            Thread::wait(1);
        }
        else{
            if(newABCSpaceCommand == 1){
                newABCSpaceCommand = 0;
                kin.goToAngles(ABCSpaceCommand.A,ABCSpaceCommand.B,ABCSpaceCommand.C);
                pitch.setAngle(ABCSpaceCommand.pitch);
                yaw.setAngle(ABCSpaceCommand.yaw);
            }
            if(newXYZSpaceCommand == 1){
                newXYZSpaceCommand = 0;
                error += kin.goToPos(XYZSpaceCommand.pos.x,XYZSpaceCommand.pos.y,XYZSpaceCommand.pos.z);
                if(error) buffered_pc.printf("There was an error moving ");
                pitch.setAngle(XYZSpaceCommand.pitch);
                yaw.setAngle(XYZSpaceCommand.yaw);
            }
            if(updatedESKF == 1){
                updatedESKF = 0;
                if(loopCounter % 1000 == 0){
                    Vector3f pos = eskfPTR->getPos();
                    buffered_pc.printf(" position x y z %f %f %f\r\n",pos[0],pos[1],pos[2] ); 
                }
                Eigen::Affine3f here = Eigen::Translation3f(eskfPTR->getPos()) * eskfPTR->getQuat();
                //Eigen::Affine3f target = Eigen::Translation3f(0,0,1) * Eigen::Quaternionf(1,0,0,0);
                Eigen::Affine3f target = Eigen::Translation3f(targetPos) * targetRot;
                Eigen::Quaternionf targetRotInv = targetRot.inverse();
                Eigen::Vector3f up;
                up << 0,1,0;
                Eigen::Vector3f side ;
                side << 0,0,1;
                                rosTime rTime = timeTracker.getTime();
                lTime lt(rTime.seconds,rTime.nSeconds);
                //up = targetRot._transformVector(up);
                up = 0.05*cos(2.0*lt.toSec())*up;
                //side = targetRot._transformVector(side);
                side = 0.05*sin(2.0*lt.toSec())*side;
                up = up + side;
                up = targetRot._transformVector(up);
                //side = targetRot.inverse()._transformVector(side);

                target =  Eigen::Translation3f(up) * target;
                Eigen::Vector3f angRates = eskfPTR->lastImu_.gyro;
                float ffGain = 0.025;
                //buffered_pc.printf(" here x y z %f %f %f\r\n",eskfPTR->getPos()[0],eskfPTR->getPos()[1],eskfPTR->getPos()[2] );
                //buffered_pc.printf(" hereQ w x y z %f %f %f %f\r\n",eskfPTR->getQuat().coeffs()[0],eskfPTR->getQuat().coeffs()[1],eskfPTR->getQuat().coeffs()[2],eskfPTR->getQuat().coeffs()[2]  );
                int notReachable = kin.goToWorldPos(here,target, angRates, ffGain);
                if (notReachable) UVLed = 0;
                else UVLed =1;
            }
    }
        Thread::signal_wait(0x1);
        //Thread::yield();
        if(loopCounter % 200000 == 0) {
            batteryV = OD1.readBattery();
        }
    }

}

float detlaTimu = 0.001;
void accelInterrupt(){
            //newData = 1;
            
             //if(!accelPending){
                timeMes = timeTracker.getTime();
                 accelT.signal_set(0x1);
             //}

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

    ImuData dataOut; 

    SocketAddress transmit("10.0.0.51", BROADCAST_PORT_T);
    ImuDataChunk chunk;
    chunk.type = imuDataChunk;
    
    Timer debugT;
    debugT.start();
    while(1) {
        //Thread::wait(200);

        //if(newData){
            //newData = 0;
            // imuDataBuffer.time.seconds = timeMes.seconds;
            // imuDataBuffer.time.nSeconds = timeMes.nSeconds;
            dataCount ++;
            rosTime timeMesLocal = timeMes;
            mpu.readAccelData(accelCount);  // Read the x/y/z adc values
            mpu.getAres();
            mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu.getGres();
            


                        // Now we'll calculate the accleration value into actual g's

            // imuDataBuffer.accel.x  = accel[0];  // get actual g value, this depends on scale being set
            // imuDataBuffer.accel.y = accel[1];
            // imuDataBuffer.accel.z = accel[2];

            // imuDataBuffer.gyro.x = gyro[0]; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            // imuDataBuffer.gyro.y = gyro[1]; // - gyroBias[1];
            // imuDataBuffer.gyro.z = gyro[2]; // - gyroBias[2];

            // imuDataBuffer.type = imuData;
            //int ret = socket.sendto(transmit, &imuDataBuffer, sizeof(imuDataBuffer));
            // chunk.array[dataCount] = imuDataBuffer;
            // if(dataCount == 19){
            //     dataCount = 0;
            //     int ret = socket.sendto(transmit, &chunk, sizeof(chunk));
            // }
            //imuToSend = 1;
            detlaTimu = timeTracker.difference(lastIMUTime, timeMesLocal);
            lastIMUTime = timeMesLocal;
            if(detlaTimu != 0.0){
                imuMail *mail = mail_box.alloc();
                mail->accel <<  (float)accelCount[0]*aRes,(float)accelCount[1]*aRes,(float)accelCount[2]*aRes;

                mail->gyro <<(float)gyroCount[0]*gRes,(float)gyroCount[1]*gRes,(float)gyroCount[2]*gRes;

                mail->stamp = timeMesLocal;
                mail_box.put(mail);
                ESKFT.signal_set(0x1);
            }
            // if(dataCount % 2000 ==0){
            //     float fps = 1000000 * dataCount / debugT.read_us();
            //     buffered_pc.printf("Ax:%f \t Ay:%f\t Az:%f\t Gx:%f\t Gy:%f\t Gz:%f\t at time %d\ts %d\tns %fframesPerS\r\n",accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],timeMes.seconds,timeMes.nSeconds, fps);
            //     dataCount = 0;
            //     debugT.reset();
            // }
        //}
        //Thread::yield();
        accelPending = 0;
        Thread::signal_wait(0x1);
        accelPending = 1;

    }
}

void batteryThread()
{
    while(1) {
        Thread::wait(5000);
        buffered_pc.printf("batteryVoltage = %f\n\r", batteryV);
    }


}

int lastMocapT = 0;
int lastIMUT = 0;
void ESKFThread(){

    //set up the ESKF as per the desktop example. 
        float sigma_accel = 0.124; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 0.00276; // [rad/s] (value derived from Noise Spectral Density in datasheet)
    float sigma_accel_drift = 0.0025; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 5e-5; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 100*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 100*sigma_gyro_drift; // [rad/s]

    float sigma_mocap_pos = 0.001; // [m]
    float sigma_mocap_rot = 0.01; // [rad]
    eskfPTR = new ESKF(
            Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            ESKF::makeState(
                Vector3f(0, 0, 0), // init pos
                Vector3f(0, 0, 0), // init vel
                Quaternionf(AngleAxisf(0.5f, Vector3f(1, 0, 0))), // init quaternion
                Vector3f(0, 0, 0), // init accel bias
                Vector3f(0, 0, 0) // init gyro bias
            ),
            ESKF::makeP(
                SQ(sigma_init_pos) * I_3,
                SQ(sigma_init_vel) * I_3,
                SQ(sigma_init_dtheta) * I_3,
                SQ(sigma_init_accel_bias) * I_3,
                SQ(sigma_init_gyro_bias) * I_3
            ),
            SQ(sigma_accel),
            SQ(sigma_gyro),
            SQ(sigma_accel_drift),
            SQ(sigma_gyro_drift),
            ESKF::delayTypes::applyUpdateToNew,100);

    //loop waiting for data to come in
    int imuCount = 0;
    int mocapCount = 0;
    float degToRad = M_PI/180.0;
        //rotation mat for the accel, to correct for mounting, result should be ros convention. 
    Eigen::Matrix<float, 3,3> rotMat;
    rotMat <<   -1, 0, 0
                ,0, 0,-1
                ,0,-1, 0;



    while(1){
        if(newMocapLocation==1){
            newMocapLocation = 0;
            Quaternionf quat(mocapLocation.quat.w,mocapLocation.quat.x,mocapLocation.quat.y,mocapLocation.quat.z);
            Vector3f pos(mocapLocation.pos.x,mocapLocation.pos.y,mocapLocation.pos.z);
            lTime stamp(mocapLocation.time.seconds,mocapLocation.time.nSeconds);
            rosTime nowRos = timeTracker.getTime();
            lTime now(nowRos.seconds,nowRos.nSeconds);
            eskfPTR->measurePos(pos,SQ(sigma_mocap_pos)*I_3,stamp,now);
            Thread::yield();
            eskfPTR->measureQuat(quat,SQ(sigma_mocap_rot)*I_3,stamp,now);
            Thread::yield();

            mocapCount ++;
            // if(mocapCount == 200){

            //     float fps = 1000000 * mocapCount / (debugTimer.read_us() - lastMocapT);
            //     buffered_pc.printf("mocapFPS %fframesPerS\r\n",fps);
            //     mocapCount = 0;
            //     lastMocapT = debugTimer.read_us();
            //     Vector3f posD = eskfPTR->getPos();
            //         buffered_pc.printf(" position x y z %f %f %f\r\n",posD[0],posD[1],posD[2] );
            // }
        }
        //if(imuToSend==1){
        
            
        while(!mail_box.empty()){
            osEvent evt = mail_box.get();
            if(evt.status == osEventMail){
            imuMail *mail = (imuMail*)evt.value.p;
            imuToSend = 0;
                        

            Vector3f accelLocal = GRAVITY * rotMat * mail->accel;
            
            Vector3f gyroLocal = degToRad * rotMat * mail->gyro;

            rosTime timeLocal = mail->stamp;
            lTime stamp(timeLocal.seconds,timeLocal.nSeconds);
            rosTime nowRos = timeTracker.getTime();
            lTime now(nowRos.seconds,nowRos.nSeconds);
            mail_box.free(mail);
            static lTime lastStamp(0,0);
            lTime duration = stamp - lastStamp;
            if(duration.toSec() > 1.0) duration.fromSec(0.001);
            if(duration.toSec() == 0.0) duration.fromSec(0.001);

            eskfPTR->predictIMU(accelLocal,gyroLocal,duration.toSec(),stamp);

            updatedESKF = 1;
            odriveThread.signal_set(0x1);
                        imuCount ++;
                        if(imuCount %10 == 0){
                            ImuData *mail = mail_box_transmit_imu.alloc();
                            mail->type = imuData;
                            mail->accel.x  = accelLocal[0];  // get actual g value, this depends on scale being set
                            mail->accel.y = accelLocal[1];
                            mail->accel.z = accelLocal[2];

                            mail->gyro.x = gyroLocal[0]; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
                            mail->gyro.y = gyroLocal[1]; // - gyroBias[1];
                            mail->gyro.z = gyroLocal[2]; // - gyroBias[2];
                            mail->time = timeLocal;
                            mail_box_transmit_imu.put(mail);
                            imuToSend = 1;
                            transmitterT.signal_set(0x1);
                        }
            // if(imuCount == 2000){
            //     float fps = 1000000 * imuCount / (debugTimer.read_us() - lastIMUT);
            //     buffered_pc.printf("imuFPS %fframesPerS  delta time %f\r\n",fps, detlaTimu);
            //     imuCount = 0;
            //     lastIMUT = debugTimer.read_us();
            // }
            Thread::yield();
            }
        }
        //Thread::yield();
        Thread::signal_wait(0x1);
    }


}




int main()
{
    debugTimer.start();
    // testing eigen run time. 
    buffered_pc.baud(115200);
    buffered_pc.printf("hello\r\n");
    UVLed = 1;


    calibration.e = 58.095;
    calibration.f = 60.722;
    calibration.re = 150;
    calibration.rf = 137.0;
    calibration.Aoffset = 0.43 - 0.111701 + 0.0142;
    calibration.Boffset = 0.43 - 0.111701 + 0.0142;
    calibration.Coffset = 0.43 - 0.111701 + 0.0142;
    calibration.gearRatio = 89.0/24.0;
    
    i2c.frequency(1000000);

    

    //set switches up
    homeGND = false;
    homeSwitchA.mode(PullUp);
    homeSwitchB.mode(PullUp);
    homeSwitchC.mode(PullUp);
    trigger.mode(PullUp);



    //buffered_pc.printf("Controller IP Address is %s\r\n", eth.get_ip_address());
    Thread::wait(1000);

    
    receiverT.start(receive);
    Thread::wait(7000);
    ESKFT.start(ESKFThread);
    transmitterT.start(transmit);
    UVLed = 0;
    odriveThread.start(runOdrive);
    accelT.start(accelThread);
    printBattery.start(batteryThread);

    while (true) {
        //led1 = !led1;
        Thread::wait(500);
    }
}
