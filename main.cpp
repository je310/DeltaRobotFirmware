
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
#include "path.h"

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

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
Thread receiverT(osPriorityNormal, 8 * 1024,NULL, "receiverThread");
Thread odriveThread(osPriorityBelowNormal, 16 * 1024,NULL, "OdriveThread");
Thread accelT(osPriorityAboveNormal, 8 * 1024,NULL, "AccelThread");

Thread printBattery(osPriorityNormal, 2 * 1024,NULL, "printBatteryThread");
Thread ESKFT(osPriorityNormal, 64 * 1024,NULL, "ESKFThread") /* 32K stack */;
Thread performanceThread(osPriorityBelowNormal, 8 * 1024,NULL, "OdriveThread");


//io declarations
DigitalOut led1(LED1);
DigitalOut UVLed(EXTPIN4);
DigitalOut homeGND(PF_5);
DigitalOut extGND(EXTPIN6);
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

string hubAddress = "10.0.0.52";

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


float travelSpeedVal  = 0.1;
//Path management 

PathManager* PM;

//performance tracking;

int imuCount = 0;
int updateCount = 0;
int estimateCount = 0;
int motorCount = 0;
lTime lastPerformanceT;


typedef struct{
    Vector3f accel;
    Vector3f gyro;
    rosTime stamp;
} imuMail;
Mail<imuMail, 128> mail_box;

Mail<ImuData, 64> mail_box_transmit_imu;

Mail<KinematicsInfo, 16> mail_box_kinematics_info;

Mail<LocationIn, 16> mail_box_headPosOut;

Mail<LocationIn, 128> mail_box_mocapOut;

Mail<PathUpdate, 64> mail_box_PathUpdate;

Mail<LocationOut,16> mail_box_mocapToProcess;

// Mail<LocationIn,8> mail_box_locationIn;


void transmit()
{
    while(isCon != 0) {
        Thread::wait(1);
    }
    string out_buffer = "very important data";
    SocketAddress transmit(hubAddress.c_str(), BROADCAST_PORT_T);
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
        if(!mail_box_transmit_imu.empty()){
            osEvent evt = mail_box_transmit_imu.get();
            if(evt.status == osEventMail){
                ImuData *mail = (ImuData*)evt.value.p;
                int ret = socket.sendto(transmit, mail, sizeof(ImuData));

                mail_box_transmit_imu.free(mail);
                Thread::yield();
            }
        }
        if(!mail_box_kinematics_info.empty()){
            osEvent evt = mail_box_kinematics_info.get();
            if(evt.status == osEventMail){
                KinematicsInfo *mail = (KinematicsInfo*)evt.value.p;
                int ret = socket.sendto(transmit, mail, sizeof(KinematicsInfo));

                mail_box_kinematics_info.free(mail);
                Thread::yield();
            }
        }
        if(!mail_box_headPosOut.empty()){
            osEvent evt = mail_box_headPosOut.get();
            if(evt.status == osEventMail){
                LocationIn *mail = (LocationIn*)evt.value.p;
                int ret = socket.sendto(transmit, mail, sizeof(LocationIn));

                mail_box_headPosOut.free(mail);
                Thread::yield();
            }
        }
        if(!mail_box_PathUpdate.empty()){
            osEvent evt = mail_box_PathUpdate.get();
            if(evt.status == osEventMail){
                PathUpdate *mail = (PathUpdate*)evt.value.p;
                int ret = socket.sendto(transmit, mail, sizeof(PathUpdate));

                mail_box_PathUpdate.free(mail);
                Thread::yield();
            }
        }

        if(!mail_box_mocapOut.empty()){
            osEvent evt = mail_box_mocapOut.get();
            if(evt.status == osEventMail){
                LocationIn *mail = (LocationIn*)evt.value.p;
                if(mail->refTime.seconds!= 0){
                    int ret = socket.sendto(transmit, mail, sizeof(LocationIn));
                }
                mail_box_mocapOut.free(mail);
                Thread::yield();
            }
        }
        
        Thread::signal_wait(0x1);
    }
}

int loopCounter = 0;
//Eigen::Vector3f current(0,0,0);
int needNewSeg = 0;
float findHorizon = 0.8;
int retID = -1;
int wasCompleted = 0;
int pathMode = sideToSide;
int movementActive = 1;



void receive()
{
    eth.connect();

    //socket.open(&eth);
    
    //socket.set_blocking(false);
    

    if(isCon)
        buffered_pc.printf(RED"Ethernet returned %d \n\r"RESET, isCon);
    else
        buffered_pc.printf(GRN"Ethernet returned %d \n\r"RESET, isCon);
    Thread::wait(100);
    if(isCon !=0) eth.disconnect();
    while(isCon != 0) {
        Thread::yield();
    }
    socket.open(&eth);
    buffered_pc.printf(GRN"Ethernet is connected at %s \n\r"RESET, eth.get_ip_address());
    Thread::wait(100);
    SocketAddress receive;
    //UDPSocket socket(&eth);
    int bind = socket.bind(BROADCAST_PORT_R);
    SocketAddress transmit(hubAddress.c_str(), BROADCAST_PORT_T);
    //printf("bind return: %d", bind);

    char buffer[256];
    
    
    buffered_pc.printf(GRN"starting receive loop\r\n"RESET);
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
                    buffered_pc.printf(BLU"Hard resetting time to  %ds and %dns \n\r"RESET,inmsg.time.seconds,inmsg.time.nSeconds);
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
                LocationOut* mail = mail_box_mocapToProcess.alloc();
                *mail = mocapLocation;
                mail_box_mocapToProcess.put(mail);
                ESKFT.signal_set(0x1);

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
            case pathMsg:{
                PathMsg inmsg;
                memcpy(&inmsg, buffer, n);
                buffered_pc.printf("Got %d \r\n", inmsg.seg.ID);
                PM->addNewPath(inmsg.seg);

            }
                break;

            case generalSettings:
                GeneralSettings inmsg;
                memcpy(&inmsg, buffer, n);
                switch(inmsg.settingType){
                case resetPaths:
                    PM->rebuildReset();
                    retID = -1;
                    needNewSeg = 1;
                    wasCompleted = 0;
                    buffered_pc.printf(BLU"Resetting the Octree + line status \n\r"RESET);
                    break;
                case sideToSideMode:
                    pathMode = sideToSide;
                    break;
                case slowClosestPath:
                    pathMode = closestSlow;
                    break;
                case fastClosestPath:
                    pathMode = octreeMode;
                    break;
                case biasedFastPath:
                    pathMode = octreeModeBiased;
                    break;
                case active:
                    movementActive = inmsg.settingInt;
                    break;
                case travelSpeed:
                    travelSpeedVal = inmsg.setting1;
                default:
                    break;
                }

                break;

            case pingIn:

                buffered_pc.printf(RED"This is an outgoing type !! Type: %d  \n\r"RESET,typeInt );

                break;

            case locationIn:
                buffered_pc.printf(RED"This is an outgoing type !! Type: %d  \n\r"RESET,typeInt );
                break;

            case userIn:
                buffered_pc.printf(RED"This is an outgoing type !! Type: %d  \n\r"RESET,typeInt );
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
    ServoAxis pitch(EXTPIN1,35, -15, 1510, -(1200.0)/120.0, 6.0);
    ServoAxis yaw(EXTPIN2,60, -60, 1445, (1200.0)/120.0, 2.0);
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
    if(error == 0)
        buffered_pc.printf(GRN"there were %d errors in the read/write\r\n"RESET,error);
    else
        buffered_pc.printf(RED"there were %d errors in the read/write\r\n"RESET,error);
    Kinematics kin(&A, &B, &C,&yaw, &pitch, calibration); // the Kinematics class contains everything
    buffered_pc.printf(BLU"setting motors to idle\r\n"RESET);
    kin.goIdle();
    kin.goIdle();
    error = 1;
    while(error > 0){
        error = kin.setSafeParams();
        Thread::wait(100);
        if(error == 0)
            buffered_pc.printf(GRN"there were %d errors in the read/write\r\n"RESET,error);
        else
            buffered_pc.printf(RED"there were %d errors in the read/write\r\n"RESET,error);
        Thread::wait(100);
    }

    pitch.setAngle(0);
    yaw.setAngle(0);
    //    kin.goIdle();//for some reason we need to do it multiple times!
    int even = 1 ;
    float t = 0;
    while(*B.homeSwitch_|| trigger) {
        Thread::wait(10);
        // t+= 0.01;
        //  pitch.setAngle(25.0*sin(t));
        //  yaw.setAngle(25.0*cos(t));

        batteryV = OD1.readBattery();
    } // wait till user
    buffered_pc.printf(GRN"we have recieved this many lines %d\r\n"RESET, PM->pathCount);
    buffered_pc.printf(BLU"finding index\r\n"RESET);
    kin.findIndex();
    buffered_pc.printf(BLU"activating motors\r\n"RESET);
    kin.activateMotors();
    buffered_pc.printf(BLU"homing motors\r\n"RESET);
    kin.homeMotors();
    //error += kin.goToPos(120,0,0);


    kin.goToAngles(0.5,0.5,0.5);
    //    while( trigger) {
    //   Thread::wait(100);

    //   batteryV = OD1.readBattery();
    //   }
    //   kin.goToPos(100,0,0);
    //   Thread::wait(1000);
    //           while( trigger) {
    //   Thread::wait(100);

    //   batteryV = OD1.readBattery();
    //   }
    //   kin.goToPos(150,0,0);
    //   Thread::wait(1000);
    //           while( trigger) {
    //   Thread::wait(100);

    //   batteryV = OD1.readBattery();
    //   }
    //   kin.goToPos(200,0,0);
    //   Thread::wait(1000);
    //          while( trigger) {
    //   Thread::wait(100);

    //   batteryV = OD1.readBattery();
    //   }
    //   kin.goToAngles(M_PI/8,M_PI/8,M_PI/8);
    //   Thread::wait(1000);
    //   while( trigger) {
    //   Thread::wait(100);

    //   batteryV = OD1.readBattery();
    //   }
    //   kin.goToAngles(M_PI/4,M_PI/4,M_PI/4);
    //   Thread::wait(1000);
    // while(1){}
    Thread::wait(500);
    error = 1;
    while(error > 0){
        error = kin.setFastParams();
        Thread::wait(100);
        buffered_pc.printf(RED"there were %d errors in the read/write\r\n"RESET,error);
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
    float envRad = 0.07;
    //   int loopCounter = 0;
    //Eigen::Vector3f current(0,0,0);
    int end = 1;
    //    int needNewSeg = 0;
    //    float findHorizon = 0.8;
    //    int retID = -1;
    //    int wasCompleted = 0;
    //    int pathMode = sideToSide;
    rosTime rTimeNow = timeTracker.getTime();
    lTime lastTime(rTimeNow.seconds,rTimeNow.nSeconds);
    Eigen::Affine3f current =  Eigen::Translation3f(0.385,0,0.03) * Eigen::AngleAxisf(0,Eigen::Vector3f(0,0,1));
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
            //PM->setNotComplete();
            //PM->rebuildReset();

            // current =  Eigen::Translation3f(0.385,0,0.03) * Eigen::AngleAxisf(0,Eigen::Vector3f(0,0,1));
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
                motorCount++;
                if(loopCounter % 1000 == 0){
                    Vector3f pos = eskfPTR->getPos();
                    //buffered_pc.printf(" position x y z %f %f %f\r\n",pos[0],pos[1],pos[2] );
                }

                Eigen::Affine3f here = Eigen::Translation3f(eskfPTR->getPos()) * eskfPTR->getQuat();
                Eigen::Affine3f origin = here* kin.imuToOrigin;
                Eigen::Affine3f centreOffsetInv;
                centreOffsetInv =  Eigen::Translation3f(Eigen::Vector3f(-0.28,0,0));
                
                //centreLocation.translation() = Eigen::Vector3f(0,0,0);
                //Eigen::Affine3f target = Eigen::Translation3f(0,0,1) * Eigen::Quaternionf(1,0,0,0);
                Eigen::Affine3f target = Eigen::Translation3f(targetPos) * targetRot;
                Eigen::Affine3f centreLocation = target.inverse()*here*centreOffsetInv;
                Eigen::Quaternionf targetRotInv = targetRot.inverse();

                // path finding section
                // path finding section
                std::vector<octree::dataPtr> priorityList;


                int end;
                // path finding section
                if(needNewSeg){
                    //std::vector<std::pair<int,int> > IDList = PM->makeSeriesPrediction(current,here,target,findHorizon,envRad,15);
                    if(wasCompleted){
                        wasCompleted = 0;
                        //               retID = PM->getBestHint(retID,end);
                        //               cacheCount++;
                        //               if(retID == -1){
                        //                    cacheCount--;
                        //                    nonCacheCount++;
                        //                   retID = PM->getClosestPath(current,baseLocation,targetLocation,findHorizon*envRad,end);
                        //               }
                    }
                    //else{
                    if(pathMode == closestSlow){
                        retID = PM->getClosestPath(current,here,target,findHorizon*envRad,end);
                    }
                    else if(pathMode == octreeModeBiased){
                        retID = PM->getClosestPathBiased(current,here,target,findHorizon*envRad,end,true,0.5,priorityList);
                    }
                    else if(pathMode == sideToSide){
                        int biasPointNum = 10;
                        std::vector<Eigen::Vector3f> biasPoints(biasPointNum);
                        for(int i = 0; i < biasPoints.size(); i++){
                            Eigen::Vector3f thisOne(0,-1,0.5 - (float)i/ biasPoints.size());
                            //Eigen::Vector3f thisOne(0,0,0);
                            biasPoints[i] = thisOne;
                        }
                        retID = PM->getClosestPathBiasedWithGlobal(current,here,target,biasPoints,findHorizon*envRad,end,false,0, 1.0,priorityList);
                    }
                    else if(pathMode == octreeMode){
                        retID = PM->getClosestPathFast(current,here,target,findHorizon*envRad,end,true,priorityList);
                    }
                    else if(pathMode == inOrder){
                        static int currentPath  = 0;
                        if(PM->isComplete(currentPath)) currentPath++;
                        if(currentPath == PM->pathCount) currentPath = -1;
                        retID  = currentPath;
                    }
                    static int pastID = retID;
                    if(pastID != retID){
                        //               pastID = retID;
                        //               heatMap = PM->getHeatMap(0.07,Eigen::Vector3f(0,0,0),Eigen::Vector3f(0,2,0),Eigen::Vector3f(0,0,1));
                        //               cv::imshow("window",1000*heatMap);
                        //               cv::waitKey(1);
                    }

                    //}
                    if(retID == 174){
                        std::cout << "here" << std::endl;
                    }
                    std::cout << "id" << retID << std::endl;
                    if(!PM->isClose(retID,end,current,target,0.002)){
                        if(end == 1){
                            PM->setupTravel(current,PM->getStartPos(retID),target,PM->getNormal(retID),travelSpeedVal,retID,end);
                        }
                        if(end == 2){
                            PM->setupTravel(current,PM->getEndPos(retID),target,PM->getNormal(retID),travelSpeedVal,retID,end);
                        }
                        end = 1;
                        retID = -2;

                    }
                    else{
                        //should take seg out of octree.
                        if(octreeMode){
                            if(retID >= 0){
                                octree::dataPtr startPoint, endPoint;
                                startPoint.data = retID;
                                startPoint.point = PM->getStartEndPos(retID,1);
                                endPoint.data = retID;
                                endPoint.point = PM->getStartEndPos(retID,2);
                                bool  val = PM->oct->remove(startPoint);
                                if(val == false) std::cout << "remove error" << std::endl;
                                val = PM->oct->remove(endPoint);
                                if(val == false) std::cout << "remove error" << std::endl;
                            }
                        }
                    }
                    needNewSeg = 0;
                }
                //PM->constrainHead(current,here, envRad); //// should I be constraining the head.
                Eigen::Affine3f posTest;

                rosTime rNow = timeTracker.getTime();
                lTime nowTime(rNow.seconds,rNow.nSeconds);
                float dt = (nowTime - lastTime).toSec();
                lastTime = nowTime;
                if(dt > 0.01){
                    buffered_pc.printf(RED"DT:%f \r\n"RESET, dt);
                    dt = 0.01;
                }
                if(dt < 0.001){
                    buffered_pc.printf(RED"DT:%f \r\n"RESET, dt);
                    dt = 0.001;
                }
                int isReach = PM->thisEndReachable(retID,current,target,here, envRad,posTest,end);
                if(!isReach){
                    buffered_pc.printf(RED"cannot reach:%d \r\n"RESET, retID);
                }
                if(isReach && !PM->isComplete(retID)){
                    PM->stepTime(retID,dt,end);
                    current.translation() = PM->getPos(retID,end,target);
                }
                else{
                    if(PM->isComplete(retID)){
                        wasCompleted =1;
                        buffered_pc.printf(GRN"completed on end ID:%d\r\n"RESET, retID);
                    }
                    else{ // here is a problem.
                        if(retID >= 0){
                            if(octreeMode){
                                octree::dataPtr startPoint, endPoint;
                                startPoint.data = retID;
                                startPoint.point = PM->getStartEndPos(retID,1);
                                endPoint.data = retID;
                                endPoint.point = PM->getStartEndPos(retID,2);
                                int  val = PM->oct->insert(startPoint);
                                val = PM->oct->insert(endPoint);
                            }
                        }
                    }
                    retID=-2;
                    needNewSeg = 1;
                    //should put the seg back into the octree.
                }

                // end of path finding section.
                // Eigen::Vector3f up;
                // up << 0,1,0;
                // Eigen::Vector3f side ;
                // side << 0,0,1;
                //                 rosTime rTime = timeTracker.getTime();
                // lTime lt(rTime.seconds,rTime.nSeconds);
                // //up = targetRot._transformVector(up);
                // up = 0.05*cos(2.0*lt.toSec())*up;
                // //side = targetRot._transformVector(side);
                // side = 0.05*sin(2.0*lt.toSec())*side;
                // up = up + side;
                //up = targetRot._transformVector(up);
                //current = targetRot._transformVector(current);
                //side = targetRot.inverse()._transformVector(side);

                target =  posTest;
                if(loopCounter%1000 == 0){
                    // buffered_pc.printf("here %f,%f,%f   target %f,%f,%f \r\n", here.translation()[0],here.translation()[1],here.translation()[2],target.translation()[0],target.translation()[1],target.translation()[2]);
                }
                if(loopCounter%50 == 0  || PM->isComplete(retID)){

                    if(retID >=0){
                        segment currentSeg = PM->hashLookupSegment(retID);
                        PathUpdate *mail = mail_box_PathUpdate.alloc();
                        mail->type = pathUpdate;
                        mail->ID = currentSeg.ID;
                        mail->complete = currentSeg.complete;
                        mail->fromEndT = currentSeg.fromEndT;
                        mail->fromStartT = currentSeg.fromStartT;
                        mail_box_PathUpdate.put(mail);
                        transmitterT.signal_set(0x1);
                    }
                }
                Eigen::Vector3f angRates = eskfPTR->lastImu_.gyro;
                float ffGain = 0.025;
                //buffered_pc.printf(" here x y z %f %f %f\r\n",eskfPTR->getPos()[0],eskfPTR->getPos()[1],eskfPTR->getPos()[2] );
                //buffered_pc.printf(" hereQ w x y z %f %f %f %f\r\n",eskfPTR->getQuat().coeffs()[0],eskfPTR->getQuat().coeffs()[1],eskfPTR->getQuat().coeffs()[2],eskfPTR->getQuat().coeffs()[2]  );
                float yawOut = 0;
                float pitchOut = 0;
                Eigen::Vector3f deltaPos;
                Eigen::Affine3f outPos2;
                int notReachable = 1;
                if(movementActive) notReachable = kin.goToWorldPos(here,target, angRates, posTest,outPos2, ffGain, deltaPos, yawOut, pitchOut);
                current = outPos2;

                if(loopCounter%50 == 0){
                    KinematicsInfo *mail = mail_box_kinematics_info.alloc();
                    mail->yaw = yawOut;
                    mail->pitch = pitchOut;
                    mail->deltaOffset.x = deltaPos[0];
                    mail->deltaOffset.y = deltaPos[1];
                    mail->deltaOffset.z = deltaPos[2];
                    mail->time = rNow;
                    mail->type = kinematicsInfo;

                    mail_box_kinematics_info.put(mail);


                    LocationIn *mail2 = mail_box_headPosOut.alloc();
                    Eigen::Quaternionf outPos2Quat(outPos2.rotation());
                    Eigen::Vector3f outPos2Pos = outPos2.translation();
                    mail2->quat.x = outPos2Quat.x();
                    mail2->quat.y = outPos2Quat.y();
                    mail2->quat.z = outPos2Quat.z();
                    mail2->quat.w = outPos2Quat.w();
                    mail2->pos.x = outPos2Pos[0];
                    mail2->pos.y = outPos2Pos[1];
                    mail2->pos.z = outPos2Pos[2];
                    mail2->time = rNow;
                    mail2->type = locationIn;

                    mail_box_headPosOut.put(mail2);
                    transmitterT.signal_set(0x1);
                }
                //     LocationIn retMsg;
                //     LocationIn *mail = mail_box_locationIn.alloc();
                //     mail->pos.x = posTest.translation()[0];
                //     mail->pos.y = posTest.translation()[1];
                //     mail->pos.z = posTest.translation()[2];
                //     Eigen::Quaternionf quatLocationReturn;
                //     quatLocationReturn= posTest.rotation();
                //     mail->quat.x = quatLocationReturn.x();
                //     mail->quat.y = quatLocationReturn.y();
                //     mail->quat.z = quatLocationReturn.z();
                //     mail->quat.w = quatLocationReturn.w();
                //     mail->time = timeTracker.getTime();
                //     mail->type = locationIn;

                //     mail_box_locationIn.put(mail);

                // transmitterT.signal_set(0x1);

                //newLocationReturnMsg = 1;
                if (notReachable || retID == -2 || retID == -1) UVLed = 0;
                else UVLed =1;
            }
        }
        Thread::signal_wait(0x1);
        Thread::yield();
        if(loopCounter % 200000 == 0) {
            if(PM->pathCount > 0)
                buffered_pc.printf(GRN"we have recieved this many lines %d\r\n"RESET, PM->pathCount);
            else
                buffered_pc.printf(RED"we have recieved this many lines %d\r\n"RESET, PM->pathCount);
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
    buffered_pc.printf(BLU"I AM 0x%x\n\r"RESET, whoami);
    buffered_pc.printf(BLU"I SHOULD BE 0x68\n\r"RESET);
    mpu.resetMPU6050(); // Reset registers to default in preparation for device calibration
    //mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    mpu.initMPU6050();
    buffered_pc.printf(BLU"MPU6050 initialized for active data mode....\n\r"RESET); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    intPin.rise(&accelInterrupt);
    Timer t;
    t.start();
    float sum = 0;
    uint32_t sumCount = 0;
    int dataCount = -1 ;

    ImuData dataOut;

    SocketAddress transmit(hubAddress.c_str(), BROADCAST_PORT_T);
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
            if(!mail_box.full()){
                imuMail *mail = mail_box.alloc();
                mail->accel <<  (float)accelCount[0]*aRes,(float)accelCount[1]*aRes,(float)accelCount[2]*aRes;

                mail->gyro <<(float)gyroCount[0]*gRes,(float)gyroCount[1]*gRes,(float)gyroCount[2]*gRes;

                mail->stamp = timeMesLocal;
                mail_box.put(mail);
                ESKFT.signal_set(0x1);
            }
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
        imuCount++;
        accelPending = 1;

    }
}

void batteryThread()
{
    while(1) {
        Thread::wait(5000);
        if(batteryV > 19)
            buffered_pc.printf(GRN"batteryVoltage = %f\n\r"RESET, batteryV);
        else
            buffered_pc.printf(RED"batteryVoltage = %f\n\r"RESET, batteryV);
    }


}

int lastMocapT = 0;
int lastIMUT = 0;
void ESKFThread(){

    //set up the ESKF as per the desktop example.
    float sigma_accel = 0.0124; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 0.00276; // [rad/s] (value derived from Noise Spectral Density in datasheet)
    float sigma_accel_drift = 0.001f*sigma_accel; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 0.001f*sigma_gyro; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.01; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 1000.0*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 1000.0*sigma_gyro_drift; // [rad/s]

    float sigma_mocap_pos = 0.0003; // [m]
    float sigma_mocap_rot = 0.003; // [rad]
    eskfPTR = new ESKF(
                Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
                ESKF::makeState(
                    Vector3f(0, 0, 1), // init pos
                    Vector3f(0, 0, 0), // init vel
                    Quaternionf(AngleAxisf(0.0f, Vector3f(0, 0, 1))), // init quaternion
                    Vector3f(-1.26, -1.09, -1.977), // init accel bias
                    Vector3f(0.114, -0.01, 0) // init gyro bias
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

    AngleAxisf xRot(0*degToRad,Eigen::Vector3f(1,0,0));
    AngleAxisf zRot(0*degToRad,Eigen::Vector3f(0,0,1));
    AngleAxisf yRot(-1.5*degToRad,Eigen::Vector3f(0,0,1));
    //rotation mat for the accel, to correct for mounting, result should be ros convention.
    Eigen::Matrix<float, 3,3> rotMat;
    rotMat <<   -1, 0, 0
            ,0, 0,-1
            ,0,-1, 0;
    rotMat = rotMat;//          *xRot*zRot*yRot;


    while(1){
        while(!mail_box_mocapToProcess.empty()){

            osEvent evt = mail_box_mocapToProcess.get();
            if(evt.status == osEventMail){
                rosTime nowRos = timeTracker.getTime();
                LocationOut *mailIn = (LocationOut*)evt.value.p;
                locationReturnMsg.type = locationIn;
                locationReturnMsg.pos = mailIn->pos;
                locationReturnMsg.quat = mailIn->quat;
                locationReturnMsg.refTime = mailIn->time;
                locationReturnMsg.time = nowRos;


                Quaternionf quat(mailIn->quat.w,mailIn->quat.x,mailIn->quat.y,mailIn->quat.z);
                Vector3f pos(mailIn->pos.x,mailIn->pos.y,mailIn->pos.z);
                lTime stamp(mailIn->time.seconds,mailIn->time.nSeconds);

                lTime now(nowRos.seconds,nowRos.nSeconds);
                float confidence  = mailIn->confidence;
                if(!(fabs(quat.x()) < 0.00001 || fabs(quat.y()) < 0.00001 ||fabs(quat.z()) < 0.00001 || stamp.sec == 0 || stamp.nsec == 0)){
                    //                    if(!mail_box_mocapOut.full()){
                    //                        LocationIn* mail = mail_box_mocapOut.alloc();
                    //                        *mail = locationReturnMsg;
                    //                        mail_box_mocapOut.put(mail);
                    //                    }

                    eskfPTR->measurePos(pos,confidence * SQ(sigma_mocap_pos)*I_3,stamp,now);
                    Thread::yield();
                    eskfPTR->measureQuat(quat,confidence *SQ(sigma_mocap_rot)*I_3,stamp,now);
                    Thread::yield();

                    mocapCount ++;
                    updateCount++;
                    //transmitterT.signal_set(0x1);
                }
                mail_box_mocapToProcess.free(mailIn);

                Thread::yield();
            }
        }

        

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
                estimateCount++;

                updatedESKF = 1;
                odriveThread.signal_set(0x1);
                imuCount ++;
                if(imuCount %50 == 0){
                    if(!mail_box_transmit_imu.full()){
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




void performance(){
    while(1){
        rosTime rT = timeTracker.getTime();
        lTime nowT(rT.seconds,rT.nSeconds);
        lTime dT = nowT - lastPerformanceT;
        lastPerformanceT = nowT;
        if(batteryV < 19){
            buffered_pc.printf(RED"performance: Accel:%fHz\t Mocap: %fHz\t  Update %fHz\t  Motors%fHz\t  Battery %fV \r\n"RESET,imuCount/dT.toSec(),updateCount/dT.toSec(), estimateCount/dT.toSec(), motorCount/dT.toSec(),batteryV );
        }
        //buffered_pc.printf("performance: Accel:%fHz\t Mocap: %fHz\t  Update %fHz\t  Motors%fHz\t  Battery %fV \r\n",imuCount/dT.toSec(),updateCount/dT.toSec(), estimateCount/dT.toSec(), motorCount/dT.toSec(),batteryV );
        imuCount = 0;
        updateCount = 0;
        estimateCount = 0;
        motorCount = 0;
        Thread::wait(10000);
    }
}




int main()
{
    debugTimer.start();
    // testing eigen run time.
    buffered_pc.baud(115200);
    buffered_pc.printf(GRN"hello\r\n"RESET);
    UVLed = 1;


    calibration.e = 58.095;
    calibration.f = 2*60.622;
    calibration.re = 135;
    calibration.rf = 133.0;
    calibration.Aoffset = 0.105;
    calibration.Boffset = 0.105;
    calibration.Coffset = 0.105;
    calibration.gearRatio = 89.0/24.0;
    
    i2c.frequency(1000000);

    Eigen::Vector3f centre(0,0,0);
    float range = 1.0;
    octree::node* nodeBuffer = new octree::node[(int)(1000)];
    octree myOct(32,2,centre,range,nodeBuffer,(int)(1000));
    PM = new PathManager(&myOct);
    

    //set switches up
    homeGND = false;
    extGND = false;
    homeSwitchA.mode(PullUp);
    homeSwitchB.mode(PullUp);
    homeSwitchC.mode(PullUp);
    trigger.mode(PullUp);

    odriveThread.start(runOdrive);

    //buffered_pc.printf("Controller IP Address is %s\r\n", eth.get_ip_address());
    Thread::wait(1000);

    
    receiverT.start(receive);
    Thread::wait(7000);
    ESKFT.start(ESKFThread);
    transmitterT.start(transmit);
    UVLed = 0;
    
    accelT.start(accelThread);
    // printBattery.start(batteryThread);
    performanceThread.start(performance);

    while (true) {
        //led1 = !led1;
        Thread::wait(500);
    }
}
