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



using std::string;
const int BROADCAST_PORT_T = 58080;
const int BROADCAST_PORT_R = 58081;
EthernetInterface eth;

SyncTime timeTracker(0,0);
float batteryV = -1;
UDPSocket socket;
int isCon = 0;
volatile int imuToSend = 0;
ImuData imuDataBuffer;
void transmit()
{
    while(isCon != 0) {
        Thread::yield();
    }
    string out_buffer = "very important data";
    SocketAddress transmit("10.0.0.160", BROADCAST_PORT_T);
    // fromRobot msg;
    // msg.motor1.busVoltage = 69;
    // buffered_pc.printf("starting send loop");
    while (true) {
      //  msg.motor1.busVoltage += 1;
        //int ret = socket.sendto(transmit, &msg, sizeof(msg));
        //printf("sendto return: %d\n", ret);
        if(imuToSend == 1){
            imuToSend = 0;
            int ret = socket.sendto(transmit, &imuDataBuffer, sizeof(imuDataBuffer));
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
        //debugTimer.start();
                int32_t typeInt = (int32_t)buffer[0];
        switch(typeInt){
        case angleSpaceCmd:

            break;

        case cartSpaceCmd:

            break;

        case pingOut:{
                PingOut inmsg;
                memcpy(&inmsg, buffer, n);

                now = timeTracker.getTime();
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
                //debugTimer.stop();
                
                //buffered_pc.printf("sectionTime %d \r\n", debugTimer.read_high_resolution_us());
                
                //buffered_pc.printf("current time is %ds and %dns \r\n", now.seconds, now.nSeconds);
            
                if(inmsg.timeOffset !=0.0f){
                    //buffered_pc.printf("Applying offset %fs \n\r",inmsg.timeOffset); 
                    timeTracker.updateTime(inmsg.timeOffset);            
                }
            }
            break;

        case locationOut:

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

void runOdrive()
{
    //start servos on endEffector
    ServoAxis pitch(EXTPIN1,30, -30, 1500, 1200.0/120.0);
    ServoAxis yaw(EXTPIN2,30, -30, 1500, 1200.0/120.0);
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
    Kinematics kin(&A, &B, &C, calibration); // the Kinematics class contains everything
    buffered_pc.printf("setting motors to idle\r\n");
    kin.goIdle();
    kin.goIdle();
    error += kin.setSafeParams();
    while(error) {}
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
    //int error = kin.goToPos(0,0,-10);

    kin.goToAngles(pi/4,pi/4,pi/4);
    Thread::wait(500);
    error += kin.setFastParams();
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
            i += inc;
            if(up == 1)k+=0.2;
            else k-=0.2;
            if(k >= -min) up = 0;
            if(k <= -max) up = 1;
            radius =0.6* (span - abs(abs(k) - mid));
            //buffered_pc.printf("x,y,z = %f %f %f",radius*sin(i),radius*cos(i),k);
            int error = kin.goToPos(40.0*sin(i),40.0*cos(i),k);
            pitch.setAngle(25.0*sin(i));
            yaw.setAngle(25.0*cos(i));
            //int error = kin.goToPos(0,0,k);
            if(i > 2*pi) i = 0;

//            buffered_pc.printf("bus voltage %f\r\n",OD1.readBattery());
//            buffered_pc.printf("k= %f\r\n",k);
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
            static rosTime prevTime;
            accelDeltaT = timeTracker.difference(prevTime,timeMes);
            prevTime = timeMes;

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
    int dataCount = 0 ;
    while(1) {
        //Thread::wait(200);
        if(newData){
            newData = 0;
            dataCount ++;
            ImuData dataOut; 
            
                        static rosTime oldTime{0,0};
            mpu.readAccelData(accelCount);  // Read the x/y/z adc values
            mpu.getAres();

            // Now we'll calculate the accleration value into actual g's
            imuDataBuffer.accel.x  = (float)accelCount[0]*aRes;  // get actual g value, this depends on scale being set
            imuDataBuffer.accel.y = (float)accelCount[1]*aRes;
            imuDataBuffer.accel.z = (float)accelCount[2]*aRes;

            mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu.getGres();

            // Calculate the gyro value into actual degrees per second
            imuDataBuffer.gyro.x = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            imuDataBuffer.gyro.y = (float)gyroCount[1]*gRes; // - gyroBias[1];
            imuDataBuffer.gyro.z = (float)gyroCount[2]*gRes; // - gyroBias[2];
            imuDataBuffer.type = imuData;
            imuDataBuffer.time.seconds = timeMes.seconds;
            imuDataBuffer.time.nSeconds = timeMes.nSeconds;
            imuToSend = 1;
            
            if(dataCount % 2000 ==0){
               // buffered_pc.printf("Ax:%f \t Ay:%f\t Az:%f\t Gx:%f\t Gy:%f\t Gz:%f\t at time %d\ts %d\tns %fhz\r\n",ax,ay,az,gx,gy,gz,timeMes.seconds,timeMes.nSeconds, 1.0/accelDeltaT);
            }
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
    calibration.rf = 143.0;
    calibration.Aoffset = 0.43 - 0.111701;
    calibration.Boffset = 0.43 - 0.111701;
    calibration.Coffset = 0.43 - 0.111701;
    calibration.gearRatio = 89.0/24.0;
    
    i2c.frequency(400000);
    Thread transmitter;
    Thread receiver;
    Thread odriveThread;
    Thread accel;
    Thread timeUpdate;
    Thread printBattery;
    Thread ESKFT(osPriorityNormal, 32 * 1024) /* 8K stack */;
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
    //odriveThread.start(runOdrive);
    accel.start(accelThread);
    printBattery.start(batteryThread);

    while (true) {
        //led1 = !led1;
        Thread::wait(500);
    }
}
