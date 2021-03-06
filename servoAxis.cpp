#include "servoAxis.h" 

extern BufferedSerial buffered_pc;
ServoAxis::ServoAxis(PinName pin,float maxAngle_, float minAngle_, float middleUs_, float usPerDeg_, float degOffset_){
    maxAngle = maxAngle_;
    minAngle = minAngle_;
    middleUs = middleUs_;
    usPerDeg = usPerDeg_;
    degOffset = degOffset_;
    pwm = new FastPWM(pin);
    pwm->period_ms(2);
    pwm->pulsewidth_us(middleUs);
    
}
    
    
int ServoAxis::setAngle(float angle){
    int error = 0;
    angle += degOffset;
    if(angle > maxAngle){
        angle = maxAngle;
        error +=1;
    } 
    if(angle < minAngle){
       angle = minAngle; 
       error +=1;
    } 
    pwm->pulsewidth_us(middleUs + angle * usPerDeg);
    return error;
    
}

int ServoAxis::anglePermissable(float angle){
    if(angle < maxAngle && angle > minAngle) return 1;
    else return 0;
}