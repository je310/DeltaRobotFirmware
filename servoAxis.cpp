#include "servoAxis.h" 

extern BufferedSerial buffered_pc;
ServoAxis::ServoAxis(PinName pin,float maxAngle_, float minAngle_, float middleUs_, float usPerDeg_, float degOffset_){
    maxAngle = maxAngle_;
    minAngle = minAngle_;
    middleUs = middleUs_;
    usPerDeg = usPerDeg_;
    degOffset = degOffset_;
    pwm = new PwmOut(pin);
    pwm->period_ms(4);
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