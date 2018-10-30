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
    
    
    void ServoAxis::setAngle(float angle){
        angle += degOffset;
        if(angle > maxAngle) angle = maxAngle;
        if(angle < minAngle) angle = minAngle;
        pwm->pulsewidth_us(middleUs + angle * usPerDeg);
        
        }