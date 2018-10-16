#include "servoAxis.h" 

ServoAxis::ServoAxis(PinName pin,float maxAngle_, float minAngle_, float middleUs_, float usPerDeg_){
    maxAngle = maxAngle_;
    minAngle = minAngle_;
    middleUs = middleUs_;
    usPerDeg = usPerDeg_;
    pwm = new PwmOut(pin);
    pwm->period_ms(4);
    pwm->pulsewidth_us(middleUs);
    
    }
    
    
    void ServoAxis::setAngle(float angle){
        if(angle > maxAngle) angle = maxAngle;
        if(angle < minAngle) angle = minAngle;
        pwm->pulsewidth_us(middleUs + angle * usPerDeg);
        
        }