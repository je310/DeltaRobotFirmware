#include "syncTime.h"
extern BufferedSerial buffered_pc;
SyncTime::SyncTime( int seconds, int nSeconds ){
    averageOffset = 0;
    clockMultiplier = 0;
    averageOffsetCount = 0;
    refTime.seconds = seconds;
    refTime.nSeconds = nSeconds;
    sinceRefTimer.reset();
    sinceRefTimer.start();
    
}
    const int million = 1000000;
    const int billion = 1000000000;
    
rosTime SyncTime::getTime(){
    rosTime ret;
    uint64_t uSecs =  sinceRefTimer.read_high_resolution_us();
    //uSecs = uSecs * (float)(1.0000136);
    ret.seconds = (int)(uSecs/million);
    ret.nSeconds = (int)((uSecs% million)*1000);
    ret.seconds += refTime.seconds;
    ret.nSeconds += refTime.nSeconds;
        if(ret.nSeconds >= (int)1e9){
        ret.seconds ++;
        ret.nSeconds -= (int)1e9;
    }
    if(ret.nSeconds < 0){
        ret.seconds --;
        ret.nSeconds += (int)1e9;
        }
    return ret;
}

void SyncTime::setMultipler(float mult){
    clockMultiplier = mult;
}

void SyncTime::resetOffsetCounter(){
    rosTime update = getTime();
    refTime = update;
    sinceRefTimer.reset();   
    //buffered_pc.printf("resetting counter %dus\r\n", sinceRefTimer.read_high_resolution_us());
    
}
float SyncTime::difference(rosTime first, rosTime later){
    int secs = later.seconds - first.seconds;
    int nSecs = later.nSeconds - first.nSeconds;
    return (float)secs + (float)nSecs/billion;
}

void SyncTime::hardReset(int seconds, int nSeconds){
        refTime.seconds = seconds;
    refTime.nSeconds = nSeconds;
    sinceRefTimer.reset();
    sinceRefTimer.start();
    }
    

void SyncTime::updateTime(float correction){
    int seconds = (int)correction;
    int nSeconds = (correction - (float)seconds) * 1e9;
    // averageOffsetCount ++;
    // averageOffset = 0.9*averageOffset + 0.1*correction;
    // if(averageOffsetCount > 10){
    //     averageOffset = 0.99*averageOffset + 0.01*correction;
    //     setMultipler(averageOffset);
    //     buffered_pc.printf("setting mult to %f", averageOffset);
    // }
    // if(averageOffsetCount > 1 && averageOffsetCount < 10) averageOffset = 0.9*averageOffset + 0.1*correction;
    // if(averageOffsetCount == 1) averageOffset = correction;
//    buffered_pc.printf("time before %ds %dns \n\r", refTime.seconds, refTime.nSeconds);
//    buffered_pc.printf("updating by  %ds %dns \n\r", seconds, nSeconds);
    refTime.seconds += seconds;
    refTime.nSeconds += nSeconds;
    if(refTime.nSeconds >= billion){
        refTime.seconds ++;
        refTime.nSeconds -= billion;
    }
    if(refTime.nSeconds < 0){
        refTime.seconds --;
        refTime.nSeconds += billion;
        }
        //buffered_pc.printf("time after %ds %dns \n\r", refTime.seconds, refTime.nSeconds);
        resetOffsetCounter();
}