#ifndef SYNC_TIME_H
#define SYNC_TIME_H

#include "comms.h"
#include "mbed.h"
#include "BufferedSerial.h"

class SyncTime{
   public:
   SyncTime( int seconds, int nSeconds);
   rosTime getTime();
   void updateTime(float correction);
   void hardReset(int seconds, int nSeconds);
   float difference(rosTime first, rosTime later);
   void setMultipler(float mult);
   
   private:
   void resetOffsetCounter();
   rosTime refTime;
   Timer sinceRefTimer;
   float clockMultiplier;
   float averageOffset;
   int averageOffsetCount;
    
    
    };



#endif