#ifndef ACCEL_MANAGER_H
#define ACCEL_MANAGER_H

#include <Adafruit_Sensor.h>

bool AccelManager_begin(uint8_t intPin);
bool AccelManager_isReady();
bool AccelManager_isMoving();
bool AccelManager_popMotionEvent();
bool AccelManager_popFallEvent();
void AccelManager_process();
void AccelManager_printDebug();
void AccelManager_setMotionThreshold(uint8_t threshold);
void AccelManager_deepSleep();
void AccelManager_wakeUp();
void AccelManager_startFallDetection();
void AccelManager_stopFallDetection();
bool AccelManager_readFreeFallInterrupt();
void AccelManager_clearInterruptLatch();

#endif
