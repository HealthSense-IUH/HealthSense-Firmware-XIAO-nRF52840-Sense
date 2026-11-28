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

// Phát hiện cử động tay khi đang đo sàng lọc AFib (Pha 1) để dừng đo tránh nhiễu PPG
void AccelManager_startScreeningMotion();
void AccelManager_stopScreeningMotion();

// Đếm bước chân trong chế độ tập luyện (MODE_WORKOUT)
void AccelManager_startStepCounting();
void AccelManager_setStepCountingPaused(bool paused);
void AccelManager_stopStepCounting();
uint32_t AccelManager_getStepCount();

#endif
