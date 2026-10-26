#ifndef PPG_MANAGER_H
#define PPG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

bool PPGManager_begin(uint8_t interruptPin);
void PPGManager_process();
bool PPGManager_popPacket(char* outBuf, size_t bufSize, size_t* outLen);
void PPGManager_discardPacket();
uint8_t PPGManager_getBPM();
uint8_t PPGManager_getSpO2();
void PPGManager_handleInterrupt();
void PPGManager_wakeUp(uint8_t interruptPin);
void PPGManager_shutDown();
bool PPGManager_popNoFingerEvent();
void PPGManager_setupPhase1();
void PPGManager_setupPhase2(bool lowPower);
long PPGManager_readIR();


#endif
