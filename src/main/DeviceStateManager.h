#pragma once

#include <Arduino.h>
#include "BLEManager.h"
#include "PPGManager.h"
#include "AccelManager.h"
#include "DisplayPower.h"

enum DeviceMode{
    MODE_IDLE,
    MODE_MEASURE,
    MODE_WORKOUT,
    MODE_SCREENING,
    MODE_EMERGENCY,
    MODE_SHUTDOWN,
    MODE_TURN_ON
};

enum DeviceEvent{
    EVT_BUTTON_SHORT,
    EVT_BUTTON_LONG,
    EVT_MOTION,
    EVT_NOT_WEARING,
    EVT_FALL_DETECTED,
    EVT_BLE_CANCEL_SOS,
    EVT_BLE_START_MEASURE,
    EVT_BLE_START_WORKOUT,
    EVT_BLE_START_SCREENING,
    EVT_BLE_TEST_SOS
};

void DeviceStateManager_begin(uint8_t buttonPin, uint8_t accelVddPin, uint8_t vibratorPin);
void DeviceStateManager_handleButton();
void DeviceStateManager_handleBLECommand(const char* cmd);
void DeviceStateManager_onEvent(DeviceEvent event);
void DeviceStateManager_loop();
DeviceMode DeviceStateManager_getMode();
uint8_t DeviceStateManager_getScreeningPhase();
