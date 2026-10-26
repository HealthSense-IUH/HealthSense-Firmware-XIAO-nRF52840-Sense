#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stddef.h>
#include <Arduino.h>

void BLEManager_begin();
bool BLEManager_isConnected();
void BLEManager_notify(const char* data, size_t len);
void BLEManager_notifyReport(const char* data, size_t len);
void sendBLECommand(const char* command);
void BLEManager_notifyEmergency(const char* data, size_t len);
void BLEManager_notifyEmergencyStatus(const char* data, size_t len);
void BLEManager_startAdvertising();
void BLEManager_stopAdvertising();

// Hàng đợi nhận lệnh từ điện thoại qua BLE Write (an toàn, không block BLE task)
bool BLEManager_popCommand(char* outBuf, size_t maxLen);

// Đăng ký hàm xử lý lệnh nhận từ điện thoại qua BLE Write (legacy)
void BLEManager_setCommandCallback(void (*callback)(const char* cmd));

// Đọc và cập nhật thời lượng pin (Battery Service 0x180F)
uint8_t BLEManager_readBatteryLevel();
void BLEManager_updateBatteryLevel();

#endif
