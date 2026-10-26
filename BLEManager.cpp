#include "BLEManager.h"
#include <bluefruit.h>

// Service & Characteristic UUIDs
BLEService        healthService = BLEService(BLEUuid("4fafc201-1fb5-459e-8fcc-c5c9c331914b"));
BLECharacteristic ppgChar       = BLECharacteristic(BLEUuid("beb5483e-36e1-4688-b7f5-ea07361b26a8"));
BLECharacteristic writeChar     = BLECharacteristic(BLEUuid("beb5483e-36e1-4688-b7f5-ea07361b26a9"));
BLECharacteristic reportChar    = BLECharacteristic(BLEUuid("beb5483e-36e1-4688-b7f5-ea07361b26aa"));

// Standard Battery Service (0x180F)
BLEDis  bledis;
BLEBas  blebas;

static bool deviceConnected = false;
static void (*commandCallback)(const char* cmd) = nullptr;

uint8_t BLEManager_readBatteryLevel() {
  // 1. Bật cầu phân áp để đọc (VBAT_ENABLE active-LOW)
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  delay(1);  // Chờ điện áp ổn định

  // 2. Đọc ADC 12-bit (0–4095) — nắm52840 mặc định 12-bit
  analogReadResolution(12);
  uint32_t rawADC = analogRead(PIN_VBAT);

  // 3. Khóa VBAT_ENABLE ở mức LOW (tránh quá áp SAADC theo LESSONS_LEARNED.md)
  digitalWrite(VBAT_ENABLE, LOW);

  // Vref của nRF52840 (Adafruit/Seeed core) = 3.6V, ADC 12-bit = 4096 bước
  // V_pin (mV) = rawADC * 3600 / 4096
  uint32_t pinMilliVolts = (rawADC * 3600) / 4096;

  // Mạch XIAO dùng cầu phân áp 1MΩ + 510kΩ
  // V_bat = V_pin * (1000 + 510) / 510 = V_pin * 1510 / 510
  uint32_t batteryMilliVolts = (pinMilliVolts * 1510) / 510;

  Serial.print("[BATTERY XIAO] raw=");
  Serial.print(rawADC);
  Serial.print(" | Dien ap pin: ");
  Serial.print(batteryMilliVolts);
  Serial.println(" mV");

  if (batteryMilliVolts >= 4200) return 100;
  if (batteryMilliVolts <= 3300) return 0;
  return (uint8_t)(((batteryMilliVolts - 3300) * 100) / (4200 - 3300));
}

void BLEManager_updateBatteryLevel() {
  uint8_t level = BLEManager_readBatteryLevel();
  blebas.write(level);
  if (deviceConnected) {
    blebas.notify(level);
  }
}

static void connect_callback(uint16_t conn_handle) {
  (void) conn_handle;
  deviceConnected = true;
  BLEManager_updateBatteryLevel();
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;
  deviceConnected = false;
  BLEManager_startAdvertising();
}

static char pendingBleCommand[64] = {0};
static volatile bool hasPendingBleCommand = false;

static void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void) conn_hdl;
  (void) chr;
  if (len == 0) return;

  char buf[64];
  uint16_t copyLen = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';

  char* start = buf;
  while (*start == ' ' || *start == '\r' || *start == '\n') start++;
  char* end = start + strlen(start);
  while (end > start && (*(end - 1) == ' ' || *(end - 1) == '\r' || *(end - 1) == '\n')) {
    end--;
  }
  *end = '\0';

  if (strlen(start) > 0) {
    noInterrupts();
    strncpy(pendingBleCommand, start, sizeof(pendingBleCommand) - 1);
    pendingBleCommand[sizeof(pendingBleCommand) - 1] = '\0';
    hasPendingBleCommand = true;
    interrupts();
  }
}

bool BLEManager_popCommand(char* outBuf, size_t maxLen) {
  if (!hasPendingBleCommand || outBuf == nullptr || maxLen == 0) return false;
  noInterrupts();
  hasPendingBleCommand = false;
  strncpy(outBuf, pendingBleCommand, maxLen - 1);
  outBuf[maxLen - 1] = '\0';
  interrupts();
  return true;
}

void BLEManager_begin() {
  // VBAT_ENABLE được quản lý trong BLEManager_readBatteryLevel() (bật lúc cần, tắt ngay sau đọc)
  // Cấu hình băng thông và MTU tối đa (247 bytes trên SoftDevice S140)
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(4); // Max power +4dBm

  // Lấy tên thiết bị từ MAC nRF52840 (đồng bộ tiền tố HuyWatch_ với Mobile App)
  uint8_t mac[6];
  Bluefruit.getAddr(mac);
  static char deviceName[20];
  sprintf(deviceName, "HuyWatch_%02X%02X", mac[1], mac[0]);
  Bluefruit.setName(deviceName);

  // Set Callbacks kết nối
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Device Info Service
  bledis.setManufacturer("HealthSense-IUH");
  bledis.setModel("HealthSense");
  bledis.begin();

  // Battery Service (0x180F)
  blebas.begin();
  blebas.write(100); // Đặt mặc định 100%, đọc ADC sau trong loop

  // Primary Health Service (0x4FAF...)
  healthService.begin();

  // PPG Raw Data Notify Characteristic (512 bytes buffer)
  ppgChar.setProperties(CHR_PROPS_NOTIFY);
  ppgChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ppgChar.setMaxLen(512);
  ppgChar.begin();

  // Write Characteristic
  writeChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  writeChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  writeChar.setMaxLen(64);
  writeChar.setWriteCallback(write_callback);
  writeChar.begin();

  // Report Notify Characteristic
  reportChar.setProperties(CHR_PROPS_NOTIFY);
  reportChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  reportChar.setMaxLen(64);
  reportChar.begin();

  BLEManager_startAdvertising();
}

bool BLEManager_isConnected() {
  return deviceConnected;
}

void BLEManager_notify(const char* data, size_t len) {
  if (deviceConnected) {
    ppgChar.notify(data, len);
  }
}

void BLEManager_notifyReport(const char* data, size_t len) {
  if (deviceConnected) {
    reportChar.notify(data, len);
  }
}

void BLEManager_notifyEmergency(const char* data, size_t len) {
  if (deviceConnected) {
    ppgChar.notify(data, len);
  }
}

void BLEManager_notifyEmergencyStatus(const char* data, size_t len) {
  if (deviceConnected) {
    reportChar.notify(data, len);
  }
}

void sendBLECommand(const char* command) {
  BLEManager_notify(command, strlen(command));
}

void BLEManager_startAdvertising() {
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(healthService);
  Bluefruit.Advertising.addService(blebas);
  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void BLEManager_stopAdvertising() {
  Bluefruit.Advertising.stop();
}

void BLEManager_setCommandCallback(void (*callback)(const char* cmd)) {
  commandCallback = callback;
}
