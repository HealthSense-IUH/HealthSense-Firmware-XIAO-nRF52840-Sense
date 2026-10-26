#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Wire.h>
#include "BLEManager.h"
#include "PPGManager.h"
#include "AccelManager.h"
#include "DisplayPower.h"
#include "DeviceStateManager.h"

// >>> CẤU HÌNH PINOUT CHO XIAO nRF52840 SENSE <<<
#define UBLOX_MOSFET_PIN     D0 // Chân điều khiển mosfet cho mạch định vị gps u-blox
#define BUCK_MOSFET_PIN      D1 // Chân điều khiển mosfet cho mạch tăng áp 3.3V lên 5V
#define BUTTON_PIN           D2 // Nút bấm vật lý
#define MAX30101_INT_PIN     D3 // Ngắt MAX30102/30101
#define VIBRATOR_MOSFET_PIN  D8 // Chân điều khiển mosfet cho mô tơ rung

// Khai báo chân riêng của linh kiện tích hợp (Onboard)
#define LSM6DS3_INT_PIN      PIN_LSM6DS3TR_C_INT1 
#define IMU_POWER_PIN        PIN_LSM6DS3TR_C_POWER

void setup() {
  // 1. KHỞI TẠO CỔNG USB CDC SERIAL NGAY LẬP TỨC
  Serial.begin(115200);

  // --- CẤU HÌNH BẢO VỆ NGUỒN NGAY LẬP TỨC ---
  // Phải cấu hình MOSFET ngay đầu setup để tránh rò rỉ điện lúc khởi động
  pinMode(UBLOX_MOSFET_PIN, OUTPUT);
  digitalWrite(UBLOX_MOSFET_PIN, HIGH); // TẮT GPS (P-MOSFET: HIGH là tắt)

  pinMode(BUCK_MOSFET_PIN, OUTPUT);
  digitalWrite(BUCK_MOSFET_PIN, HIGH);  // BẬT mạch tăng áp 5V (TPS61023: EN pin active-HIGH → HIGH = enable/BẬT)

  delay(200);

  // MỞ KHÓA I2C NGAY SAU KHI ĐÃ CÓ NGUỒN 5V
  Wire.begin();

  // THIẾT LẬP INPUT CHO CHÂN NÚT BẤM
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // KHỞI TẠO LED (Mới)
  DisplayPower_begin();

  Serial.println("\n=============================================");
  Serial.println(" KHOI DONG FIRMWARE HEALTHSENSE (nRF52840)   ");
  Serial.println("=============================================");
  Serial.flush();

  // KHỞI TẠO BLUETOOTH
  Serial.println("[STEP 1] Dang khoi tao BLE...");
  Serial.flush();
  BLEManager_begin();
  Serial.println("[STEP 1] BLE -> OK!");
  Serial.flush();

  // KHỞI TẠO CẢM BIẾN GIA TỐC LSM6DS3
  Serial.println("[STEP 3] Dang khoi tao LSM6DS3...");
  Serial.flush();
  if (!AccelManager_begin(LSM6DS3_INT_PIN)) {
    Serial.println("LOI: Khong tim thay LSM6DS3!");
  } else {
    Serial.println("LSM6DS3 OK!");
  }
  Serial.flush();

  // 8. KHỞI TẠO CẢM BIẾN ĐO NHỊP TIM MAX30102
  Serial.println("[STEP 4] Dang khoi tao MAX30101...");
  Serial.flush();
  if (!PPGManager_begin(MAX30101_INT_PIN)) {
    Serial.println("LOI: Khong tim thay MAX30101!");
    while (1) {
      delay(100);
    }
  }
  Serial.println("MAX30101 OK!");
  Serial.flush();

  // 9. KHỞI TẠO MÁY TRẠNG THÁI HỆ THỐNG
  DeviceStateManager_begin(BUTTON_PIN, IMU_POWER_PIN, VIBRATOR_MOSFET_PIN);
  Serial.println(">>> TOAN BO HE THONG KHOI DONG THANH CONG! <<<");
  Serial.flush();
}

void loop() {
  DeviceStateManager_handleButton();

  // Xử lý lệnh nhận được từ điện thoại qua BLE Write (an toàn trong loop, tránh deadlock với SoftDevice)
  char bleCmd[64];
  if (BLEManager_popCommand(bleCmd, sizeof(bleCmd))) {
    DeviceStateManager_handleBLECommand(bleCmd);
  }

  // Đọc và xử lý sự kiện gia tốc MPU6050
  AccelManager_process();
  AccelManager_printDebug();

  if (AccelManager_popMotionEvent()) {
    DeviceStateManager_onEvent(EVT_MOTION);
  }

  if (AccelManager_popFallEvent()) {
    DeviceStateManager_onEvent(EVT_FALL_DETECTED);
  }

  // Đọc dữ liệu từ hàng đợi FIFO của MAX30102
  PPGManager_process();

  if (PPGManager_popNoFingerEvent()) {
    DeviceStateManager_onEvent(EVT_NOT_WEARING);
    Serial.println("[UNWEAR] Da thao dong ho. Tat LED tiet kiem pin!");
  }

  DeviceMode mode = DeviceStateManager_getMode();

  if (mode == MODE_WORKOUT) {
    // Chế độ tập luyện (Workout): Chỉ gửi nhịp tim/SpO2 trung bình mỗi 1 giây
    static unsigned long lastVitalsSend = 0;
    if (millis() - lastVitalsSend >= 1000 && BLEManager_isConnected()) {
      lastVitalsSend = millis();
      char buf[64];
      size_t len = (size_t)snprintf(buf, sizeof(buf), "W:%lu,%u,%u\n",
                                    millis(), PPGManager_getBPM(), PPGManager_getSpO2());
      BLEManager_notifyReport(buf, len);
    }
    // Giải phóng buffer tránh tràn bộ nhớ (giống ESP32-C3 dùng discardPacket)
    PPGManager_discardPacket();

  } else if (mode == MODE_MEASURE || mode == MODE_SCREENING) {
    // Chế độ đo đạc (Measure / Screening): Gửi gói dữ liệu PPG raw + bpm/spo2
    char packet[512];
    size_t len = 0;
    if (PPGManager_popPacket(packet, sizeof(packet), &len)) {
      if (BLEManager_isConnected()) {
        BLEManager_notify(packet, len);
      }
    }
  }

  DeviceStateManager_loop();

  // Cập nhật phần trăm dung lượng pin định kỳ mỗi 30 giây
  static unsigned long lastBatteryUpdateTime = 0;
  if (millis() - lastBatteryUpdateTime >= 30000) {
    lastBatteryUpdateTime = millis();
    BLEManager_updateBatteryLevel();
  }

  if (!BLEManager_isConnected()) delay(1);
}
