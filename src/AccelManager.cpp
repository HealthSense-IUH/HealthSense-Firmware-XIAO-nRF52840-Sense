#include "AccelManager.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include <Arduino.h>

// Khởi tạo đối tượng cảm biến (I2C mặc định)
LSM6DS3 myIMU(I2C_MODE, 0x6A); 
static bool imuReady = false;
volatile bool isMoving = false;
static volatile uint32_t motionIsrCount = 0;
static volatile bool fallEventFlag = false;
static volatile bool imuInterruptFlag = false;
static uint8_t storedImuPin;
static bool fallDetectionEnabled = false;

static constexpr uint8_t LSM6DS3_FREE_FALL_REG = 0x5D;
static constexpr uint8_t LSM6DS3_WAKE_UP_SRC_REG = 0x1B;
// TAP_CFG1 = INTERRUPTS_ENABLE (bit7) | LIR (bit0). Thiếu LIR thì FF_IA tự xóa khi hết free-fall -> dễ lỡ sự kiện.
static constexpr uint8_t LSM6DS3_TAP_CFG1_VALUE = 0x81;

// Ngưỡng cử động cho AccelManager_isMoving(): tổng |Δx|+|Δy|+|Δz| raw (~0.12g) và thời gian giữ trạng thái "đang cử động"
static constexpr int32_t MOTION_DELTA_RAW = 2000;
static constexpr unsigned long MOTION_HOLD_MS = 1500UL;

// --- 3-STAGE FALL VERIFICATION FSM ---
// Giai đoạn 1: Free-fall (Phần cứng INT1 bắt rơi tự do)
// Giai đoạn 2: Impact (Gia tốc tổng >= 1.8g hoặc bão hòa cảm biến trong vòng 600ms sau rơi)
// Giai đoạn 3: Immobility (Bất động tối thiểu 1.5s sau va chạm; loại bỏ báo động giả khi vung tay/chạy bộ)
enum FallCheckState {
  FALL_STATE_IDLE = 0,
  FALL_STATE_WAIT_IMPACT,
  FALL_STATE_WAIT_IMMOBILITY
};

static FallCheckState fallState = FALL_STATE_IDLE;
static unsigned long fallStateStartMs = 0;
static unsigned long impactDetectedAt = 0;
static unsigned long fallLastSampleMs = 0;
static int16_t fallPrevX = 0;
static int16_t fallPrevY = 0;
static int16_t fallPrevZ = 0;

static constexpr unsigned long FALL_IMPACT_TIMEOUT_MS = 600UL;
static constexpr unsigned long FALL_SETTLE_MS = 500UL;
static constexpr unsigned long FALL_IMMOBILITY_WINDOW_MS = 1500UL;
static constexpr unsigned long FALL_SAMPLE_INTERVAL_MS = 50UL;
static constexpr float FALL_IMPACT_MAG_SQ_THRESH = 3.24f; // (1.8g)^2
static constexpr int16_t FALL_SATURATION_THRESH = 28000;
static constexpr int32_t FALL_MOTION_DELTA_THRESH_RAW = 2500; // ~0.15g (2500 / 16384)
static constexpr uint8_t FALL_MOTION_STREAK_THRESH = 4;       // Cần cử động liên tục >= 4 mẫu (~200ms) mới hủy
static uint8_t fallMotionStreak = 0;

static void AccelManager_handleInterrupt() {
  imuInterruptFlag = true;
}

bool AccelManager_begin(uint8_t intPin) {
  storedImuPin = intPin;
  
  // Bật nguồn cho cảm biến trên mạch XIAO Sense
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(50); // Chờ điện áp ổn định

  if (myIMU.begin() != 0) {
    imuReady = false;
    Serial.println("LỖI: Không tìm thấy LSM6DS3TR-C!");
    return false;
  }
  imuReady = true;

  // --- CẤU HÌNH THUẬT TOÁN ĐẾM BƯỚC VÀ PHÁT HIỆN CHUYỂN ĐỘNG ---
  // Tương tự chức năng setAccelerometerRange(MPU6050_RANGE_4_G) cũ[cite: 9]
  // nhưng làm việc trực tiếp với thanh ghi của STMicroelectronics
  
  // // Bật chức năng Pedometer (Đếm bước) & Motion Detection
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x50); // Bật Accelerometer, 208 Hz, 2G
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09); // Bật Filter
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);  // Kích hoạt chức năng tính toán nhúng
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10); // Đẩy cờ Ngắt (Interrupt) Pedometer/Motion ra chân INT1

  // --- CẤU HÌNH THUẬT TOÁN WAKE-UP (PHÁT HIỆN VUNG TAY) ---
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x50); // Bật Accelerometer, 208 Hz, 2G
  
  // TAP_CFG1: INTERRUPTS_ENABLE (0x80) + LIR (0x01) = 0x81 để giữ INT1/FF_IA HIGH cho đến khi WAKE_UP_SRC được đọc
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, LSM6DS3_TAP_CFG1_VALUE); 
  
  // WAKE_UP_THS: Cài đặt ngưỡng nhạy chuyển động. Số càng nhỏ càng nhạy (0x01 đến 0x3F)
  // Bạn có thể chỉnh số 0x02 này lên 0x04 hoặc 0x05 nếu thấy nó quá nhạy
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x20); 
  
  // MD1_CFG: Route Free-Fall interrupt to INT1 (theo app note / ví dụ chuẩn của ST)
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x10);

  // Free-fall threshold/duration.
  // 0x33 là cấu hình gần với ví dụ chuẩn của SparkFun/ST cho FF detection ổn định hơn 0x7F.
  myIMU.writeRegister(LSM6DS3_FREE_FALL_REG, 0x33);
  fallDetectionEnabled = true;

  // Xóa chốt INT1 (nếu có) trước khi gắn ngắt
  AccelManager_clearInterruptLatch();

  // Cấu hình chân ngắt trên nRF52840
  pinMode(storedImuPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(storedImuPin), AccelManager_handleInterrupt, RISING);

  Serial.println("LSM6DS3TR-C OK và đã bật Ngắt chuyển động!");
  return true;
}

bool AccelManager_isReady() { return imuReady; }
// Phát hiện cử động bằng cách so sánh mẫu gia tốc giữa 2 lần gọi (dùng cho kiểm tra bất động khi SOS).
// Đọc raw (±2g => ~16393 LSB/g); không dùng readFloat* vì thư viện scale theo accelRange mặc định 16g.
bool AccelManager_isMoving() {
  if (!imuReady) return false;
  static int16_t prevX = 0, prevY = 0, prevZ = 0;
  static bool hasPrev = false;
  static unsigned long lastMotionAt = 0;

  int16_t x = myIMU.readRawAccelX();
  int16_t y = myIMU.readRawAccelY();
  int16_t z = myIMU.readRawAccelZ();
  unsigned long now = millis();

  if (!hasPrev) {
    hasPrev = true;
    lastMotionAt = now;
  } else {
    int32_t delta = abs((int32_t)x - prevX) + abs((int32_t)y - prevY) + abs((int32_t)z - prevZ);
    if (delta > MOTION_DELTA_RAW) lastMotionAt = now;
  }
  prevX = x;
  prevY = y;
  prevZ = z;
  return (now - lastMotionAt) < MOTION_HOLD_MS;
}

bool AccelManager_popFallEvent() {
  bool fallEvent = false;
  noInterrupts();
  if (fallEventFlag) {
    fallEvent = true;
    fallEventFlag = false;
  }
  interrupts();
  return fallEvent;
}

bool AccelManager_popMotionEvent() {
  bool motionEvent = false;
  noInterrupts();
  if (isMoving) {
    motionEvent = true;
    isMoving = false;
  }
  interrupts();
  return motionEvent;
}

void AccelManager_process() {
  if (!imuReady) return;

  // 1. Xử lý cờ ngắt phần cứng INT1 từ LSM6DS3TR-C
  if (imuInterruptFlag) {
    noInterrupts();
    imuInterruptFlag = false;
    interrupts();

    isMoving = true;
    motionIsrCount++;
    // Đọc WAKE_UP_SRC đúng 1 lần: vừa lấy cờ FF_IA (bit5) vừa xóa chốt INT1 (LIR)
    uint8_t wakeUpSrc = 0;
    myIMU.readRegister(&wakeUpSrc, LSM6DS3_WAKE_UP_SRC_REG);
    Serial.printf("[ACCEL] INT1 WAKE_UP_SRC=0x%02X fallArmed=%d\n", wakeUpSrc, fallDetectionEnabled ? 1 : 0);

    // Giai đoạn 1: Phần cứng phát hiện rơi tự do (Free-fall bit 5)
    if (fallDetectionEnabled && (wakeUpSrc & 0x20)) {
      if (fallState == FALL_STATE_IDLE || fallState == FALL_STATE_WAIT_IMPACT) {
        fallState = FALL_STATE_WAIT_IMPACT;
        fallStateStartMs = millis();
        Serial.println("[ACCEL] Giai doan 1: Free-fall detected -> Cho va cham (WAIT_IMPACT)");
      }
    }
  }

  // 2. FSM xác minh va chạm & bất động (chỉ chạy khi tính năng fall detection được bật)
  if (!fallDetectionEnabled) {
    fallState = FALL_STATE_IDLE;
    return;
  }

  if (fallState == FALL_STATE_IDLE) {
    return;
  }

  unsigned long now = millis();

  if (fallState == FALL_STATE_WAIT_IMPACT) {
    // Quá thời gian 600ms mà không có va chạm -> cú rơi giả / an toàn
    if (now - fallStateStartMs > FALL_IMPACT_TIMEOUT_MS) {
      fallState = FALL_STATE_IDLE;
      fallMotionStreak = 0;
      Serial.println("[ACCEL] Giai doan 2: Khong co va cham sau 600ms -> Bo qua");
    } else {
      int16_t rx = myIMU.readRawAccelX();
      int16_t ry = myIMU.readRawAccelY();
      int16_t rz = myIMU.readRawAccelZ();

      float ax = (float)rx / 16384.0f;
      float ay = (float)ry / 16384.0f;
      float az = (float)rz / 16384.0f;
      float magSq = ax * ax + ay * ay + az * az;

      // Va chạm: gia tốc tổng >= 1.8g (magSq >= 3.24) hoặc cảm biến bão hòa (|raw| >= 28000)
      if (magSq >= FALL_IMPACT_MAG_SQ_THRESH ||
          abs((int32_t)rx) >= FALL_SATURATION_THRESH ||
          abs((int32_t)ry) >= FALL_SATURATION_THRESH ||
          abs((int32_t)rz) >= FALL_SATURATION_THRESH) {
        fallState = FALL_STATE_WAIT_IMMOBILITY;
        impactDetectedAt = now;
        fallLastSampleMs = now;
        fallPrevX = rx;
        fallPrevY = ry;
        fallPrevZ = rz;
        fallMotionStreak = 0;
        Serial.printf("[ACCEL] Giai doan 2: Phat hien va cham (magSq=%.2f) -> Cho bat dong (WAIT_IMMOBILITY)\n", magSq);
      }
    }
  } else if (fallState == FALL_STATE_WAIT_IMMOBILITY) {
    unsigned long timeSinceImpact = now - impactDetectedAt;

    // Trong FALL_SETTLE_MS (500ms) đầu tiên sau va chạm: giai đoạn rung chấn cơ học / nảy đập,
    // liên tục ghi nhận mẫu baseline mà không bắt lỗi cử động
    if (timeSinceImpact < FALL_SETTLE_MS) {
      fallPrevX = myIMU.readRawAccelX();
      fallPrevY = myIMU.readRawAccelY();
      fallPrevZ = myIMU.readRawAccelZ();
      fallLastSampleMs = now;
      fallMotionStreak = 0;
    } else {
      // Sau thời gian rung chấn: kiểm tra cử động định kỳ mỗi 50ms
      if (now - fallLastSampleMs >= FALL_SAMPLE_INTERVAL_MS) {
        fallLastSampleMs = now;
        int16_t rx = myIMU.readRawAccelX();
        int16_t ry = myIMU.readRawAccelY();
        int16_t rz = myIMU.readRawAccelZ();

        int32_t delta = abs((int32_t)rx - fallPrevX) + abs((int32_t)ry - fallPrevY) + abs((int32_t)rz - fallPrevZ);
        fallPrevX = rx;
        fallPrevY = ry;
        fallPrevZ = rz;

        if (delta > FALL_MOTION_DELTA_THRESH_RAW) {
          fallMotionStreak++;
          Serial.printf("[ACCEL] Giai doan 3: Cu dong nhe (delta=%ld, streak=%u/%u)\n",
                        (long)delta, fallMotionStreak, FALL_MOTION_STREAK_THRESH);

          // Chỉ khi cử động liên tục >= 4 mẫu (~200ms) chứng tỏ người dùng đang hoạt động bình thường / đứng dậy
          if (fallMotionStreak >= FALL_MOTION_STREAK_THRESH) {
            fallState = FALL_STATE_IDLE;
            fallMotionStreak = 0;
            Serial.printf("[ACCEL] Giai doan 3: Phat hien hoat dong lien tuc (streak>=%u) -> Huy bao dong (Nguoi dung van hoat dong)\n", FALL_MOTION_STREAK_THRESH);
            return;
          }
        } else {
          // Lắng xuống: giảm streak dần
          if (fallMotionStreak > 0) fallMotionStreak--;
        }
      }

      // Nếu đã qua đủ thời gian kiểm tra bất động (FALL_SETTLE_MS + FALL_IMMOBILITY_WINDOW_MS)
      // mà không bị hủy bởi chuỗi cử động liên tục -> Xác nhận té ngã thật sự!
      if (timeSinceImpact >= (FALL_SETTLE_MS + FALL_IMMOBILITY_WINDOW_MS)) {
        fallEventFlag = true;
        fallState = FALL_STATE_IDLE;
        fallMotionStreak = 0;
        Serial.println("[ACCEL] Giai doan 3: Xac nhan bat dong -> TE NGA XAC NHAN! (fallEventFlag=1)");
      }
    }
  }
}

void AccelManager_deepSleep() {
  if (!imuReady) return;
  // Đưa cảm biến vào chế độ Power-Down
  detachInterrupt(digitalPinToInterrupt(storedImuPin));
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00);
  fallDetectionEnabled = false;
  fallState = FALL_STATE_IDLE;
  fallMotionStreak = 0;
  Serial.println("[ACCEL] Đã đưa LSM6DS3TR-C vào chế độ Deep Sleep");
}

void AccelManager_wakeUp() {
  if (!imuReady) return;
  fallState = FALL_STATE_IDLE;
  fallMotionStreak = 0;
  // Đánh thức cảm biến (208 Hz)
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x50);
  pinMode(storedImuPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(storedImuPin), AccelManager_handleInterrupt, RISING);
  Serial.println("[ACCEL] Đã đánh thức LSM6DS3TR-C");
}

void AccelManager_printDebug() {
  static unsigned long lastDebugPrint = 0;
  if (!imuReady) return;

  if (millis() - lastDebugPrint >= 1000) {
    lastDebugPrint = millis();

    // Serial.print("[");
    // Serial.print(millis());
    // Serial.print("] [ACCEL-DBG] isMoving=");
    // Serial.print(isMoving ? "1" : "0");
    // Serial.print(" | isrCount=");
    // Serial.print((uint32_t)motionIsrCount);
    // Serial.println();
  }
}

void AccelManager_setMotionThreshold(uint8_t threshold) {
  if (!imuReady) return;
  
  // Thanh ghi WAKE_UP_THS chỉ nhận giá trị tối đa 6 bit (0x3F tức là 63)
  uint8_t regValue = threshold; 
  if (regValue > 0x3F) {
      regValue = 0x3F; // Giới hạn chống tràn thanh ghi
  }
  
  // Ghi giá trị mới vào thanh ghi cấu hình ngưỡng vung tay
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, regValue);
  
  Serial.print("[ACCEL] Đã đổi ngưỡng nhạy vung tay thành: ");
  Serial.println(regValue);
}

void AccelManager_startFallDetection() {
  if (!imuReady) return;
  fallDetectionEnabled = true;
  fallState = FALL_STATE_IDLE;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, LSM6DS3_TAP_CFG1_VALUE);
  myIMU.writeRegister(LSM6DS3_FREE_FALL_REG, 0x33);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x10);
  // Xóa chốt INT1 còn tồn đọng để cạnh lên (RISING) kế tiếp luôn được bắt
  AccelManager_clearInterruptLatch();
  Serial.println("[ACCEL] Fall detection armed on INT1");
}

void AccelManager_stopFallDetection() {
  if (!imuReady) return;
  fallDetectionEnabled = false;
  fallState = FALL_STATE_IDLE;
}

bool AccelManager_readFreeFallInterrupt() {
  if (!imuReady) return false;
  uint8_t value = 0;
  myIMU.readRegister(&value, LSM6DS3_WAKE_UP_SRC_REG);
  return (value & 0x20) != 0;
}

void AccelManager_clearInterruptLatch() {
  if (!imuReady) return;
  uint8_t dummy = 0;
  myIMU.readRegister(&dummy, LSM6DS3_WAKE_UP_SRC_REG);
}
