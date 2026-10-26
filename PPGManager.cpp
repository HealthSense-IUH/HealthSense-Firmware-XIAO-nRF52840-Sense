#include "PPGManager.h"
#include <MAX30105.h>
#include <heartRate.h>
#include <Arduino.h>

#define MAX30101_INT_PIN     D3

static MAX30105 particleSensor;
static const byte* interruptPinPtr = nullptr;
static volatile bool dataReady = false;
static volatile bool isSensorShutdown = false;

#define PPG_PACKET_SIZE_LOCAL 5  // Gom 10 mẫu / gói (giống ESP32-C3)
static char ppgPayload[512] = "";
static size_t ppgPayloadLen = 0;
static uint8_t ppgSampleCount = 0;

// buffers for SpO2 calculation (Red + IR)
#define BUFFER_SIZE_LOCAL 167
static uint32_t redBuffer[BUFFER_SIZE_LOCAL];
static uint32_t irBuffer[BUFFER_SIZE_LOCAL];
static int bufferIndex = 0;
static bool bufferFull = false;

// Buffer BPM từ LED xanh lá (Green)
static uint32_t greenBeatBuffer = 0;  // Giá trị green mẫu hiện tại để checkForBeat

static uint8_t finalBPM = 0;
static uint8_t finalSpO2 = 0;
static long lastBeatTime = 0;
static float beatAvg = 0;

// no-finger detection (mirror original behavior)
static unsigned long noFingerStartTime = 0;
static volatile bool noFingerEventFlag = false;

static uint8_t storedMaxPin = 32;

void PPGManager_handleInterrupt() {
  dataReady = true;
}

bool PPGManager_begin(uint8_t interruptPin) {
  storedMaxPin = interruptPin;

  // 3. GỌI I2C KẾT NỐI SENSOR
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    return false;
  }

  particleSensor.setup(60, 4, 3, 400, 215, 16384);
  particleSensor.setPulseAmplitudeRed(85);
  particleSensor.setPulseAmplitudeIR(85);
  particleSensor.setPulseAmplitudeGreen(85);

  pinMode(storedMaxPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(storedMaxPin), PPGManager_handleInterrupt, FALLING);
  particleSensor.enableDATARDY();

  delay(10);
  particleSensor.getINT1();
  particleSensor.getINT2();
  dataReady = false;

  return true;
}

void PPGManager_shutDown() {
  if (isSensorShutdown) return;
  isSensorShutdown = true;
  dataReady = false;
  noFingerStartTime = 0;
  noFingerEventFlag = false;

  // 1. Tắt dòng phát quang của cả 3 LED về 0 để dập tắt LED triệt để
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeIR(0);
  particleSensor.setPulseAmplitudeGreen(0);

  // 2. Đưa IC cảm biến vào chế độ Low-Power Shutdown (IC ngừng lấy mẫu)
  particleSensor.shutDown();

  // 3. Gỡ ngắt khỏi chân vi điều khiển và giữ PULLUP (tránh nhiễu hoặc pulldown kéo mức LOW giả)
  detachInterrupt(digitalPinToInterrupt(storedMaxPin));
  pinMode(storedMaxPin, INPUT_PULLUP);
}

void PPGManager_wakeUp(uint8_t interruptPin) {
  storedMaxPin = interruptPin;

  // 1. Đánh thức IC và xóa dữ liệu cũ trong FIFO
  particleSensor.wakeUp();
  particleSensor.clearFIFO();

  // 2. Cài đặt lại cấu hình đo và biên độ LED mặc định (17-bit, 16384nA chống bão hòa)
  particleSensor.setup(60, 4, 3, 400, 215, 16384);
  particleSensor.setPulseAmplitudeRed(85);
  particleSensor.setPulseAmplitudeIR(85);
  particleSensor.setPulseAmplitudeGreen(85);

  // 3. Cấu hình lại chân ngắt
  pinMode(storedMaxPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(storedMaxPin), PPGManager_handleInterrupt, FALLING);
  particleSensor.enableDATARDY();

  delay(10);
  particleSensor.getINT1();
  particleSensor.getINT2();
  dataReady = false;
  isSensorShutdown = false;
}

static uint8_t filter3_BPM(uint8_t newVal) {
  static uint8_t b_bpm[3] = {0,0,0};
  b_bpm[0] = b_bpm[1]; b_bpm[1] = b_bpm[2]; b_bpm[2] = newVal;
  uint8_t x=b_bpm[0], y=b_bpm[1], z=b_bpm[2];
  if ((x - y) * (z - x) >= 0) return x;
  if ((y - x) * (z - y) >= 0) return y;
  return z;
}



static uint8_t filter3_SpO2(uint8_t newVal) {
  static uint8_t b_spo2[3] = {0,0,0};
  b_spo2[0] = b_spo2[1]; b_spo2[1] = b_spo2[2]; b_spo2[2] = newVal;
  uint8_t x=b_spo2[0], y=b_spo2[1], z=b_spo2[2];
  if ((x - y) * (z - x) >= 0) return x;
  if ((y - x) * (z - y) >= 0) return y;
  return z;
}

void PPGManager_process() {
  if (isSensorShutdown) return;

  // Bắt tín hiệu ngắt ngay trong loop (Active LOW)
  if (digitalRead(storedMaxPin) == LOW) {
    dataReady = true;
  }

  if (!dataReady) return;
  dataReady = false;

  particleSensor.getINT1();
  particleSensor.getINT2();
  particleSensor.check();

  while (particleSensor.available()) {
    long irValue  = particleSensor.getFIFOIR();
    long redValue = particleSensor.getFIFORed();
    long greenValue = particleSensor.getFIFOGreen();  // Dùng cho BPM

    // Phát hiện tháo tay dựa trên IR (ngưỡng như ESP32-C3)
    if (irValue < 50000) {
      if (noFingerStartTime == 0) {
        noFingerStartTime = millis();
      }
      if (millis() - noFingerStartTime > 2000) {
        // Reset trạng thái đo
        finalBPM = 0;
        finalSpO2 = 0;
        beatAvg = 0;
        bufferIndex = 0;
        bufferFull = false;
        noFingerEventFlag = true;
        noFingerStartTime = 0;
        break;
      }
    } else {
      noFingerStartTime = 0;

      // Đóng gói BLE: millis,red,ir,bpm,spo2 (tương thích format ESP32-C3 & app)
      char tempStr[64];
      int slen = snprintf(tempStr, sizeof(tempStr), "%lu,%lu,%lu,%u,%u\n",
                          (unsigned long)millis(), (unsigned long)redValue,
                          (unsigned long)irValue, finalBPM, finalSpO2);
      if (slen > 0 && ppgPayloadLen + (size_t)slen < sizeof(ppgPayload)) {
        memcpy(ppgPayload + ppgPayloadLen, tempStr, (size_t)slen + 1);
        ppgPayloadLen += (size_t)slen;
        ppgSampleCount++;
      }

      // Lưu Red + IR vào cửa sổ SpO2
      redBuffer[bufferIndex] = (uint32_t)redValue;
      irBuffer[bufferIndex]  = (uint32_t)irValue;
      bufferIndex++;
      if (bufferIndex >= BUFFER_SIZE_LOCAL) { bufferIndex = 0; bufferFull = true; }

      // ---- BPM từ LED XANH LÁ (Green) ----
      if (checkForBeat(greenValue)) {
        long delta = millis() - lastBeatTime;
        lastBeatTime = millis();
        float beatsPerMinute = 60000.0f / delta;
        if (beatsPerMinute > 40 && beatsPerMinute < 255) {
          beatAvg = (beatAvg == 0) ? beatsPerMinute : (beatAvg * 0.2f) + (beatsPerMinute * 0.8f);
          finalBPM = filter3_BPM((uint8_t)beatAvg);
        }
      }

      // ---- SpO2 từ Red + IR (tính lại mỗi giây, sau khi buffer đầy) ----
      if (bufferFull) {
        static uint8_t spo2CalcCounter = 0;
        if (++spo2CalcCounter >= 100) {  // 400Hz / 4 avg = ~100 mẫu/s
          spo2CalcCounter = 0;
          uint32_t minRed = redBuffer[0], maxRed = redBuffer[0];
          uint32_t minIR  = irBuffer[0],  maxIR  = irBuffer[0];
          for (int i = 1; i < BUFFER_SIZE_LOCAL; i++) {
            if (redBuffer[i] < minRed) minRed = redBuffer[i];
            if (redBuffer[i] > maxRed) maxRed = redBuffer[i];
            if (irBuffer[i]  < minIR)  minIR  = irBuffer[i];
            if (irBuffer[i]  > maxIR)  maxIR  = irBuffer[i];
          }
          long acRed = maxRed - minRed, dcRed = minRed;
          long acIR  = maxIR  - minIR,  dcIR  = minIR;
          if (acIR > 0 && dcRed > 0) {
            float rValue = ((float)acRed / dcRed) / ((float)acIR / dcIR);
            float spo2Calc = 110.0f - (17.0f * rValue);
            if (spo2Calc > 100.0f) spo2Calc = 100.0f;
            if (spo2Calc >= 80.0f) finalSpO2 = filter3_SpO2((uint8_t)spo2Calc);
          }
        }
      }

      // LOG CSV: millis,ir,red,bpm,spo2,green (cột cuối thay motion bằng green raw)
      if (Serial) {
        Serial.printf("%lu,%lu,%lu,%u,%u,%lu\n",
                      (unsigned long)millis(), (unsigned long)irValue,
                      (unsigned long)redValue, finalBPM, finalSpO2,
                      (unsigned long)greenValue);
      }
    }
    particleSensor.nextSample();
  }
}

bool PPGManager_popNoFingerEvent() {
  if (noFingerEventFlag) {
    noFingerEventFlag = false;
    return true;
  }
  return false;
}

long PPGManager_readIR() {
  return particleSensor.getIR();
}

bool PPGManager_popPacket(char* outBuf, size_t bufSize, size_t* outLen) {
  if (ppgSampleCount < PPG_PACKET_SIZE_LOCAL || ppgPayloadLen >= bufSize) return false;
  memcpy(outBuf, ppgPayload, ppgPayloadLen + 1);
  if (outLen) *outLen = ppgPayloadLen;
  PPGManager_discardPacket();
  return true;
}

/** Bỏ gói đang gom (chế độ không gửi PPG thô, tránh đầy bộ đệm - giống ESP32-C3). */
void PPGManager_discardPacket() {
  ppgPayload[0] = '\0';
  ppgPayloadLen = 0;
  ppgSampleCount = 0;
}

uint8_t PPGManager_getBPM() { return finalBPM; }
uint8_t PPGManager_getSpO2() { return finalSpO2; }

void PPGManager_setupPhase1() {
  isSensorShutdown = false;
  particleSensor.setup(60, 4, 3, 400, 215, 16384);
  particleSensor.setPulseAmplitudeRed(85);
  particleSensor.setPulseAmplitudeIR(85);
  particleSensor.setPulseAmplitudeGreen(85);
  particleSensor.enableDATARDY();
  particleSensor.clearFIFO();
  // Clear internal state/buffers
  bufferIndex = 0;
  bufferFull = false;
  finalBPM = 0;
  finalSpO2 = 0;
  beatAvg = 0;
}

void PPGManager_setupPhase2(bool lowPower) {
  isSensorShutdown = false;
  if (lowPower) {
    particleSensor.setup(60, 4, 3, 400, 215, 16384);
    particleSensor.setPulseAmplitudeRed(5);
    particleSensor.setPulseAmplitudeIR(75);
    particleSensor.setPulseAmplitudeGreen(75);
  } else {
    particleSensor.setup(60, 4, 3, 400, 215, 16384);
    particleSensor.setPulseAmplitudeRed(75);
    particleSensor.setPulseAmplitudeIR(75);
    particleSensor.setPulseAmplitudeGreen(75);
  }
  particleSensor.enableDATARDY();
  particleSensor.clearFIFO();
  // Clear internal state/buffers
  bufferIndex = 0;
  bufferFull = false;
  finalBPM = 0;
  finalSpO2 = 0;
  beatAvg = 0;
}

