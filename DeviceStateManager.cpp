#include "DeviceStateManager.h"

#define MAX30101_INT_PIN     D3

static constexpr unsigned long EMERGENCY_CANCEL_WINDOW_MS = 15000UL;
static constexpr unsigned long EMERGENCY_IMMOBILE_WINDOW_MS = 10000UL;
static constexpr unsigned long EMERGENCY_FALL_CONFIRM_WINDOW_MS = 2000UL;
static constexpr unsigned long EMERGENCY_PROBE_INTERVAL_MS = 500UL;
static constexpr long EMERGENCY_NO_FINGER_IR_THRESHOLD = 50000L;
static constexpr uint8_t EMERGENCY_BPM_LOW_THRESHOLD = 40;
static constexpr uint8_t EMERGENCY_BPM_HIGH_THRESHOLD = 150;
static constexpr uint8_t EMERGENCY_SPO2_THRESHOLD = 90;
static constexpr unsigned long EMERGENCY_IR_SETTLE_MS = 1500UL;      // chờ cảm biến ổn định trước khi xét "không đeo tay"
static constexpr uint8_t EMERGENCY_NO_FINGER_CONFIRM_COUNT = 3;      // cần 3 lần thăm dò IR thấp liên tiếp (~1.5s)
static constexpr unsigned long EMERGENCY_ALERT_INTERVAL_MS = 2000UL; // tần suất tối đa của SOS:ALERT

enum EmergencyType {
    EMERGENCY_NONE,
    EMERGENCY_FALL,
    EMERGENCY_SYNCOPE,
    EMERGENCY_CARDIAC
};

static DeviceMode currentMode = MODE_IDLE;
static DeviceMode pendingMode = MODE_IDLE;
static unsigned long lastModeChangeTime = 0;
static unsigned long lastWearCheck = 0;
static unsigned long screeningStartTime = 0;
static unsigned long screeningEndTime = 0;
static unsigned long ledWakeTime = 0;
static bool isLEDOn = false;
static uint8_t wakeButtonPin;
static uint8_t accelVddPin_stored;

static bool isWearing = false;
static long btnPressTime = 0;
static bool isBtnPressed = false;
static bool isLongPressHandled = false;
static bool isScreening = false;

// Screening cycle management (10-minute cycle with 2 phases)
static unsigned long screeningCycleStart = 0;
// 0: idle, 1: Phase 1 (1 min measure), 2: Phase 1 sleep dynamic, 3: Phase 2 (30s measure), 4: Phase 2 sleep dynamic, 5: Phase 1 micro-retry wait (30s)
static uint8_t screeningRetryCount = 0;
static uint8_t screeningPhase = 0;
static unsigned long phaseStartTime = 0;
static unsigned long retrySleepStart = 0;
static unsigned long phaseSleepStart = 0;
static unsigned long phaseSleepDuration = 0;
static bool isSpo2Missed = false;
static uint8_t phase1AvgSpO2 = 0;

static bool emergencyActive = false;
static bool emergencyCountdownActive = false;
static EmergencyType emergencyType = EMERGENCY_NONE;
static unsigned long emergencyStartedAt = 0;
static unsigned long emergencyAlarmStartedAt = 0;
static unsigned long emergencyLastProbeAt = 0;
static uint8_t emergencyCancelSecondsLeft = 0;
static uint8_t emergencyBpm = 0;
static uint8_t emergencySpo2 = 0;
static long emergencyIr = 0;
static uint8_t vibratorPinStored = 0;
static bool emergencySimulated = false;           // true khi kích hoạt bằng CMD:TEST_SOS (bỏ qua kiểm tra đeo tay)
static uint8_t emergencyNoFingerCount = 0;        // số lần thăm dò IR thấp liên tiếp
static unsigned long emergencyLastAlertAt = 0;

// Accumulators for averaging vitals
static unsigned long bpmSum = 0;
static unsigned long spo2Sum = 0;
static unsigned long validSampleCount = 0;
static unsigned long lastSampleAccumTime = 0;

static void enterMode(DeviceMode mode);
static void exitMode(DeviceMode mode);
static void DeviceStateManager_requestMode(DeviceMode newMode);
static void DeviceStateManager_enterEmergency(EmergencyType type);
static void DeviceStateManager_processEmergency();
static void DeviceStateManager_abortEmergency(const char* reason);
static void DeviceStateManager_emitEmergencyStatus(const char* payload);
static void DeviceStateManager_triggerVibrator(bool on);

static const char* DeviceStateManager_emergencyTypeName(EmergencyType type) {
    return type == EMERGENCY_FALL ? "FALL" :
           type == EMERGENCY_SYNCOPE ? "SYNCOPE" : "CARDIAC";
}

// Hàm xử lý lệnh nhận từ điện thoại qua BLE Write (được gọi từ loop chính, an toàn tuyệt đối với I2C và SoftDevice)
void DeviceStateManager_handleBLECommand(const char* cmd) {
    if (cmd == nullptr || cmd[0] == '\0') return;
    Serial.print("[BLE-RX] Nhận lệnh từ điện thoại: ");
    Serial.println(cmd);

    if (strcmp(cmd, "CMD:START_MEASURE") == 0) {
        DeviceStateManager_onEvent(EVT_BLE_START_MEASURE);
    } else if (strcmp(cmd, "CMD:START_WORKOUT") == 0) {
        DeviceStateManager_onEvent(EVT_BLE_START_WORKOUT);
    } else if (strcmp(cmd, "CMD:START_SCREENING") == 0) {
        DeviceStateManager_onEvent(EVT_BLE_START_SCREENING);
    } else if (strcmp(cmd, "CMD:IDLE") == 0) {
        DeviceStateManager_onEvent(EVT_NOT_WEARING); // Về IDLE
    } else if (strcmp(cmd, "CMD:CANCEL_SOS") == 0) {
        DeviceStateManager_onEvent(EVT_BLE_CANCEL_SOS);
    } else if (strcmp(cmd, "CMD:TEST_SOS") == 0) {
        DeviceStateManager_onEvent(EVT_BLE_TEST_SOS);
    } else {
        Serial.print("[BLE-RX] Lệnh không xác định: ");
        Serial.println(cmd);
    }
}

void DeviceStateManager_begin(uint8_t buttonPin, uint8_t accelVddPin, uint8_t vibratorPin) {
    currentMode = MODE_TURN_ON;
    pendingMode = MODE_TURN_ON;
    lastModeChangeTime = millis();
    lastWearCheck = 0;
    screeningStartTime = 0;
    screeningEndTime = 0;
    ledWakeTime = 0;
    isLEDOn = false;
    btnPressTime = 0;
    isBtnPressed = false;
    isLongPressHandled = false;
    isWearing = false;
    isScreening = false;
    emergencyActive = false;
    emergencyCountdownActive = false;
    emergencyType = EMERGENCY_NONE;
    emergencyStartedAt = 0;
    emergencyAlarmStartedAt = 0;
    emergencyLastProbeAt = 0;
    emergencyCancelSecondsLeft = 0;
    emergencyBpm = 0;
    emergencySpo2 = 0;
    emergencyIr = 0;
    emergencySimulated = false;
    emergencyNoFingerCount = 0;
    emergencyLastAlertAt = 0;
    wakeButtonPin = buttonPin;
    // nrf_gpio_cfg_input(wakeButtonPin, NRF_GPIO_PIN_PULLUP);
    pinMode(wakeButtonPin, INPUT_PULLUP);
    accelVddPin_stored = accelVddPin;
    vibratorPinStored = vibratorPin;
    pinMode(vibratorPinStored, OUTPUT);
    digitalWrite(vibratorPinStored, LOW);
    enterMode(MODE_TURN_ON);
}

void DeviceStateManager_handleButton() {
    if (digitalRead(wakeButtonPin) == LOW) {
        if (!isBtnPressed) {
            isBtnPressed = true;
            btnPressTime = millis();
            isLongPressHandled = false;
        } else if (!isLongPressHandled && (millis() - btnPressTime >= 4000)) {
            isLongPressHandled = true;
            DeviceStateManager_onEvent(EVT_BUTTON_LONG);
        }
    } else {
        if (isBtnPressed) {
            isBtnPressed = false;
            if (!isLongPressHandled) {
                DeviceStateManager_onEvent(EVT_BUTTON_SHORT);
            }
        }
    }

    if (isLEDOn && (millis() - ledWakeTime > 3000)) {
        DisplayPower_showOff();
        isLEDOn = false;
    }
}

void DeviceStateManager_onEvent(DeviceEvent event) {
    switch (event) {
        case EVT_BUTTON_SHORT:
            if (currentMode == MODE_EMERGENCY && emergencyActive) {
                // Nhấn nút khi đang SOS = xác nhận/tắt cảnh báo tại thiết bị (theo STATE_MACHINE.md)
                Serial.println("event BUTTON_SHORT -> hủy SOS tại thiết bị");
                DeviceStateManager_abortEmergency("BUTTON");
                break;
            }
            DisplayPower_showOn();
            isLEDOn = true;
            ledWakeTime = millis();
            Serial.println("event BUTTON_SHORT, LED on");
            break;

        case EVT_BUTTON_LONG:
            Serial.println("event BUTTON_LONG, device shutdown");
            DeviceStateManager_requestMode(MODE_SHUTDOWN);
            break;

        case EVT_MOTION:
            // Cảm biến gia tốc không còn ảnh hưởng đến việc đo trên nRF52840.
            // Chỉ log để debug, không dừng / đổi chế độ nào cả.
            Serial.printf("[%lu] event MOTION (bỏ qua - không ảnh hưởng đo)\n", (unsigned long)millis());
            break;

        case EVT_NOT_WEARING:
            Serial.println("event NOT_WEARING");
            if (currentMode == MODE_EMERGENCY && emergencyActive) {
                // CMD:TEST_SOS bỏ qua kiểm tra đeo tay; SOS thật mà tháo tay thì hủy và BÁO cho app (trước đây thoát im lặng)
                if (!emergencySimulated) {
                    Serial.println("[EMERGENCY] Phát hiện không đeo tay -> hủy SOS");
                    DeviceStateManager_abortEmergency("NO_WEAR");
                }
                break;
            }
            sendBLECommand("CMD:NOT_WEARING\n");
            DeviceStateManager_requestMode(MODE_IDLE);
            break;

        case EVT_FALL_DETECTED:
            if (currentMode == MODE_EMERGENCY || pendingMode == MODE_EMERGENCY) {
                Serial.println("event FALL_DETECTED (bỏ qua - đang trong EMERGENCY)");
                break;
            }
            Serial.println("event FALL_DETECTED");
            DeviceStateManager_enterEmergency(EMERGENCY_FALL);
            break;

        case EVT_BLE_CANCEL_SOS:
            Serial.println("event BLE_CANCEL_SOS");
            if (currentMode == MODE_EMERGENCY && emergencyActive) {
                DeviceStateManager_abortEmergency(nullptr);
            }
            break;

        case EVT_BLE_START_MEASURE:
            Serial.println("event BLE_START_MEASURE");
            DeviceStateManager_requestMode(MODE_MEASURE);
            break;

        case EVT_BLE_START_WORKOUT:
            Serial.println("event BLE_START_WORKOUT");
            DeviceStateManager_requestMode(MODE_WORKOUT);
            break;

        case EVT_BLE_START_SCREENING:
            Serial.println("event BLE_START_SCREENING");
            sendBLECommand("CMD:START_SCREENING\n");
            if (currentMode == MODE_SCREENING) {
                // Ép khởi động lại chu kỳ nếu đang ở trong chế độ chờ/ngủ của Screening
                exitMode(MODE_SCREENING);
                enterMode(MODE_SCREENING);
            } else {
                DeviceStateManager_requestMode(MODE_SCREENING);
            }
            break;

        case EVT_BLE_TEST_SOS:
            // Giả lập té ngã để kiểm tra đường BLE -> app mà không cần ngã thật (bỏ qua kiểm tra đeo tay)
            Serial.println("event BLE_TEST_SOS (mô phỏng té ngã, bỏ qua kiểm tra đeo tay)");
            emergencySimulated = true;
            DeviceStateManager_enterEmergency(EMERGENCY_FALL);
            break;
    }
}

void DeviceStateManager_loop() {
    if (pendingMode != currentMode) {
        exitMode(currentMode);
        currentMode = pendingMode;
        enterMode(currentMode);
        lastModeChangeTime = millis();
    }

    if (currentMode == MODE_IDLE) {
        if (millis() - lastWearCheck >= 4000) {
            lastWearCheck = millis();

            PPGManager_wakeUp(MAX30101_INT_PIN);
            delay(50);

            long testIR = PPGManager_readIR();
            if (testIR > 50000) {
                Serial.println("[WEAR] Kich hoat do nhip tim!");
                DeviceStateManager_requestMode(MODE_SCREENING);
            } else {
                PPGManager_shutDown();
            }
        }
    }

    if (currentMode == MODE_EMERGENCY && emergencyActive) {
        DeviceStateManager_processEmergency();
    }

    if (currentMode == MODE_SCREENING) {
        unsigned long now = millis();
        
        if (screeningPhase == 1) {
            // ĐANG ĐO PHA 1 (1 phút)
            // Tích luỹ BPM/SpO2 mỗi 1 giây
            if (now - lastSampleAccumTime >= 1000) {
                lastSampleAccumTime = now;
                uint8_t bpm = PPGManager_getBPM();
                uint8_t spo2 = PPGManager_getSpO2();
                if (bpm > 0 && spo2 > 0) { // Chỉ lấy dữ liệu khi hợp lệ
                    bpmSum += bpm;
                    spo2Sum += spo2;
                    validSampleCount++;
                }
            }
            
            // Đã đo đủ 1 phút liên tục
            if (now - phaseStartTime >= 60000UL) {
                PPGManager_shutDown();
                
                uint8_t avgBPM = 0;
                uint8_t avgSpO2 = 0;
                if (validSampleCount > 0) {
                    avgBPM = bpmSum / validSampleCount;
                    avgSpO2 = spo2Sum / validSampleCount;
                }
                
                phase1AvgSpO2 = avgSpO2; // Lưu lại để dùng ở Pha 2
                isSpo2Missed = false;
                
                Serial.print("[SCREENING] Pha 1 thành công! Avg BPM: ");
                Serial.print(avgBPM);
                Serial.print(" | Avg SpO2: ");
                Serial.println(avgSpO2);
                
                // Gửi qua Characteristic 2 (Báo cáo Pha 1 hoàn tất)
                if (BLEManager_isConnected()) {
                    char report[32];
                    snprintf(report, sizeof(report), "R1:%u,%u\n", avgBPM, avgSpO2);
                    BLEManager_notifyReport(report, strlen(report));
                }
                
                // Chuyển sang pha nghỉ chờ Pha 2 (mốc 150s = 2.5 phút)
                screeningPhase = 2;
                phaseSleepStart = millis();
                long targetTime = screeningCycleStart + 150 * 1000UL;
                long remaining = targetTime - (long)millis();
                phaseSleepDuration = remaining > 0 ? remaining : 0;
                
                Serial.print("[SCREENING] Ngủ động ");
                Serial.print(phaseSleepDuration / 1000);
                Serial.println(" giây để đến mốc 2.5 phút (Pha 2)...");
            }
        } 
        else if (screeningPhase == 9) {
            // ĐANG NGỦ MICRO-RETRY (30 giây)
            if (now - retrySleepStart >= 30000UL) {
                long timeRemainingForP1 = (long)(screeningCycleStart + 150000UL) - (long)now;
                if (timeRemainingForP1 < 60000L) {
                    // Thời gian còn lại đến mốc Pha 2 < 60s, không đủ để đo trọn vẹn 1 phút Pha 1
                    Serial.println("[SCREENING] Thời gian còn lại < 60s, không đủ đo trọn vẹn Pha 1. Hủy đo Pha 1 và chuyển sang chờ Pha 2...");
                    isSpo2Missed = true;
                    screeningPhase = 2;
                    phaseSleepStart = millis();
                    phaseSleepDuration = timeRemainingForP1 > 0 ? timeRemainingForP1 : 0;
                } else {
                    // Đủ thời gian (>= 60s) -> Thức dậy đo lại Pha 1
                    Serial.println("[SCREENING] Hết 30 giây ngủ micro-retry. Thức dậy đo lại Pha 1...");
                    screeningPhase = 1;
                    phaseStartTime = millis();
                    bpmSum = 0;
                    spo2Sum = 0;
                    validSampleCount = 0;
                    lastSampleAccumTime = millis();
                    
                    PPGManager_wakeUp(MAX30101_INT_PIN);
                    PPGManager_setupPhase1();
                }
            }
        }
        else if (screeningPhase == 2 || screeningPhase == 4 || screeningPhase == 6) {
            // ĐANG NGHỈ CHỜ PHA TIẾP THEO
            if (now - phaseSleepStart >= phaseSleepDuration) {
                uint8_t nextMeasurePhase = 3;
                if (screeningPhase == 2) nextMeasurePhase = 3;      // Pha 2 (mốc 2.5 min)
                else if (screeningPhase == 4) nextMeasurePhase = 5; // Pha 3 (mốc 5.0 min)
                else if (screeningPhase == 6) nextMeasurePhase = 7; // Pha 4 (mốc 7.5 min)

                screeningPhase = nextMeasurePhase;
                phaseStartTime = millis();
                bpmSum = 0;
                spo2Sum = 0;
                validSampleCount = 0;
                lastSampleAccumTime = millis();
                
                Serial.print("[SCREENING] Thức dậy bắt đầu Pha đo thứ ");
                Serial.print((screeningPhase / 2) + 1);
                Serial.println("...");

                PPGManager_wakeUp(MAX30101_INT_PIN);
                PPGManager_setupPhase2(isSpo2Missed == false); // lowPower = true nếu không bị lỡ SpO2
            }
        }
        else if (screeningPhase == 3 || screeningPhase == 5 || screeningPhase == 7) {
            // ĐANG ĐO PHA PHỤ (Pha 2, 3, 4 - 30 giây)
            // Tích luỹ BPM/SpO2 mỗi 1 giây
            if (now - lastSampleAccumTime >= 1000) {
                lastSampleAccumTime = now;
                uint8_t bpm = PPGManager_getBPM();
                uint8_t spo2 = PPGManager_getSpO2();
                if (bpm > 0) {
                    bpmSum += bpm;
                    if (isSpo2Missed && spo2 > 0) {
                        spo2Sum += spo2;
                    }
                    validSampleCount++;
                }
            }
            
            // Đã đo đủ 30 giây
            if (now - phaseStartTime >= 30000UL) {
                PPGManager_shutDown();
                
                uint8_t avgBPM = 0;
                uint8_t avgSpO2 = 0;
                if (validSampleCount > 0) {
                    avgBPM = bpmSum / validSampleCount;
                    if (isSpo2Missed) {
                        avgSpO2 = spo2Sum / validSampleCount;
                    }
                }
                
                // Lọc BPM trước khi gửi
                uint8_t finalBPMReport = 0;
                if (avgBPM > 40 && avgBPM < 255) {
                    finalBPMReport = avgBPM;
                } else {
                    finalBPMReport = 0;
                }
                
                // Xác định SpO2 gửi đi
                uint8_t finalSpO2Report = isSpo2Missed ? avgSpO2 : phase1AvgSpO2;
                uint8_t phaseNumber = (screeningPhase / 2) + 1;
                
                Serial.print("[SCREENING] Pha ");
                Serial.print(phaseNumber);
                Serial.print(" hoàn tất! Avg BPM: ");
                Serial.print(avgBPM);
                Serial.print(" (Gửi: ");
                Serial.print(finalBPMReport);
                Serial.print(") | SpO2: ");
                Serial.println(finalSpO2Report);
                
                // Gửi qua Characteristic 2 (Báo cáo trung bình các pha 2, 3, 4)
                if (BLEManager_isConnected()) {
                    char report[32];
                    snprintf(report, sizeof(report), "R2:%u,%u\n", finalBPMReport, finalSpO2Report);
                    BLEManager_notifyReport(report, strlen(report));
                }
                
                // Tính toán mốc mục tiêu cho pha tiếp theo
                uint8_t nextSleepPhase = 4;
                unsigned long targetMs = 300000UL; // Mốc mặc định Pha 2 -> 3 (300s = 5 min)

                if (screeningPhase == 3) {
                    nextSleepPhase = 4;
                    targetMs = 300000UL; // 5.0 phút
                } else if (screeningPhase == 5) {
                    nextSleepPhase = 6;
                    targetMs = 450000UL; // 7.5 phút
                } else if (screeningPhase == 7) {
                    nextSleepPhase = 8;
                    targetMs = 600000UL; // 10.0 phút (Kết thúc chu kỳ)
                }

                screeningPhase = nextSleepPhase;
                phaseSleepStart = millis();
                long targetTime = screeningCycleStart + targetMs;
                long remaining = targetTime - (long)millis();
                phaseSleepDuration = remaining > 0 ? remaining : 0;
                
                Serial.print("[SCREENING] Ngủ động ");
                Serial.print(phaseSleepDuration / 1000);
                Serial.print(" giây để đến mốc ");
                Serial.print(targetMs / 1000);
                Serial.println("s...");
            }
        }
        else if (screeningPhase == 8) {
            // ĐANG NGHỈ KẾT THÚC CHU KỲ (đến mốc 600s = 10 phút) -> Bắt đầu lại chu kỳ 10 phút mới
            if (now - phaseSleepStart >= phaseSleepDuration) {
                Serial.println("[SCREENING] Đã xong 10 phút chu kỳ. Bắt đầu chu kỳ 10 phút mới (Pha 1)...");
                screeningCycleStart = millis();
                screeningPhase = 1;
                phaseStartTime = millis();
                bpmSum = 0;
                spo2Sum = 0;
                validSampleCount = 0;
                screeningRetryCount = 0;
                isSpo2Missed = false;
                lastSampleAccumTime = millis();
                
                PPGManager_wakeUp(MAX30101_INT_PIN);
                PPGManager_setupPhase1();
            }
        }
    }

}

DeviceMode DeviceStateManager_getMode() {
    return currentMode;
}

static void DeviceStateManager_requestMode(DeviceMode newMode) {
    if (newMode == currentMode || newMode == pendingMode) {
        return;
    }
    pendingMode = newMode;
}

static void enterMode(DeviceMode mode) {
    switch (mode) {
        case MODE_IDLE:
            Serial.println("Mode IDLE");
            // 1. Cấu hình I2C cho IMU trước khi sập nguồn
            AccelManager_setMotionThreshold(20);
            AccelManager_startFallDetection();
            PPGManager_shutDown();
            // Giữ BLE advertising bật khi IDLE để điện thoại luôn tìm thấy và chủ động kết nối được
            if (!BLEManager_isConnected()) {
                BLEManager_startAdvertising();
                Serial.println("[BLE] Đang bật advertising (IDLE)");
            }
            break;

        case MODE_MEASURE:
            Serial.println("Mode MEASURE");
            AccelManager_startFallDetection();
            PPGManager_wakeUp(MAX30101_INT_PIN);
            // Không cần chỉnh ngưỡng gia tốc vì bên nRF52840 không dừng đo khi có chuyển động
            break;

        case MODE_WORKOUT:
            Serial.println("Mode WORKOUT");
            PPGManager_wakeUp(MAX30101_INT_PIN);
            AccelManager_setMotionThreshold(20);
            AccelManager_startFallDetection();
            break;

        case MODE_SCREENING:
            Serial.println("Mode SCREENING");
            AccelManager_startFallDetection();
            screeningCycleStart = millis();
            screeningPhase = 1; // Bắt đầu Pha 1
            screeningRetryCount = 0;
            isSpo2Missed = false;
            phase1AvgSpO2 = 0;
            phaseStartTime = millis();
            
            bpmSum = 0;
            spo2Sum = 0;
            validSampleCount = 0;
            lastSampleAccumTime = millis();
            
            PPGManager_wakeUp(MAX30101_INT_PIN);
            PPGManager_setupPhase1();
            // Không cần chỉnh ngưỡng gia tốc vì bên nRF52840 không dừng đo khi có chuyển động
            break;

        case MODE_SHUTDOWN:
            Serial.println("Mode SHUTDOWN");
            PPGManager_shutDown();
            DisplayPower_showRed();  
            delay(1000);
            DisplayPower_showOff();
            AccelManager_deepSleep();
            Serial.println("[POWER] Đã tắt nguồn MPU6050");
            delay(100);
#ifdef ESP32
            esp_deep_sleep_enable_gpio_wakeup(1ULL << wakeButtonPin, ESP_GPIO_WAKEUP_GPIO_LOW);
            esp_deep_sleep_start();
#else
            // SỬA Ở ĐÂY: Bọc wakeButtonPin trong g_ADigitalPinMap
            nrf_gpio_cfg_sense_input(g_ADigitalPinMap[wakeButtonPin], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
            sd_power_system_off();
#endif
            break;
        case MODE_EMERGENCY:
            Serial.println("Mode EMERGENCY");
            emergencyActive = true;
            emergencyCountdownActive = true;
            emergencyType = emergencyType == EMERGENCY_NONE ? EMERGENCY_FALL : emergencyType;
            emergencyAlarmStartedAt = millis();
            emergencyStartedAt = millis();
            emergencyLastProbeAt = millis(); // chờ cảm biến ổn định trước lần thăm dò đầu tiên
            emergencyLastAlertAt = 0;
            emergencyNoFingerCount = 0;
            emergencyCancelSecondsLeft = 15;
            emergencyBpm = 0;
            emergencySpo2 = 0;
            emergencyIr = 0;
            // Dùng cấu hình LED biên độ 85 (giống phép thử đeo tay ở IDLE) để ngưỡng IR 50000 có cùng hiệu chuẩn.
            // setupPhase2(false) hạ LED xuống 75 làm IR thấp hơn ~12% -> dễ bị coi là không đeo tay.
            PPGManager_wakeUp(MAX30101_INT_PIN);
            PPGManager_setupPhase1();
            // LED đỏ 3s (tự tắt bởi handleButton) + rung để thấy ngay là firmware đã nhận té ngã
            DisplayPower_showRed();
            isLEDOn = true;
            ledWakeTime = millis();
            DeviceStateManager_triggerVibrator(true);
            Serial.printf("[EMERGENCY] Enter type=%s simulated=%d connected=%d\n",
                          DeviceStateManager_emergencyTypeName(emergencyType), emergencySimulated ? 1 : 0,
                          BLEManager_isConnected() ? 1 : 0);
            DeviceStateManager_emitEmergencyStatus("SOS:COUNTDOWN,15\n");
            break;
        case MODE_TURN_ON:
            Serial.println("Mode TURN_ON");
            DisplayPower_showOn();
            AccelManager_wakeUp();
            isLEDOn = true;
            ledWakeTime = millis();
            DeviceStateManager_requestMode(MODE_IDLE);
            break;
    }
}

static void exitMode(DeviceMode mode) {
    switch (mode) {
        case MODE_SCREENING:
            screeningStartTime = 0;
            screeningPhase = 0;
            PPGManager_shutDown();
            break;

        case MODE_IDLE:
            // Bật lại BLE advertising khi thoát IDLE (bắt đầu đo)
            BLEManager_startAdvertising();
            Serial.println("[BLE] Đã bật advertising (thoát IDLE)");
            break;

        case MODE_MEASURE:
        case MODE_WORKOUT:
            PPGManager_shutDown();
            break;
        case MODE_EMERGENCY:
            // Rời EMERGENCY bằng đường khác (CMD:START_*, giữ nút...) -> báo app để UI SOS không bị kẹt
            if (emergencyActive) {
                DeviceStateManager_emitEmergencyStatus("SOS:CANCELLED\n");
            }
            DisplayPower_showOff();
            DeviceStateManager_triggerVibrator(false);
            AccelManager_stopFallDetection();
            PPGManager_shutDown();
            emergencyActive = false;
            emergencyCountdownActive = false;
            emergencySimulated = false;
            emergencyNoFingerCount = 0;
            emergencyType = EMERGENCY_NONE;
            break;
        default:
            break;
    }
}

static void DeviceStateManager_triggerVibrator(bool on) {
    digitalWrite(vibratorPinStored, on ? HIGH : LOW);
}

static void DeviceStateManager_emitEmergencyStatus(const char* payload) {
    Serial.printf("[EMERGENCY] TX connected=%d: %s", BLEManager_isConnected() ? 1 : 0, payload);
    BLEManager_notifyEmergencyStatus(payload, strlen(payload));
    BLEManager_notifyEmergency(payload, strlen(payload));
}

static void DeviceStateManager_enterEmergency(EmergencyType type) {
    emergencyType = type;
    Serial.println("[EMERGENCY] Enter request from fall/syncope trigger");
    if (currentMode != MODE_EMERGENCY) {
        pendingMode = MODE_EMERGENCY;
    }
}

static void DeviceStateManager_abortEmergency(const char* reason) {
    // reason: nullptr = hủy từ app ("SOS:CANCELLED"); "NO_WEAR"/"BUTTON" được nối thêm để app biết lý do
    char payload[48];
    if (reason != nullptr && reason[0] != '\0') {
        snprintf(payload, sizeof(payload), "SOS:CANCELLED,%s\n", reason);
    } else {
        snprintf(payload, sizeof(payload), "SOS:CANCELLED\n");
    }
    DeviceStateManager_emitEmergencyStatus(payload);
    DeviceStateManager_triggerVibrator(false);
    AccelManager_stopFallDetection();
    PPGManager_shutDown();
    emergencyActive = false;
    emergencyCountdownActive = false;
    emergencySimulated = false;
    emergencyNoFingerCount = 0;
    emergencyType = EMERGENCY_NONE;
    DeviceStateManager_requestMode(MODE_IDLE);
}

static void DeviceStateManager_processEmergency() {
    if (!emergencyActive) return;

    unsigned long now = millis();

    if (emergencyCountdownActive) {
        if (now - emergencyAlarmStartedAt >= EMERGENCY_CANCEL_WINDOW_MS) {
            emergencyCountdownActive = false;
            {
                char confirmed[40];
                snprintf(confirmed, sizeof(confirmed), "SOS:CONFIRMED,%s\n", DeviceStateManager_emergencyTypeName(emergencyType));
                DeviceStateManager_emitEmergencyStatus(confirmed);
            }
        } else {
            uint8_t secondsLeft = (uint8_t)((EMERGENCY_CANCEL_WINDOW_MS - (now - emergencyAlarmStartedAt) + 999UL) / 1000UL);
            if (secondsLeft != emergencyCancelSecondsLeft) {
                emergencyCancelSecondsLeft = secondsLeft;
                char countdown[32];
                snprintf(countdown, sizeof(countdown), "SOS:COUNTDOWN,%u\n", emergencyCancelSecondsLeft);
                DeviceStateManager_emitEmergencyStatus(countdown);
            }
        }
    }

    if (now - emergencyLastProbeAt >= EMERGENCY_PROBE_INTERVAL_MS) {
        emergencyLastProbeAt = now;
        emergencyIr = PPGManager_readIR();
        emergencyBpm = PPGManager_getBPM();
        emergencySpo2 = PPGManager_getSpO2();
        Serial.printf("[EMERGENCY] probe t=%lums IR=%ld BPM=%u SpO2=%u lowIR=%u\n",
                      (unsigned long)(now - emergencyStartedAt), emergencyIr, emergencyBpm, emergencySpo2,
                      emergencyNoFingerCount);

        // Chỉ xét "không đeo tay" sau khi cảm biến đã ổn định và IR thấp liên tiếp nhiều lần
        // (tránh hủy nhầm ngay lúc vừa bật LED). CMD:TEST_SOS bỏ qua kiểm tra này.
        if (!emergencySimulated && (now - emergencyStartedAt) >= EMERGENCY_IR_SETTLE_MS) {
            if (emergencyIr < EMERGENCY_NO_FINGER_IR_THRESHOLD) {
                emergencyNoFingerCount++;
                if (emergencyNoFingerCount >= EMERGENCY_NO_FINGER_CONFIRM_COUNT) {
                    Serial.println("[EMERGENCY] IR thấp liên tiếp -> không đeo tay, hủy SOS");
                    DeviceStateManager_abortEmergency("NO_WEAR");
                    return;
                }
            } else {
                emergencyNoFingerCount = 0;
            }
        }

        bool abnormalVitals =
            (emergencyBpm > 0 && emergencyBpm < EMERGENCY_BPM_LOW_THRESHOLD) ||
            (emergencyBpm > EMERGENCY_BPM_HIGH_THRESHOLD) ||
            (emergencySpo2 > 0 && emergencySpo2 < EMERGENCY_SPO2_THRESHOLD);

        // Giới hạn tần suất ALERT (2s/lần) để không làm ngập kênh BLE
        bool immobile = !AccelManager_isMoving() && (now - emergencyStartedAt >= EMERGENCY_IMMOBILE_WINDOW_MS);
        if ((abnormalVitals || immobile) && (now - emergencyLastAlertAt >= EMERGENCY_ALERT_INTERVAL_MS)) {
            emergencyLastAlertAt = now;
            if (abnormalVitals) {
                char payload[96];
                snprintf(payload, sizeof(payload), "SOS:ALERT,%s,ACTIVE,%lu,%u,%u,%ld\n",
                         DeviceStateManager_emergencyTypeName(emergencyType), (unsigned long)now,
                         emergencyBpm, emergencySpo2, emergencyIr);
                DeviceStateManager_emitEmergencyStatus(payload);
            } else {
                DeviceStateManager_emitEmergencyStatus("SOS:ALERT,SYNCOPE,ACTIVE\n");
            }
        }

        if (emergencyType == EMERGENCY_FALL && (now - emergencyStartedAt >= EMERGENCY_FALL_CONFIRM_WINDOW_MS)) {
            if (!abnormalVitals && AccelManager_isMoving()) {
                emergencyType = EMERGENCY_SYNCOPE;
            }
        }

        if (emergencyType == EMERGENCY_FALL && emergencyCountdownActive && (now - emergencyStartedAt >= EMERGENCY_CANCEL_WINDOW_MS)) {
            emergencyType = EMERGENCY_CARDIAC;
        }
    }
}

uint8_t DeviceStateManager_getScreeningPhase() {
    return screeningPhase;
}
