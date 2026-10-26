# HealthSense Firmware — XIAO nRF52840 Sense

Firmware cho thiết bị đeo theo dõi sức khỏe và phát hiện té ngã **HealthSense**, xây dựng trên nền tảng vi điều khiển **Seeed Studio XIAO nRF52840 Sense** kết hợp cùng cảm biến quang học **MAX30101 / MAX30102** và module định vị **u-blox MAX-M10S**.

---

## 📌 Tính năng nổi bật

- **Phát hiện té ngã 3 giai đoạn (3-Stage Fall Verification FSM)**:
  - **Giai đoạn 1 (Free-fall)**: Nhận diện rơi tự do qua ngắt phần cứng `INT1` (bit `FF_IA` của LSM6DS3TR-C, $g < 0.35g$).
  - **Giai đoạn 2 (Impact)**: Bắt va chạm mặt sàn ($g \ge 1.8g$ hoặc bão hòa gia tốc trong cửa sổ $600\text{ ms}$).
  - **Giai đoạn 3 (Immobility)**: Chờ $500\text{ ms}$ dập tắt hoàn toàn rung chấn nảy cơ học (`FALL_SETTLE_MS`), sau đó kiểm tra bất động trong $1500\text{ ms}$ với bộ lọc chuỗi cử động (`fallMotionStreak`) nhằm chống báo động giả khi vung tay/chạy bộ.
- **Quy trình cấp cứu SOS thông minh qua BLE**:
  - Tự động kích hoạt chế độ khẩn cấp `MODE_EMERGENCY`.
  - Đếm ngược $15\text{ giây}$ (`SOS:COUNTDOWN,15` $\rightarrow$ `1`) đồng bộ lên điện thoại, nháy LED đỏ và rung haptic.
  - **Cơ chế hủy linh hoạt**: Hủy bằng nút bấm vật lý (`BUTTON`), hủy từ ứng dụng di động (`CMD:CANCEL_SOS`), hoặc tự hủy khi không đeo trên tay (`NO_WEAR` qua kiểm tra IR).
  - Khi hết thời gian đếm ngược: chốt cảnh báo (`SOS:CONFIRMED,FALL`) và liên tục truyền sinh hiệu nhịp tim/SpO2/IR (`SOS:ALERT,FALL,ACTIVE,...`) mỗi $2\text{ giây}$.
- **Đo nhịp tim (BPM) & Nồng độ oxy trong máu (SpO2)**:
  - Thu thập tín hiệu PPG quang học $100\text{ Hz}$ qua hàng đợi phần cứng FIFO của MAX30101.
  - Lọc trung vị và tính toán BPM thời gian thực; tự động phát hiện tháo đồng hồ để tắt LED tiết kiệm pin.
- **Sàng lọc rung nhĩ (AFib Screening)**:
  - Chu kỳ sàng lọc đa pha kéo dài 10 phút, tự động kích hoạt định kỳ khi người dùng đeo đồng hồ ở trạng thái nghỉ.
- **Chế độ tập luyện (Workout Mode)**:
  - Đếm bước chân thông minh bằng thuật toán nhận diện đỉnh gia tốc $25\text{ Hz}$.
  - Gói tin gọn nhẹ gửi định kỳ mỗi $1 - 3\text{ giây}$ lên ứng dụng di động.
- **Quản lý năng lượng tối ưu (Ultra Low Power)**:
  - Ngắt nguồn cảm biến và chip ngoại vi qua các cổng khóa MOSFET (`TPS61023` 5V Boost, `UBLOX` GPS P-MOSFET, `LSM6DS3` Power Pin).
  - Hỗ trợ chế độ System Deep Sleep với dòng tiêu thụ $< 15\,\mu\text{A}$, đánh thức bằng nhấn giữ nút nguồn (`BUTTON_PIN`).

---

## 🔌 Sơ đồ chân phần cứng (Pinout)

Thiết bị sử dụng vi điều khiển **Nordic nRF52840** (ARM Cortex-M4F @ 64 MHz, Bluetooth 5.0 BLE) trên bo mạch **Seeed Studio XIAO nRF52840 Sense**:

| Chân trên XIAO | GPIO nRF52840 | Tên cấu hình | Chức năng / Mô tả |
|---|---|---|---|
| **D0** | `P0.02` (AIN0) | `UBLOX_MOSFET_PIN` | Cổng P-MOSFET cấp nguồn module GPS (Active-LOW: LOW = BẬT, HIGH = TẮT) |
| **D1** | `P0.03` (AIN1) | `BUCK_MOSFET_PIN` | Chân Enable mạch Boost 5V TPS61023 cấp cho LED MAX30101 (HIGH = BẬT) |
| **D2** | `P0.28` (AIN4) | `BUTTON_PIN` | Nút bấm người dùng (`INPUT_PULLUP`), đánh thức máy từ Deep Sleep |
| **D3** | `P0.29` (AIN5) | `MAX30101_INT_PIN` | Ngắt Data-Ready từ cảm biến quang MAX30101/MAX30102 (`FALLING`) |
| **D4** | `P0.04` (AIN2) | `SDA` | Bus dữ liệu phần cứng I2C (Wire) kết nối MAX30101 và LSM6DS3 |
| **D5** | `P0.05` (AIN3) | `SCL` | Bus xung nhịp phần cứng I2C (Wire) |
| **D6** | `P1.11` | `UART_TX` (`Serial1 TX`)| Chân truyền UART sang RX module GPS u-blox MAX-M10S |
| **D7** | `P1.12` | `UART_RX` (`Serial1 RX`)| Chân nhận UART từ TX module GPS u-blox MAX-M10S |
| **D8** | `P1.13` | `VIBRATOR_MOSFET_PIN` | Kích cổng N-MOSFET rung motor phản hồi xúc giác / báo động khẩn cấp |
| **VBAT_ENABLE**| `P0.14` | Cầu phân áp Pin | **LƯU Ý:** Luôn khóa ở mức `LOW` để bảo vệ chân ADC khi sạc USB |
| **PIN_VBAT** | `P0.31` (AIN7) | Cảm biến điện áp Pin | Đọc ADC 12-bit mức pin LiPo qua cầu trở $1\,\text{M}\Omega / 510\,\text{k}\Omega$ |
| **IMU_INT1** | `P0.11` | `PIN_LSM6DS3TR_C_INT1` | Ngắt phần cứng từ cảm biến gia tốc LSM6DS3TR-C tích hợp |
| **IMU_POWER** | `P0.12` | `PIN_LSM6DS3TR_C_POWER`| Cấp/ngắt nguồn VDD cho cảm biến LSM6DS3TR-C onboard |
| **LED_RED** | `P0.17` | LED Đỏ Onboard | Báo động khẩn cấp SOS, pin yếu hoặc tắt nguồn (Active-LOW) |
| **LED_GREEN** | `P0.15` | LED Xanh lá Onboard | Báo trạng thái hoạt động / đo thành công (Active-LOW) |
| **LED_BLUE** | `P0.16` | LED Xanh dương Onboard| Báo trạng thái kết nối và quảng bá BLE (Active-LOW) |

---

## 📡 Giao thức BLE (Bluetooth Low Energy)

- **Tên thiết bị quảng bá**: `HuyWatch_XXXX` (`XXXX` là 2 byte cuối địa chỉ MAC).
- **Service UUID chính**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
  - **DATA Characteristic** (`beb5483e-36e1-4688-b7f5-ea07361b26a8` - `NOTIFY`): Gói dữ liệu thô PPG và sự kiện hệ thống.
  - **COMMAND Characteristic** (`beb5483e-36e1-4688-b7f5-ea07361b26a9` - `WRITE` / `WRITE_NR`): Tiếp nhận lệnh điều khiển từ điện thoại.
  - **REPORT Characteristic** (`beb5483e-36e1-4688-b7f5-ea07361b26aa` - `NOTIFY`): Gói tin kết quả đo, vitals tập luyện và bản tin SOS khẩn cấp.
- **Battery Service chuẩn Bluetooth SIG**: Service `0x180F` / Characteristic `0x2A19` (`READ` & `NOTIFY`).

### Các chuỗi lệnh và gói tin:
| Chiều | Chuỗi / Định dạng | Ý nghĩa |
|---|---|---|
| **App $\rightarrow$ Watch** | `CMD:START_MEASURE` | Bắt đầu đo nhịp tim / SpO2 chủ động |
| **App $\rightarrow$ Watch** | `CMD:START_WORKOUT` / `CMD:WORKOUT_STOP` | Bắt đầu / kết thúc phiên tập luyện |
| **App $\rightarrow$ Watch** | `CMD:CANCEL_SOS` | Hủy trạng thái báo động khẩn cấp SOS từ điện thoại |
| **App $\rightarrow$ Watch** | `CMD:TEST_SOS` | Giả lập kích hoạt SOS để kiểm thử quy trình |
| **Watch $\rightarrow$ App** | `SOS:COUNTDOWN,<sec>\n` | Đếm ngược thời gian chờ trước khi gửi thông báo khẩn cấp |
| **Watch $\rightarrow$ App** | `SOS:CONFIRMED,FALL\n` | Xác nhận chính thức sự kiện té ngã sau khi hết thời gian đếm |
| **Watch $\rightarrow$ App** | `SOS:ALERT,FALL,ACTIVE,...` | Luồng dữ liệu sinh hiệu khẩn cấp định kỳ kèm nhịp tim & SpO2 |
| **Watch $\rightarrow$ App** | `SOS:CANCELLED[,reason]\n` | Báo hủy SOS (lý do: `BUTTON`, `NO_WEAR` hoặc từ app) |
| **Watch $\rightarrow$ App** | `W:<timestamp>,<bpm>,<steps>\n`| Báo cáo nhịp tim và bước chân định kỳ trong chế độ tập luyện |

---

## 🚀 Hướng dẫn nạp Firmware

### Cách 1: Nạp siêu tốc bằng kéo-thả file `.uf2` (Khuyên dùng)
Bạn không cần mở Arduino IDE hay cài đặt thư viện:
1. Kết nối XIAO nRF52840 Sense với máy tính qua cáp USB Type-C.
2. Dùng nhíp hoặc que nhỏ **bấm nhanh nút RESET trên mạch 2 lần liên tiếp** (Double-click).
3. Máy tính sẽ xuất hiện ổ đĩa USB mới mang tên **`XIAO-SENSE`** (hoặc `XIAO-BLE`).
4. Kéo thả file **[`HealthSense-Firmware-XIAO-nRF52840.uf2`](HealthSense-Firmware-XIAO-nRF52840.uf2)** vào ổ đĩa đó.
5. Ổ đĩa sẽ tự đóng và mạch tự động khởi động chạy firmware mới trong vòng 2 giây!

### Cách 2: Biên dịch qua Arduino IDE
1. Cài đặt **Arduino IDE 2.x**.
2. Thêm URL Bo mạch Seeed Studio vào *Preferences $\rightarrow$ Additional Board Manager URLs*:
   ```text
   https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   ```
3. Vào *Boards Manager*, tìm và cài đặt gói: **Seeeduino nRF52 Boards** (chọn bo mạch: **Seeed XIAO nRF52840 Sense**).
4. Cài đặt các thư viện cần thiết qua Library Manager:
   - `Seeed Arduino LSM6DS3`
   - `SparkFun MAX3010x Pulse and Proximity Sensor Library`
   - `Adafruit Bluefruit nRF52 Libraries`
5. Mở file [`HealthSense-Firmware-XIAO-nRF52840.ino`](HealthSense-Firmware-XIAO-nRF52840.ino), chọn cổng COM tương ứng và bấm **Upload**.

---

## 📂 Cấu trúc mã nguồn

```text
├── HealthSense-Firmware-XIAO-nRF52840.ino   # File sketch chính (setup, loop & điều phối sự kiện)
├── HealthSense-Firmware-XIAO-nRF52840.uf2   # File binary nạp sẵn dạng USB Drag & Drop
├── AccelManager.h / .cpp                    # Trình điều khiển LSM6DS3TR-C, đếm bước & FSM té ngã 3 giai đoạn
├── PPGManager.h / .cpp                      # Trình điều khiển MAX30101, thuật toán nhịp tim, SpO2 & dò đeo tay
├── BLEManager.h / .cpp                      # Quản lý Bluetooth Low Energy GATT, kết nối & truyền nhận gói tin
├── DeviceStateManager.h / .cpp              # Máy trạng thái hữu hạn (FSM), xử lý chuỗi khẩn cấp SOS & quản lý nguồn
├── DisplayPower.h / .cpp                    # Trình điều khiển đèn LED RGB báo trạng thái
└── .context/                                # Tài liệu kỹ thuật kiến trúc, sơ đồ chân & hợp đồng giao thức BLE
```

---

## 👥 Nhóm phát triển
Đồ án tốt nghiệp / Dự án hệ thống chăm sóc sức khỏe thông minh **HealthSense** — **IUH**.
