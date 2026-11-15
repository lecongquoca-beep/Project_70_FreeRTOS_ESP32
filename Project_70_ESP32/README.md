# Project Cá Nhân (70%) – Hệ Thống Nhúng Đa Nhiệm Thời Gian Thực FreeRTOS trên ESP32

- **Sinh viên thực hiện:** Lê Công Quốc Anh — **MSSV:** `24119009`
- **Môn học:** Real-Time Operating Systems (Hệ điều hành thời gian thực)
- **Giảng viên hướng dẫn:** ThS. Huỳnh Hoàng Hà
- **Đơn vị:** Khoa Điện – Điện tử, Trường Đại học Sư phạm Kỹ thuật TP.HCM (HCMUTE)
- **Báo cáo chi tiết:** [`Report_RTOS_ESP32_Project70_v7.docx`](./Report_RTOS_ESP32_Project70_v7.docx)

---

## 1. Tổng quan dự án

Dự án thiết kế và triển khai một nút điều khiển và giám sát thời gian thực đa nhiệm trên vi điều khiển lõi kép **ESP32-WROOM-32** sử dụng nhân **SMP FreeRTOS**. Hệ thống gồm **8 Task** và **1 Software Timer** được phân tải bất đối xứng trên hai nhân CPU (`Core 0` và `Core 1`), minh họa **12 cơ chế cốt lõi của RTOS** (`[K1]`–`[K12]`) nhằm loại bỏ hiện tượng trôi pha thời gian (timing drift), xung đột bus I²C dùng chung và chồng chéo dữ liệu UART.

### Kết quả thực nghiệm trên phần cứng thật (> 1,111 giây chạy liên tục)
| Hạng mục kiểm thử | Kết quả đo thực tế (Serial Monitor & OLED) |
|---|---|
| **Chu kỳ nháy LED1 (Task 1)** | 600 ms (đảo trạng thái mỗi 300 ms qua `vTaskDelayUntil`), độ lệch pha tích lũy **0 ms** |
| **Cảm biến ánh sáng BH1750 (Task 2)** | Bình thường trong phòng: **97.5 – 113.3 lx** (`66% – 68%`); che tay: giảm mượt xuống **50.8 lx** (`56%`) |
| **Phản hồi nút nhấn BTN1 (Task 3)** | Lọc chống dội 30 ms (`INPUT_PULLUP`), đảo trạng thái LED2 tức thì, **0 lỗi nhiễu tiếp điểm** |
| **Đồng bộ Binary Semaphore BTN2 → LED3** | Độ trễ chiếm quyền (preemption) `< 1 ms`; LED3 sáng chuẩn xác **3,000 ms**; bấm dồn báo `da co san` (bão hòa mức 1) |
| **Màn hình OLED SH1106 1.3" (Task 6)** | Làm tươi 500 ms/lần, đồng bộ khởi động bằng `Event Group`, chia sẻ bus I²C với BH1750 qua `Mutex` không treo bus |
| **Gatekeeper Queue & Heartbeat (Task 7)** | Hàng đợi 16 phần tử (`g_logQueue`), chu kỳ heartbeat **5,000 ms**, số gói tin bị rơi **`drop = 0`** |
| **Tài nguyên bộ nhớ (Task 8 Monitor)** | Flash: **304,032 B** · SRAM Heap trống: **318,512 B** · Stack trống tối thiểu mỗi task: **≥ 1,252 B** (mức dùng `< 60%`) |

---

## 2. Sơ đồ kết nối phần cứng (ESP32-WROOM-32 DevKitC 38 chân)

Mạch sử dụng bo mạch mở rộng Domino (Screw-Terminal Shield) kết nối với ngoại vi qua bus I²C 100 kHz và các chân GPIO:

| Linh kiện | Chân ESP32 (`config.h`) | Chế độ / Giao tiếp | Chức năng trong hệ thống |
|---|---|---|---|
| **LED1** (+ điện trở 220 Ω – 1 kΩ) | `GPIO25` | Digital Output | Nháy định kỳ chu kỳ `600 ms` (Task 1) |
| **LED2** (+ điện trở 220 Ω – 1 kΩ) | `GPIO26` | Digital Output | Đảo trạng thái ON/OFF mỗi lần nhấn BTN1 (Task 3) |
| **LED3** (+ điện trở 220 Ω – 1 kΩ) | `GPIO27` | Digital Output | Sáng đúng `3,000 ms` khi nhận Binary Semaphore từ BTN2 (Task 5) |
| **Nút nhấn BTN1** (xanh) | `GPIO18` | `INPUT_PULLUP` (nối GND) | Điều khiển bật/tắt LED2, chống dội phần mềm `30 ms` (Task 3) |
| **Nút nhấn BTN2** (trắng) | `GPIO19` | `INPUT_PULLUP` (nối GND) | Cấp token vào `g_btn2Sem` đánh thức Task 5 (Task 4) |
| **Nút BOOT** (có sẵn trên kit) | `GPIO0` | `INPUT_PULLUP` + Ngắt ngoài (`FALLING`) | Kích hoạt ISR đánh thức Task 8 in thống kê Heap & Stack HWM |
| **Cảm biến ánh sáng BH1750** | `SDA = GPIO21`, `SCL = GPIO22` | I²C (`0x23`, chân `ADO` nối `GND`) | Đo độ rọi (lux) và quy đổi logarit ra `%` mỗi `500 ms` (Task 2) |
| **Màn hình OLED SH1106 1.3"** | `SDA = GPIO21`, `SCL = GPIO22` | I²C (`0x3C`, `128x64`) | Hiển thị trạng thái LED1/2/3, lux, `%`, đếm ngược 3.0s và `seq` (Task 6) |

*(Ghi chú: Các chân mở rộng `PIN_TEMP = GPIO35` và `PIN_RGB = GPIO16/17/5` đã được khai báo sẵn trong `include/config.h` cho giai đoạn phát triển tiếp theo).*

---

## 3. Kiến trúc phần mềm & Phân bổ Task trên 2 nhân

| Task | Tên hàm | Nhân CPU | Priority | Stack | Chu kỳ / Sự kiện | Cơ chế FreeRTOS áp dụng |
|---|---|---|---|---|---|---|
| **Task 1** | `task1_led1` | Core 1 | 1 | 2,048 B | Chu kỳ `600 ms` (đảo mỗi `300 ms`) | `vTaskDelayUntil()` (`[K2]`) |
| **Task 2** | `task2_sensor` | Core 0 | 2 | 3,072 B | Chu kỳ `500 ms` | `g_i2cMutex`, `g_sensorMutex` (`[K5]`), `xEventGroupSetBits` (`[K7]`), tự khởi tạo lại khi cắm nóng |
| **Task 3** | `task3_btn1` | Core 1 | 3 | 3,072 B | Quét mỗi `10 ms` | Chống dội phím `30 ms`, `vTaskDelay()` (`[K3]`) |
| **Task 4** | `task4_btn2` | Core 1 | 3 | 3,072 B | Quét mỗi `10 ms` | `xSemaphoreGive(g_btn2Sem)` (`[K4]`), `portENTER_CRITICAL` (`[K9]`) |
| **Task 5** | `task5_led3` | Core 1 | **4** | 3,072 B | Chờ sự kiện (`portMAX_DELAY`) | `xSemaphoreTake(g_btn2Sem)` (`[K4]`), ưu tiên cao nhất phản hồi `< 1 ms` |
| **Task 6** | `task6_oled` | Core 0 | 1 | 4,096 B | Chu kỳ `500 ms` | `xEventGroupWaitBits` (`[K7]`), `g_i2cMutex`, `g_sensorMutex` (`[K5]`) |
| **Task 7** | `task7_logger` | Core 0 | 1 | 3,072 B | Chờ hàng đợi (`portMAX_DELAY`) | Mẫu thiết kế **Gatekeeper** độc quyền xuất UART qua `g_logQueue` (`[K6]`) |
| **Task 8** | `task8_monitor` | Core 0 | 2 | 4,096 B | Chờ thông báo từ `isr_boot()` | **Deferred ISR** (`vTaskNotifyGiveFromISR`, `[K8]`), `uxTaskGetStackHighWaterMark()` (`[K11]`) |
| **Timer** | `heartbeat_cb` | Timer Svc | – | – | Auto-reload `5,000 ms` | `xTimerCreate()` (`[K10]`) báo `uptime`, `btn2Count`, `drop` |
| **Setup** | `loop()` | – | – | – | Chạy 1 lần lúc khởi động | `vTaskDelete(NULL)` (`[K12]`) giải phóng ~8 KB SRAM của `loopTask` |

---

## 4. Cấu trúc thư mục dự án

```text
├── include/
│   └── config.h                # Định nghĩa chân GPIO, địa chỉ I2C, chu kỳ, Priority và Stack
├── src/
│   └── main.cpp                # Mã nguồn chính triển khai 8 Task, ISR, Software Timer & IPC
├── platformio.ini              # Cấu hình build PlatformIO (espressif32 @ 6.9.0) & thư viện
├── Report_RTOS_ESP32_Project70_v7.docx  # Báo cáo tiểu luận hoàn chỉnh
└── README.md                   # Tài liệu hướng dẫn dự án