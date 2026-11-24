#pragma once
#include <Arduino.h>

// ---- Chân kết nối ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ESP32-S3 DevKitC-1 (N8R2/N16R8)
#define PIN_LED1       4
#define PIN_LED2       5
#define PIN_LED3       6
#define PIN_BTN1      15   // nút nhấn nối GND, dùng INPUT_PULLUP
#define PIN_BTN2      16
#define PIN_BTN_BOOT   0   // nút BOOT có sẵn trên kit -> ngắt (ISR) cho task Monitor
#define PIN_SDA       10
#define PIN_SCL       11
// Chân mở rộng cho ESP32-S3
#define PIN_TEMP       2
#define PIN_RGB_R      7
#define PIN_RGB_G      8
#define PIN_RGB_B      9
#else
// ESP32 NodeMCU / DevKitC 38 chân (ESP32-WROOM-32)
// --- Hàng chân bên TRÁI bo mạch (cụm LED & Cảm biến mở rộng) ---
#define PIN_LED1      25   // G25 -> Điện trở 220Ω - 1kΩ -> LED1 -> GND
#define PIN_LED2      26   // G26 -> Điện trở 220Ω - 1kΩ -> LED2 -> GND
#define PIN_LED3      27   // G27 -> Điện trở 220Ω - 1kΩ -> LED3 -> GND
#define PIN_TEMP      35   // G35 (ADC1_CH7): Cảm biến nhiệt độ (mở rộng)

// --- Hàng chân bên PHẢI bo mạch (cụm Bus I2C dùng chung OLED + BH1750, Nút nhấn & LED RGB) ---
#define PIN_SDA       21   // G21 -> Chân SDA dùng chung (OLED + BH1750)
#define PIN_SCL       22   // G22 -> Chân SCL dùng chung (OLED + BH1750)
#define PIN_BTN1      18   // G18 -> Nút nhấn 1 -> GND (dùng INPUT_PULLUP)
#define PIN_BTN2      19   // G19 -> Nút nhấn 2 -> GND (dùng INPUT_PULLUP)
#define PIN_BTN_BOOT   0   // G0  -> Nút BOOT có sẵn trên bo mạch (ISR Monitor)
#define PIN_RGB_R     16   // G16 -> Điện trở -> Chân R của LED RGB
#define PIN_RGB_G     17   // G17 -> Điện trở -> Chân G của LED RGB
#define PIN_RGB_B      5   // G5  -> Điện trở -> Chân B của LED RGB
#endif

// ---- Địa chỉ các thiết bị trên Bus I2C ----
#define OLED_ADDR     0x3C // Địa chỉ màn hình OLED SH1106
#define BH1750_ADDR   0x23 // Địa chỉ cảm biến ánh sáng BH1750 (ADO nối GND)

// ---- Thang đo cảm biến ánh sáng ----
#define LUX_FULL      1000.0f   // Độ rọi (lux) quy ước ứng với mức 100%

// ---- Chu kỳ các task (ms) ----
#define T_LED1_MS        600   // Task 1: chu kỳ nháy 600ms (mỗi 300ms đảo trạng thái 1 lần)
#define T_SENSOR_MS      500   // Task 2: chu kỳ đọc cảm biến BH1750
#define T_BTN_POLL_MS     10   // Task 3, Task 4: chu kỳ quét nút nhấn
#define T_DEBOUNCE_MS     30   // Thời gian chống dội phím (ms)
#define T_LED3_ON_MS    3000   // Task 5: thời gian giữ sáng LED3 khi nhận Semaphore
#define T_OLED_MS        500   // Task 6: chu kỳ cập nhật màn hình OLED
#define T_TEMP_MS       1000   // Task mở rộng: đọc cảm biến nhiệt độ & cập nhật LED RGB
#define T_HEARTBEAT_MS  5000   // Software timer heartbeat (5 giây/lần)

// ---- Priority: số lớn = ưu tiên cao (FreeRTOS) ----
#define PRIO_LED1     1
#define PRIO_SENSOR   2
#define PRIO_TEMP     2
#define PRIO_BTN1     3
#define PRIO_BTN2     3
#define PRIO_LED3     4   // Phản hồi Binary Semaphore nhanh nhất
#define PRIO_OLED     1
#define PRIO_LOGGER   1
#define PRIO_MONITOR  2

// ---- Stack (byte, trên ESP32 đơn vị của xTaskCreate là byte) ----
#define STACK_SMALL  2048
#define STACK_OLED   4096
#define STACK_LOG    3072

// ---- Event group bits ----
#define EVT_OLED_READY    (1 << 0)
#define EVT_SENSOR_READY  (1 << 1)
#define EVT_TEMP_READY    (1 << 2)

// ---- Cấu hình mở rộng cho Task 9 (CLI) & Task 10 (Supervisor/PWM) ----
#define T_SUPERVISOR_MS    250    // Chu kỳ giám sát an toàn & cập nhật PWM (250ms)
#define PRIO_CLI           2      // Mức ưu tiên Task nhận lệnh UART
#define PRIO_SUPERVISOR    3      // Mức ưu tiên cao để giám sát an toàn hệ thống
#define STACK_CLI          4096   // Stack cho bộ phân tích cú pháp lệnh
#define STACK_SUPERVISOR   3072   // Stack cho tác vụ giám sát & PWM
#define PWM_CH_LED1        0      // Kênh LEDC PWM 0 gắn vào PIN_LED1 (G25)
#define PWM_FREQ_HZ        5000   // Tần số PWM 5 kHz
#define PWM_RES_BITS       8      // Độ phân giải 8-bit (0 - 255)
#define DEFAULT_LUX_THRESH 30     // Ngưỡng cảnh báo tối mặc định (30%)