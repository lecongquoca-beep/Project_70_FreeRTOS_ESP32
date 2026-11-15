/*
 * ======================================================================================
 * DỰ ÁN CÁ NHÂN: HỆ THỐNG NHÚNG THỜI GIAN THỰC (RTOS - FreeRTOS TRÊN ESP32)
 * ======================================================================================
 * Phần cứng sử dụng:
 *   - Vi điều khiển : ESP32-WROOM-32 (Dual-core Xtensa LX6, chạy FreeRTOS)
 *   - Màn hình OLED  : SH1106 1.3 inch, giao tiếp I2C (địa chỉ 0x3C, SDA=21, SCL=22)
 *   - Cảm biến sáng  : BH1750 (GY-302), giao tiếp I2C dùng chung bus (địa chỉ 0x23)
 *   - Ngoại vi I/O   : 3 LED đơn (G25, G26, G27) + 2 nút nhấn ngoài (G18, G19) + nút BOOT (G0)
 *
 * SƠ ĐỒ ÁNH XẠ KỸ THUẬT FREERTOS -> MÃ NGUỒN (Dùng đối chiếu cho báo cáo):
 *   [K1]  xTaskCreatePinnedToCore, Priority, Stack .... Hàm setup()
 *   [K2]  vTaskDelayUntil (chu kỳ định thời chính xác) . task1_led1, task2_sensor, task6_oled
 *   [K3]  vTaskDelay (nhường CPU sang trạng thái Blocked) task3_btn1, task4_btn2, task5_led3
 *   [K4]  Binary Semaphore (đồng bộ sự kiện giữa 2 task) task4_btn2 -> task5_led3
 *   [K5]  Mutex (khóa tương hỗ bảo vệ dữ liệu & bus I2C) g_sensorMutex, g_i2cMutex
 *   [K6]  Queue + Gatekeeper Task (điều phối cổng Serial) logf(), task7_logger
 *   [K7]  Event Group (cờ đồng bộ trạng thái khởi tạo) . g_sysEvents (EVT_SENSOR_READY, EVT_OLED_READY)
 *   [K8]  ISR + Task Notification (xử lý ngắt trì hoãn) . isr_boot() -> task8_monitor
 *   [K9]  Critical Section (Spinlock cho chip 2 nhân) .. g_mux (bảo vệ biến dùng chung & deadline LED3)
 *   [K10] Software Timer (bộ định thời tự nạp lại) ..... g_heartbeat, heartbeat_cb()
 *   [K11] Giám sát tài nguyên (Stack High Water Mark) .. task8_monitor
 *   [K12] Hủy Task không dùng để thu hồi RAM .......... vTaskDelete(NULL) trong loop()
 * ======================================================================================
 */

#include <Arduino.h>
#include <Wire.h>               // Thư viện giao tiếp chuẩn I2C (SDA, SCL)
#include <BH1750.h>             // Thư viện điều khiển cảm biến cường độ ánh sáng số BH1750
#include <Adafruit_GFX.h>       // Thư viện đồ họa lõi (vẽ chữ, đường thẳng, tọa độ)
#include <Adafruit_SH110X.h>    // Thư viện driver chuyên dụng cho chip màn hình OLED SH1106
#include <freertos/FreeRTOS.h>  // Nhân hệ điều hành thời gian thực FreeRTOS
#include <freertos/task.h>      // API quản lý tác vụ (Task)
#include <freertos/semphr.h>    // API quản lý Semaphore và Mutex
#include <freertos/queue.h>     // API quản lý hàng đợi (Queue)
#include <freertos/event_groups.h> // API quản lý nhóm cờ sự kiện (Event Group)
#include <freertos/timers.h>    // API quản lý bộ định thời phần mềm (Software Timer)
#include "config.h"             // File cấu hình tập trung: sơ đồ chân, chu kỳ, mức ưu tiên, stack
#include <esp_log.h>            // Thư viện kiểm soát mức log hệ thống của ESP-IDF

// ======================================================================================
// 1. CẤU TRÚC DỮ LIỆU & BIẾN TOÀN CỤC DÙNG CHUNG GIỮA CÁC TASK
// ======================================================================================

// Cấu trúc đóng gói dữ liệu đo từ cảm biến ánh sáng BH1750
struct SensorData {
  uint16_t raw;      // Giá trị độ rọi làm tròn nguyên (lux)
  float    lux;      // Độ rọi thực tế đo từ BH1750 (đơn vị: lux)
  uint8_t  percent;  // Mức độ sáng quy đổi sang phần trăm (0% - 100%) theo thang logarit
  uint32_t seq;      // Bộ đếm số lần đọc cảm biến thành công kể từ lúc khởi động
};

// Biến lưu dữ liệu cảm biến: Task 2 ghi vào, Task 6 đọc ra -> Bắt buộc bảo vệ bằng g_sensorMutex [K5]
static SensorData g_sensor = {0, 0.0f, 0, 0};

// Các biến cờ trạng thái 1 byte (bool): chỉ 1 task ghi, các task khác đọc (truy xuất nguyên tử)
static volatile bool g_led1On = false;           // Trạng thái LED1 (Task 1 ghi, Task 2 & Task 6 đọc)
static volatile bool g_led2On = false;           // Trạng thái LED2 (Task 3 ghi, Task 6 đọc)
static volatile bool g_led3On = false;           // Trạng thái LED3 (Task 5 ghi, Task 6 đọc)

// Các biến dùng chung cần truy cập đồng bộ nhiều biến hoặc trên cả 2 nhân CPU -> Bảo vệ bằng Spinlock [K9]
static volatile TickType_t g_led3OffTick = 0;    // Thời điểm (tính theo tick) mà LED3 sẽ tự tắt
static volatile uint32_t   g_btn2Count   = 0;    // Tổng số lần nút BTN2 được nhấn
static volatile uint32_t   g_logDropped  = 0;    // Đếm số bản tin log bị rơi nếu hàng đợi Queue đầy

// Khóa Spinlock (Critical Section) dùng cho kiến trúc Dual-Core ESP32 [K9]
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Các Handle (con trỏ quản lý đối tượng nhân FreeRTOS)
static SemaphoreHandle_t  g_btn2Sem;     // [K4] Binary Semaphore: báo hiệu từ Task 4 (BTN2) sang Task 5 (LED3)
static SemaphoreHandle_t  g_sensorMutex; // [K5] Mutex: ngăn xung đột đọc/ghi cấu trúc g_sensor
static SemaphoreHandle_t  g_i2cMutex;    // [K5] Mutex: khóa bus I2C (G21/G22) vì BH1750 và OLED dùng chung dây
static QueueHandle_t      g_logQueue;    // [K6] Queue: hàng đợi chứa tối đa 16 bản tin log gửi ra Serial
static EventGroupHandle_t g_sysEvents;   // [K7] Event Group: lưu các bit trạng thái sẵn sàng của hệ thống
static TimerHandle_t      g_heartbeat;   // [K10] Software Timer: bộ đếm định kỳ báo hệ thống còn sống

// Handle của 8 Task trong hệ thống (dùng để giám sát bộ nhớ Stack và gửi thông báo từ ngắt)
static TaskHandle_t h_led1, h_sensor, h_btn1, h_btn2, h_led3, h_oled, h_logger, h_monitor;

// Đối tượng điều khiển phần cứng ngoại vi
static Adafruit_SH1106G oled(128, 64, &Wire, -1); // Màn hình OLED 128x64, không dùng chân Reset cứng (-1)
static BH1750 lightMeter(0x23);                   // Cảm biến ánh sáng BH1750 ở địa chỉ 0x23 (chân ADO nối GND)
static bool g_bh1750Ok = false;                   // Cờ đánh dấu cảm biến BH1750 đã khởi tạo thành công chưa

// ======================================================================================
// 2. HỆ THỐNG GHI LOG AN TOÀN QUA QUEUE - MẪU THIẾT KẾ GATEKEEPER [K6]
// ======================================================================================
// Thay vì để nhiều task cùng gọi Serial.print() gây lẫn lộn ký tự trên Terminal,
// mọi task đều đẩy bản tin vào g_logQueue; chỉ duy nhất Task 7 (logger) lấy ra và in.

struct LogMsg {
  TickType_t tick;  // Mốc thời gian (tính theo OS tick) tại thời điểm tạo bản tin
  char text[56];    // Nội dung thông điệp đã được định dạng (tối đa 55 ký tự + '\0')
};

// Hàm tiện ích giống printf(), có thể gọi từ bất kỳ Task nào (lưu ý: không gọi từ hàm ngắt ISR)
static void logf(const char *fmt, ...) {
  LogMsg m;
  m.tick = xTaskGetTickCount(); // Lấy thời gian hệ thống hiện tại

  // Định dạng chuỗi ký tự từ danh sách tham số biến đổi (...)
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.text, sizeof(m.text), fmt, ap);
  va_end(ap);

  // Gửi bản tin vào cuối hàng đợi với thời gian chờ = 0 (không làm treo/chậm task đang gọi)
  if (xQueueSend(g_logQueue, &m, 0) != pdPASS) {
    // Nếu Queue đang đầy (16 bản tin chưa kịp in), tăng biến đếm bản tin bị bỏ qua
    portENTER_CRITICAL(&g_mux);
    g_logDropped = g_logDropped + 1;
    portEXIT_CRITICAL(&g_mux);
  }
}

// ======================================================================================
// 3. TASK 1: ĐIỀU KHIỂN LED1 NHẤP NHÁY THEO CHU KỲ CỐ ĐỊNH [K2]
// ======================================================================================
static void task1_led1(void *arg) {
  TickType_t last = xTaskGetTickCount();              // Mốc thời gian bắt đầu chu kỳ
  const TickType_t half = pdMS_TO_TICKS(T_LED1_MS / 2); // Nửa chu kỳ bật, nửa chu kỳ tắt
  bool on = false;

  for (;;) {
    on = !on;                       // Đảo trạng thái logic (tắt -> bật, bật -> tắt)
    g_led1On = on;                  // Cập nhật vào biến toàn cục để Task 2 & Task 6 hiển thị
    digitalWrite(PIN_LED1, on);     // Xuất mức điện áp HIGH/LOW ra chân G25

    // [K2] vTaskDelayUntil đảm bảo tần số nháy chính xác tuyệt đối, không bị trôi pha theo thời gian
    vTaskDelayUntil(&last, half);
  }
}

// ======================================================================================
// 4. TASK 2: ĐỌC CẢM BIẾN ÁNH SÁNG BH1750 QUA BUS I2C [K2, K5, K7]
// ======================================================================================

// Hàm quy đổi độ rọi (lux) sang phần trăm (0 - 100%) theo thang logarit
// Lý do: Mắt người cảm nhận độ sáng theo hàm logarit (từ 1 lux trong tối tới LUX_FULL = 100%)
static uint8_t lux_to_percent(float lux) {
  if (lux <= 1.0f) return 0;
  float p = log10f(lux) / log10f(LUX_FULL) * 100.0f;
  return (uint8_t)(p > 100.0f ? 100.0f : p);
}

static void task2_sensor(void *arg) {
  TickType_t last = xTaskGetTickCount();
  uint32_t nlog = 0; // Bộ đếm số vòng lặp để giảm tần suất in log (2 lần đọc mới in 1 lần)

  for (;;) {
    float lux = -1.0f;

    // [K5] Xin giữ khóa Mutex của bus I2C trước khi giao tiếp với cảm biến BH1750
    // Mục đích: Đảm bảo Task 6 (OLED) không truyền dữ liệu I2C cùng lúc gây treo bus
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!g_bh1750Ok) {
        // Nếu cảm biến chưa sẵn sàng (hoặc mới cắm lại dây), thử khởi tạo ở chế độ đo phân giải cao
        g_bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      }
      if (g_bh1750Ok) {
        lux = lightMeter.readLightLevel(); // Đọc giá trị độ rọi (đơn vị: lux) từ chip BH1750
        if (lux < 0.0f) {
          // Thư viện trả về số âm khi mất kết nối I2C -> Đánh dấu để tự động khởi tạo lại ở chu kỳ sau
          g_bh1750Ok = false;
        }
      }
      xSemaphoreGive(g_i2cMutex); // Trả lại khóa bus I2C ngay sau khi đọc xong
    }

    if (lux >= 0.0f) {
      uint8_t pct = lux_to_percent(lux);
      uint16_t rawLux = (uint16_t)(lux > 65535.0f ? 65535 : lux);

      // Cứ 2 chu kỳ đọc (500ms x 2 = 1000ms) thì gửi 1 bản tin trạng thái ánh sáng + LED1 ra Terminal
      if ((++nlog & 1) == 0) {
        logf("BH1750: %.1f lx (%u%%) | LED1:%s", lux, pct, g_led1On ? "ON" : "OFF");
      }

      // [K5] Xin khóa g_sensorMutex để cập nhật bộ dữ liệu g_sensor một cách nhất quán
      if (xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor.raw = rawLux;
        g_sensor.lux = lux;
        g_sensor.percent = pct;
        g_sensor.seq++;
        xSemaphoreGive(g_sensorMutex); // Giải phóng Mutex cho Task 6 đọc
      }

      // [K7] Bật cờ EVT_SENSOR_READY báo hiệu cho Task 6 (OLED) biết đã có dữ liệu hợp lệ đầu tiên
      xEventGroupSetBits(g_sysEvents, EVT_SENSOR_READY);
    } else {
      // Báo lỗi ra Terminal nếu dây cảm biến BH1750 bị lỏng hoặc chưa cắm đúng địa chỉ 0x23
      if ((++nlog & 1) == 0) {
        logf("LOI: Khong doc duoc BH1750 (0x23) | LED1:%s", g_led1On ? "ON" : "OFF");
      }
    }

    // [K2] Chờ đúng chu kỳ T_SENSOR_MS (500ms) rồi thực hiện lần đo kế tiếp
    vTaskDelayUntil(&last, pdMS_TO_TICKS(T_SENSOR_MS));
  }
}

// ======================================================================================
// 5. BỘ LỌC CHỐNG DỘI PHÍM CƠ HỌC (SOFTWARE DEBOUNCE) CHO CÁC NÚT NHẤN
// ======================================================================================
struct Debounce {
  uint8_t pin;         // Chân GPIO nối với nút nhấn
  bool stable;         // Trạng thái logic đã ổn định (HIGH = đang nhả do dùng INPUT_PULLUP)
  bool lastRead;       // Trạng thái vừa đọc được ở lần quét trước đó
  TickType_t since;    // Mốc thời gian bắt đầu phát hiện sự thay đổi mức logic
};

// Hàm trả về true ĐÚNG 1 LẦN DUY NHẤT mỗi khi người dùng nhấn nút xuống (cạnh xuống HIGH -> LOW)
static bool pressed_edge(Debounce &d) {
  bool r = digitalRead(d.pin);
  TickType_t now = xTaskGetTickCount();

  // Nếu tín hiệu đang dao động (nhiễu cơ học khi tiếp điểm chạm nhau), cập nhật lại mốc thời gian
  if (r != d.lastRead) {
    d.lastRead = r;
    d.since = now;
  }

  // Chỉ xác nhận trạng thái mới khi tín hiệu giữ ổn định liên tục quá T_DEBOUNCE_MS (30ms)
  if ((now - d.since) >= pdMS_TO_TICKS(T_DEBOUNCE_MS) && r != d.stable) {
    d.stable = r;
    return r == LOW; // Nút nhấn nối GND nên mức LOW tương ứng với hành động nhấn xuống
  }
  return false;
}

// ======================================================================================
// 6. TASK 3: ĐỌC NÚT NHẤN BTN1 -> ĐẢO TRẠNG THÁI BẬT/TẮT LED2 [K3]
// ======================================================================================
static void task3_btn1(void *arg) {
  Debounce d = {PIN_BTN1, HIGH, HIGH, 0};
  for (;;) {
    if (pressed_edge(d)) {
      g_led2On = !g_led2On;             // Đảo trạng thái LED2
      digitalWrite(PIN_LED2, g_led2On); // Xuất tín hiệu ra chân G26
      logf("BTN1 -> LED2 %s", g_led2On ? "ON" : "OFF");
    }
    // [K3] Đưa Task vào trạng thái Blocked trong 10ms để nhường CPU cho các Task ưu tiên thấp hơn
    vTaskDelay(pdMS_TO_TICKS(T_BTN_POLL_MS));
  }
}

// ======================================================================================
// 7. TASK 4: ĐỌC NÚT NHẤN BTN2 -> CẤP TÍN HIỆU BINARY SEMAPHORE CHO TASK 5 [K3, K4, K9]
// ======================================================================================
static void task4_btn2(void *arg) {
  Debounce d = {PIN_BTN2, HIGH, HIGH, 0};
  for (;;) {
    if (pressed_edge(d)) {
      // [K4] Phát "token" vào Binary Semaphore để đánh thức Task 5 đang ngủ chờ
      // Nếu Task 5 đang chạy dở (chưa hết 3s) và semaphore đã có sẵn token, hàm trả về pdFALSE
      BaseType_t ok = xSemaphoreGive(g_btn2Sem);

      // [K9] Cập nhật biến đếm số lần nhấn trong vùng găng (Critical Section)
      portENTER_CRITICAL(&g_mux);
      g_btn2Count = g_btn2Count + 1;
      portEXIT_CRITICAL(&g_mux);

      logf("BTN2 give sem: %s", ok == pdTRUE ? "OK" : "da co san");
    }
    vTaskDelay(pdMS_TO_TICKS(T_BTN_POLL_MS));
  }
}

// ======================================================================================
// 8. TASK 5: CHỜ SEMAPHORE TỪ BTN2 -> BẬT SÁNG LED3 TRONG 3 GIÂY [K3, K4, K9]
// ======================================================================================
static void task5_led3(void *arg) {
  for (;;) {
    // [K4] Ngủ vô thời hạn (Blocked, không tiêu tốn 0% CPU) cho tới khi nhận được Semaphore từ Task 4
    xSemaphoreTake(g_btn2Sem, portMAX_DELAY);

    // Tính trước mốc thời gian tắt LED3 (hiện tại + 3000ms) để Task 6 hiển thị đếm ngược lên OLED
    TickType_t off = xTaskGetTickCount() + pdMS_TO_TICKS(T_LED3_ON_MS);

    // [K9] Cập nhật đồng thời cả 2 biến g_led3OffTick và g_led3On để Task 6 không đọc phải dữ liệu lệch pha
    portENTER_CRITICAL(&g_mux);
    g_led3OffTick = off;
    g_led3On = true;
    portEXIT_CRITICAL(&g_mux);

    digitalWrite(PIN_LED3, HIGH); // Bật sáng LED3 (chân G27)
    logf("LED3 ON 3s");

    // Giữ sáng LED3 trong 3 giây (trong lúc này Task ngủ Blocked, CPU vẫn chạy các Task khác bình thường)
    vTaskDelay(pdMS_TO_TICKS(T_LED3_ON_MS));

    digitalWrite(PIN_LED3, LOW);  // Tắt LED3 sau khi hết 3 giây
    portENTER_CRITICAL(&g_mux);
    g_led3On = false;
    portEXIT_CRITICAL(&g_mux);
    logf("LED3 OFF, cho tiep");
  }
}

// ======================================================================================
// 9. HÀM KHỞI TẠO & DÒ ĐỊA CHỈ MÀN HÌNH OLED SH1106 TRÊN BUS I2C
// ======================================================================================
static uint8_t g_oledAddr = OLED_ADDR; // Mặc định 0x3C

// Lưu ý: Hàm này thao tác trực tiếp trên bus I2C nên chỉ gọi trong setup() hoặc khi đã giữ g_i2cMutex
static bool oled_init_locked(bool verbose) {
  uint8_t foundAddr = 0;
  int count = 0;

  // Quét toàn bộ dải địa chỉ I2C 7-bit (từ 0x01 đến 0x7E)
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { // Thiết bị phản hồi ACK
      count++;
      if (addr == 0x3C || addr == 0x3D) foundAddr = addr; // Nhận diện địa chỉ OLED
      if (verbose) Serial.printf("[I2C] Tim thay thiet bi tai: 0x%02X\n", addr);
    }
  }

  if (verbose) {
    Serial.printf("[I2C] SDA=%d SCL=%d: %d thiet bi (OLED=0x%02X)\n", PIN_SDA, PIN_SCL, count, foundAddr);
  }

  if (foundAddr == 0) return false;              // Không tìm thấy màn hình OLED trên bus
  if (!oled.begin(foundAddr, true)) return false; // Khởi tạo bộ điều khiển màn hình SH1106

  g_oledAddr = foundAddr;
  oled.clearDisplay();                 // Xóa sạch bộ nhớ đệm khung hình (RAM nội của màn hình)
  oled.setTextColor(SH110X_WHITE);     // Thiết lập màu chữ sáng (đơn sắc)
  oled.setTextWrap(false);             // Không tự động xuống dòng khi chữ chạm mép phải
  oled.setContrast(120);               // Đặt độ tương phản mức vừa (120/255) giúp màn sáng êm, tránh sụt áp
  oled.display();                      // Đẩy dữ liệu từ RAM ra hiển thị trên tấm nền OLED
  return true;
}

// ======================================================================================
// 10. TASK 6: HIỂN THỊ TRẠNG THÁI TOÀN HỆ THỐNG LÊN MÀN HÌNH OLED [K2, K5, K7, K9]
// ======================================================================================
static void task6_oled(void *arg) {
  // [K7] Đợi Task 2 đọc xong mẫu cảm biến đầu tiên (EVT_SENSOR_READY) mới bắt đầu vẽ màn hình
  xEventGroupWaitBits(g_sysEvents, EVT_SENSOR_READY, pdFALSE, pdTRUE, portMAX_DELAY);

  bool haveOled = xEventGroupGetBits(g_sysEvents) & EVT_OLED_READY;
  uint8_t retry = 0, oledFail = 0;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // Bước 1: Sao chép nhanh dữ liệu cảm biến ra biến cục bộ dưới sự bảo vệ của g_sensorMutex [K5]
    SensorData s;
    if (xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      s = g_sensor;
      xSemaphoreGive(g_sensorMutex);
    } else {
      s = {0, 0, 0, 0};
    }

    // Bước 2: Đọc an toàn trạng thái và thời điểm tắt của LED3 thông qua Critical Section [K9]
    bool led3;
    TickType_t offTick;
    portENTER_CRITICAL(&g_mux);
    led3 = g_led3On;
    offTick = g_led3OffTick;
    portEXIT_CRITICAL(&g_mux);

    // Tính thời gian còn lại (mili-giây) trước khi LED3 tắt để hiển thị đếm ngược
    int32_t remainMs = 0;
    if (led3) {
      int32_t d = (int32_t)(offTick - xTaskGetTickCount());
      remainMs = d > 0 ? d * portTICK_PERIOD_MS : 0;
    }

    // Bước 3: Xin giữ khóa bus I2C [K5] để vẽ khung hình mới lên OLED (tránh đụng độ với BH1750)
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (haveOled) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SH110X_WHITE);

        // Dòng tiêu đề trên cùng
        oled.setCursor(0, 0);
        oled.println("RTOS ESP32 - Quoc");
        oled.drawFastHLine(0, 10, 128, SH110X_WHITE); // Vẽ đường kẻ ngang phân cách

        // Dòng 1: Hiển thị trạng thái LED1 và LED2
        oled.setCursor(0, 16);
        oled.printf("LED1:%s  LED2:%s\n", g_led1On ? "ON" : "OFF", g_led2On ? "ON" : "OFF");

        // Dòng 2: Hiển thị phần trăm ánh sáng và độ rọi lux từ BH1750
        oled.setCursor(0, 28);
        oled.printf("Light: %u%% (%.1f lx)\n", s.percent, s.lux);

        // Dòng 3: Hiển thị trạng thái LED3 kèm đồng hồ đếm ngược 3.0s -> 0.0s
        oled.setCursor(0, 40);
        if (led3) oled.printf("LED3 : ON  %.1fs\n", remainMs / 1000.0f);
        else      oled.println("LED3 : OFF");

        // Dòng 4: Hiển thị số thứ tự mẫu đo và địa chỉ I2C của màn hình
        oled.setCursor(0, 54);
        oled.printf("seq=%lu addr=0x%02X", (unsigned long)s.seq, g_oledAddr);

        oled.display(); // Truyền toàn bộ khung hình 128x64 pixel qua I2C ra màn hình

        // Cơ chế tự phục hồi nóng (Hot-plug): kiểm tra xem màn hình có còn phản hồi ACK không
        Wire.beginTransmission(g_oledAddr);
        if (Wire.endTransmission() != 0) {
          if (++oledFail >= 6) { // Nếu lỗi liên tiếp 6 lần (3 giây) -> đánh dấu mất kết nối để dò lại
            oledFail = 0;
            haveOled = false;
            xEventGroupClearBits(g_sysEvents, EVT_OLED_READY);
            logf("[OLED] Mat ket noi, dang thu lai...");
          }
        } else {
          oledFail = 0;
        }
      } else {
        // Nếu trước đó chưa cắm OLED, cứ mỗi 5 giây (10 chu kỳ x 500ms) sẽ tự động quét lại bus I2C
        if (++retry >= 10) {
          retry = 0;
          if (oled_init_locked(false)) {
            haveOled = true;
            xEventGroupSetBits(g_sysEvents, EVT_OLED_READY);
            logf("[OLED] Da ket noi lai 0x%02X", g_oledAddr);
          }
        }
      }
      xSemaphoreGive(g_i2cMutex); // Trả lại khóa bus I2C sau khi vẽ xong
    }

    // [K2] Cập nhật màn hình đều đặn mỗi 500ms (2 khung hình/giây)
    vTaskDelayUntil(&last, pdMS_TO_TICKS(T_OLED_MS));
  }
}

// ======================================================================================
// 11. TASK 7: GATEKEEPER LOGGER - ĐỘC QUYỀN XUẤT DỮ LIỆU RA CỔNG SERIAL [K6]
// ======================================================================================
static void task7_logger(void *arg) {
  LogMsg m;
  for (;;) {
    // Ngủ chờ cho tới khi có bản tin mới trong g_logQueue, sau đó in ra Terminal kèm timestamp và số nhân CPU
    if (xQueueReceive(g_logQueue, &m, portMAX_DELAY) == pdPASS) {
      Serial.printf("[%7lu ms][c%d] %s\n",
                    (unsigned long)(m.tick * portTICK_PERIOD_MS), xPortGetCoreID(), m.text);
    }
  }
}

// ======================================================================================
// 12. NGẮT NGOẠI VI (ISR) & TASK 8: GIÁM SÁT TÀI NGUYÊN HỆ THỐNG [K8, K11]
// ======================================================================================

// Trình phục vụ ngắt (ISR) gắn vào nút BOOT (G0), đặt trong bộ nhớ nhanh IRAM để phản hồi tức thì
static void IRAM_ATTR isr_boot() {
  BaseType_t woken = pdFALSE;
  // [K8] Mô hình Deferred Interrupt Processing: ISR không tự in log nặng nề mà chỉ gửi thông báo
  // (Task Notification) siêu nhẹ để đánh thức Task 8 xử lý phần việc nặng ở ngữ cảnh Task
  vTaskNotifyGiveFromISR(h_monitor, &woken);
  if (woken) portYIELD_FROM_ISR(); // Chuyển ngữ cảnh ngay nếu Task 8 có độ ưu tiên cao hơn Task đang chạy
}

// Hàm phụ trợ in thông tin mức ưu tiên và dung lượng Stack trống tối thiểu (High Water Mark) của từng Task [K11]
static void print_task(const char *name, TaskHandle_t h) {
  logf("%-8s prio=%u stackFree=%u B", name, (unsigned)uxTaskPriorityGet(h),
       (unsigned)uxTaskGetStackHighWaterMark(h));
}

static void task8_monitor(void *arg) {
  for (;;) {
    // Chờ tín hiệu thông báo từ ngắt nút BOOT (isr_boot)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(50)); // Chống dội phím 50ms
    ulTaskNotifyTake(pdTRUE, 0);   // Xóa các xung nhiễu dội phát sinh thêm trong lúc chờ 50ms

    // [K11] Thống kê tổng số Task đang hoạt động, dung lượng RAM Heap trống và Stack dư của 8 Task
    logf("=== MONITOR: %u tasks, heap %u B ===",
         (unsigned)uxTaskGetNumberOfTasks(), (unsigned)ESP.getFreeHeap());
    print_task("led1", h_led1);
    print_task("sensor", h_sensor);
    print_task("btn1", h_btn1);
    print_task("btn2", h_btn2);
    print_task("led3", h_led3);
    print_task("oled", h_oled);
    print_task("logger", h_logger);
    print_task("monitor", h_monitor);
  }
}

// ======================================================================================
// 13. SOFTWARE TIMER: BỘ ĐỊNH THỜI PHẦN MỀM BÁO NHỊP TIM HỆ THỐNG [K10]
// ======================================================================================
// Hàm callback này chạy trong ngữ cảnh của Timer Service Task (không được dùng lệnh block/delay lâu)
static void heartbeat_cb(TimerHandle_t t) {
  uint32_t cnt, drop;
  portENTER_CRITICAL(&g_mux);
  cnt = g_btn2Count;
  drop = g_logDropped;
  portEXIT_CRITICAL(&g_mux);

  // Định kỳ mỗi 5 giây báo thời gian hoạt động (uptime), số lần nhấn BTN2 và số log bị rơi
  logf("heartbeat up=%lus btn2=%lu drop=%lu", (unsigned long)(millis() / 1000),
       (unsigned long)cnt, (unsigned long)drop);
}

// ======================================================================================
// 14. HÀM KHỞI TẠO CHÍNH (SETUP) & CẤU HÌNH ĐA NHÂN FREERTOS [K1]
// ======================================================================================
void setup() {
#if ARDUINO_USB_CDC_ON_BOOT
  // Tăng bộ đệm truyền USB CDC lên 2048 byte để không bị rớt ký tự khi Task 8 in 9 dòng cùng lúc
  Serial.setTxBufferSize(2048);
#endif
  Serial.begin(115200);

  // Cấu hình chiều tín hiệu cho các chân LED (ngõ ra) và Nút nhấn (ngõ vào kéo trở nội lên nguồn)
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

  // Khởi tạo các đối tượng đồng bộ hóa của FreeRTOS
  g_btn2Sem     = xSemaphoreCreateBinary();       // Tạo Binary Semaphore (khởi đầu ở trạng thái rỗng)
  g_sensorMutex = xSemaphoreCreateMutex();        // Tạo Mutex bảo vệ biến g_sensor (có cơ chế kế thừa ưu tiên)
  g_i2cMutex    = xSemaphoreCreateMutex();        // Tạo Mutex bảo vệ bus I2C dùng chung
  g_logQueue    = xQueueCreate(16, sizeof(LogMsg)); // Tạo hàng đợi chứa tối đa 16 cấu trúc LogMsg
  g_sysEvents   = xEventGroupCreate();            // Tạo nhóm cờ sự kiện hệ thống
  configASSERT(g_btn2Sem && g_sensorMutex && g_i2cMutex && g_logQueue && g_sysEvents); // Dừng nếu thiếu RAM cấp phát

  // Tắt bớt thông báo lỗi I2C nội bộ của driver ESP-IDF (do Task 6 đã tự đếm và báo gọn)
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  // Khởi động bus I2C phần cứng tại chân G21 (SDA) và G22 (SCL) với tốc độ chuẩn 100kHz
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  // Khởi tạo màn hình OLED SH1106 (0x3C) và cảm biến ánh sáng BH1750 (0x23)
  if (oled_init_locked(true)) xEventGroupSetBits(g_sysEvents, EVT_OLED_READY);
  else Serial.println("[OLED] Chua thay man hinh, Task 6 se thu lai");

  g_bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  if (g_bh1750Ok) Serial.println("[BH1750] Khoi tao thanh cong tai 0x23!");
  else Serial.println("[BH1750] Chua thay cam bien tai 0x23, Task 2 se thu lai");

  // [K1] Tạo 8 Task và ghim cố định lên 2 nhân xử lý (Core 0 & Core 1) của ESP32:
  //   - Core 1: Các tác vụ điều khiển I/O trực tiếp (LED1, BTN1, BTN2, LED3) cần phản hồi nhanh
  //   - Core 0: Các tác vụ giao tiếp ngoại vi, hiển thị OLED, xuất log Serial và giám sát hệ thống
  xTaskCreatePinnedToCore(task7_logger,  "logger",  STACK_LOG,          NULL, PRIO_LOGGER,  &h_logger,  0);
  xTaskCreatePinnedToCore(task1_led1,    "led1",    STACK_SMALL,        NULL, PRIO_LED1,    &h_led1,    1);
  xTaskCreatePinnedToCore(task2_sensor,  "sensor",  STACK_SMALL + 1024, NULL, PRIO_SENSOR,  &h_sensor,  0);
  xTaskCreatePinnedToCore(task3_btn1,    "btn1",    STACK_SMALL + 1024, NULL, PRIO_BTN1,    &h_btn1,    1);
  xTaskCreatePinnedToCore(task4_btn2,    "btn2",    STACK_SMALL + 1024, NULL, PRIO_BTN2,    &h_btn2,    1);
  xTaskCreatePinnedToCore(task5_led3,    "led3",    STACK_SMALL + 1024, NULL, PRIO_LED3,    &h_led3,    1);
  xTaskCreatePinnedToCore(task6_oled,    "oled",    STACK_OLED,         NULL, PRIO_OLED,    &h_oled,    0);
  xTaskCreatePinnedToCore(task8_monitor, "monitor", STACK_OLED,         NULL, PRIO_MONITOR, &h_monitor, 0);

  // Đăng ký ngắt phần cứng cạnh xuống (FALLING) cho nút nhấn BOOT (G0)
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_BOOT), isr_boot, FALLING);

  // [K10] Tạo và kích hoạt Software Timer tự lặp lại (pdTRUE) mỗi T_HEARTBEAT_MS (5000ms)
  g_heartbeat = xTimerCreate("hb", pdMS_TO_TICKS(T_HEARTBEAT_MS), pdTRUE, NULL, heartbeat_cb);
  xTimerStart(g_heartbeat, 0);

  logf("Khoi dong xong, %u tasks", (unsigned)uxTaskGetNumberOfTasks());
}

// ======================================================================================
// 15. HÀM LOOP CỦA ARDUINO [K12]
// ======================================================================================
void loop() {
  // Vì toàn bộ ứng dụng đã được chia thành 8 Task độc lập của FreeRTOS nên loopTask của Arduino
  // không còn tác vụ nào cần xử lý -> Tự xóa chính nó để thu hồi ~8KB RAM trả về cho hệ thống.
  vTaskDelete(NULL);
}