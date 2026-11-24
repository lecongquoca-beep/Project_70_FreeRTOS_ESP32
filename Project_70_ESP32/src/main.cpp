/*
 * ======================================================================================
 * DỰ ÁN CÁ NHÂN: HỆ THỐNG NHÚNG THỜI GIAN THỰC (RTOS - FreeRTOS TRÊN ESP32)
 * ======================================================================================
 * Phần cứng sử dụng:
 *   - Vi điều khiển : ESP32-WROOM-32 (Dual-core Xtensa LX6, chạy FreeRTOS)
 *   - Màn hình OLED  : SH1106 1.3 inch, giao tiếp I2C (địa chỉ 0x3C, SDA=21, SCL=22)
 *   - Cảm biến sáng  : BH1750 (GY-302), giao tiếp I2C dùng chung bus (địa chỉ 0x23)
 *   - Ngoại vi I/O   : 3 LED đơn (G25, G26, G27) + LED báo nguồn (G16) + 2 nút nhấn ngoài (G18, G19) + nút BOOT (G0)
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
 *   [K13] Stream Buffer (giao tiếp UART 2 chiều) ....... g_uartRxStream, task9_cli_ctrl
 *   [K14] Hardware Watchdog & LEDC PWM tự động ......... esp_task_wdt, task10_supervisor
 * ======================================================================================
 */

#include <Arduino.h>
#include <Wire.h>                   // Thư viện giao tiếp chuẩn I2C (SDA, SCL)
#include <BH1750.h>                 // Thư viện điều khiển cảm biến cường độ ánh sáng số BH1750
#include <Adafruit_GFX.h>           // Thư viện đồ họa lõi (vẽ chữ, đường thẳng, tọa độ)
#include <Adafruit_SH110X.h>        // Thư viện driver chuyên dụng cho chip màn hình OLED SH1106
#include <freertos/FreeRTOS.h>      // Nhân hệ điều hành thời gian thực FreeRTOS
#include <freertos/task.h>          // API quản lý tác vụ (Task)
#include <freertos/semphr.h>        // API quản lý Semaphore và Mutex
#include <freertos/queue.h>         // API quản lý hàng đợi (Queue)
#include <freertos/event_groups.h>  // API quản lý nhóm cờ sự kiện (Event Group)
#include <freertos/timers.h>        // API quản lý bộ định thời phần mềm (Software Timer)
#include <freertos/stream_buffer.h> // Stream Buffer tối ưu cho truyền dòng byte UART
#include <esp_task_wdt.h>           // Thư viện Watchdog phần cứng của ESP-IDF
#include "config.h"                 // File cấu hình tập trung: sơ đồ chân, chu kỳ, mức ưu tiên, stack
#include <esp_log.h>                // Thư viện kiểm soát mức log hệ thống của ESP-IDF

// ======================================================================================
// 1. CẤU TRÚC DỮ LIỆU & BIẾN TOÀN CỤC DÙNG CHUNG GIỮA CÁC TASK
// ======================================================================================

struct SensorData {
  uint16_t raw;      // Giá trị độ rọi làm tròn nguyên (lux)
  float    lux;      // Độ rọi thực tế đo từ BH1750 (đơn vị: lux)
  uint8_t  percent;  // Mức độ sáng quy đổi sang phần trăm (0% - 100%) theo thang logarit
  uint32_t seq;      // Bộ đếm số lần đọc cảm biến thành công kể từ lúc khởi động
};

static SensorData g_sensor = {0, 0.0f, 0, 0};

static volatile bool g_led1On = false;           // Trạng thái nhịp nháy LED1 (Task 1 ghi)
static volatile bool g_led2On = false;           // Trạng thái LED2 (Task 3 ghi)
static volatile bool g_led3On = false;           // Trạng thái LED3 (Task 5 ghi)

static volatile TickType_t g_led3OffTick = 0;    // Thời điểm (tính theo tick) mà LED3 sẽ tự tắt
static volatile uint32_t   g_btn2Count   = 0;    // Tổng số lần nút BTN2 được nhấn
static volatile uint32_t   g_logDropped  = 0;    // Đếm số bản tin log bị rơi nếu hàng đợi Queue đầy

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t    g_btn2Sem;      // [K4] Binary Semaphore: Task 4 (BTN2) -> Task 5 (LED3)
static SemaphoreHandle_t    g_sensorMutex;  // [K5] Mutex bảo vệ cấu trúc g_sensor
static SemaphoreHandle_t    g_i2cMutex;     // [K5] Mutex khóa bus I2C (G21/G22)
static QueueHandle_t        g_logQueue;     // [K6] Queue chứa tối đa 16 bản tin log gửi ra Serial
static EventGroupHandle_t   g_sysEvents;    // [K7] Event Group lưu các bit trạng thái sẵn sàng
static TimerHandle_t        g_heartbeat;    // [K10] Software Timer định kỳ báo hệ thống còn sống
static StreamBufferHandle_t g_uartRxStream; // [K13] Stream Buffer nhận dữ liệu từ Serial RX

static TaskHandle_t h_led1, h_sensor, h_btn1, h_btn2, h_led3, h_oled, h_logger, h_monitor;
static TaskHandle_t h_cli, h_supervisor;

// Biến điều khiển chế độ PWM tự động & độ sáng mục tiêu cho Task 9 & Task 10
static volatile bool    g_autoMode  = true;               // true: độ sáng nhịp nháy theo lux; false: sáng 100%
static volatile uint8_t g_luxThresh = DEFAULT_LUX_THRESH; // Ngưỡng % ánh sáng kích hoạt bù sáng mạnh
static volatile uint8_t g_pwmDuty   = 255;                // Độ sáng PWM (1..255) khi LED1 ở nửa chu kỳ ON

static Adafruit_SH1106G oled(128, 64, &Wire, -1);
static BH1750 lightMeter(0x23);
static bool g_bh1750Ok = false;

// ======================================================================================
// 2. HỆ THỐNG GHI LOG AN TOÀN QUA QUEUE - MẪU THIẾT KẾ GATEKEEPER [K6]
// ======================================================================================
struct LogMsg {
  TickType_t tick;
  char text[56];
};

static void logf(const char *fmt, ...) {
  LogMsg m;
  m.tick = xTaskGetTickCount();

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.text, sizeof(m.text), fmt, ap);
  va_end(ap);

  if (xQueueSend(g_logQueue, &m, 0) != pdPASS) {
    portENTER_CRITICAL(&g_mux);
    g_logDropped = g_logDropped + 1;
    portEXIT_CRITICAL(&g_mux);
  }
}

// ======================================================================================
// 3. TASK 1: ĐIỀU KHIỂN LED1 NHẤP NHÁY ĐỘC LẬP THEO CHU KỲ CỐ ĐỊNH 600MS [K2]
// ======================================================================================
static void task1_led1(void *arg) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t half = pdMS_TO_TICKS(T_LED1_MS / 2); // 300ms ON, 300ms OFF
  bool on = false;

  for (;;) {
    on = !on;
    g_led1On = on;

    // LED1 luôn nhấp nháy độc lập mỗi 300ms:
    // - Khi OFF: tắt hẳn (duty = 0)
    // - Khi ON : sáng với độ sáng g_pwmDuty (do Task 10 tính theo lux ở chế độ AUTO, hoặc 255 ở MANUAL)
    uint8_t duty = g_autoMode ? g_pwmDuty : 255;
    ledcWrite(PWM_CH_LED1, on ? duty : 0);

    // [K2] vTaskDelayUntil đảm bảo tần số nháy chính xác tuyệt đối, không bị trôi pha
    vTaskDelayUntil(&last, half);
  }
}

// ======================================================================================
// 4. TASK 2: ĐỌC CẢM BIẾN ÁNH SÁNG BH1750 QUA BUS I2C [K2, K5, K7]
// ======================================================================================
static uint8_t lux_to_percent(float lux) {
  if (lux <= 1.0f) return 0;
  float p = log10f(lux) / log10f(LUX_FULL) * 100.0f;
  return (uint8_t)(p > 100.0f ? 100.0f : p);
}

static void task2_sensor(void *arg) {
  TickType_t last = xTaskGetTickCount();
  uint32_t nlog = 0;

  for (;;) {
    float lux = -1.0f;

    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!g_bh1750Ok) {
        g_bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      }
      if (g_bh1750Ok) {
        lux = lightMeter.readLightLevel();
        if (lux < 0.0f) {
          g_bh1750Ok = false;
        }
      }
      xSemaphoreGive(g_i2cMutex);
    }

    if (lux >= 0.0f) {
      uint8_t pct = lux_to_percent(lux);
      uint16_t rawLux = (uint16_t)(lux > 65535.0f ? 65535 : lux);

      if ((++nlog & 1) == 0) {
        logf("BH1750: %.1f lx (%u%%) | LED1:%s (PWM=%u)",
             lux, pct, g_led1On ? "ON" : "OFF", (unsigned)(g_autoMode ? g_pwmDuty : 255));
      }

      if (xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_sensor.raw = rawLux;
        g_sensor.lux = lux;
        g_sensor.percent = pct;
        g_sensor.seq++;
        xSemaphoreGive(g_sensorMutex);
      }

      xEventGroupSetBits(g_sysEvents, EVT_SENSOR_READY);
    } else {
      if ((++nlog & 1) == 0) {
        logf("LOI: Khong doc duoc BH1750 (0x23) | LED1:%s", g_led1On ? "ON" : "OFF");
      }
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(T_SENSOR_MS));
  }
}

// ======================================================================================
// 5. BỘ LỌC CHỐNG DỘI PHÍM CƠ HỌC (SOFTWARE DEBOUNCE) CHO CÁC NÚT NHẤN
// ======================================================================================
struct Debounce {
  uint8_t pin;
  bool stable;
  bool lastRead;
  TickType_t since;
};

static bool pressed_edge(Debounce &d) {
  bool r = digitalRead(d.pin);
  TickType_t now = xTaskGetTickCount();

  if (r != d.lastRead) {
    d.lastRead = r;
    d.since = now;
  }

  if ((now - d.since) >= pdMS_TO_TICKS(T_DEBOUNCE_MS) && r != d.stable) {
    d.stable = r;
    return r == LOW;
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
      g_led2On = !g_led2On;
      digitalWrite(PIN_LED2, g_led2On);
      logf("BTN1 -> LED2 %s", g_led2On ? "ON" : "OFF");
    }
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
      BaseType_t ok = xSemaphoreGive(g_btn2Sem);

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
    xSemaphoreTake(g_btn2Sem, portMAX_DELAY);

    TickType_t off = xTaskGetTickCount() + pdMS_TO_TICKS(T_LED3_ON_MS);

    portENTER_CRITICAL(&g_mux);
    g_led3OffTick = off;
    g_led3On = true;
    portEXIT_CRITICAL(&g_mux);

    digitalWrite(PIN_LED3, HIGH);
    logf("LED3 ON 3s");

    vTaskDelay(pdMS_TO_TICKS(T_LED3_ON_MS));

    digitalWrite(PIN_LED3, LOW);
    portENTER_CRITICAL(&g_mux);
    g_led3On = false;
    portEXIT_CRITICAL(&g_mux);
    logf("LED3 OFF, cho tiep");
  }
}

// ======================================================================================
// 9. HÀM KHỞI TẠO & DÒ ĐỊA CHỈ MÀN HÌNH OLED SH1106 TRÊN BUS I2C
// ======================================================================================
static uint8_t g_oledAddr = OLED_ADDR;

static bool oled_init_locked(bool verbose) {
  uint8_t foundAddr = 0;
  int count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      count++;
      if (addr == 0x3C || addr == 0x3D) foundAddr = addr;
      if (verbose) Serial.printf("[I2C] Tim thay thiet bi tai: 0x%02X\n", addr);
    }
  }

  if (verbose) {
    Serial.printf("[I2C] SDA=%d SCL=%d: %d thiet bi (OLED=0x%02X)\n", PIN_SDA, PIN_SCL, count, foundAddr);
  }

  if (foundAddr == 0) return false;
  if (!oled.begin(foundAddr, true)) return false;

  g_oledAddr = foundAddr;
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextWrap(false);
  oled.setContrast(120);
  oled.display();
  return true;
}

// ======================================================================================
// 10. TASK 6: HIỂN THỊ TRẠNG THÁI TOÀN HỆ THỐNG LÊN MÀN HÌNH OLED [K2, K5, K7, K9]
// ======================================================================================
static void task6_oled(void *arg) {
  xEventGroupWaitBits(g_sysEvents, EVT_SENSOR_READY, pdFALSE, pdTRUE, portMAX_DELAY);

  bool haveOled = xEventGroupGetBits(g_sysEvents) & EVT_OLED_READY;
  uint8_t retry = 0, oledFail = 0;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    SensorData s;
    if (xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      s = g_sensor;
      xSemaphoreGive(g_sensorMutex);
    } else {
      s = {0, 0, 0, 0};
    }

    bool led3;
    TickType_t offTick;
    portENTER_CRITICAL(&g_mux);
    led3 = g_led3On;
    offTick = g_led3OffTick;
    portEXIT_CRITICAL(&g_mux);

    int32_t remainMs = 0;
    if (led3) {
      int32_t d = (int32_t)(offTick - xTaskGetTickCount());
      remainMs = d > 0 ? d * portTICK_PERIOD_MS : 0;
    }

    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (haveOled) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SH110X_WHITE);

        oled.setCursor(0, 0);
        oled.printf("RTOS ESP32 [%s]\n", g_autoMode ? "AUTO" : "MAN");
        oled.drawFastHLine(0, 10, 128, SH110X_WHITE);

        oled.setCursor(0, 16);
        oled.printf("LED1:%s  LED2:%s\n", g_led1On ? "ON" : "OFF", g_led2On ? "ON" : "OFF");

        oled.setCursor(0, 28);
        oled.printf("Light: %u%% (%.1f lx)\n", s.percent, s.lux);

        oled.setCursor(0, 40);
        if (led3) oled.printf("LED3 : ON  %.1fs\n", remainMs / 1000.0f);
        else      oled.println("LED3 : OFF");

        oled.setCursor(0, 54);
        oled.printf("seq=%lu PWM=%u", (unsigned long)s.seq, (unsigned)(g_autoMode ? g_pwmDuty : 255));

        oled.display();

        Wire.beginTransmission(g_oledAddr);
        if (Wire.endTransmission() != 0) {
          if (++oledFail >= 6) {
            oledFail = 0;
            haveOled = false;
            xEventGroupClearBits(g_sysEvents, EVT_OLED_READY);
            logf("[OLED] Mat ket noi, dang thu lai...");
          }
        } else {
          oledFail = 0;
        }
      } else {
        if (++retry >= 10) {
          retry = 0;
          if (oled_init_locked(false)) {
            haveOled = true;
            xEventGroupSetBits(g_sysEvents, EVT_OLED_READY);
            logf("[OLED] Da ket noi lai 0x%02X", g_oledAddr);
          }
        }
      }
      xSemaphoreGive(g_i2cMutex);
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(T_OLED_MS));
  }
}

// ======================================================================================
// 11. TASK 7: GATEKEEPER LOGGER - ĐỘC QUYỀN XUẤT DỮ LIỆU RA CỔNG SERIAL [K6]
// ======================================================================================
static void task7_logger(void *arg) {
  LogMsg m;
  for (;;) {
    if (xQueueReceive(g_logQueue, &m, portMAX_DELAY) == pdPASS) {
      Serial.printf("[%7lu ms][c%d] %s\n",
                    (unsigned long)(m.tick * portTICK_PERIOD_MS), xPortGetCoreID(), m.text);
    }
  }
}

// ======================================================================================
// 12. NGẮT NGOẠI VI (ISR) & TASK 8: GIÁM SÁT TÀI NGUYÊN HỆ THỐNG [K8, K11]
// ======================================================================================
static void IRAM_ATTR isr_boot() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(h_monitor, &woken);
  if (woken) portYIELD_FROM_ISR();
}

static void print_task(const char *name, TaskHandle_t h) {
  logf("%-10s prio=%u stackFree=%u B", name, (unsigned)uxTaskPriorityGet(h),
       (unsigned)uxTaskGetStackHighWaterMark(h));
}

static void task8_monitor(void *arg) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(50));
    ulTaskNotifyTake(pdTRUE, 0);

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
    print_task("cli_ctrl", h_cli);
    print_task("supervisor", h_supervisor);
  }
}

// ======================================================================================
// 13. SOFTWARE TIMER: BỘ ĐỊNH THỜI PHẦN MỀM BÁO NHỊP TIM HỆ THỐNG [K10]
// ======================================================================================
static void heartbeat_cb(TimerHandle_t t) {
  uint32_t cnt, drop;
  portENTER_CRITICAL(&g_mux);
  cnt = g_btn2Count;
  drop = g_logDropped;
  portEXIT_CRITICAL(&g_mux);

  logf("heartbeat up=%lus btn2=%lu drop=%lu", (unsigned long)(millis() / 1000),
       (unsigned long)cnt, (unsigned long)drop);
}

// ======================================================================================
// 16. TASK 9: BỘ XỬ LÝ LỆNH TƯƠNG TÁC HAI CHIỀU QUA STREAM BUFFER [K13]
// ======================================================================================
static void task9_cli_ctrl(void *arg) {
  char cmdBuf[48];
  size_t idx = 0;

  for (;;) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      xStreamBufferSend(g_uartRxStream, &c, 1, 0);
    }

    char ch;
    if (xStreamBufferReceive(g_uartRxStream, &ch, 1, pdMS_TO_TICKS(20)) == 1) {
      if (ch == '\n' || ch == '\r') {
        if (idx > 0) {
          cmdBuf[idx] = '\0';
          if (strcmp(cmdBuf, "auto") == 0) {
            g_autoMode = true;
            logf("[CLI] Chuyen sang che do AUTO (PWM theo lux)");
          } else if (strcmp(cmdBuf, "manual") == 0) {
            g_autoMode = false;
            g_pwmDuty = 255;
            logf("[CLI] Chuyen sang che do MANUAL (PWM=255)");
          } else if (strncmp(cmdBuf, "thresh=", 7) == 0) {
            int val = atoi(cmdBuf + 7);
            if (val >= 0 && val <= 100) {
              g_luxThresh = (uint8_t)val;
              logf("[CLI] Cap nhat nguong canh bao = %u%%", g_luxThresh);
            }
          } else if (strcmp(cmdBuf, "stat") == 0) {
            xTaskNotifyGive(h_monitor);
          } else {
            logf("[CLI] Lenh hop le: auto | manual | thresh=<0..100> | stat");
          }
          idx = 0;
        }
      } else if (idx < sizeof(cmdBuf) - 1) {
        cmdBuf[idx++] = ch;
      }
    }
  }
}

// ======================================================================================
// 17. TASK 10: GIÁM SÁT AN TOÀN (WATCHDOG) & TÍNH TOÁN ĐỘ SÁNG PWM TỰ ĐỘNG [K14]
// ======================================================================================
static void task10_supervisor(void *arg) {
  esp_task_wdt_init(5, true);
  esp_task_wdt_add(NULL);

  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // 1. Nạp lại bộ đếm Watchdog chứng minh hệ thống không bị treo
    esp_task_wdt_reset();

    // 2. Đọc dữ liệu phần trăm ánh sáng hiện tại từ cảm biến BH1750 qua Mutex
    uint8_t curPct = 50;
    bool sensorAlive = false;
    if (xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
      curPct = g_sensor.percent;
      sensorAlive = (g_sensor.seq > 0) && g_bh1750Ok;
      xSemaphoreGive(g_sensorMutex);
    }

    // 3. Tính toán độ sáng PWM (g_pwmDuty) để Task 1 nháy LED1 với cường độ bù sáng thông minh:
    //    - Khi che tối (< g_luxThresh): LED1 nháy cực sáng (255)
    //    - Khi phòng sáng mạnh: LED1 vẫn nháy đều 600ms nhưng dịu xuống (40..200) tiết kiệm điện
    if (g_autoMode && sensorAlive) {
      if (curPct < g_luxThresh) {
        g_pwmDuty = 255;
      } else {
        g_pwmDuty = (uint8_t)map(curPct, g_luxThresh, 100, 200, 35);
      }
    } else {
      g_pwmDuty = 255;
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(T_SUPERVISOR_MS));
  }
}

// ======================================================================================
// 14. HÀM KHỞI TẠO CHÍNH (SETUP) & CẤU HÌNH ĐA NHÂN FREERTOS [K1]
// ======================================================================================
void setup() {
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxBufferSize(2048);
#endif
  Serial.begin(115200);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);

  // Kích hoạt chân G16 (PIN_RGB_R trên ESP32-WROOM-32) làm LED phát sáng liên tục
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  pinMode(PIN_RGB_R, OUTPUT);
  digitalWrite(PIN_RGB_R, HIGH);
#endif

  g_btn2Sem      = xSemaphoreCreateBinary();
  g_sensorMutex  = xSemaphoreCreateMutex();
  g_i2cMutex     = xSemaphoreCreateMutex();
  g_logQueue     = xQueueCreate(16, sizeof(LogMsg));
  g_sysEvents    = xEventGroupCreate();
  g_uartRxStream = xStreamBufferCreate(64, 1);
  configASSERT(g_btn2Sem && g_sensorMutex && g_i2cMutex && g_logQueue && g_sysEvents && g_uartRxStream);

  // Gắn kênh PWM phần cứng vào chân LED1 (G25)
  ledcSetup(PWM_CH_LED1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_LED1, PWM_CH_LED1);

  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  if (oled_init_locked(true)) xEventGroupSetBits(g_sysEvents, EVT_OLED_READY);
  else Serial.println("[OLED] Chua thay man hinh, Task 6 se thu lai");

  g_bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  if (g_bh1750Ok) Serial.println("[BH1750] Khoi tao thanh cong tai 0x23!");
  else Serial.println("[BH1750] Chua thay cam bien tai 0x23, Task 2 se thu lai");

  // [K1] Tạo 10 Task và ghim cố định lên 2 nhân xử lý (Core 0 & Core 1) của ESP32:
  xTaskCreatePinnedToCore(task7_logger,     "logger",     STACK_LOG,          NULL, PRIO_LOGGER,     &h_logger,     0);
  xTaskCreatePinnedToCore(task1_led1,       "led1",       STACK_SMALL,        NULL, PRIO_LED1,       &h_led1,       1);
  xTaskCreatePinnedToCore(task2_sensor,     "sensor",     STACK_SMALL + 1024, NULL, PRIO_SENSOR,     &h_sensor,     0);
  xTaskCreatePinnedToCore(task3_btn1,       "btn1",       STACK_SMALL + 1024, NULL, PRIO_BTN1,       &h_btn1,       1);
  xTaskCreatePinnedToCore(task4_btn2,       "btn2",       STACK_SMALL + 1024, NULL, PRIO_BTN2,       &h_btn2,       1);
  xTaskCreatePinnedToCore(task5_led3,       "led3",       STACK_SMALL + 1024, NULL, PRIO_LED3,       &h_led3,       1);
  xTaskCreatePinnedToCore(task6_oled,       "oled",       STACK_OLED,         NULL, PRIO_OLED,       &h_oled,       0);
  xTaskCreatePinnedToCore(task8_monitor,    "monitor",    STACK_OLED,         NULL, PRIO_MONITOR,    &h_monitor,    0);
  xTaskCreatePinnedToCore(task9_cli_ctrl,   "cli_ctrl",   STACK_CLI,          NULL, PRIO_CLI,        &h_cli,        0);
  xTaskCreatePinnedToCore(task10_supervisor,"supervisor", STACK_SUPERVISOR,   NULL, PRIO_SUPERVISOR, &h_supervisor, 1);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_BOOT), isr_boot, FALLING);

  g_heartbeat = xTimerCreate("hb", pdMS_TO_TICKS(T_HEARTBEAT_MS), pdTRUE, NULL, heartbeat_cb);
  xTimerStart(g_heartbeat, 0);

  logf("Khoi dong xong, %u tasks", (unsigned)uxTaskGetNumberOfTasks());
}

void loop() {
  vTaskDelete(NULL);
} 