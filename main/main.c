/**
 * ============================================================
 *  SMART STUDY ASSISTANT - Hệ Thống Nhúng Học Sâu
 *  Trường Đại học Bách Khoa Đà Nẵng - Khoa Điện tử Viễn thông
 * ============================================================
 *  Vi điều khiển : ESP32-C3
 *  RTOS          : FreeRTOS
 *  Tác vụ        : Task_Input, Task_Sensor, Task_Logic,
 *                  Task_Display, Task_Light, Task_Alert, Task_WiFi
 *  Đồng bộ hóa  : Queue, Mutex, Semaphore
 *  LED           : WS2812B 100 LED/m (RMT)
 * ============================================================
 */

#include "esp_crt_bundle.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
static const char *TAG = "SMART_STUDY";

/* ============================================================
 *  CẤU HÌNH CHÂN GPIO
 * ============================================================ */
#define BTN_START_GPIO      4
#define BTN_STOP_GPIO       5
#define BTN_RESET_GPIO      6
#define BUZZER_GPIO         7
#define I2C_MASTER_SDA      2
#define I2C_MASTER_SCL      3
#define I2C_MASTER_NUM      I2C_NUM_0
#define LED_DATA_GPIO       8
#define LED_COUNT           100

/* Địa chỉ I2C */
#define DS3231_ADDR         0x68
#define OLED_ADDR           0x3C
#define BH1750_ADDR         0x23

#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      pdMS_TO_TICKS(1000)

/* WiFi & Server */
#define WIFI_SSID           "Trifnh"
#define WIFI_PASS           "01072025"
// Bỏ chữ "s" trong https, bỏ ":5000", giữ nguyên đường dẫn API phía sau
#define SERVER_URL  "https://web-study-lamp.onrender.com"

/* Pomodoro */
#define POMODORO_WORK_MS    (25 * 60 * 1000)
#define POMODORO_BREAK_MS   (5  * 60 * 1000)

/* ============================================================
 *  MACRO BCD — FIX: thêm ngoặc bảo vệ argument
 * ============================================================ */
#define BCD2DEC(v)  ((((v) >> 4) & 0x0F) * 10 + ((v) & 0x0F))
#define DEC2BCD(v)  ((((v) / 10) << 4)   | ((v) % 10))

/* ============================================================
 *  KIỂU DỮ LIỆU
 * ============================================================ */
typedef enum { CMD_START, CMD_STOP, CMD_RESET } btn_cmd_t;
typedef enum { STATE_IDLE, STATE_WORK, STATE_BREAK, STATE_PAUSE } clock_state_t;
typedef enum { ALERT_NONE, ALERT_WORK_END, ALERT_BREAK_START } alert_code_t;

typedef enum {
    WEB_CMD_NONE = 0,
    WEB_CMD_START,
    WEB_CMD_STOP,
    WEB_CMD_RESET,
    WEB_CMD_COLOR,
} web_cmd_t;

typedef struct { web_cmd_t cmd; uint8_t r, g, b; } web_ctrl_t;
typedef struct { uint8_t r, g, b, brightness; } led_cmd_t;

typedef struct {
    clock_state_t state;
    uint32_t remaining_sec;
    float    lux;
    uint8_t  led_brt;
    uint8_t  hour, min, sec;
    uint8_t  date, month, year;   // ← THÊM DÒNG NÀY
} disp_data_t;

typedef struct {
    uint16_t      sessions_today;
    uint32_t      total_time_sec;
    clock_state_t current_state;
} wifi_data_t;

typedef struct {
    uint8_t sec, min, hour;
    uint8_t day, date, month, year;
} rtc_time_t;

/* ============================================================
 *  BIẾN TOÀN CỤC
 * ============================================================ */
static volatile clock_state_t g_state      = STATE_IDLE;
static volatile uint32_t      g_remain_sec = 0;
static volatile float         g_lux        = 0.0f;
static volatile uint16_t      g_sessions   = 0;
static volatile uint32_t      g_total_sec  = 0;
static volatile alert_code_t  g_alert_code = ALERT_NONE;
static rtc_time_t             g_rtc_time   = {0};

static volatile bool    g_manual_color = false;
static volatile uint8_t g_manual_r     = 0;
static volatile uint8_t g_manual_g     = 0;
static volatile uint8_t g_manual_b     = 0;

static led_strip_handle_t led_strip = NULL;

/* ============================================================
 *  HANDLE QUEUE / MUTEX / SEMAPHORE
 * ============================================================ */
static QueueHandle_t     Queue_Input   = NULL;
static QueueHandle_t     Queue_Command = NULL;
static QueueHandle_t     Queue_Sensor  = NULL;
static QueueHandle_t     Queue_Light   = NULL;
static QueueHandle_t     Queue_Display = NULL;
static QueueHandle_t     Queue_WiFi    = NULL;
static QueueHandle_t     Queue_WebCtrl = NULL;
static SemaphoreHandle_t Mutex_I2C     = NULL;
static SemaphoreHandle_t Mutex_State   = NULL;
static SemaphoreHandle_t Sem_Alert     = NULL;

/* ============================================================
 *  FONT OLED 5x8
 * ============================================================ */
static const uint8_t font5x8[][5] = {
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},{0x00,0x00,0x00,0x00,0x00}
};

/* ============================================================
 *  OLED DRIVER
 * ============================================================ */
static void oled_send_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, 2, pdMS_TO_TICKS(100));
}
static void oled_send_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, 2, pdMS_TO_TICKS(100));
}
static void oled_set_cursor(uint8_t row, uint8_t col) {
    oled_send_cmd(0xB0 + row);
    oled_send_cmd(0x00 | (col & 0x0F));
    oled_send_cmd(0x10 | ((col >> 4) & 0x0F));
}
static void oled_clear(void) {
    for (uint8_t i = 0; i < 8; i++) {
        oled_set_cursor(i, 0);
        for (uint8_t j = 0; j < 128; j++) oled_send_data(0);
    }
}
static void oled_init(void) {
    uint8_t cmds[] = {
        0xAE,0x20,0x00,0x40,0xA1,0xC8,0xA6,0xA4,
        0xA8,0x3F,0xD3,0x00,0xD5,0x80,0xD9,0xF1,
        0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF
    };
    for (int i = 0; i < (int)sizeof(cmds); i++) oled_send_cmd(cmds[i]);
    oled_clear();
}
static void oled_set_brightness(uint8_t level) {
    oled_send_cmd(0x81);
    oled_send_cmd(level);
}
static void oled_print(uint8_t row, uint8_t col_char, const char *str) {
    oled_set_cursor(row, col_char * 6);
    while (*str) {
        if (*str >= 32 && *str <= 126) {
            for (int i = 0; i < 5; i++) oled_send_data(font5x8[*str - 32][i]);
            oled_send_data(0x00);
        }
        str++;
    }
}

/* ============================================================
 *  DS3231 RTC DRIVER
 *  Datasheet DS3231: registers 0x00-0x06
 *  reg[0]=sec, [1]=min, [2]=hour, [3]=day, [4]=date, [5]=month, [6]=year
 * ============================================================ */
static esp_err_t ds3231_read_time(rtc_time_t *t) {
    uint8_t raw[7] = {0};
    uint8_t reg    = 0x00;
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM, DS3231_ADDR, &reg, 1, raw, 7, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    t->sec   = BCD2DEC(raw[0] & 0x7F);
    t->min   = BCD2DEC(raw[1] & 0x7F);
    t->hour  = BCD2DEC(raw[2] & 0x3F);
    t->day   = BCD2DEC(raw[3] & 0x07);
    t->date  = BCD2DEC(raw[4] & 0x3F);
    t->month = BCD2DEC(raw[5] & 0x1F);
    t->year  = BCD2DEC(raw[6]);
    return ESP_OK;
}

static esp_err_t ds3231_set_time(const rtc_time_t *t) {
    /* Kiểm tra giá trị hợp lệ trước khi ghi */
    if (t->sec > 59 || t->min > 59 || t->hour > 23 ||
        t->date < 1 || t->date > 31 ||
        t->month < 1 || t->month > 12 ||
        t->year > 99) {
        ESP_LOGE("RTC", "Gio khong hop le! sec=%d min=%d hour=%d date=%d month=%d year=%d",
                 t->sec, t->min, t->hour, t->date, t->month, t->year);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[8];
    buf[0] = 0x00;                  /* bắt đầu từ register 0x00 */
    buf[1] = DEC2BCD(t->sec);       /* 0x00: seconds */
    buf[2] = DEC2BCD(t->min);       /* 0x01: minutes */
    buf[3] = DEC2BCD(t->hour);      /* 0x02: hours (24h mode, bit6=0) */
    buf[4] = DEC2BCD(t->day);       /* 0x03: day of week (1-7) */
    buf[5] = DEC2BCD(t->date);      /* 0x04: date (1-31) */
    buf[6] = DEC2BCD(t->month);     /* 0x05: month (1-12) */
    buf[7] = DEC2BCD(t->year);      /* 0x06: year (0-99) */

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM, DS3231_ADDR, buf, 8, I2C_TIMEOUT_MS);

    if (err == ESP_OK) {
        /* Đọc lại để xác nhận ghi thành công */
        rtc_time_t verify = {0};
        vTaskDelay(pdMS_TO_TICKS(50));
        ds3231_read_time(&verify);
        ESP_LOGI("RTC", "Xac nhan: %02d:%02d:%02d ngay %02d/%02d/20%02d",
                 verify.hour, verify.min, verify.sec,
                 verify.date, verify.month, verify.year);
    }
    return err;
}

/* ============================================================
 *  BH1750 DRIVER
 * ============================================================ */
static bool bh1750_ready = false;

static void bh1750_init(void) {
    uint8_t cmd;
    esp_err_t err;
    cmd = 0x01;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR,
                                     &cmd, 1, pdMS_TO_TICKS(200));
    if (err != ESP_OK) { bh1750_ready = false; return; }
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd = 0x07;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR,
                                     &cmd, 1, pdMS_TO_TICKS(200));
    if (err != ESP_OK) { bh1750_ready = false; return; }
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd = 0x10;
    err = i2c_master_write_to_device(I2C_MASTER_NUM, BH1750_ADDR,
                                     &cmd, 1, pdMS_TO_TICKS(200));
    if (err != ESP_OK) { bh1750_ready = false; return; }
    vTaskDelay(pdMS_TO_TICKS(200));
    bh1750_ready = true;
    ESP_LOGI("BH1750", "GY-302 khoi tao OK!");
}

static float bh1750_read_lux(void) {
    if (!bh1750_ready) { bh1750_init(); if (!bh1750_ready) return -1.0f; }
    uint8_t data[2] = {0, 0};
    esp_err_t err = i2c_master_read_from_device(
        I2C_MASTER_NUM, BH1750_ADDR, data, 2, pdMS_TO_TICKS(300));
    if (err != ESP_OK) { bh1750_ready = false; return -1.0f; }
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    if (raw == 0 || raw == 0xFFFF) return -1.0f;
    return (float)raw / 1.2f;
}

/* ============================================================
 *  WS2812B DRIVER (RMT)
 * ============================================================ */
static void ws2812_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num   = LED_DATA_GPIO,
        .max_leds         = LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model        = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    esp_err_t err = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip);
    if (err != ESP_OK) {
        ESP_LOGE("LED", "WS2812B init that bai: %s", esp_err_to_name(err));
        return;
    }
    led_strip_clear(led_strip);
    ESP_LOGI("LED", "WS2812B OK - %d LED", LED_COUNT);
}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brt_pct) {
    if (led_strip == NULL) return;
    uint32_t scale = brt_pct;
    uint8_t fr = (uint8_t)((uint32_t)r * scale / 100);
    uint8_t fg = (uint8_t)((uint32_t)g * scale / 100);
    uint8_t fb = (uint8_t)((uint32_t)b * scale / 100);
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel(led_strip, i, fr, fg, fb);
    }
    led_strip_refresh(led_strip);
}

static void led_flash_alert(void) {
    if (led_strip == NULL) return;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < LED_COUNT; j++)
            led_strip_set_pixel(led_strip, j, 255, 255, 255);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ============================================================
 *  ISR NÚT NHẤN
 * ============================================================ */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    btn_cmd_t cmd;
    if      (gpio_num == BTN_START_GPIO) cmd = CMD_START;
    else if (gpio_num == BTN_STOP_GPIO)  cmd = CMD_STOP;
    else                                  cmd = CMD_RESET;
    xQueueSendFromISR(Queue_Input, &cmd, NULL);
}

/* ============================================================
 *  I2C MASTER INIT
 * ============================================================ */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA,
        .scl_io_num       = I2C_MASTER_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* ============================================================
 *  TASK 1: Task_Input
 * ============================================================ */
void task_input(void *pv) {
    btn_cmd_t  raw_cmd;
    TickType_t last_tick = 0;
    const TickType_t debounce = pdMS_TO_TICKS(50);
    while (1) {
        if (xQueueReceive(Queue_Input, &raw_cmd, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_tick) >= debounce) {
                last_tick = now;
                xQueueSend(Queue_Command, &raw_cmd, 0);
                ESP_LOGI("INPUT", "Nut hop le: %d", raw_cmd);
                gpio_set_level(BUZZER_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(80));
                gpio_set_level(BUZZER_GPIO, 0);
            }
        }
    }
}

/* ============================================================
 *  TASK 2: Task_Sensor
 * ============================================================ */
void task_sensor(void *pv) {
    if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
        bh1750_init();
        xSemaphoreGive(Mutex_I2C);
    }
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        float lux;
        if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
            lux = bh1750_read_lux();
            xSemaphoreGive(Mutex_I2C);
        } else continue;
        if (lux < 0.0f) continue;
        xQueueOverwrite(Queue_Sensor, &lux);
    }
}

/* ============================================================
 *  TASK 3: Task_Logic
 * ============================================================ */
void task_logic(void *pv) {
    clock_state_t state         = STATE_IDLE;
    uint32_t      remain_sec    = 0;
    uint8_t       led_brt       = 20;
    alert_code_t  pending_alert = ALERT_NONE;
    rtc_time_t    local_time    = {0};
    btn_cmd_t     cmd;
    web_ctrl_t    wctrl;
    float         lux_val       = 0.0f;
    TickType_t    last_sec_tick = xTaskGetTickCount();

    while (1) {
        /* ---- 1. Lệnh nút nhấn ---- */
        if (xQueueReceive(Queue_Command, &cmd, 0) == pdTRUE) {
            switch (cmd) {
                case CMD_START:
                    if (state == STATE_IDLE || state == STATE_BREAK) {
                        state = STATE_WORK;
                        remain_sec = POMODORO_WORK_MS / 1000;
                        ESP_LOGI("LOGIC", "BTN -> WORK");
                    } else if (state == STATE_PAUSE) {
                        state = STATE_WORK;
                        ESP_LOGI("LOGIC", "BTN -> RESUME");
                    }
                    g_manual_color = false;
                    break;
                case CMD_STOP:
                    if (state == STATE_WORK || state == STATE_BREAK) {
                        state = STATE_PAUSE;
                        ESP_LOGI("LOGIC", "BTN -> PAUSE");
                    }
                    break;
                case CMD_RESET:
                    state = STATE_IDLE; remain_sec = 0;
                    g_sessions = 0; g_total_sec = 0;
                    g_manual_color = false;
                    ESP_LOGI("LOGIC", "BTN -> RESET");
                    break;
            }
        }

        /* ---- 2. Lệnh Web Dashboard ---- */
        if (xQueueReceive(Queue_WebCtrl, &wctrl, 0) == pdTRUE) {
            switch (wctrl.cmd) {
                case WEB_CMD_START:
                    if (state == STATE_IDLE || state == STATE_BREAK) {
                        state = STATE_WORK;
                        remain_sec = POMODORO_WORK_MS / 1000;
                        ESP_LOGI("LOGIC", "WEB -> WORK");
                    } else if (state == STATE_PAUSE) {
                        state = STATE_WORK;
                        ESP_LOGI("LOGIC", "WEB -> RESUME");
                    }
                    g_manual_color = false;
                    break;
                case WEB_CMD_STOP:
                    if (state == STATE_WORK || state == STATE_BREAK) {
                        state = STATE_PAUSE;
                        ESP_LOGI("LOGIC", "WEB -> PAUSE");
                    }
                    break;
                case WEB_CMD_RESET:
                    state = STATE_IDLE; remain_sec = 0;
                    g_sessions = 0; g_total_sec = 0;
                    g_manual_color = false;
                    ESP_LOGI("LOGIC", "WEB -> RESET");
                    break;
                case WEB_CMD_COLOR:
                    g_manual_color = true;
                    g_manual_r = wctrl.r;
                    g_manual_g = wctrl.g;
                    g_manual_b = wctrl.b;
                    ESP_LOGI("LOGIC", "WEB -> COLOR R=%d G=%d B=%d",
                             wctrl.r, wctrl.g, wctrl.b);
                    break;
                default: break;
            }
        }

    /* ---- 3. Cảm biến ánh sáng ---- */
if (xQueueReceive(Queue_Sensor, &lux_val, 0) == pdTRUE) {
    g_lux = lux_val;
    float clamped = lux_val > 1000.0f ? 1000.0f : lux_val;

    /* FIX: đảo ngược — lux cao thì brightness thấp */
    led_brt = (uint8_t)(100.0f - clamped * 70.0f / 1000.0f);
    /* 
     * lux=0    → led_brt=100%  (phòng tối, cần đèn sáng nhất)
     * lux=500  → led_brt=65%   (phòng vừa)
     * lux=1000 → led_brt=30%   (phòng đã rất sáng, đèn bổ sung nhẹ)
     */
}

        /* ---- 4. Đếm ngược 1 giây ---- */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_sec_tick) >= pdMS_TO_TICKS(1000)) {
            last_sec_tick = now;
            if ((state == STATE_WORK || state == STATE_BREAK) && remain_sec > 0) {
                remain_sec--;
                if (state == STATE_WORK) g_total_sec++;
            }
            if (remain_sec == 0 && (state == STATE_WORK || state == STATE_BREAK)) {
                if (state == STATE_WORK) {
                    pending_alert = ALERT_WORK_END;
                    g_sessions++;
                    state = STATE_BREAK;
                    remain_sec = POMODORO_BREAK_MS / 1000;
                    ESP_LOGI("LOGIC", "WORK_END -> BREAK");
                } else {
                    pending_alert = ALERT_BREAK_START;
                    state = STATE_IDLE;
                    remain_sec = 0;
                    ESP_LOGI("LOGIC", "BREAK_END -> IDLE");
                }
                g_alert_code = pending_alert;
                xSemaphoreGive(Sem_Alert);
            }
        }

        /* ---- 5. Cập nhật global ---- */
        if (xSemaphoreTake(Mutex_State, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_state      = state;
            g_remain_sec = remain_sec;
            xSemaphoreGive(Mutex_State);
        }

        /* ---- 6. Đọc giờ RTC ---- */
        if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(20)) == pdTRUE) {
            ds3231_read_time(&local_time);
            g_rtc_time = local_time;
            xSemaphoreGive(Mutex_I2C);
        }

        /* ---- 7. Tính màu LED ---- */
        led_cmd_t led_cmd;
        led_cmd.brightness = led_brt;
        if (g_manual_color) {
            led_cmd.r = g_manual_r;
            led_cmd.g = g_manual_g;
            led_cmd.b = g_manual_b;
        } else {
            switch (state) {
                case STATE_WORK:
                    led_cmd.r=200; led_cmd.g=220; led_cmd.b=255; break;
                case STATE_BREAK:
                    led_cmd.r=255; led_cmd.g=200; led_cmd.b=80;  break;
                case STATE_PAUSE:
                    led_cmd.r=180; led_cmd.g=100; led_cmd.b=30;
                    led_cmd.brightness = led_brt / 3; break;
                default:
                    led_cmd.r=0; led_cmd.g=0; led_cmd.b=0;
                    led_cmd.brightness = 0; break;
            }
        }

        xQueueOverwrite(Queue_Light, &led_cmd);

        /* ---- 8. Gửi Queue_Display ---- */
       disp_data_t disp = {
    .state         = state,
    .remaining_sec = remain_sec,
    .lux           = g_lux,
    .led_brt       = led_brt,
    .hour          = local_time.hour,
    .min           = local_time.min,
    .sec           = local_time.sec,
    .date          = local_time.date,    // ← THÊM
    .month         = local_time.month,   // ← THÊM
    .year          = local_time.year,    // ← THÊM
};
        xQueueOverwrite(Queue_Display, &disp);

        /* ---- 9. Gửi Queue_WiFi ---- */
        wifi_data_t wdata = {
            .sessions_today = g_sessions,
            .total_time_sec = g_total_sec,
            .current_state  = state,
        };
        xQueueOverwrite(Queue_WiFi, &wdata);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
 *  TASK 4: Task_Display
 * ============================================================ */
void task_display(void *pv) {
    if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
        oled_init();
        xSemaphoreGive(Mutex_I2C);
    }
    disp_data_t data;
    char buf[32];
    while (1) {
        if (xQueueReceive(Queue_Display, &data, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        uint8_t brightness;
        if      (data.lux < 15.0f)  brightness = 10;
        else if (data.lux < 150.0f) brightness = 100;
        else                         brightness = 255;
        oled_set_brightness(brightness);
        const char *mode_str =
            (data.state == STATE_WORK)  ? "WORK " :
            (data.state == STATE_BREAK) ? "BREAK" :
            (data.state == STATE_PAUSE) ? "PAUSE" : "IDLE ";
        snprintf(buf, sizeof(buf), "%s %02u:%02u:%02u", mode_str,
                 (unsigned)data.hour, (unsigned)data.min, (unsigned)data.sec);
        oled_print(0, 0, buf);
        unsigned int m = (unsigned int)(data.remaining_sec / 60);
        unsigned int s = (unsigned int)(data.remaining_sec % 60);
        snprintf(buf, sizeof(buf), "Pomo %02u:%02u      ", m, s);
        oled_print(2, 0, buf);
        snprintf(buf, sizeof(buf), "Lux: %-6.1f     ", data.lux);
        oled_print(4, 0, buf);
        snprintf(buf, sizeof(buf), "LED BRT: %3u%%   ", (unsigned)data.led_brt);
        oled_print(6, 0, buf);
        xSemaphoreGive(Mutex_I2C);
    }
}

/* ============================================================
 *  TASK 5: Task_Light
 * ============================================================ */
void task_light(void *pv) {
    ws2812_init();
    led_cmd_t cmd;
    while (1) {
        if (xQueueReceive(Queue_Light, &cmd, portMAX_DELAY) == pdTRUE) {
            led_set_color(cmd.r, cmd.g, cmd.b, cmd.brightness);
        }
    }
}

/* ============================================================
 *  TASK 6: Task_Alert
 * ============================================================ */
void task_alert(void *pv) {
    while (1) {
        if (xSemaphoreTake(Sem_Alert, portMAX_DELAY) == pdTRUE) {
            alert_code_t code = g_alert_code;
            if (code == ALERT_WORK_END) {
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(BUZZER_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(BUZZER_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(200));
                }
                ESP_LOGI("ALERT", "WORK_END: 3 beep");
            } else if (code == ALERT_BREAK_START) {
                gpio_set_level(BUZZER_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(BUZZER_GPIO, 0);
                ESP_LOGI("ALERT", "BREAK_START: 1 beep dai");
            }
            led_flash_alert();
        }
    }
}

/* ============================================================
 *  TASK 7: Task_WiFi
 * ============================================================ */
static bool wifi_connected   = false;
static int  wifi_retry_count = 0;
#define WIFI_MAX_RETRY  5

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        wifi_retry_count = 0;
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            wifi_retry_count++;
            ESP_LOGW("WIFI", "Mat ket noi, thu lai %d/%d...",
                     wifi_retry_count, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE("WIFI", "Khong the ket noi sau %d lan.", WIFI_MAX_RETRY);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_connected   = true;
        wifi_retry_count = 0;
        ESP_LOGI("WIFI", "Da ket noi WiFi!");
    }
}

static void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               wifi_event_handler, NULL);
    wifi_config_t wcfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();
}

/* ============================================================
 * KHỐI XỬ LÝ MQTT (MỚI)
 * ============================================================ */
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("MQTT", "Da ket noi Broker!");
            mqtt_connected = true;
            // Đăng ký kênh nhận lệnh từ Web ngay khi kết nối thành công
            esp_mqtt_client_subscribe(mqtt_client, "dut/smartclock/trung/command", 0);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW("MQTT", "Mat ket noi Broker!");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_DATA:
            // ĐÂY LÀ NƠI NHẬN LỆNH TỪ WEB (Độ trễ ~0.1 giây)
            ESP_LOGI("MQTT", "Co lenh tu Web!");
            char resp[160] = {0};
            snprintf(resp, sizeof(resp), "%.*s", event->data_len, event->data);
            
            web_ctrl_t ctrl = { .cmd = WEB_CMD_NONE };
            
            // Phân tích lệnh JSON giống hệt như cách cũ của bạn
            if      (strstr(resp, "\"START\"")) ctrl.cmd = WEB_CMD_START;
            else if (strstr(resp, "\"STOP\""))  ctrl.cmd = WEB_CMD_STOP;
            else if (strstr(resp, "\"RESET\"")) ctrl.cmd = WEB_CMD_RESET;
            else if (strstr(resp, "\"COLOR\"")) {
                ctrl.cmd = WEB_CMD_COLOR;
                int r=0, g=0, b=0;
                char *pr = strstr(resp, "\"r\":"); if (pr) sscanf(pr+4, "%d", &r);
                char *pg = strstr(resp, "\"g\":"); if (pg) sscanf(pg+4, "%d", &g);
                char *pb = strstr(resp, "\"b\":"); if (pb) sscanf(pb+4, "%d", &b);
                ctrl.r = (uint8_t)(r & 0xFF);
                ctrl.g = (uint8_t)(g & 0xFF);
                ctrl.b = (uint8_t)(b & 0xFF);
            }
            if (ctrl.cmd != WEB_CMD_NONE) {
                ESP_LOGI("MQTT", "Thuc thi: cmd=%d r=%d g=%d b=%d", ctrl.cmd, ctrl.r, ctrl.g, ctrl.b);
                xQueueSend(Queue_WebCtrl, &ctrl, 0); // Đẩy lệnh vào Queue cho Task_Logic xử lý
            }
            break;
            
        default:
            break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.emqx.io:1883", // Trạm bưu điện trung tâm
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/* ============================================================
 * TASK 7: Task_WiFi (Phiên bản MQTT siêu nhẹ)
 * ============================================================ */
void task_wifi(void *pv) {
    wifi_init_sta(); // Chờ kết nối WiFi cục bộ
    
    // Phải có mạng WiFi thì mới bật MQTT
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    mqtt_app_start();

    wifi_data_t wdata;
    while (1) {
        // Tốc độ cập nhật: 1 giây / 1 lần
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        
        if (!mqtt_connected) continue;

        // Nếu có dữ liệu mới thì đóng gói và BẮN lên Web
        if (xQueuePeek(Queue_WiFi, &wdata, 0) == pdTRUE) {
            uint8_t lr=0, lg=0, lb=0;
            if (g_manual_color) {
                lr=g_manual_r; lg=g_manual_g; lb=g_manual_b;
            } else {
                switch (wdata.current_state) {
                    case STATE_WORK:  lr=200; lg=220; lb=255; break;
                    case STATE_BREAK: lr=255; lg=200; lb= 80; break;
                    case STATE_PAUSE: lr=180; lg=100; lb= 30; break;
                    default:          lr=  0; lg=  0; lb=  0; break;
                }
            }
            
            uint8_t brt = (uint8_t)(100.0f - (g_lux > 1000.0f ? 1000.0f : g_lux) * 70.0f / 1000.0f);
            const char *s = (wdata.current_state == STATE_WORK)  ? "WORK"  :
                            (wdata.current_state == STATE_BREAK) ? "BREAK" :
                            (wdata.current_state == STATE_PAUSE) ? "PAUSE" : "IDLE";
            
            // Format JSON giữ nguyên 100% như cũ để Web không bị sốc
            char payload[320];
            snprintf(payload, sizeof(payload),
                "{\"state\":\"%s\",\"remaining_sec\":%lu,\"lux\":%.1f,\"led_brt\":%u,"
                "\"hour\":%u,\"min\":%u,\"sec\":%u,\"date\":%u,\"month\":%u,\"year\":%u,"
                "\"sessions_today\":%u,\"total_time_sec\":%lu,\"led_r\":%u,\"led_g\":%u,\"led_b\":%u}",
                s, (unsigned long)g_remain_sec, g_lux, brt,
                (unsigned)g_rtc_time.hour, (unsigned)g_rtc_time.min, (unsigned)g_rtc_time.sec,
                (unsigned)g_rtc_time.date, (unsigned)g_rtc_time.month, (unsigned)g_rtc_time.year,
                (unsigned)wdata.sessions_today, (unsigned long)wdata.total_time_sec,
                (unsigned)lr, (unsigned)lg, (unsigned)lb);

            // Bắn một phát lên kênh Status ngay lập tức
            esp_mqtt_client_publish(mqtt_client, "dut/smartclock/trung/status", payload, 0, 0, 0);
        }
    }
}
/* ============================================================
 *  APP_MAIN
 * ============================================================ */
void app_main(void) {
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* I2C */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C OK");

    /* ============================================================
     *  SET GIỜ DS3231
     *
     *  Cách dùng đúng:
     *  1. Đổi SET_RTC_TIME → 1, chỉnh giờ bên dưới cho đúng thực tế
     *  2. Build + Flash → Serial Monitor hiện "Xac nhan: XX:XX:XX..."
     *  3. Đổi SET_RTC_TIME → 0 ngay lập tức
     *  4. Build + Flash lần nữa → xong, DS3231 giữ giờ bằng pin CR2032
     *
     *  KHÔNG để SET_RTC_TIME = 1 khi hoạt động bình thường!
     * ============================================================ */
#define SET_RTC_TIME  0/* <<< 1 = set giờ, 0 = đọc giờ bình thường */

#if SET_RTC_TIME
    /* Chỉnh giờ/phút/giây/ngày/tháng/năm cho đúng thời điểm nạp firmware */
    rtc_time_t setup_time = {
        .sec   = 0,
        .min   = 15,      /* <<< SỬA */
        .hour  = 3,      /* <<< SỬA */
        .day   = 1,      /* 1=CN, 2=T2 ... 7=T7 */
        .date  = 20,     /* <<< SỬA ngày */
        .month = 4,      /* <<< SỬA tháng */
        .year  = 26,     /* 2026 → ghi 26 */
    };
    if (ds3231_set_time(&setup_time) == ESP_OK) {
        ESP_LOGI("RTC", ">>> Set gio OK! Doi SET_RTC_TIME=0 roi flash lai <<<");
    } else {
        ESP_LOGE("RTC", "Set gio THAT BAI - kiem tra I2C va gia tri gio!");
    }
    /* Dừng lại sau khi set để không tiếp tục khởi động
     * → buộc người dùng flash lại với SET_RTC_TIME=0 */
    ESP_LOGW("RTC", "Vui long doi SET_RTC_TIME=0 va flash lai!");
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
#else
    /* Đọc và in giờ hiện tại để xác nhận DS3231 hoạt động đúng */
    {
        rtc_time_t cur = {0};
        vTaskDelay(pdMS_TO_TICKS(100)); /* chờ I2C ổn định */
        if (ds3231_read_time(&cur) == ESP_OK) {
            ESP_LOGI("RTC", "Gio hien tai: %02d:%02d:%02d  %02d/%02d/20%02d",
                     cur.hour, cur.min, cur.sec,
                     cur.date, cur.month, cur.year);
            /* Cảnh báo nếu giờ bằng 0 — có thể pin yếu hoặc chưa set */
            if (cur.hour == 0 && cur.min == 0 && cur.sec == 0 &&
                cur.date == 1 && cur.month == 1) {
                ESP_LOGW("RTC", "CANH BAO: Gio dang la 00:00:00 01/01 - "
                         "co the pin het hoac chua set gio!");
            }
        } else {
            ESP_LOGE("RTC", "Doc gio THAT BAI - kiem tra ket noi DS3231!");
        }
    }
#endif

    /* GPIO nút nhấn TTP223 */
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_POSEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BTN_START_GPIO) |
                        (1ULL<<BTN_STOP_GPIO)  |
                        (1ULL<<BTN_RESET_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    /* GPIO Buzzer */
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BUZZER_GPIO);
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, 0);

    /* Tạo Queue */
    Queue_Input   = xQueueCreate(10, sizeof(btn_cmd_t));
    Queue_Command = xQueueCreate(5,  sizeof(btn_cmd_t));
    Queue_Sensor  = xQueueCreate(1,  sizeof(float));
    Queue_Light   = xQueueCreate(1,  sizeof(led_cmd_t));
    Queue_Display = xQueueCreate(1,  sizeof(disp_data_t));
    Queue_WiFi    = xQueueCreate(1,  sizeof(wifi_data_t));
    Queue_WebCtrl = xQueueCreate(5,  sizeof(web_ctrl_t));

    /* Tạo Mutex & Semaphore */
    Mutex_I2C   = xSemaphoreCreateMutex();
    Mutex_State = xSemaphoreCreateMutex();
    Sem_Alert   = xSemaphoreCreateBinary();

    configASSERT(Queue_Input && Queue_Command && Queue_Sensor &&
                 Queue_Light && Queue_Display && Queue_WiFi  &&
                 Queue_WebCtrl && Mutex_I2C && Mutex_State && Sem_Alert);

    /* ISR nút nhấn */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_START_GPIO, gpio_isr_handler, (void*)BTN_START_GPIO);
    gpio_isr_handler_add(BTN_STOP_GPIO,  gpio_isr_handler, (void*)BTN_STOP_GPIO);
    gpio_isr_handler_add(BTN_RESET_GPIO, gpio_isr_handler, (void*)BTN_RESET_GPIO);

    /* Tạo Task */
    xTaskCreate(task_logic,   "Task_Logic",   4096, NULL, 5, NULL);
    xTaskCreate(task_alert,   "Task_Alert",   2048, NULL, 4, NULL);
    xTaskCreate(task_display, "Task_Display", 4096, NULL, 4, NULL);
    xTaskCreate(task_sensor,  "Task_Sensor",  3072, NULL, 4, NULL);
    xTaskCreate(task_light,   "Task_Light",   3072, NULL, 3, NULL);
    xTaskCreate(task_input,   "Task_Input",   2048, NULL, 3, NULL);
    xTaskCreate(task_wifi,    "Task_WiFi",    8192, NULL, 1, NULL);

    ESP_LOGI(TAG, "=== Smart Study san sang! ===");
}