/**
 * ============================================================
 *  SMART STUDY ASSISTANT - Hệ Thống Nhúng Học Sâu
 *  Trường Đại học Bách Khoa Đà Nẵng - Khoa Điện tử Viễn thông
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
#include "nvs.h"
#include "esp_netif.h"
#include "mqtt_client.h"
static const char *TAG = "SMART_STUDY";

#define BTN_START_GPIO      4
#define BTN_STOP_GPIO       5
#define BTN_RESET_GPIO      6
#define BUZZER_GPIO         7
#define I2C_MASTER_SDA      2
#define I2C_MASTER_SCL      3
#define I2C_MASTER_NUM      I2C_NUM_0
#define LED_DATA_GPIO       8
#define LED_COUNT           100

#define DS3231_ADDR         0x68
#define OLED_ADDR           0x3C
#define BH1750_ADDR         0x23

#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      pdMS_TO_TICKS(1000)

#define WIFI_SSID           "Van Lan"
#define WIFI_PASS           "23091969"
#define SERVER_URL          "https://web-study-lamp.onrender.com"

static volatile uint32_t g_work_min  = 25;
static volatile uint32_t g_break_min = 5;
static uint8_t g_last_date = 0;
#define BCD2DEC(v)  ((((v) >> 4) & 0x0F) * 10 + ((v) & 0x0F))
#define DEC2BCD(v)  ((((v) / 10) << 4)   | ((v) % 10))

typedef enum { CMD_START, CMD_STOP, CMD_RESET } btn_cmd_t;
typedef enum { STATE_IDLE, STATE_WORK, STATE_BREAK, STATE_PAUSE } clock_state_t;
typedef enum { ALERT_NONE, ALERT_WORK_END, ALERT_BREAK_START } alert_code_t;

typedef enum {
    WEB_CMD_NONE = 0,
    WEB_CMD_START, WEB_CMD_STOP, WEB_CMD_RESET, WEB_CMD_COLOR,
    WEB_CMD_SET_TIME, WEB_CMD_SET_RTC
} web_cmd_t;

typedef struct {
    web_cmd_t cmd;
    uint8_t r, g, b;
    uint16_t work_min, break_min;
    uint8_t hour, min, sec, date, month, year;
} web_ctrl_t;

typedef struct { uint8_t r, g, b, brightness; } led_cmd_t;

typedef struct {
    clock_state_t state;
    uint32_t remaining_sec;
    float    lux;
    uint8_t  led_brt;
    uint8_t  hour, min, sec;
    uint8_t  date, month, year;
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
 *  OLED DRIVER (giữ nguyên)
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

/* ============================================================
 *  ██████████████████████████████████████████████████████████
 *  FRAMEBUFFER + LOPAKA DISPLAY ENGINE  ← THÊM MỚI HOÀN TOÀN
 *  ██████████████████████████████████████████████████████████
 *
 *  Đây là phần DUY NHẤT thay đổi so với firmware gốc của bạn.
 *  Tất cả code bên dưới đây đến hết task_display() là MỚI.
 * ============================================================ */

/* Framebuffer 128×64 pixel trong RAM */
static uint8_t fb[8][128];

static void fb_clear(void) {
    memset(fb, 0, sizeof(fb));
}

/* Ghi 1 pixel vào framebuffer */
static void fb_pixel(int x, int y, uint8_t on) {
    if (x < 0 || x > 127 || y < 0 || y > 63) return;
    if (on) fb[y/8][x] |=  (1 << (y%8));
    else    fb[y/8][x] &= ~(1 << (y%8));
}

/* Đẩy toàn bộ framebuffer lên OLED 1 lần — không nhấp nháy */
static void fb_flush(void) {
    for (uint8_t page = 0; page < 8; page++) {
        oled_send_cmd(0xB0 | page);
        oled_send_cmd(0x00);
        oled_send_cmd(0x10);
        for (uint8_t col = 0; col < 128; col++) {
            oled_send_data(fb[page][col]);
        }
    }
}

/* ── Primitives ───────────────────────────────────────────── */
static void fb_hline(int x0, int x1, int y, uint8_t on) {
    if (x0 > x1) { int t=x0; x0=x1; x1=t; }
    for (int x = x0; x <= x1; x++) fb_pixel(x, y, on);
}
static void fb_vline(int x, int y0, int y1, uint8_t on) {
    if (y0 > y1) { int t=y0; y0=y1; y1=t; }
    for (int y = y0; y <= y1; y++) fb_pixel(x, y, on);
}

/* Hình chữ nhật bo góc — viền */
static void fb_round_rect(int x, int y, int w, int h, int r) {
    fb_hline(x+r, x+w-r-1, y,     1);
    fb_hline(x+r, x+w-r-1, y+h-1, 1);
    fb_vline(x,     y+r, y+h-r-1, 1);
    fb_vline(x+w-1, y+r, y+h-r-1, 1);
    int cx=0, cy=r, err=1-r;
    while (cx <= cy) {
        fb_pixel(x+w-r-1+cx, y+r-1  -cy, 1);
        fb_pixel(x+r    -cx, y+r-1  -cy, 1);
        fb_pixel(x+w-r-1+cx, y+h-r-1+cy, 1);
        fb_pixel(x+r    -cx, y+h-r-1+cy, 1);
        fb_pixel(x+w-r-1+cy, y+r-1  -cx, 1);
        fb_pixel(x+r    -cy, y+r-1  -cx, 1);
        fb_pixel(x+w-r-1+cy, y+h-r-1+cx, 1);
        fb_pixel(x+r    -cy, y+h-r-1+cx, 1);
        if (err < 0) {
            err += 2*cx+3;
        } else {
            err += 2*(cx-cy)+5;
            cy--;
        }
        cx++;
    }
}

/* Hình chữ nhật bo góc — đặc (fill) */
static void fb_fill_round_rect(int x, int y, int w, int h, int r) {
    for (int row = y+r; row <= y+h-r-1; row++) fb_hline(x, x+w-1, row, 1);
    int cx=0, cy=r, err=1-r;
    while (cx <= cy) {
        fb_hline(x+r-cy, x+w-r-1+cy, y+r-cx,     1);
        fb_hline(x+r-cx, x+w-r-1+cx, y+r-cy,     1);
        fb_hline(x+r-cy, x+w-r-1+cy, y+h-r-1+cx, 1);
        fb_hline(x+r-cx, x+w-r-1+cx, y+h-r-1+cy, 1);
        if (err < 0) {
            err += 2*cx+3;
        } else {
            err += 2*(cx-cy)+5;
            cy--;
        }
        cx++;
    }
}

/* Vòng tròn rỗng */
static void fb_circle(int cx, int cy, int r) {
    int x=0, y=r, err=1-r;
    while (x <= y) {
        fb_pixel(cx+x,cy+y,1); fb_pixel(cx-x,cy+y,1);
        fb_pixel(cx+x,cy-y,1); fb_pixel(cx-x,cy-y,1);
        fb_pixel(cx+y,cy+x,1); fb_pixel(cx-y,cy+x,1);
        fb_pixel(cx+y,cy-x,1); fb_pixel(cx-y,cy-x,1);
        if (err < 0) {
            err += 2*x+3;
        } else {
            err += 2*(x-y)+5;
            y--;
        }
        x++;
    }
}

/* ── Text size 1 (font5x8, 6px/char) ─────────────────────── */
static void fb_text(int x, int y, const char *str) {
    while (*str) {
        uint8_t c = (uint8_t)(*str++);
        if (c < 32 || c > 126) { x += 6; continue; }
        const uint8_t *g = font5x8[c-32];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (int bit = 0; bit < 8; bit++)
                if (bits & (1<<bit)) fb_pixel(x+col, y+bit, 1);
        }
        x += 6;
    }
}

/* ── Text size 2 (scale ×2, 12px/char) — dùng cho RTC ────── */
static void fb_text_x2(int x, int y, const char *str) {
    while (*str) {
        uint8_t c = (uint8_t)(*str++);
        if (c < 32 || c > 126) { x += 12; continue; }
        const uint8_t *g = font5x8[c-32];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (int bit = 0; bit < 8; bit++) {
                if (bits & (1<<bit)) {
                    fb_pixel(x+col*2,   y+bit*2,   1);
                    fb_pixel(x+col*2+1, y+bit*2,   1);
                    fb_pixel(x+col*2,   y+bit*2+1, 1);
                    fb_pixel(x+col*2+1, y+bit*2+1, 1);
                }
            }
        }
        x += 12;
    }
}

/* ── Format tổng thời gian học ──────────────────────────── */
static void fmt_total(char *buf, size_t sz, uint32_t sec) {
    uint32_t h = sec / 3600;
    uint32_t m = (sec % 3600) / 60;
    if (h > 0) snprintf(buf, sz, "%luh%02lu", (unsigned long)h, (unsigned long)m);
    else        snprintf(buf, sz, "%02lu:%02lu",(unsigned long)m,(unsigned long)(sec%60));
}

/* ── Vẽ 1 frame hoàn chỉnh theo Lopaka layout ────────────── */
static void draw_frame(const disp_data_t *d) {
    char buf[24];
    fb_clear();

    /* ① RTC — setCursor(17,3) + setTextSize(2) */
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
             (unsigned)d->hour, (unsigned)d->min, (unsigned)d->sec);
    fb_text_x2(17, 3, buf);

    /* ② Đường kẻ ngang — drawLine(1,19,128,19) */
    fb_hline(1, 127, 19, 1);

    /* ③ Badge trạng thái — drawRoundRect(3,22,36,13,4) */
    const char *state_str;
    int badge_w;
    switch (d->state) {
        case STATE_WORK:  state_str="WORK";  badge_w=36; break;
        case STATE_BREAK: state_str="BREAK"; badge_w=42; break;
        case STATE_PAUSE: state_str="PAUSE"; badge_w=42; break;
        default:          state_str="IDLE";  badge_w=30; break;
    }
    fb_round_rect(3, 22, badge_w, 13, 4);
    /* Text bên trong badge — setCursor(9,26) */
    fb_text(9, 26, state_str);

    /* ④ Dấu chấm tròn — drawCircle(38,30,1) */
    if (d->state != STATE_IDLE) {
        fb_circle(38, 30, 1);
    }

    /* ⑤ Pomodoro countdown — setCursor(85,23) */
    if (d->state != STATE_IDLE) {
        snprintf(buf, sizeof(buf), "%02u:%02u",
                 (unsigned)(d->remaining_sec/60),
                 (unsigned)(d->remaining_sec%60));
    } else {
        snprintf(buf, sizeof(buf), "%02lu:00", (unsigned long)g_work_min);
    }
    fb_text(85, 23, buf);

    /* ⑥ Progress bar
     *   Viền — drawRoundRect(1,36,126,10,4)
     *   Fill — fillRoundRect(2,37,fill_w,8,3)  remaining → đầy=còn nhiều */
    fb_round_rect(1, 36, 126, 10, 4);
    if (d->state != STATE_IDLE) {
        uint32_t total_s = (d->state == STATE_BREAK)
                           ? g_break_min * 60
                           : g_work_min  * 60;
        if (total_s > 0 && d->remaining_sec > 0) {
            int fill_w = (int)((uint32_t)d->remaining_sec * 122 / total_s);
            if (fill_w > 0) fb_fill_round_rect(2, 37, fill_w, 8, 3);
        }
    }

    /* ⑦ Đường kẻ ngang footer — drawLine(1,47,129,47) */
    fb_hline(1, 127, 47, 1);

    /* ⑧ Đường kẻ dọc phân 3 cột — drawLine(43,48,43,63) & (87,47,87,65) */
    fb_vline(43, 48, 63, 1);
    fb_vline(87, 47, 63, 1);

    /* ⑨ Cột 1: LUX — setCursor(4,49) label, setCursor(24,56) value */
    fb_text(4, 49, "Lux");
    if (d->lux >= 0)
        snprintf(buf, sizeof(buf), "%4.0f", d->lux);
    else
        snprintf(buf, sizeof(buf), " ---");
    fb_text(4, 57, buf);

    /* ⑩ Cột 2: BRT — setCursor(48,49) label, setCursor(66,57) value */
    fb_text(48, 49, "BRT");
    snprintf(buf, sizeof(buf), "%3u%%", (unsigned)d->led_brt);
    fb_text(50, 57, buf);

    /* ⑪ Cột 3: TOTAL — setCursor(89,49) label, setCursor(104,57) value */
    fb_text(89, 49, "TOTAL");
    fmt_total(buf, sizeof(buf), g_total_sec);
    fb_text(92, 57, buf);

    /* Flush toàn bộ framebuffer lên OLED 1 lần */
    fb_flush();
}

/* ============================================================
 *  TASK 4: Task_Display  ← CHỈ HÀM NÀY THAY ĐỔI SO VỚI GỐC
 * ============================================================ */
void task_display(void *pv) {
    if (xSemaphoreTake(Mutex_I2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
        oled_init();
        xSemaphoreGive(Mutex_I2C);
    }

    disp_data_t data;

    while (1) {
        if (xQueueReceive(Queue_Display, &data, pdMS_TO_TICKS(200)) != pdTRUE) continue;
        if (xSemaphoreTake(Mutex_I2C,   pdMS_TO_TICKS(200)) != pdTRUE) continue;

        /* Điều chỉnh độ sáng OLED theo lux */
        uint8_t oled_brt;
        if      (data.lux < 15.0f)  oled_brt = 10;
        else if (data.lux < 150.0f) oled_brt = 128;
        else                         oled_brt = 255;
        oled_set_brightness(oled_brt);

        /* Vẽ frame và đẩy lên màn hình */
        draw_frame(&data);

        xSemaphoreGive(Mutex_I2C);
    }
}
/* ============================================================
 *  ██████████████████████████████████████████████████████████
 *  KẾT THÚC PHẦN THAY ĐỔI — TẤT CẢ CODE BÊN DƯỚI GIỮ NGUYÊN
 *  ██████████████████████████████████████████████████████████
 * ============================================================ */

/* ============================================================
 *  DS3231 RTC DRIVER
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
    if (t->sec > 59 || t->min > 59 || t->hour > 23 ||
        t->date < 1 || t->date > 31 ||
        t->month < 1 || t->month > 12 || t->year > 99) {
        ESP_LOGE("RTC", "Gio khong hop le!");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[8];
    buf[0]=0x00; buf[1]=DEC2BCD(t->sec);   buf[2]=DEC2BCD(t->min);
    buf[3]=DEC2BCD(t->hour); buf[4]=DEC2BCD(t->day);
    buf[5]=DEC2BCD(t->date); buf[6]=DEC2BCD(t->month); buf[7]=DEC2BCD(t->year);
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, DS3231_ADDR, buf, 8, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
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
    uint8_t cmd; esp_err_t err;
    cmd=0x01; err=i2c_master_write_to_device(I2C_MASTER_NUM,BH1750_ADDR,&cmd,1,pdMS_TO_TICKS(200));
    if(err!=ESP_OK){bh1750_ready=false;return;}
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd=0x07; err=i2c_master_write_to_device(I2C_MASTER_NUM,BH1750_ADDR,&cmd,1,pdMS_TO_TICKS(200));
    if(err!=ESP_OK){bh1750_ready=false;return;}
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd=0x10; err=i2c_master_write_to_device(I2C_MASTER_NUM,BH1750_ADDR,&cmd,1,pdMS_TO_TICKS(200));
    if(err!=ESP_OK){bh1750_ready=false;return;}
    vTaskDelay(pdMS_TO_TICKS(200));
    bh1750_ready=true;
    ESP_LOGI("BH1750","GY-302 khoi tao OK!");
}

static float bh1750_read_lux(void) {
    if(!bh1750_ready){bh1750_init();if(!bh1750_ready)return -1.0f;}
    uint8_t data[2]={0,0};
    esp_err_t err=i2c_master_read_from_device(I2C_MASTER_NUM,BH1750_ADDR,data,2,pdMS_TO_TICKS(300));
    if(err!=ESP_OK){bh1750_ready=false;return -1.0f;}
    uint16_t raw=((uint16_t)data[0]<<8)|data[1];
    if(raw==0||raw==0xFFFF)return -1.0f;
    return (float)raw/1.2f;
}

/* ============================================================
 *  WS2812B DRIVER (RMT)
 * ============================================================ */
static void ws2812_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num=LED_DATA_GPIO, .max_leds=LED_COUNT,
        .led_pixel_format=LED_PIXEL_FORMAT_GRB,
        .led_model=LED_MODEL_WS2812, .flags.invert_out=false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src=RMT_CLK_SRC_DEFAULT,
        .resolution_hz=10*1000*1000, .flags.with_dma=false,
    };
    esp_err_t err=led_strip_new_rmt_device(&strip_cfg,&rmt_cfg,&led_strip);
    if(err!=ESP_OK){ESP_LOGE("LED","WS2812B init that bai");return;}
    led_strip_clear(led_strip);
    ESP_LOGI("LED","WS2812B OK - %d LED",LED_COUNT);
}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brt_pct) {
    if(!led_strip)return;
    uint8_t fr=(uint8_t)((uint32_t)r*brt_pct/100);
    uint8_t fg=(uint8_t)((uint32_t)g*brt_pct/100);
    uint8_t fb_=(uint8_t)((uint32_t)b*brt_pct/100);
    for(int i=0;i<LED_COUNT;i++) led_strip_set_pixel(led_strip,i,fr,fg,fb_);
    led_strip_refresh(led_strip);
}

static void led_flash_alert(void) {
    if(!led_strip)return;
    for(int i=0;i<3;i++){
       for(int j=0;j<LED_COUNT;j++) led_strip_set_pixel(led_strip,j,20,20,20); 
        led_strip_refresh(led_strip); vTaskDelay(pdMS_TO_TICKS(200));
        led_strip_clear(led_strip);   led_strip_refresh(led_strip); vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ============================================================
 *  ISR + I2C INIT
 * ============================================================ */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num=(uint32_t)arg;
    btn_cmd_t cmd;
    if(gpio_num==BTN_START_GPIO) cmd=CMD_START;
    else if(gpio_num==BTN_STOP_GPIO) cmd=CMD_STOP;
    else cmd=CMD_RESET;
    xQueueSendFromISR(Queue_Input,&cmd,NULL);
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf={
        .mode=I2C_MODE_MASTER, .sda_io_num=I2C_MASTER_SDA,
        .scl_io_num=I2C_MASTER_SCL,
        .sda_pullup_en=GPIO_PULLUP_ENABLE, .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM,&conf);
    return i2c_driver_install(I2C_MASTER_NUM,conf.mode,0,0,0);
}

/* ============================================================
 *  TASK 1: Task_Input
 * ============================================================ */
void task_input(void *pv) {
    btn_cmd_t raw_cmd;
    TickType_t last_tick=0;
    const TickType_t debounce=pdMS_TO_TICKS(50);
    while(1){
        if(xQueueReceive(Queue_Input,&raw_cmd,portMAX_DELAY)){
            TickType_t now=xTaskGetTickCount();
            if((now-last_tick)>=debounce){
                last_tick=now;
                xQueueSend(Queue_Command,&raw_cmd,0);
                gpio_set_level(BUZZER_GPIO,1); vTaskDelay(pdMS_TO_TICKS(80));
                gpio_set_level(BUZZER_GPIO,0);
            }
        }
    }
}

/* ============================================================
 *  TASK 2: Task_Sensor
 * ============================================================ */
void task_sensor(void *pv) {
    if(xSemaphoreTake(Mutex_I2C,pdMS_TO_TICKS(1000))==pdTRUE){
        bh1750_init(); xSemaphoreGive(Mutex_I2C);
    }
    while(1){
        vTaskDelay(pdMS_TO_TICKS(200));
        float lux;
        if(xSemaphoreTake(Mutex_I2C,pdMS_TO_TICKS(1000))==pdTRUE){
            lux=bh1750_read_lux(); xSemaphoreGive(Mutex_I2C);
        } else continue;
        if(lux<0.0f) continue;
        xQueueOverwrite(Queue_Sensor,&lux);
    }
}

/* ============================================================
 *  NVS MEMORY (LƯU TRỮ DỮ LIỆU CHỐNG MẤT ĐIỆN)
 * ============================================================ */
static void save_study_data(void) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u32(my_handle, "total_sec", g_total_sec);
        nvs_set_u16(my_handle, "sessions", g_sessions);
        nvs_set_u32(my_handle, "work_min", g_work_min);
        nvs_set_u32(my_handle, "break_min", g_break_min);
        nvs_set_u8(my_handle, "last_date", g_last_date); // LƯU THÊM NGÀY
        nvs_commit(my_handle); 
        nvs_close(my_handle);
    }
}

static void load_study_data(void) {
    nvs_handle_t my_handle;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        uint32_t temp_sec = 0; uint16_t temp_ses = 0;
        uint32_t temp_work = 25; uint32_t temp_break = 5;
        uint8_t temp_date = 0;
        
        if (nvs_get_u32(my_handle, "total_sec", &temp_sec) == ESP_OK) g_total_sec = temp_sec;
        if (nvs_get_u16(my_handle, "sessions", &temp_ses) == ESP_OK)  g_sessions = temp_ses;
        if (nvs_get_u32(my_handle, "work_min", &temp_work) == ESP_OK) g_work_min = temp_work;
        if (nvs_get_u32(my_handle, "break_min", &temp_break) == ESP_OK) g_break_min = temp_break;
        if (nvs_get_u8(my_handle, "last_date", &temp_date) == ESP_OK) g_last_date = temp_date; // ĐỌC NGÀY LÊN
        nvs_close(my_handle);
    }
}
/* ============================================================
 *  TASK 3: Task_Logic
 * ============================================================ */
void task_logic(void *pv) {
    load_study_data();
    clock_state_t state=STATE_IDLE;
    uint32_t remain_sec=0;
    uint8_t led_brt=20;
    alert_code_t pending_alert=ALERT_NONE;
    rtc_time_t local_time={0};
    btn_cmd_t cmd;
    web_ctrl_t wctrl;
    float lux_val=0.0f;
    TickType_t last_sec_tick=xTaskGetTickCount();

    while(1){
        /* Nút nhấn */
      /* Nút nhấn */
        if(xQueueReceive(Queue_Command,&cmd,0)==pdTRUE){
            switch(cmd){
                case CMD_START:
                    if(state==STATE_IDLE||state==STATE_BREAK){
                        state=STATE_WORK; remain_sec=g_work_min*60;
                    } else if(state==STATE_PAUSE) {
                        state=STATE_WORK; // Đã thêm ngoặc nhọn ở đây
                    }
                    g_manual_color=false; break;
                case CMD_STOP:
                    if(state==STATE_WORK||state==STATE_BREAK) {
                        state=STATE_PAUSE; // Thêm ngoặc cho an toàn
                    }
                    break;
                case CMD_RESET:
                    state=STATE_IDLE; remain_sec=0;
                     g_manual_color=false;
                    save_study_data();
                     break;
            }
        }

        /* Web */
        if(xQueueReceive(Queue_WebCtrl,&wctrl,0)==pdTRUE){
            // --- THÊM ĐOẠN CODE NÀY ĐỂ KÍCH HOẠT CÒI ---
            // Nếu lệnh từ Web là Start, Stop hoặc Reset thì cho còi kêu 1 tiếng "bíp" ngắn
            if (wctrl.cmd == WEB_CMD_START || wctrl.cmd == WEB_CMD_STOP || wctrl.cmd == WEB_CMD_RESET) {
                gpio_set_level(BUZZER_GPIO, 1); 
                vTaskDelay(pdMS_TO_TICKS(80)); // Kêu trong 80ms
                gpio_set_level(BUZZER_GPIO, 0);
            }
            // ------------------------------------------
            switch(wctrl.cmd){
                case WEB_CMD_START:
                    if(state==STATE_IDLE||state==STATE_BREAK){
                        state=STATE_WORK; remain_sec=g_work_min*60;
                    } else if(state==STATE_PAUSE) {
                        state=STATE_WORK; // Đã thêm ngoặc nhọn ở đây
                    }
                    g_manual_color=false; break;
                case WEB_CMD_STOP:
                    if(state==STATE_WORK||state==STATE_BREAK) {
                        state=STATE_PAUSE; // Thêm ngoặc cho an toàn
                    }
                    break;
                case WEB_CMD_RESET:
                    state=STATE_IDLE; remain_sec=0;
                    g_manual_color=false; 
                    save_study_data();
                    break;
                case WEB_CMD_COLOR:
                    g_manual_color=true;
                    g_manual_r=wctrl.r; g_manual_g=wctrl.g; g_manual_b=wctrl.b; break;
                case WEB_CMD_SET_TIME:
                    g_work_min=wctrl.work_min; g_break_min=wctrl.break_min;
                    save_study_data();
                     break;
                case WEB_CMD_SET_RTC:
                    if(xSemaphoreTake(Mutex_I2C,pdMS_TO_TICKS(100))==pdTRUE){
                        rtc_time_t nr={
                            .hour=wctrl.hour,.min=wctrl.min,.sec=wctrl.sec,
                            .date=wctrl.date,.month=wctrl.month,.year=wctrl.year,.day=1
                        };
                        ds3231_set_time(&nr); xSemaphoreGive(Mutex_I2C);
                    } break;
                default: break;
            }
        }

        /* Cảm biến lux */
        if(xQueueReceive(Queue_Sensor,&lux_val,0)==pdTRUE){
            g_lux=lux_val;
            float clamped=lux_val>1000.0f?1000.0f:lux_val;
            led_brt=(uint8_t)(100.0f-clamped*70.0f/1000.0f);
        }

        /* Đếm ngược */
        TickType_t now=xTaskGetTickCount();
        if((now-last_sec_tick)>=pdMS_TO_TICKS(1000)){
            last_sec_tick=now;
            if((state==STATE_WORK||state==STATE_BREAK)&&remain_sec>0){
                remain_sec--;
                if(state==STATE_WORK) g_total_sec++;
                if (g_total_sec % 60 == 0) {
                        save_study_data();
                    }
            }
            if(remain_sec==0&&(state==STATE_WORK||state==STATE_BREAK)){
                if(state==STATE_WORK){
                    pending_alert=ALERT_WORK_END; g_sessions++;
                    state=STATE_BREAK; remain_sec=g_break_min*60;
                } else {
                    pending_alert=ALERT_BREAK_START;
                    state=STATE_IDLE; remain_sec=0;
                }
                g_alert_code=pending_alert; xSemaphoreGive(Sem_Alert);
                save_study_data();
            }
        }

        if(xSemaphoreTake(Mutex_State,pdMS_TO_TICKS(10))==pdTRUE){
            g_state=state; g_remain_sec=remain_sec; xSemaphoreGive(Mutex_State);
        }
        if(xSemaphoreTake(Mutex_I2C,pdMS_TO_TICKS(20))==pdTRUE){
            ds3231_read_time(&local_time); g_rtc_time=local_time; xSemaphoreGive(Mutex_I2C);
            /* === BỘ DÒ NỬA ĐÊM TỰ RESET === */
        if (local_time.year > 0) { // Đảm bảo đã đọc được giờ thật
            if (g_last_date == 0) {
                // Lần đầu cấp điện, lưu ngày hiện tại làm mốc
                g_last_date = local_time.date;
                save_study_data();
            } else if (local_time.date != g_last_date) {
                // ĐÃ QUA NGÀY MỚI! Tiến hành xóa thành tích
                g_total_sec = 0;
                g_sessions = 0;
                g_last_date = local_time.date; 
                save_study_data();
                ESP_LOGI(TAG, "Da qua nua dem! Reset Total ve 0");
            }
        }
        }

        /* LED màu */
        led_cmd_t led_cmd; led_cmd.brightness=led_brt;
        if(g_manual_color){
            led_cmd.r=g_manual_r; led_cmd.g=g_manual_g; led_cmd.b=g_manual_b;
        } else {
            switch(state){
                case STATE_WORK:  led_cmd.r=200;led_cmd.g=220;led_cmd.b=255; break;
                case STATE_BREAK: led_cmd.r=255;led_cmd.g=200;led_cmd.b=80;  break;
                case STATE_PAUSE: led_cmd.r=180;led_cmd.g=100;led_cmd.b=30;
                                  led_cmd.brightness=led_brt/3; break;
                default: led_cmd.r=0;led_cmd.g=0;led_cmd.b=0;led_cmd.brightness=0; break;
            }
        }
        xQueueOverwrite(Queue_Light,&led_cmd);

        disp_data_t disp={
            .state=state, .remaining_sec=remain_sec, .lux=g_lux, .led_brt=led_brt,
            .hour=local_time.hour, .min=local_time.min, .sec=local_time.sec,
            .date=local_time.date, .month=local_time.month, .year=local_time.year,
        };
        xQueueOverwrite(Queue_Display,&disp);

        wifi_data_t wdata={.sessions_today=g_sessions,.total_time_sec=g_total_sec,.current_state=state};
        xQueueOverwrite(Queue_WiFi,&wdata);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
 *  TASK 5: Task_Light
 * ============================================================ */
void task_light(void *pv) {
    ws2812_init();
    led_cmd_t cmd;
    while(1){
        if(xQueueReceive(Queue_Light,&cmd,portMAX_DELAY)==pdTRUE)
            led_set_color(cmd.r,cmd.g,cmd.b,cmd.brightness);
    }
}

/* ============================================================
 *  TASK 6: Task_Alert
 * ============================================================ */
void task_alert(void *pv) {
    while(1){
        if(xSemaphoreTake(Sem_Alert,portMAX_DELAY)==pdTRUE){
            alert_code_t code=g_alert_code;
            if(code==ALERT_WORK_END){
                for(int i=0;i<3;i++){
                    gpio_set_level(BUZZER_GPIO,1); vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(BUZZER_GPIO,0); vTaskDelay(pdMS_TO_TICKS(200));
                }
            } else if(code==ALERT_BREAK_START){
                gpio_set_level(BUZZER_GPIO,1); vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(BUZZER_GPIO,0);
            }
            led_flash_alert();
        }
    }
}

/* ============================================================
 *  TASK 7: Task_WiFi + MQTT
 * ============================================================ */
static bool wifi_connected=false;
static int  wifi_retry_count=0;
#define WIFI_MAX_RETRY 5

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data){
    if(base==WIFI_EVENT&&id==WIFI_EVENT_STA_START){ wifi_retry_count=0; esp_wifi_connect(); }
    else if(base==WIFI_EVENT&&id==WIFI_EVENT_STA_DISCONNECTED){
        wifi_connected=false;
        if(wifi_retry_count<WIFI_MAX_RETRY){ wifi_retry_count++; esp_wifi_connect(); }
    } else if(base==IP_EVENT&&id==IP_EVENT_STA_GOT_IP){ wifi_connected=true; wifi_retry_count=0; }
}

static void wifi_init_sta(void){
    esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event_handler,NULL);
    esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event_handler,NULL);
    wifi_config_t wcfg={.sta={.ssid=WIFI_SSID,.password=WIFI_PASS}};
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA,&wcfg); esp_wifi_start();
}

static esp_mqtt_client_handle_t mqtt_client=NULL;
static bool mqtt_connected=false;

static void mqtt_event_handler(void *ha, esp_event_base_t base, int32_t eid, void *ed){
    esp_mqtt_event_handle_t event=ed;
    switch((esp_mqtt_event_id_t)eid){
        case MQTT_EVENT_CONNECTED:
            mqtt_connected=true;
            esp_mqtt_client_subscribe(mqtt_client,"dut/smartclock/trung/command",0); break;
        case MQTT_EVENT_DISCONNECTED: mqtt_connected=false; break;
        case MQTT_EVENT_DATA: {
            char resp[160]={0};
            snprintf(resp,sizeof(resp),"%.*s",event->data_len,event->data);
            web_ctrl_t ctrl={.cmd=WEB_CMD_NONE};
            if     (strstr(resp,"\"START\"")) ctrl.cmd=WEB_CMD_START;
            else if(strstr(resp,"\"STOP\""))  ctrl.cmd=WEB_CMD_STOP;
            else if(strstr(resp,"\"RESET\"")) ctrl.cmd=WEB_CMD_RESET;
            else if(strstr(resp,"\"COLOR\"")){
                ctrl.cmd=WEB_CMD_COLOR;
                int r=0,g=0,b=0;
                char *pr=strstr(resp,"\"r\":"); if(pr) sscanf(pr+4,"%d",&r);
                char *pg=strstr(resp,"\"g\":"); if(pg) sscanf(pg+4,"%d",&g);
                char *pb=strstr(resp,"\"b\":"); if(pb) sscanf(pb+4,"%d",&b);
                ctrl.r=(uint8_t)r; ctrl.g=(uint8_t)g; ctrl.b=(uint8_t)b;
            }
            else if(strstr(resp,"\"SET_TIME\"")){
                ctrl.cmd=WEB_CMD_SET_TIME; int w=25,b=5;
                char *pw=strstr(resp,"\"work\":"); if(pw) sscanf(pw+7,"%d",&w);
                char *pb=strstr(resp,"\"break\":"); if(pb) sscanf(pb+8,"%d",&b);
                ctrl.work_min=(uint16_t)w; ctrl.break_min=(uint16_t)b;
            }
            else if(strstr(resp,"\"SET_RTC\"")){
                ctrl.cmd=WEB_CMD_SET_RTC; int h=0,m=0,s=0,d=1,mo=1,y=0;
                char *ph=strstr(resp,"\"hour\":");  if(ph) sscanf(ph+7,"%d",&h);
                char *pm=strstr(resp,"\"min\":");   if(pm) sscanf(pm+6,"%d",&m);
                char *ps=strstr(resp,"\"sec\":");   if(ps) sscanf(ps+6,"%d",&s);
                char *pd=strstr(resp,"\"date\":");  if(pd) sscanf(pd+7,"%d",&d);
                char *pmo=strstr(resp,"\"month\":"); if(pmo) sscanf(pmo+8,"%d",&mo);
                char *py=strstr(resp,"\"year\":");  if(py) sscanf(py+7,"%d",&y);
                ctrl.hour=h; ctrl.min=m; ctrl.sec=s;
                ctrl.date=d; ctrl.month=mo; ctrl.year=y;
            }
            if(ctrl.cmd!=WEB_CMD_NONE) xQueueSend(Queue_WebCtrl,&ctrl,0);
            break;
        }
        default: break;
    }
}

static void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_cfg={.broker.address.uri="mqtt://broker.emqx.io:1883"};
    mqtt_client=esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client,ESP_EVENT_ANY_ID,mqtt_event_handler,NULL);
    esp_mqtt_client_start(mqtt_client);
}

void task_wifi(void *pv){
    wifi_init_sta();
    while(!wifi_connected) vTaskDelay(pdMS_TO_TICKS(1000));
    mqtt_app_start();

    wifi_data_t wdata;
    while(1){
       vTaskDelay(pdMS_TO_TICKS(500));
        if(!mqtt_connected) continue;
        if(xQueuePeek(Queue_WiFi,&wdata,0)==pdTRUE){
            uint8_t lr=0,lg=0,lb=0;
            if(g_manual_color){ lr=g_manual_r; lg=g_manual_g; lb=g_manual_b; }
            else {
                switch(wdata.current_state){
                    case STATE_WORK:  lr=200;lg=220;lb=255; break;
                    case STATE_BREAK: lr=255;lg=200;lb=80;  break;
                    case STATE_PAUSE: lr=180;lg=100;lb=30;  break;
                    default: break;
                }
            }
            uint8_t brt=(uint8_t)(100.0f-(g_lux>1000.0f?1000.0f:g_lux)*70.0f/1000.0f);
            const char *s=(wdata.current_state==STATE_WORK)?"WORK":
                          (wdata.current_state==STATE_BREAK)?"BREAK":
                          (wdata.current_state==STATE_PAUSE)?"PAUSE":"IDLE";
            char payload[320];
            snprintf(payload,sizeof(payload),
                "{\"state\":\"%s\",\"remaining_sec\":%lu,\"lux\":%.1f,\"led_brt\":%u,"
                "\"hour\":%u,\"min\":%u,\"sec\":%u,\"date\":%u,\"month\":%u,\"year\":%u,"
                "\"sessions_today\":%u,\"total_time_sec\":%lu,\"led_r\":%u,\"led_g\":%u,\"led_b\":%u}",
                s,(unsigned long)g_remain_sec,g_lux,brt,
                (unsigned)g_rtc_time.hour,(unsigned)g_rtc_time.min,(unsigned)g_rtc_time.sec,
                (unsigned)g_rtc_time.date,(unsigned)g_rtc_time.month,(unsigned)g_rtc_time.year,
                (unsigned)wdata.sessions_today,(unsigned long)wdata.total_time_sec,
                (unsigned)lr,(unsigned)lg,(unsigned)lb);
            esp_mqtt_client_publish(mqtt_client,"dut/smartclock/trung/status",payload,0,0,0);
        }
    }
}

/* ============================================================
 *  APP_MAIN
 * ============================================================ */
void app_main(void) {
    esp_err_t ret=nvs_flash_init();
    if(ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase(); nvs_flash_init();
    }
    ESP_ERROR_CHECK(i2c_master_init());

#define SET_RTC_TIME 0
#if SET_RTC_TIME
    rtc_time_t setup_time={.sec=0,.min=05,.hour=22,.day=1,.date=30,.month=4,.year=26};
    if(ds3231_set_time(&setup_time)==ESP_OK)
        ESP_LOGI("RTC","Set gio OK! Doi SET_RTC_TIME=0 roi flash lai");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
#else
    {
        rtc_time_t cur={0};
        vTaskDelay(pdMS_TO_TICKS(100));
        if(ds3231_read_time(&cur)==ESP_OK)
            ESP_LOGI("RTC","Gio: %02d:%02d:%02d  %02d/%02d/20%02d",
                     cur.hour,cur.min,cur.sec,cur.date,cur.month,cur.year);
        else
            ESP_LOGE("RTC","Doc gio THAT BAI!");
    }
#endif

    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_POSEDGE, .mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<BTN_START_GPIO)|(1ULL<<BTN_STOP_GPIO)|(1ULL<<BTN_RESET_GPIO),
        .pull_down_en=GPIO_PULLDOWN_DISABLE, .pull_up_en=GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    io_conf.intr_type=GPIO_INTR_DISABLE; io_conf.mode=GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask=(1ULL<<BUZZER_GPIO);
    gpio_config(&io_conf); gpio_set_level(BUZZER_GPIO,0);

    Queue_Input  =xQueueCreate(10,sizeof(btn_cmd_t));
    Queue_Command=xQueueCreate(5, sizeof(btn_cmd_t));
    Queue_Sensor =xQueueCreate(1, sizeof(float));
    Queue_Light  =xQueueCreate(1, sizeof(led_cmd_t));
    Queue_Display=xQueueCreate(1, sizeof(disp_data_t));
    Queue_WiFi   =xQueueCreate(1, sizeof(wifi_data_t));
    Queue_WebCtrl=xQueueCreate(5, sizeof(web_ctrl_t));
    Mutex_I2C  =xSemaphoreCreateMutex();
    Mutex_State=xSemaphoreCreateMutex();
    Sem_Alert  =xSemaphoreCreateBinary();

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_START_GPIO,gpio_isr_handler,(void*)BTN_START_GPIO);
    gpio_isr_handler_add(BTN_STOP_GPIO, gpio_isr_handler,(void*)BTN_STOP_GPIO);
    gpio_isr_handler_add(BTN_RESET_GPIO,gpio_isr_handler,(void*)BTN_RESET_GPIO);

    /* Tăng stack cho Task_Display vì có framebuffer 1KB */
    xTaskCreate(task_logic,  "Task_Logic",  4096,NULL,5,NULL);
    xTaskCreate(task_alert,  "Task_Alert",  2048,NULL,4,NULL);
    xTaskCreate(task_display,"Task_Display",5120,NULL,4,NULL); /* ← stack 5120 thay vì 4096 */
    xTaskCreate(task_sensor, "Task_Sensor", 3072,NULL,4,NULL);
    xTaskCreate(task_light,  "Task_Light",  3072,NULL,3,NULL);
    xTaskCreate(task_input,  "Task_Input",  2048,NULL,3,NULL);
    xTaskCreate(task_wifi,   "Task_WiFi",   8192,NULL,1,NULL);

    ESP_LOGI(TAG,"=== Smart Study san sang! ===");
}