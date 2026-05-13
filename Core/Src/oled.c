/* OLED SSD1306 驱动 — 软件 I2C (PB6=SCL, PB7=SDA) */
#include "oled.h"
#include <string.h>

#define OLED_ADDR  0x78  // 7-bit: 0x3C, shifted: 0x78

/* ── 软件 I2C 引脚操作 ── */
#define SCL_PORT  GPIOB
#define SDA_PORT  GPIOB
#define SCL_PIN   GPIO_PIN_6
#define SDA_PIN   GPIO_PIN_7

#define SCL_H  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define SCL_L  HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
#define SDA_H  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)
#define SDA_L  HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
#define SDA_R  HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN)

/* ── 帧缓冲 (128x64, 每字节 8 像素) ── */
static uint8_t framebuf[OLED_WIDTH * OLED_HEIGHT / 8];
static uint8_t oled_ok = 0;

/* ── 软件 I2C 时序 ── */
static void i2c_delay(void) {
    for (volatile int i = 0; i < 8; i++);
}

static void i2c_start(void) {
    SDA_H; SCL_H; i2c_delay();
    SDA_L; i2c_delay();
    SCL_L; i2c_delay();
}

static void i2c_stop(void) {
    SCL_L; i2c_delay();
    SDA_L; i2c_delay();
    SCL_H; i2c_delay();
    SDA_H; i2c_delay();
}

static int i2c_ack(void) {
    SCL_L; i2c_delay();
    SDA_H; i2c_delay();
    SCL_H; i2c_delay();
    int timeout = 100;
    while (SDA_R && timeout--);  // wait ACK with timeout
    SCL_L; i2c_delay();
    return timeout > 0;
}

static int i2c_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H; else SDA_L;
        i2c_delay();
        SCL_H; i2c_delay();
        SCL_L; i2c_delay();
        data <<= 1;
    }
    return i2c_ack();
}

static int i2c_write_cmd(uint8_t cmd) {
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR)) { i2c_stop(); return 0; }
    if (!i2c_write_byte(0x00)) { i2c_stop(); return 0; }
    if (!i2c_write_byte(cmd)) { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static int i2c_write_data(uint8_t *data, uint16_t len) {
    i2c_start();
    if (!i2c_write_byte(OLED_ADDR)) { i2c_stop(); return 0; }
    if (!i2c_write_byte(0x40)) { i2c_stop(); return 0; }
    for (uint16_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return 0; }
    }
    i2c_stop();
    return 1;
}

/* ── 初始化 ── */
void OLED_Init(void) {
    // 使能 GPIOB 时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = SCL_PIN | SDA_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);

    SDA_H; SCL_H;

    // 检测 OLED 是否存在
    i2c_start();
    int ok = i2c_write_byte(OLED_ADDR);
    i2c_stop();
    if (!ok) return;  // OLED 未连接, 静默跳过
    oled_ok = 1;

    // SSD1306 初始化序列
    i2c_write_cmd(0xAE);  // display off
    i2c_write_cmd(0xD5); i2c_write_cmd(0x80);  // clock divide
    i2c_write_cmd(0xA8); i2c_write_cmd(0x3F);  // mux ratio
    i2c_write_cmd(0xD3); i2c_write_cmd(0x00);  // display offset
    i2c_write_cmd(0x40);  // start line
    i2c_write_cmd(0x8D); i2c_write_cmd(0x14);  // charge pump
    i2c_write_cmd(0x20); i2c_write_cmd(0x00);  // memory mode = horizontal
    i2c_write_cmd(0xA1);  // segment remap (column 127 = SEG0)
    i2c_write_cmd(0xC8);  // COM scan direction
    i2c_write_cmd(0xDA); i2c_write_cmd(0x12);  // COM pins
    i2c_write_cmd(0x81); i2c_write_cmd(0xCF);  // contrast
    i2c_write_cmd(0xD9); i2c_write_cmd(0xF1);  // pre-charge
    i2c_write_cmd(0xDB); i2c_write_cmd(0x40);  // deselect
    i2c_write_cmd(0xA4);  // display on resume
    i2c_write_cmd(0xA6);  // normal display
    i2c_write_cmd(0x2E);  // deactivate scroll
    i2c_write_cmd(0xAF);  // display on

    OLED_Clear();
    OLED_Update();
}

void OLED_Clear(void) {
    memset(framebuf, 0, sizeof(framebuf));
}

void OLED_Update(void) {
    if (!oled_ok) return;
    for (uint8_t page = 0; page < 8; page++) {
        i2c_write_cmd(0xB0 + page);  // page addr
        i2c_write_cmd(0x00);  // lower column = 0
        i2c_write_cmd(0x10);  // upper column = 0
        i2c_write_data(framebuf + page * OLED_WIDTH, OLED_WIDTH);
    }
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    if (color)
        framebuf[x + (y / 8) * OLED_WIDTH] |=  (1 << (y % 8));
    else
        framebuf[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
}

void OLED_DrawBar(uint8_t x, uint8_t w, uint8_t h) {
    if (h > OLED_HEIGHT) h = OLED_HEIGHT;
    for (uint8_t i = 0; i < w; i++) {
        for (uint8_t j = 0; j < h; j++) {
            OLED_DrawPixel(x + i, OLED_HEIGHT - 1 - j, 1);
        }
    }
}

/* ──  Bresenham 直线算法 ── */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (1) {
        OLED_DrawPixel(x0, y0, 1);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
}

/* ── 6×8 字体 ── */
static const uint8_t font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // #
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // $
    {0x23,0x13,0x08,0x64,0x62,0x00}, // %
    {0x36,0x49,0x55,0x22,0x50,0x00}, // &
    {0x00,0x05,0x03,0x00,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // )
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, // *
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // +
    {0x00,0x50,0x30,0x00,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08,0x00}, // -
    {0x00,0x60,0x60,0x00,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02,0x00}, // /
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 2
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 3
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 4
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 5
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 7
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 8
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 9
    {0x00,0x36,0x36,0x00,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00,0x00}, // ;
    {0x00,0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14,0x00}, // =
    {0x41,0x22,0x14,0x08,0x00,0x00}, // >
    {0x02,0x01,0x51,0x09,0x06,0x00}, // ?
    {0x32,0x49,0x79,0x41,0x3E,0x00}, // @
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // A
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // B
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // C
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // D
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // E
    {0x7F,0x09,0x09,0x01,0x01,0x00}, // F
    {0x3E,0x41,0x41,0x51,0x32,0x00}, // G
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // H
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // J
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // K
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // L
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, // M
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // N
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // O
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // P
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // Q
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // R
    {0x46,0x49,0x49,0x49,0x31,0x00}, // S
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // T
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // U
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // V
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, // W
    {0x63,0x14,0x08,0x14,0x63,0x00}, // X
    {0x03,0x04,0x78,0x04,0x03,0x00}, // Y
    {0x61,0x51,0x49,0x45,0x43,0x00}, // Z
};

void OLED_ShowString(uint8_t x, uint8_t y, const char *str) {
    if (!oled_ok) return;
    while (*str) {
        uint8_t c = *str - 0x20;  // font starts at 0x20 (space)
        if (c >= sizeof(font6x8)) c = 0;  // out of range → space
        for (uint8_t i = 0; i < 6; i++) {
            uint8_t line = font6x8[c][i];
            for (uint8_t j = 0; j < 8; j++) {
                if (line & (1 << j))
                    OLED_DrawPixel(x + i, y + j, 1);
                else
                    OLED_DrawPixel(x + i, y + j, 0);
            }
        }
        x += 6;
        if (x > OLED_WIDTH - 6) { x = 0; y += 10; }
        str++;
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len) {
    char buf[12];
    for (uint8_t i = 0; i < len; i++) {
        buf[len - 1 - i] = '0' + (num % 10);
        num /= 10;
    }
    buf[len] = 0;
    OLED_ShowString(x, y, buf);
}
