#ifndef OLED_H
#define OLED_H

#include "main.h"

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawBar(uint8_t x, uint8_t w, uint8_t h);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len);

#endif
