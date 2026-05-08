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

#endif
