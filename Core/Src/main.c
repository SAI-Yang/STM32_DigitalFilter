/**
 * 实物测试版: PA0 接信号源, TIM3触发ADC@10kHz, DMA搬512点
 * FIR 100阶低通 (500Hz通带, 3000Hz阻带)
 * 纯寄存器串口输出 @ 115200
 */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "fir_coeffs.h"
#include "oled.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"

#define FFT_SIZE        512
#define FS              10000.0f
#define FIR_NUM_TAPS    101
#define PI              3.14159265f
#define SPEC_BARS       64

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef  htim3;

static uint16_t adc_buf[FFT_SIZE];
static float    buf_a[FFT_SIZE];
static float    buf_b[FFT_SIZE];
static float    fir_state[FIR_NUM_TAPS + FFT_SIZE - 1];
static volatile uint8_t data_ready = 0;

void SystemClock_Config(void);
void fir_filter(float *in, float *out, int n);
float goertzel(float *samples, int n, float freq);

// ---- 纯寄存器串口 ----
static void uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}
static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    data_ready = 1;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // PC13 LED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // 外设
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    OLED_Init();
    OLED_Clear();
    OLED_Update();

    // 串口: 根据实际时钟设 BRR
    USART1->CR1 &= ~USART_CR1_UE;
    {
        uint32_t pclk2 = SystemCoreClock;  // APB2=HCLK
        float div = (float)pclk2 / (16.0f * 115200.0f);
        uint32_t mantissa = (uint32_t)div;
        uint32_t fraction = (uint32_t)((div - mantissa) * 16.0f + 0.5f);
        USART1->BRR = (mantissa << 4) | (fraction & 0xF);
    }
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
    uart_puts("=== STM32 FIR Filter (ADC Hardware Test) ===\r\n");
    {
        char tmp[32];
        sprintf(tmp, "SYSCLK=%luMHz\r\n", SystemCoreClock/1000000);
        uart_puts(tmp);
    }
    uart_puts("fs=10000Hz  FIR=100阶  Fpass=800Hz\r\n");
    uart_puts("Frame\tFreq(Hz)\tVin(V)\tVout(V)\tAtten(dB)\r\n");

    // 启动
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, FFT_SIZE);
    HAL_TIM_Base_Start(&htim3);

    uint32_t frame = 0;
    while (1) {
        while (!data_ready);
        data_ready = 0;

        uart_putc('.');

        // ADC uint16 → float voltage
        for (int i = 0; i < FFT_SIZE; i++)
            buf_a[i] = adc_buf[i] * 3.3f / 4096.0f;

        // FIR
        fir_filter(buf_a, buf_b, FFT_SIZE);

        // Goertzel
        float m5_raw  = goertzel(buf_a, FFT_SIZE, 500)  / 256.0f;
        float m3_raw  = goertzel(buf_a, FFT_SIZE, 3000) / 256.0f;
        float m5_filt = goertzel(buf_b, FFT_SIZE, 500)  / 256.0f;
        float m3_filt = goertzel(buf_b, FFT_SIZE, 3000) / 256.0f;

        float atten_500  = 20.0f * log10f((m5_filt  + 0.0001f) / (m5_raw  + 0.0001f));
        float atten_3000 = 20.0f * log10f((m3_filt  + 0.0001f) / (m3_raw  + 0.0001f));

        // 串口
        char line[128];
        sprintf(line, "%lu\t500Hz\t%.3f->%.3fV\t%.1fdB\r\n", frame, m5_raw, m5_filt, atten_500);
        uart_puts(line);
        sprintf(line, "%lu\t3kHz\t%.3f->%.3fV\t%.1fdB\r\n\r\n", frame, m3_raw, m3_filt, atten_3000);
        uart_puts(line);

        // OLED
        char oled_str[24];
        sprintf(oled_str, "500:%.2f->%.2fV", m5_raw, m5_filt);
        OLED_ShowString(0, 0, oled_str);
        sprintf(oled_str, "3k:%.2f->%.3fV", m3_raw, m3_filt);
        OLED_ShowString(0, 10, oled_str);
        sprintf(oled_str, "Att:%.1f/%.1fdB", atten_500, atten_3000);
        OLED_ShowString(0, 20, oled_str);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        frame++;
    }
}

// ---- FIR ----
void fir_filter(float *in, float *out, int n) {
    for (int i = 0; i < FIR_NUM_TAPS - 1; i++)
        fir_state[i] = fir_state[i + n];
    for (int i = 0; i < n; i++)
        fir_state[FIR_NUM_TAPS - 1 + i] = in[i];
    for (int i = 0; i < n; i++) {
        float sum = 0;
        for (int j = 0; j < FIR_NUM_TAPS; j++)
            sum += fir_coeffs[j] * fir_state[FIR_NUM_TAPS - 1 + i - j];
        out[i] = sum;
    }
}

// ---- Goertzel ----
float goertzel(float *samples, int n, float freq) {
    float coeff = 2.0f * cosf(2.0f * PI * freq / FS);
    float s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) {
        s0 = samples[i] + coeff * s1 - s2;
        s2 = s1;  s1 = s0;
    }
    float power = s2*s2 + s1*s1 - coeff*s1*s2;
    return (power > 0) ? sqrtf(power) : 0;
}

// ---- 时钟: HSE 72MHz, 失败自动切 HSI 64MHz ----
void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    // 尝试 HSE
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        // HSE 失败, 切 HSI+PLL 64MHz
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_ON;
        osc.HSEState = RCC_HSE_OFF;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
        osc.PLL.PLLMUL = RCC_PLL_MUL16;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void) { while (1); }
