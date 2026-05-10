/**
 * STM32F103C8T6 数字滤波器课设
 * 手动 FIR + Goertzel 频率分析 + OLED 频谱显示
 *
 * 功能:
 *   1. TIM3 触发 ADC1 以 10kHz 采样 (PA0)
 *   2. DMA 循环搬运 512 点数据
 *   3. FIR 低通滤波 (保留 500Hz, 滤除 3000Hz)
 *   4. Goertzel 输出精确幅值 (串口)
 *   5. FFT 频谱显示 (OLED SSD1306, PB6/PB7)
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

/* -------------- 常量 -------------- */
#define FFT_SIZE        512
#define FS              10000.0f
#define FIR_NUM_TAPS    101
#define PI              3.14159265f
#define SPEC_BARS       64       // 频谱条数 (OLED 一半宽度)

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart1;

/* -------------- 全局缓冲区 -------------- */
static uint16_t adc_dma[FFT_SIZE];
static float    buf_a[FFT_SIZE];
static float    buf_b[FFT_SIZE];
static float    spectrum[FFT_SIZE / 2];  // FFT 幅值谱
static float    fir_state[FIR_NUM_TAPS + FFT_SIZE - 1];
static uint8_t  dma_ready = 0;

/* -------------- 函数声明 -------------- */
void SystemClock_Config(void);
void Println(const char *str);
void fir_filter(float *in, float *out, int n);
float goertzel(float *samples, int n, float freq);
void fft_mag(float *in, float *out, int n);
void show_spectrum(float *spec, int bins);

/* -------------- 主函数 -------------- */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();

    // PA8 toggle = program running
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &g);

    OLED_Init();
    OLED_Clear();
    OLED_Update();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma, FFT_SIZE);
    HAL_TIM_Base_Start(&htim3);

    Println("=== STM32 Digital Filter + OLED ===");

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // on

    while (1) {
        if (!dma_ready) continue;
        dma_ready = 0;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);  // blink each frame

        /* ADC uint16 -> float */
        for (int i = 0; i < FFT_SIZE; i++) {
            buf_a[i] = adc_dma[i] * 3.3f / 4096.0f;
        }

        /* FIR 滤波 */
        fir_filter(buf_a, buf_b, FFT_SIZE);

        /* Goertzel 精确幅值 */
        float m5  = goertzel(buf_a, FFT_SIZE, 500);
        float m3  = goertzel(buf_a, FFT_SIZE, 3000);
        float m5f = goertzel(buf_b, FFT_SIZE, 500);
        float m3f = goertzel(buf_b, FFT_SIZE, 3000);

        /* FFT 频谱 (滤波后, 供 OLED 显示) */
        fft_mag(buf_b, spectrum, FFT_SIZE);
        show_spectrum(spectrum, SPEC_BARS);

        /* 串口输出 */
        char str[128];
        sprintf(str, "500:%.3f->%.3f  3000:%.3f->%.3f  Atten:%.1fdB",
                m5, m5f, m3, m3f,
                20.0f * log10f((m3f + 1e-6f) / (m3 + 1e-6f)));
        Println(str);
    }
}

/* -------------- FIR 低通滤波 -------------- */
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

/* -------------- Goertzel 算法 -------------- */
float goertzel(float *samples, int n, float freq) {
    float coeff = 2.0f * cosf(2.0f * PI * freq / FS);
    float s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) {
        s0 = samples[i] + coeff * s1 - s2;
        s2 = s1;  s1 = s0;
    }
    float power = s2 * s2 + s1 * s1 - coeff * s1 * s2;
    return (power > 0) ? sqrtf(power) : 0;
}

/* -------------- 手动 FFT (幅值谱) -------------- */
void fft_mag(float *in, float *out, int n) {
    float real[n], imag[n];
    for (int i = 0; i < n; i++) { real[i] = in[i]; imag[i] = 0; }

    // 位反转
    int j = 0;
    for (int i = 0; i < n - 1; i++) {
        if (i < j) {
            float t = real[i]; real[i] = real[j]; real[j] = t;
            t = imag[i]; imag[i] = imag[j]; imag[j] = t;
        }
        int m = n >> 1;
        while (m && j >= m) { j -= m; m >>= 1; }
        j += m;
    }

    // 蝶形运算
    for (int len = 2; len <= n; len <<= 1) {
        float wlen_r = cosf(2 * PI / len);
        float wlen_i = -sinf(2 * PI / len);
        for (int i = 0; i < n; i += len) {
            float wr = 1, wi = 0;
            for (int k = 0; k < len / 2; k++) {
                float tr = wr * real[i + k + len/2] - wi * imag[i + k + len/2];
                float ti = wr * imag[i + k + len/2] + wi * real[i + k + len/2];
                real[i + k + len/2] = real[i + k] - tr;
                imag[i + k + len/2] = imag[i + k] - ti;
                real[i + k] += tr;
                imag[i + k] += ti;
                float t = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = t;
            }
        }
    }

    // 幅值 (单边谱)
    for (int i = 0; i < n / 2; i++)
        out[i] = sqrtf(real[i] * real[i] + imag[i] * imag[i]) / n * 2;
    out[0] /= 2;
}

/* -------------- OLED 频谱显示 -------------- */
static float spec_max = 1.0f;

void show_spectrum(float *spec, int bins) {
    // 每根柱子取 bin 最大幅值
    float bars[bins];
    memset(bars, 0, sizeof(bars));
    int step = (FFT_SIZE / 2) / bins;
    if (step < 1) step = 1;
    for (int i = 0; i < bins && i * step < FFT_SIZE/2; i++) {
        float maxv = 0;
        for (int k = 0; k < step && i * step + k < FFT_SIZE/2; k++)
            if (spec[i * step + k] > maxv) maxv = spec[i * step + k];
        bars[i] = maxv;
    }

    // 自适应最大值 (缓降)
    float peak = 0;
    for (int i = 0; i < bins; i++)
        if (bars[i] > peak) peak = bars[i];
    if (peak > 0.01f) spec_max = (spec_max * 0.7f) + (peak * 0.3f);
    else spec_max *= 0.99f;

    // 绘制
    int bw = OLED_WIDTH / bins;  // 每根柱子像素宽度
    if (bw < 1) bw = 1;
    OLED_Clear();

    // 标题 "FILTERED SPECTRUM"
    for (int i = 0; i < 10; i++)
        OLED_DrawPixel(4 + i * 12, 0, 1);  // 简化的装饰线

    // 频谱柱
    for (int i = 0; i < bins; i++) {
        uint8_t h = (spec_max > 0.001f)
            ? (uint8_t)(bars[i] / spec_max * (OLED_HEIGHT - 12))
            : 0;
        if (h > 0) OLED_DrawBar(i * bw, bw, h);
    }

    OLED_Update();
}

/* -------------- 系统时钟 -------------- */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Println(const char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    dma_ready = 1;
}

void Error_Handler(void) {
    while (1);
}
