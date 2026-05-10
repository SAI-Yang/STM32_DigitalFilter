/**
 * 完整数字滤波器 + PC13 诊断灯 + 纯寄存器串口
 * PC13 闪 N 次 = 程序执行到第 N 阶段
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
#define LED_PIN         GPIO_PIN_13
#define LED_PORT        GPIOC

extern UART_HandleTypeDef huart1;

static float    buf_a[FFT_SIZE];
static float    buf_b[FFT_SIZE];
static float    fir_state[FIR_NUM_TAPS + FFT_SIZE - 1];

void SystemClock_Config(void);
void Println(const char *str);
void fir_filter(float *in, float *out, int n);
float goertzel(float *samples, int n, float freq);
void fft_mag(float *in, float *out, int n);
void show_spectrum(float *spec, int bins);

// ---- 纯寄存器串口发送 ----
static void uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}
static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // PC13 板载 LED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_13;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // 外设初始化
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    OLED_Init();
    OLED_Clear();
    OLED_Update();

    // 纯寄存器 USART1: 8MHz → 115200, BRR=0x45
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR = 0x45;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
    uart_puts("=== STM32 FIR Filter Test (Software Signal) ===\r\n");
    uart_puts("Signal: sin(2pi*500t) + 0.5*sin(2pi*3000t)\r\n");
    uart_puts("fs=10000Hz  FIR Order=100  Fpass=800Hz\r\n");
    uart_puts("Frame\tFreq(Hz)\tVin(mV)\tVout(mV)\tAtten(dB)\r\n");

    float p500 = 0.0f, p3000 = 0.0f;  // 相位累加器
    float step500  = 2.0f * PI * 500.0f  / FS;
    float step3000 = 2.0f * PI * 3000.0f / FS;

    uart_puts("Goertzel running (FFT skipped), wait ~3s/frame...\r\n");

    uint32_t frame = 0;
    while (1) {
        // 软件生成测试信号
        for (int i = 0; i < FFT_SIZE; i++) {
            buf_a[i] = sinf(p500) + 0.5f * sinf(p3000);
            p500  += step500;
            p3000 += step3000;
        }
        while (p500  > 2.0f*PI) p500  -= 2.0f*PI;
        while (p3000 > 2.0f*PI) p3000 -= 2.0f*PI;

        uart_putc('S');  // Signal done

        // FIR 滤波
        fir_filter(buf_a, buf_b, FFT_SIZE);
        uart_putc('F');  // Filter done

        // Goertzel 幅值 (除以 N/2=256 转实际幅值)
        float m5_raw  = goertzel(buf_a, FFT_SIZE, 500)  / 256.0f;
        float m3_raw  = goertzel(buf_a, FFT_SIZE, 3000) / 256.0f;
        float m5_filt = goertzel(buf_b, FFT_SIZE, 500)  / 256.0f;
        float m3_filt = goertzel(buf_b, FFT_SIZE, 3000) / 256.0f;
        uart_putc('G');

        float atten_500  = 20.0f * log10f((m5_filt  + 0.0001f) / (m5_raw  + 0.0001f));
        float atten_3000 = 20.0f * log10f((m3_filt  + 0.0001f) / (m3_raw  + 0.0001f));

        char line[128];
        sprintf(line, "%lu\t500Hz\t%.3fV->%.3fV\t%.1fdB\r\n", frame, m5_raw, m5_filt, atten_500);
        uart_puts(line);
        sprintf(line, "%lu\t3kHz\t%.3fV->%.3fV\t%.1fdB\r\n\r\n", frame, m3_raw, m3_filt, atten_3000);
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

void fft_mag(float *in, float *out, int n) {
    static float real_buf[512], imag_buf[512];  // 静态分配代替VLA
    float *real = real_buf, *imag = imag_buf;
    for (int i = 0; i < n; i++) { real[i] = in[i]; imag[i] = 0; }

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

    for (int len = 2; len <= n; len <<= 1) {
        float wlen_r = cosf(2 * PI / len);
        float wlen_i = -sinf(2 * PI / len);
        for (int i = 0; i < n; i += len) {
            float wr = 1, wi = 0;
            for (int k = 0; k < len / 2; k++) {
                float tr = wr * real[i+k+len/2] - wi * imag[i+k+len/2];
                float ti = wr * imag[i+k+len/2] + wi * real[i+k+len/2];
                real[i+k+len/2] = real[i+k] - tr;
                imag[i+k+len/2] = imag[i+k] - ti;
                real[i+k] += tr;
                imag[i+k] += ti;
                float t = wr * wlen_r - wi * wlen_i;
                wi = wr * wlen_i + wi * wlen_r;
                wr = t;
            }
        }
    }

    for (int i = 0; i < n / 2; i++)
        out[i] = sqrtf(real[i] * real[i] + imag[i] * imag[i]) / n * 2;
    out[0] /= 2;
}

static float spec_max = 1.0f;
void show_spectrum(float *spec, int bins) {
    float bars[bins];
    memset(bars, 0, sizeof(bars));
    int step = (FFT_SIZE / 2) / bins;
    if (step < 1) step = 1;
    for (int i = 0; i < bins && i * step < FFT_SIZE/2; i++) {
        float maxv = 0;
        for (int k = 0; k < step && i*step+k < FFT_SIZE/2; k++)
            if (spec[i*step+k] > maxv) maxv = spec[i*step+k];
        bars[i] = maxv;
    }
    float peak = 0;
    for (int i = 0; i < bins; i++)
        if (bars[i] > peak) peak = bars[i];
    if (peak > 0.01f) spec_max = (spec_max * 0.7f) + (peak * 0.3f);
    else spec_max *= 0.99f;

    int bw = OLED_WIDTH / bins;
    if (bw < 1) bw = 1;
    OLED_Clear();
    int y_off = 30;
    for (int i = 0; i < bins; i++) {
        uint8_t max_h = OLED_HEIGHT - y_off;
        uint8_t h = (spec_max > 0.001f) ? (uint8_t)(bars[i] / spec_max * max_h) : 0;
        if (h > 0) OLED_DrawBar(i * bw, bw, h);
    }
    OLED_Update();
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR = 0;
    SystemCoreClock = 8000000;
}

void Error_Handler(void) { while (1); }
