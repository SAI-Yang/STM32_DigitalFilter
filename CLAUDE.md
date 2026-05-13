# STM32_DigitalFilter — 数字滤波器课程设计

## 项目概览

基于 STM32F103C8T6 的 FIR 低通数字滤波器：
- ADC 采样 → FIR 滤波 → Goertzel 频点分析 → FFT 频谱 → OLED + 串口输出
- 纯 C 手动实现（无 CMSIS-DSP）
- 101 阶 Hamming 窗 FIR，截止 1kHz

## Gotchas

### VLA 栈溢出
`startup_stm32f103xb.s` 默认栈仅 0x400=1KB，`fft_mag()` 里的 VLA `float real[512], imag[512]` 需要 4KB，远超栈容量，导致栈溢出冲垮全局变量。
- **必须用全局静态数组代替 VLA**

### OLED 6×8 字库限制
`oled.c` 的 `font6x8` 表只覆盖 `0x20`~`0x5A`（空格/数字/大写字母/常见符号），不含小写字母。
- 显示字符串用全大写

### 软件 I2C 时序
开漏输出 + 外部上拉，`i2c_delay` 循环次数影响速度，过慢会拖低帧率。
- `OLED_Update()` 发送 8 页 × 128 字节，约需 50-100ms

### ADC 版本注意
若切回 ADC 采样模式，需恢复 `HAL_ADC_Start_DMA` / `HAL_TIM_Base_Start` 和 `dma_ready` 标志。

## 文件结构

```
Core/Inc/  — 头文件（fir_coeffs.h, oled.h, 外设头文件）
Core/Src/  — 源文件（main.c, oled.c, 外设初始化）
Drivers/   — CMSIS + STM32F1xx HAL 库
MDK-ARM/   — Keil 工程（startup_stm32f103xb.s, uvprojx）
```
