# STM32F103C8T6 数字滤波器

基于 STM32F103C8T6 的 FIR 低通数字滤波器课设/毕设项目。采样 → FIR 滤波 → 频谱分析 → 串口输出 + OLED 显示。

## 功能

- **ADC 采样**: TIM3 触发 ADC1，10kHz，DMA 循环搬运 512 点
- **FIR 滤波**: 101 阶 Hamming 窗低通，截止 1kHz（保留 500Hz，滤除 3000Hz）
- **频率分析**: Goertzel 算法精确计算 500Hz/3000Hz 幅值 → 串口输出
- **频谱显示**: 手动 Radix-2 FFT → 128×64 OLED（SSD1306）柱状频谱
- **无 CMSIS-DSP 依赖**: FIR/FFT/Goertzel 均为纯 C 实现

## 硬件

| 组件 | 型号 |
|------|------|
| MCU | STM32F103C8T6 (Blue Pill) |
| ADC | 内置 12-bit, PA0 |
| OLED | SSD1306 128×64, I2C (PB6/PB7) |
| 串口 | USART1 PA9(TX)/PA10(RX), 115200 |

## 引脚分配

| 引脚 | 功能 |
|------|------|
| PA0 | 信号输入 (ADC1_IN0, 0~3.3V) |
| PA9 | USART1 TX |
| PA10 | USART1 RX |
| PA13 | SWDIO (ST-Link) |
| PA14 | SWCLK (ST-Link) |
| PB6 | OLED SCL |
| PB7 | OLED SDA |

## 软件架构

```
信号输入 → ADC(10kHz) → DMA(512点) → FIR滤波 → Goertzel(串口)
                                                    → FFT → OLED频谱
```

- `Core/Src/main.c` — 主逻辑
- `Core/Src/oled.c` — SSD1306 软件 I2C 驱动
- `Core/Inc/fir_coeffs.h` — 101 阶 FIR 系数（Hamming 窗）

## 开发环境

- STM32CubeMX（时钟树、外设配置）
- Keil MDK ARMCC V5
- ST-Link 烧录

## 信号链路

测试信号: 500Hz + 3000Hz 正弦叠加

```
滤波前:  500Hz ████████  3000Hz ████
滤波后:  500Hz ████████  3000Hz ░░░░  衰减 > 40dB
```

## 许可证

MIT
