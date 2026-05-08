# STM32F103C8T6 数字滤波器

基于 STM32F103C8T6 的 FIR 低通数字滤波器课程设计 / 毕业设计项目。

**核心流程**: 信号输入 → ADC 采样 (10kHz) → DMA 传输 → FIR 低通滤波 → 频谱分析 → 串口输出 + OLED 显示

---

## 目录

- [功能概述](#功能概述)
- [硬件设计](#硬件设计)
- [引脚分配](#引脚分配)
- [CubeMX 配置](#cubemx-配置)
- [信号处理详解](#信号处理详解)
- [OLED 显示](#oled-显示)
- [软件架构](#软件架构)
- [文件清单](#文件清单)
- [开发环境](#开发环境)
- [测试方法](#测试方法)
- [踩坑记录](#踩坑记录)

---

## 功能概述

| 功能模块 | 实现方式 | 详情 |
|---------|---------|------|
| ADC 采样 | TIM3 触发 ADC1 + DMA | 10kHz 采样率, 12-bit 精度, 512 点/帧 |
| FIR 滤波 | 手动卷积实现 | 101 阶 Hamming 窗低通, 截止 1kHz |
| 频率分析 | Goertzel 算法 | 精确计算 500Hz / 3000Hz 幅值 |
| 频谱显示 | 手动 Radix-2 FFT → OLED | SSD1306 128×64, 64 根频谱柱 |
| 数据输出 | USART1 | 115200-8-N-1, 每帧输出衰减值 |
| **无 CMSIS-DSP** | 纯 C 语言实现 | 无外部库依赖 |

### 信号参数

| 参数 | 值 |
|------|----|
| 采样率 (Fs) | 10 kHz |
| ADC 精度 | 12-bit (0~4095) |
| 采样点数 (N) | 512 |
| 频率分辨率 | 19.53 Hz |
| FIR 阶数 | 100 (101 taps) |
| FIR 窗函数 | Hamming |
| FIR 通带截止 | 800~1000 Hz |
| FIR 阻带衰减 | ~53 dB |
| 每帧采集时间 | 51.2 ms |
| SRAM 占用 | ~9.4 KB (共 20 KB) |

---

## 硬件设计

### 元件清单

| 组件 | 型号 | 数量 |
|------|------|------|
| MCU 最小系统板 | STM32F103C8T6 (Blue Pill) | 1 |
| 调试器 | ST-Link V2 | 1 |
| USB 串口 | USB-TTL (CH340G) | 1 |
| OLED 显示屏 | SSD1306 128×64 I2C (0.96寸) | 1 (可选) |
| 电阻 | 10 kΩ | 2 (偏置电路) |
| 电容 | 1 μF | 1 (隔直) |

### 电路连接

```
信号源 → [1μF 隔直] → 1.65V 偏置 (10k+10k 分压) → PA0
                                                    │
ST-Link:  SWDIO → PA13, SWCLK → PA14, 3.3V, GND   │
                                                    │
USB-TTL:  TX → PA10(可选), RX → PA9(TX), GND       │
                                                    │
OLED:     SCL → PB6, SDA → PB7, VCC → 3.3V, GND ──┘
```

### 信号调理

STM32 ADC 输入范围为 0~3.3V。若信号源为交流信号（正负摆动），需加偏置电路：

1. **隔直电容**：1μF 电容串联在信号路径上，滤除直流分量
2. **偏置网络**：两个 10kΩ 电阻从 3.3V 分压得到 1.65V 直流偏置
3. 信号经电容耦合后叠加在 1.65V 偏置上，使 ADC 输入始终在 0~3.3V 范围内

---

## 引脚分配

| 引脚 | 功能 | 连接对象 |
|------|------|---------|
| PA0 | ADC1_IN0 (模拟输入) | 信号源 (经调理后接入) |
| PA9 | USART1_TX | USB-TTL RX |
| PA10 | USART1_RX | USB-TTL TX (可选) |
| PA13 | SWDIO | ST-Link |
| PA14 | SWCLK | ST-Link |
| PB6 | 软件 I2C SCL | OLED SCK |
| PB7 | 软件 I2C SDA | OLED SDA |
| 3.3V | VDD | 供电 |
| GND | 地 | 共地 |

---

## CubeMX 配置

### 时钟树

```
HSE 8MHz → PLL(x9) → SYSCLK 72MHz
                      ├→ AHB 72MHz
                      ├→ APB1 36MHz (TIM3 72MHz ×2)
                      └→ APB2 72MHz
                                  └→ ADC Prescaler (/6) → ADC 12MHz
```

### 外设配置

| 外设 | 关键参数 |
|------|---------|
| ADC1 | 12-bit, 单通道 PA0, TIM3 TRGO 触发上升沿, 55.5 Cycles 采样时间 |
| DMA1 Ch1 | Circular 模式, Peripheral→Memory, Half Word, High Priority |
| TIM3 | PSC=71, ARR=99 → 10kHz, TRGO=Update Event |
| USART1 | 115200-8-N-1, PA9 TX / PA10 RX |

### DMA 工作流程

```
TIM3 每 100μs 产生 Update Event
  ↓ 触发
ADC1 启动转换 (~5.7μs)
  ↓ 转换完成
DMA 自动搬运结果到 adc_dma[]
  ↓ N=512 次
DMA 传输完成中断 → HAL_ADC_ConvCpltCallback
  ↓
主循环处理数据 (FIR → Goertzel → FFT → OLED → 串口)
  ↓
等待下一帧 (TIM3 继续触发, DMA Circular 自动循环覆盖)
```

---

## 信号处理详解

### 1. FIR 低通滤波

手动实现 FIR 滤波器，使用状态缓冲区实现卷积运算：

```c
void fir_filter(float *in, float *out, int n) {
    // 状态缓冲区: 保留上一次的最后 n 个样本
    for (int i = 0; i < FIR_NUM_TAPS - 1; i++)
        fir_state[i] = fir_state[i + n];
    // 写入新样本
    for (int i = 0; i < n; i++)
        fir_state[FIR_NUM_TAPS - 1 + i] = in[i];
    // FIR 卷积: y[n] = Σ h[k] · x[n-k]
    for (int i = 0; i < n; i++) {
        float sum = 0;
        for (int j = 0; j < FIR_NUM_TAPS; j++)
            sum += fir_coeffs[j] * fir_state[FIR_NUM_TAPS - 1 + i - j];
        out[i] = sum;
    }
}
```

系数由 MATLAB `fir1(100, 0.2, hamming(101))` 生成并导出为 `fir_coeffs.h`。

**性能**: 512 × 101 = 51,712 次乘加, 在 72MHz CM3 上约 < 2ms。

### 2. Goertzel 频率分析

替代 FFT 进行特定频率幅值测量。只计算需要的频率点，效率远高于完整 FFT：

```c
float goertzel(float *samples, int n, float freq) {
    float coeff = 2 * cos(2π · freq / Fs);
    // 递归计算
    for (int i = 0; i < n; i++) {
        s0 = samples[i] + coeff · s1 − s2;
        s2 = s1;  s1 = s0;
    }
    power = s2² + s1² − coeff · s1 · s2;
    return √power;
}
```

输出格式：`500Hz:1.523->1.501 | 3000Hz:0.492->0.008 | Atten:-35.6dB`

### 3. FFT 频谱显示

手动 Radix-2 时间抽取 FFT，用于 OLED 频谱可视化：

| 参数 | 值 |
|------|----|
| 算法 | Cooley-Tukey DIT |
| 点数 | 512 |
| 蝶形级数 | 9 |
| 总运算量 | 2304 次复数蝶形 |
| 执行时间 | ~1ms (72MHz) |
| 显示频率范围 | 0 ~ 5000 Hz |

---

## OLED 显示

- **驱动方式**: 软件 I2C bit-bang (PB6=SCL, PB7=SDA)
- **显示内容**: 滤波后信号的频谱柱状图
- **分辨率**: 128 × 64 像素, 64 根频谱柱 (每柱 2 像素宽)
- **自动缩放**: 指数滑动平均平滑最大值, 缓升缓降
- **容错**: OLED 不存在时自动跳过, 不阻塞主程序

### OLED I2C 地址

SSD1306 默认 7-bit 地址为 0x3C (写模式 0x78)。

---

## 软件架构

### 主循环

```
上电 → HAL_Init → SystemClock_Config → 外设初始化 → OLED 初始化
→ 启动 ADC/DMA/TIM3
→ while(1):
     等待 dma_ready 标志
     uint16 → float 转换
     FIR 滤波 (buf_a → buf_b)
     Goertzel 滤波前幅值 (500Hz, 3000Hz)
     Goertzel 滤波后幅值 (500Hz, 3000Hz)
     FFT 频谱 (buf_b → spectrum)
     OLED 显示频谱
     串口输出测量结果
```

### SRAM 占用

```
adc_dma[512]    1 KB    uint16_t DMA 目标
buf_a[512]      2 KB    float ADC 原始 / FFT 输入
buf_b[512]      2 KB    float FIR 输出
spectrum[256]   1 KB    float FFT 幅值谱
fir_state[612]  2.4 KB  float FIR 状态缓冲区
fir_coeffs[101] 0.4 KB  float FIR 系数
framebuf[1024]  1 KB    uint8_t OLED 帧缓冲 (128×64/8)
合计: ~9.8 KB (共 20 KB SRAM)
```

---

## 文件清单

```
STM32_DigitalFilter/
├── Core/
│   ├── Inc/
│   │   ├── main.h              CubeMX 主头文件
│   │   ├── oled.h              OLED 驱动接口
│   │   ├── fir_coeffs.h        FIR 滤波器系数 (MATLAB 生成)
│   │   ├── adc.h / tim.h / usart.h / dma.h / gpio.h  CubeMX 外设头文件
│   │   └── stm32f1xx_hal_conf.h / stm32f1xx_it.h
│   └── Src/
│       ├── main.c              主程序 (FIR + Goertzel + FFT + OLED + 串口)
│       ├── oled.c              SSD1306 软件 I2C 驱动
│       ├── adc.c / tim.c / usart.c / dma.c / gpio.c  CubeMX 初始化代码
│       └── stm32f1xx_hal_msp.c / stm32f1xx_it.c / system_stm32f1xx.c
├── Drivers/
│   ├── CMSIS/                  CMSIS-Core + DSP (仅头文件)
│   └── STM32F1xx_HAL_Driver/   ST HAL 库
├── MDK-ARM/                    Keil 工程文件
│   └── STM32_DigitalFilter.uvprojx
├── .gitignore
├── README.md
└── STM32_DigitalFilter.ioc     CubeMX 配置文件
```

---

## 开发环境

| 工具 | 版本 |
|------|------|
| MCU | STM32F103C8T6 (Cortex-M3, 72MHz, 20KB SRAM, 64KB Flash) |
| CubeMX | 生成外设初始化代码 |
| Keil MDK | ARMCC V5.06 update 7 |
| ST-Link | 烧录/调试 |
| MATLAB | FIR 系数生成 + 算法仿真 |

### 配置步骤

1. CubeMX 打开 `.ioc` 文件 → Generate Code
2. Keil 打开 `.uvprojx` → 编译
3. ST-Link 连接 → Flash Download
4. 串口助手 115200 查看输出

---

## 测试方法

### 测试信号

输入: `x(t) = sin(2π · 500t) + 0.5 · sin(2π · 3000t)`

可用信号发生器产生，或手机播放对应频率音频文件经音频变压器隔离后输入。

### 预期结果

| 频点 | 滤波前 | 滤波后 | 说明 |
|------|--------|--------|------|
| 500 Hz | 幅值 ~1.5 | 幅值 ~1.5 | 通带内, 几乎无衰减 |
| 3000 Hz | 幅值 ~0.5 | 幅值 < 0.01 | 阻带内, 衰减 > 40dB |

### 串口输出示例

```
500:1.523->1.501 | 3000:0.492->0.008 | Atten:-35.6dB
500:1.519->1.498 | 3000:0.488->0.007 | Atten:-36.8dB
500:1.521->1.502 | 3000:0.491->0.008 | Atten:-35.8dB
```

### OLED 显示

64 根频谱柱, 横轴 0~5000 Hz, 滤波后应在 500Hz 处有明显峰值, 3000Hz 处几乎无显示。

---

## 踩坑记录

### 1. ADC1 触发源选择

STM32F103C8T6 的 ADC1 \*\*不支持\*\* TIM2 TRGO。CubeMX 中 External Trigger 选项里没有 `Timer 2 Trigger Out event`:

| 问题 | 解决 |
|------|------|
| ADC1 External Trigger 找不到 TIM2 TRGO | 改用 TIM3 TRGO |
| TIM3 参数 | PSC=71, ARR=99, TRGO=Update Event 与 TIM2 完全一致 |

### 2. CMSIS-DSP 集成失败

Keil MDK ARMCC V5 下添加 CMSIS-DSP 源文件后链接器报 undefined symbol:

- 手动添加 10+ 个源文件仍有依赖缺失 (arm_fir_f32 只对 CM7/CM0 有实现)
- ARM_MATH_CM3 导致 arm_fir_f32 函数体被跳过
- **结论**: 放弃 CMSIS-DSP，纯 C 手动实现 FIR / FFT / Goertzel

### 3. 内存限制

STM32F103C8T6 仅有 20KB SRAM。双缓冲 FFT_SIZE=1024 时数组共占用 ~18.8KB，超出限制。

| 缓冲区 | FFT_SIZE=1024 | FFT_SIZE=512 |
|--------|---------------|--------------|
| adc_dma | 2 KB | 1 KB |
| buf_a + buf_b + fft_out | 12 KB | 6 KB |
| fir_state | ~4.4 KB | ~2.4 KB |
| 合计 | ~18.4 KB ❌ | ~9.4 KB ✅ |

### 4. OLED I2C 地址

SSD1306 默认 7-bit 地址为 0x3C，左移一位后的写地址为 0x78。不同厂商模块可能不同 (0x3D)，如果 OLED 无响应可检查模块背面电阻配置。

---

## 参考资料

- STM32F103C8T6 数据手册 (DS5319)
- RM0008 参考手册 (STM32F103xx)
- CMSIS-Core 文档
- SSD1306 数据手册
