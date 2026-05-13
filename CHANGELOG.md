# 更新日志

## [2026-05-10] v1.1 — 实物测试 + 软件验证

### 新增
- **软件测试版** (`main_soft_test.c`): 内置 500Hz+3000Hz 测试信号, 无需外部信号源即可验证 FIR 滤波效果
- **实物 ADC 版** (`main.c`): 支持 PA0 采集外部信号 (TIM3 10kHz + DMA)
- **时钟容错**: HSE 72MHz 优先, 失败自动降级 HSI 64MHz
- **BRR 自动计算**: 根据实际 `SystemCoreClock` 动态设置波特率

### 修复
- **串口输出**: 改用纯寄存器 (`USART1->DR`) 替代 HAL_UART_Transmit, 避免 HAL 状态机死锁
- **FFT 栈溢出**: `float real[n]` VLA 改为静态数组, 防止 512 点 FFT 栈溢出
- **Goertzel 幅值校准**: 输出除以 N/2(256) 得到真实电压值

### 测试结果 (软件测试版)
| 频率 | 输入 | FIR 输出 | 衰减 |
|------|------|---------|------|
| 500Hz | 1.000V | 0.998V | ~0dB |
| 3000Hz | 0.500V | 0.001V | ~-52dB |

### 已知问题
- Proteus 仿真无法正常运行 (HSE 晶振模型问题 + 仿真速度限制)
- 8MHz HSI 下 512 点 float FFT 需跳帧或降至 128 点

---

## [2026-05] v1.0 — 初始版本

- 手动 FIR 101 阶 Hamming 窗低通滤波器
- Goertzel 算法精确幅值测量
- 手动 Radix-2 FFT + SSD1306 OLED 频谱显示
- USART1 串口数据输出
- Keil MDK + STM32F103C8T6
