# STM32F103C8T6 模拟 DS3231（HAL 工程）使用说明

临时替代 DS3231 模块，验证 ESP32 侧驱动/关机走时逻辑。真模块到货后弃用。

## CubeMX 配置（3 项）
1. **HCLK 72MHz**：RCC 时钟树 → PLL 源 HSE(8MHz)、×9 → SYSCLK 72MHz、
   AHB Prescaler /1（HCLK 72MHz）、APB1 Prescaler /2（36MHz）
   （原 2MHz 太慢，I2C 从机时序余量不足）
2. **启用 RTC**：激活时钟源 **LSE**（32.768kHz），24h 格式
3. **I2C1 全局中断勾选**：NVIC Settings → I2C1 event interrupt ✓
   （I2C1 PB6/PB7 已配置 ✓；从机参数由 ds3231_sim.c 覆盖）

重新生成代码。

## 工程接入（3 步）
1. 把 `Hardware/ds3231_sim.c` 加入 Keil 工程（Hardware 组已有）
2. `Core/Src/stm32f1xx_it.c` 的 `I2C1_EV_IRQHandler` 函数体改为：
   ```c
   void I2C1_EV_IRQHandler(void)
   {
       ds3231_sim_i2c1_ev_irq();   /* 替代 HAL_I2C_EV_IRQHandler(&hi2c1) */
   }
   ```
   （文件头加 `#include "ds3231_sim.h"`）
3. `Core/Src/main.c` 的 USER CODE BEGIN 2 段调用：
   ```c
   ds3231_sim_init();
   ```

编译（F7）→ ST-Link SWD 烧录。

## 接线（与 ESP32 播放器）
```
PB6 (SCL) ──→ I2C SCL（AXP 总线）
PB7 (SDA) ──→ I2C SDA
GND        ──→ GND
核心板 USB 常供电（ESP32 关机时模拟器必须继续走时）
```
地址 0x68 与 AXP 0x34 不冲突。

## 验证流程（ESP32 侧）
1. `time set 2026-08-14 15:00`（自动写 DS3231=STM32，清 OSF）
2. `time` 确认；重启后日志应显示 "Time from DS3231"
3. 关机（`nav poweroff`→`key press` 或拔 USB 电池关机），等待几分钟
4. 开机 → `time`：应为 15:00 + 关机时长（真走时）

## 注意
- 首次上电 OSF=1 → ESP32 拒绝读取回退冻结版（正常），time set 写一次后生效
- STM32 走时精度 = LSE 32.768kHz（±20ppm 级，每天 ~1.7s），真 DS3231（TCXO）更准
- 核心板供电必须保持（USB 插着）；VBAT 已接 3.3V，拔电后 RTC 基准丢失需重新 time set

## LSI 走时校准（2026-08-15）

STM32 的 LSI（内部 40kHz RC）固有精度 ±1.5%~5%，本机实测偏快 ~1.67%
（关机 1 小时开机快 60 秒）。固件已实现**校准补偿**（`ds3231_sim.c` 顶部
`RTC_CAL_NUM/DEN`：60 实际秒 → 61 RTC 计数）。

**精确校准方法**：
1. ESP32 `time set` 设置准确时间
2. 关机精确等待 N 秒（建议 1 小时以上，用秒表）
3. 开机对比时间，得到"实际流逝 / 标称流逝"比值
4. 更新 `RTC_CAL_NUM/RTC_CAL_DEN`（如实测 1 小时快 61 秒 → 60/61）→ 重新烧录

校准基准在 ESP32 `time set` 时自动更新；校准后剩余误差为 LSI 温度/电压
漂移（室温 + 3.0V 稳压下很小）。

## 真 DS3231 模块替代验证（2026-08-17 实机）
DS3231 模块到货后，将 ESP32 上的 STM32 模拟器移除，换上真实 DS3231 模块：
- 供电：PMU LDO1（DS3231 官方支持 2.3V~5.5V）
- 验证：`time set` 写入 + 关机走时实测均正常
- 结论：本模拟器的 I2C 从机寄存器协议/行为近似度足够——ESP32 侧软件零改动，
  直接驱动真实 DS3231 即正常，证明模拟实现（0x68 从机 + BCD 日历 + OSF 逻辑）正确。
 (docs: 真 DS3231 模块替代验证（LDO1 供电, time set+关机走时正常, ESP32 软件零改动）)
