/* ds3231_sim.h — STM32F103 模拟 DS3231（HAL RTC + I2C1 从机）
 * 使用：CubeMX 工程（HCLK 72MHz + RTC(LSE) + I2C1(PB6/PB7) 已配置）
 * main.c USER CODE 段调用 ds3231_sim_init()；stm32f1xx_it.c 的
 * I2C1_EV_IRQHandler 改为调用 ds3231_sim_i2c1_ev_irq()。 */
#ifndef DS3231_SIM_H
#define DS3231_SIM_H

#include <stdint.h>

void ds3231_sim_init(void);          /* I2C1 从机 0x68 + RTC 初始 2000-01-01 + OSF=1 */
void ds3231_sim_i2c1_ev_irq(void);   /* I2C1 事件中断处理（寄存器级状态机） */
void ds3231_sim_i2c1_er_irq(void);   /* I2C1 错误中断（AF/BERR/OVR 计数） */
void ds3231_sim_tx_note(void);       /* 记录 I2C 事务时间戳（EV 中断内调用） */
void ds3231_sim_poll(void);          /* 主循环：LED 出错/未连接指示 */
void ds3231_sim_diag(void);          /* 主循环：USART1 诊断摘要（每秒） */
void ds3231_sim_rtc_irq(void);       /* RTC 秒中断：清标志 + 喂 IWDG */

#endif
