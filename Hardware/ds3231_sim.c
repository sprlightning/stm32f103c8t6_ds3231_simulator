/* ============================================================================
 * ds3231_sim.c — STM32F103C8T6 模拟 DS3231 外部 RTC（HAL RTC + I2C1 从机）
 *
 * 前提（CubeMX 配置）：HCLK 72MHz（HSE PLL×9）、RTC 启用（时钟源 LSE）、
 * I2C1 启用（PB6=SCL/PB7=SDA）。HAL 生成的 MX_RTC_Init/MX_I2C1_Init 保留。
 *
 * 功能：
 *  - I2C1 从机（7 位地址 0x68），寄存器映射兼容 DS3231：
 *      0x00 秒(bit7=CH) 0x01 分 0x02 时(24h) 0x03 星期 0x04 日 0x05 月 0x06 年
 *      0x0E 控制(bit7 EOSC=1 暂停)  0x0F 状态(bit7 OSF：上电=1，写清)
 *  - 时间实时映射 HAL RTC（LSE 32.768kHz 硬件走时，±20ppm 级）：
 *    读寄存器 → HAL_RTC_GetTime/GetDate 换算 BCD；写寄存器 → HAL_RTC_SetTime/SetDate
 *  - OSF 语义：上电=1（时间不可信）→ ESP32 首次 time set 写入后清除
 *
 * 接线：PB6→ESP32 I2C SCL，PB7→SDA，GND 共地，核心板 USB 常供电
 * ========================================================================== */
#include "ds3231_sim.h"
#include "stm32f1xx_hal.h"
#include "main.h"      /* LED_R_GPIO_Port / LED_R_Pin */
#include "rtc.h"       /* RTC_BKP_MAGIC（备份域持久化标记） */
#include <stdio.h>     /* printf 重定向到 USART1（诊断，MicroLIB） */
#include <string.h>    /* memcpy（本地拷贝 volatile 结构体） */
#include <time.h>      /* mktime/localtime_r（LSI 校准换算） */

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;

#define REG_CTRL  0x0E
#define REG_STAT  0x0F

/* 诊断计数（中断里只计数，主循环打印） */
static volatile uint32_t s_addr_cnt = 0;   /* ADDR 匹配次数（地址 0x68 被寻址） */
static volatile uint32_t s_rx_cnt = 0;     /* RXNE 收到字节数 */
static volatile uint32_t s_err_cnt = 0;    /* I2C 错误次数 */

/* ---------------- 模拟寄存器状态（除时间外的 DS3231 寄存器） ---------------- */
static volatile uint8_t s_ctrl = 0x00;   /* 0x0E：bit7 EOSC */
static volatile uint8_t s_stat = 0x80;   /* 0x0F：上电 OSF=1 */
static volatile uint8_t s_ptr = 0;       /* 寄存器指针 */
static volatile uint8_t s_first = 1;     /* 写事务第一字节=寄存器地址 */
static volatile uint8_t s_tx = 0;        /* 读事务进行中 */
static volatile uint8_t s_paused = 0;    /* EOSC/CH 置位时暂停走时 */
/* 闹钟/老化存储寄存器（0x07-0x0D + 0x10，ESP32 未用——读写行为与真模块一致） */
static volatile uint8_t s_regs_ext[11];

/* 初始时间：2000-01-01 00:00:00 周六 */
static void rtc_set_initial(void)
{
    RTC_DateTypeDef d = {0};
    d.Year = 0;          /* 00 → 2000 */
    d.Month = RTC_MONTH_JANUARY;
    d.Date = 1;
    d.WeekDay = RTC_WEEKDAY_SATURDAY;
    HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);

    RTC_TimeTypeDef t = {0};
    t.Hours = 0;
    t.Minutes = 0;
    t.Seconds = 0;
    HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
}

/* ---------------- BCD 工具 ---------------- */
static uint8_t bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
static uint8_t bin(uint8_t b) { return (b & 0x0F) + ((b >> 4) & 0x0F) * 10; }

/* ============ LSI 走时校准（2026-08-15） ============
 * LSI（内部 40kHz RC）固有精度 ±1.5%~5%——本机实测偏快 1.67%（关机 1 小时
 * 开机快 60 秒）。校准：RTC_CNT 计数差值 × 60/61 = 实际秒。
 * 校准系数 = 标称秒数 / 实测计数（60 实际秒 → 61 计数）。
 * 精确校准方法：time set → 关机精确 N 秒 → 开机对比 → 更新 RTC_CAL_DEN。 */
#define RTC_CAL_NUM  60   /* 实际秒 */
#define RTC_CAL_DEN  61   /* RTC 计数（LSI 偏快时计数 > 秒） */

/* 校准基准：最近一次 SetTime 后的时间 */
static volatile struct tm s_cal_base_tm;
static volatile uint8_t s_cal_base_valid = 0;

/* 校准缓存（2026-08-16）：主循环 poll 每秒用 rtc_get_calibrated（mktime/
 * localtime_r——非中断安全）计算，I2C 中断 reg_read 只读此纯 RAM 缓存。
 * 之前中断里直接调 rtc_get_calibrated → 库函数卡死 → 从机读方向挂起。 */
static volatile uint8_t s_cal_cache_sec, s_cal_cache_min, s_cal_cache_hour;
static volatile uint8_t s_cal_cache_wday, s_cal_cache_mday, s_cal_cache_mon, s_cal_cache_year;
static volatile uint8_t s_cal_cache_valid = 0;

/* 主循环调用：校准并更新缓存（非中断上下文，库函数安全） */
static void rtc_update_cal_cache(void)
{
    struct tm cal;
    if (rtc_get_calibrated(&cal) != 0) {
        s_cal_cache_valid = 0;
        return;
    }
    s_cal_cache_sec = bcd(cal.tm_sec);
    s_cal_cache_min = bcd(cal.tm_min);
    s_cal_cache_hour = bcd(cal.tm_hour);
    s_cal_cache_wday = (uint8_t)((cal.tm_wday == 0) ? 7 : cal.tm_wday);
    s_cal_cache_mday = bcd(cal.tm_mday);
    s_cal_cache_mon = bcd(cal.tm_mon + 1);
    s_cal_cache_year = bcd(cal.tm_year - 100);
    s_cal_cache_valid = 1;
}

/* SECF 风暴自愈（2026-08-16）：中断里检测两次 SECF 间隔（正常 1s） */
static volatile uint32_t s_last_secf_ms = 0;

/* SetTime 应用后更新校准基准（poll 里调用） */
static void rtc_set_base(const struct tm *t)
{
    s_cal_base_tm = *t;
    s_cal_base_valid = 1;
}

/* 读取校准后的时间（替代 HAL_RTC_GetTime——HAL 不补偿 LSI 偏差）
 * 注意：F103 RTC 计数器 CNTH:CNTL 是 BCD 编码日历（不是线性秒计数），
 * 直接做差值会在分钟/小时进位时跳变（10:00:59→10:01:00 的 diff=0x100-0x59=167）。
 * 正确做法：用 HAL 解析当前时间（BCD 正确解码）→ epoch 线性秒差 → 乘校准系数 */
static int rtc_get_calibrated(struct tm *t)
{
    if (!s_cal_base_valid) {
        return -1;
    }
    struct tm base;
    memcpy(&base, (const void *)&s_cal_base_tm, sizeof(base));   /* 本地拷贝（去 volatile；ARMCC V5 不支持 struct 强转） */
    RTC_TimeTypeDef rt;
    RTC_DateTypeDef rd;
    HAL_RTC_GetTime(&hrtc, &rt, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rd, RTC_FORMAT_BIN);
    struct tm now = {0};
    now.tm_year = rd.Year + 100;   /* HAL Year 0-99 → tm_year(1900 基准) */
    now.tm_mon = rd.Month - 1;
    now.tm_mday = rd.Date;
    now.tm_hour = rt.Hours;
    now.tm_min = rt.Minutes;
    now.tm_sec = rt.Seconds;
    now.tm_isdst = -1;
    time_t base_epoch = mktime(&base);
    time_t now_epoch = mktime(&now);
    if (now_epoch < base_epoch) {
        now_epoch = base_epoch;   /* RTC 被回拨（防倒退） */
    }
    /* HAL 标称秒差 × 系数 = 实际秒（LSI 偏快时 HAL 秒数 > 实际秒数） */
    uint64_t diff = (uint64_t)(now_epoch - base_epoch);
    time_t real_epoch = base_epoch + (time_t)(diff * RTC_CAL_NUM / RTC_CAL_DEN);
    localtime_r(&real_epoch, t);
    return 0;
}

/* ---------------- 读/写寄存器 ---------------- */

/* 读时间类寄存器（0x00-0x06）：实时从校准后的 RTC 映射（LSI 偏差补偿） */
static uint8_t reg_read(uint8_t reg)
{
    if (reg == REG_CTRL) return s_ctrl;
    if (reg == REG_STAT) return s_stat;
    if (reg <= 0x06) {
        /* 读方向（I2C EV 中断内）必须中断安全：校准计算（mktime/localtime_r）
         * 是 C 库函数（非中断安全，2026-08-16 实测中断里调用卡死 → 从机读方向
         * 挂起 → ESP32 读 DS3231 超时 → 回退 AXP 缓存 → 关机走时冻结）。
         * 校准由主循环 poll 每秒计算写入 s_cal_cache，这里只读纯 RAM。 */
        if (s_cal_cache_valid && s_cal_base_valid) {
            switch (reg) {
            case 0x00: return s_cal_cache_sec | (s_paused ? 0x80 : 0x00);
            case 0x01: return s_cal_cache_min;
            case 0x02: return s_cal_cache_hour;
            case 0x03: return s_cal_cache_wday;
            case 0x04: return s_cal_cache_mday;
            case 0x05: return s_cal_cache_mon;
            case 0x06: return s_cal_cache_year;
            }
        }
        /* 无校准基准：HAL 直读（纯寄存器读，中断安全） */
        RTC_TimeTypeDef t;
        RTC_DateTypeDef d;
        HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
        switch (reg) {
        case 0x00: return bcd(t.Seconds) | (s_paused ? 0x80 : 0x00);
        case 0x01: return bcd(t.Minutes);
        case 0x02: return bcd(t.Hours);          /* 24h */
        case 0x03: return (uint8_t)((d.WeekDay % 7) + 1);   /* HAL 1=Mon..7=Sun → DS3231 1=Sun..7=Sat */
        case 0x04: return bcd(d.Date);
        case 0x05: return bcd(d.Month);
        case 0x06: return bcd(d.Year);           /* 00-99（2000 基准） */
        }
    } else if (reg >= 0x07 && reg <= 0x0D) {
        return s_regs_ext[reg - 0x07];           /* 闹钟 1/2 寄存器（存储，ESP32 未用） */
    } else if (reg == 0x10) {
        return s_regs_ext[10];                   /* 老化偏移（存储） */
    }
    /* 0x11-0x12 温度：模拟器无温度传感器，返回 0（真模块才有） */
    return 0;
}

/* 写寄存器：时间类 → 暂存缓冲（主循环应用，HAL_RTC_SetDate 等 RTOFF 会阻塞
 * 中断——中断里调用在设置页高频写时占用 EV 中断 → 从机卡死）；控制/状态类 → 标志 */
static volatile uint8_t s_regs_tmp[7];
static volatile uint8_t s_reg_pending = 0;

static void reg_write(uint8_t reg, uint8_t val)
{
    if (reg <= 0x06) {
        if (reg == 0x00) {
            for (int i = 0; i < 7; i++) s_regs_tmp[i] = 0;   /* 防残留 */
        }
        s_regs_tmp[reg] = val;
        if (reg == 0x06) {
            s_reg_pending = 1;   /* 全部 7 字节收到：标记主循环应用 */
        }
    } else if (reg == 0x0E) {
        s_ctrl = val;
        if (val & 0x80) s_paused = 1;    /* EOSC=1 暂停 */
    } else if (reg == 0x0F) {
        s_stat = val & 0x7F;             /* 写清 OSF */
    } else if (reg >= 0x07 && reg <= 0x0D) {
        s_regs_ext[reg - 0x07] = val;    /* 闹钟寄存器（存储，ESP32 未用） */
    } else if (reg == 0x10) {
        s_regs_ext[10] = val;            /* 老化偏移（存储） */
    }
    /* 0x11-0x12 温度：只读，忽略写入 */
}

/* ---------------- I2C1 从机（寄存器级，覆盖 HAL 的主机配置） ---------------- */
void ds3231_sim_init(void)
{
    /* 初始时间 + OSF=1：仅首次上电（断电后 BKP 魔数丢失）执行；CPU 复位
     * （魔数有效——MX_RTC_Init 已跳过 BDRST/SetTime 清零）保留域内走时，
     * 模拟真 DS3231 独立芯片：复位不丢时间。 */
    HAL_PWR_EnableBkUpAccess();
    if (BKP->DR1 != RTC_BKP_MAGIC) {
        rtc_set_initial();
        BKP->DR1 = RTC_BKP_MAGIC;
    }

    /* I2C1 从机配置：PCLK1=36MHz，地址 0x68，ACK 使能 */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* PB6/PB7 由 CubeMX 配置为 AF 开漏 ✓ */
    I2C1->CR1 = 0;                       /* 先关 */
    I2C1->CR2 = 4;                       /* FREQ = PCLK1 4MHz（低功耗：HCLK 8MHz/APB1 /2） */
    I2C1->OAR1 = 0x68 << 1;              /* 7 位地址 0x68（ADDMODE=0） */
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;

    /* 事件中断：NVIC 使能（HAL 已注册向量，函数体改为 ds3231_sim_i2c1_ev_irq） */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;   /* AF/BERR/OVR 触发 ER 中断 */

    /* 独立看门狗（IWDG，LSI 驱动）：兜底主循环卡死（如 HAL_RTC_SetDate 异常）。
     * 分频 256 + RLR 4095 ≈ 26s 超时；由 RTC 秒中断每秒喂狗（睡眠中也喂）。 */
    IWDG->KR = 0x5555;   /* 解锁 */
    IWDG->PR = 6;        /* 256 分频 */
    IWDG->RLR = 4095;    /* 超时 ≈ 4096/156 ≈ 26s */
    IWDG->KR = 0xCCCC;   /* 启动 */

    /* RTC 秒中断：每 1s 唤醒 WFI + 喂狗 */
    __HAL_RTC_SECOND_ENABLE_IT(&hrtc, RTC_IT_SEC);
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(RTC_IRQn);

    /* 校准基准不在此建立：开机时 RTC 域可能保留上次时间（VBAT 有电）或已清零
     * （CNT=0 → 2000-01-01），统一走 HAL 兜底；ESP32 time set 时由 poll 里的
     * rtc_set_base 建立正确基准，校准从此起算 */
}

/* RTC 秒中断处理（stm32f1xx_it.c RTC_IRQHandler USER CODE 调用）：清标志 + 喂 IWDG。
 * SECF/风暴自愈（2026-08-16 实测）：PRL=0 时 RTC 分频=1 → SECF 40000Hz 中断风暴
 * → 主循环饿死、写挂起（RTOFF=0）。中断里检测风暴（两次 SECF 间隔 <50ms）→
 * BDRST 重置域 + 重配（RTCSEL/RTCEN/PRL）——必须在中断内完成（主循环已饿死）。 */
void ds3231_sim_rtc_irq(void)
{
    if (__HAL_RTC_SECOND_GET_IT_SOURCE(&hrtc, RTC_IT_SEC) &&
        __HAL_RTC_SECOND_GET_FLAG(&hrtc, RTC_FLAG_SEC)) {
        uint32_t now = HAL_GetTick();
        if (now - s_last_secf_ms < 50) {
            /* 风暴（正常 1s 间隔）：PRL 丢失/分频错误——完整自愈。
             * BDRST 重置域 → 重新配置 RTCSEL/RTCEN → HAL_RTC_Init 重跑
             * （含等同步/写 PRL/验证——手动写 PRL 在时钟切换期会挂起） */
            HAL_PWR_EnableBkUpAccess();
            __HAL_RCC_BACKUPRESET_FORCE();
            __HAL_RCC_BACKUPRESET_RELEASE();
            RCC->BDCR = RCC_BDCR_RTCSEL_1 | RCC_BDCR_RTCEN;   /* RTCSEL=LSI + RTCEN */
            hrtc.State = HAL_RTC_STATE_RESET;                  /* 触发 MspInit 重跑 */
            HAL_RTC_Init(&hrtc);
            __HAL_RTC_SECOND_ENABLE_IT(&hrtc, RTC_IT_SEC);
            BKP->DR1 = RTC_BKP_MAGIC;   /* BDRST 清了 BKP：重新标记，下次复位保留走时 */
            s_cal_base_valid = 0;   /* 校准基准失效（时间已重置） */
        }
        s_last_secf_ms = now;
        __HAL_RTC_SECOND_CLEAR_FLAG(&hrtc, RTC_FLAG_SEC);
        IWDG->KR = 0xAAAA;   /* 喂狗（IWDG 26s 超时，秒级喂狗充足） */
    }
}

/* I2C1 事件中断处理（stm32f1xx_it.c 的 I2C1_EV_IRQHandler 调用本函数） */
void ds3231_sim_i2c1_ev_irq(void)
{
    uint32_t sr1 = I2C1->SR1;

    /* 防御：每次 EV 中断确保 ACK/中断位（HAL_I2C_EV_IRQHandler 可能误清，
     * 见 STOPF 分支注释——若 HAL 已清，这里恢复，从机不失能） */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    if (sr1 & I2C_SR1_ADDR) {
        s_addr_cnt++;                     /* 诊断：地址 0x68 被寻址 */
        ds3231_sim_tx_note();             /* 记录 I2C 事务（ESP32 通信中） */
        uint32_t sr2 = I2C1->SR2;        /* 清 ADDR */
        if (sr2 & I2C_SR2_TRA) {
            s_tx = 1;                    /* 读方向：预载第一字节 */
            I2C1->DR = reg_read(s_ptr);
            s_ptr = (s_ptr + 1) & 0x0F;
        } else {
            s_tx = 0;
            s_first = 1;                 /* 写方向：等寄存器地址字节 */
        }
    } else if (sr1 & I2C_SR1_RXNE) {
        s_rx_cnt++;                       /* 诊断：收到字节 */
        uint8_t b = I2C1->DR;
        if (s_first) {
            s_ptr = b & 0x0F;
            s_first = 0;
        } else {
            reg_write(s_ptr, b);
            s_ptr = (s_ptr + 1) & 0x0F;
        }
    } else if (sr1 & I2C_SR1_TXE) {
        if (s_tx) {
            if (sr1 & I2C_SR1_AF) {
                /* 主机 NACK（读取字节数已够）：停止发送，AF 由 ER 中断清理 */
                s_tx = 0;
                I2C1->SR1 = 0;
            } else {
                I2C1->DR = reg_read(s_ptr);
                s_ptr = (s_ptr + 1) & 0x0F;
            }
        }
    }
    /* STOPF 独立检查（不能进 else-if 链）：RXNE 与 STOPF 常同时置位
     * （最后字节 + stop 同帧）——漏处理 STOPF 会被 HAL_I2C_EV_IRQHandler
     * 误当主机传输结束 → 清 ACK/CR2 中断位 → 从机永久失能 */
    if (sr1 & I2C_SR1_STOPF) {
        I2C1->CR1 |= I2C_CR1_ACK;   /* 读 SR1（已读）+ 写 CR1 清 STOPF（保持 ACK/PE） */
        s_tx = 0;
        s_first = 1;
    }
}

/* ============ LED 出错指示（低功耗：平时灭，仅出错闪烁） ============
 * 快闪 3 次 = I2C 协议错误（AF/BERR/OVR，检查接线/总线冲突）
 * 慢闪 1 次/30s = 无 I2C 通信（ESP32 未在读写）
 * 快闪 5 次 = RTC 读取异常 */
static volatile uint32_t s_i2c_err = 0;
static volatile uint32_t s_last_tx_tick = 0;

void ds3231_sim_i2c1_er_irq(void)   /* stm32f1xx_it.c I2C1_ER_IRQHandler USER CODE 调用（HAL 清标志前） */
{
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_AF | I2C_SR1_OVR)) {
        s_i2c_err++;
        s_err_cnt++;                      /* 诊断：错误计数 */
        I2C1->SR1 = 0;                 /* 清错误位（RC_W0）；AF 不清会永久卡死从机 */
        s_ptr = 0;
        s_first = 1;
        s_tx = 0;                      /* 复位从机状态机 */
        if (I2C1->SR1 & I2C_SR1_SB) {
            /* 总线卡在起始：PE 复位彻底恢复 */
            I2C1->CR1 &= ~I2C_CR1_PE;
            I2C1->CR1 |= I2C_CR1_PE;
        }
    }
}

/* 记录 I2C 事务（EV 中断处理里调用） */
void ds3231_sim_tx_note(void)
{
    s_last_tx_tick = HAL_GetTick();
}

/* ============ USART1 诊断（COM4，115200） ============ */
/* printf 重定向（MicroLIB fputc） */
int fputc(int ch, FILE *f)
{
    (void)f;
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)ch;
    return ch;
}

/* 中断里只计数（printf 会阻塞 I2C 响应），主循环每秒打印摘要 */
static uint32_t s_last_diag = 0;

void ds3231_sim_diag(void)   /* 主循环每秒调用 */
{
    uint32_t now = HAL_GetTick();
    if (now - s_last_diag < 1000) {
        return;
    }
    s_last_diag = now;
    printf("SIM: ADDR=%lu RX=%lu ERR=%lu SR1=%04X CR1=%04X CR2=%04X\r\n",
           (unsigned long)s_addr_cnt, (unsigned long)s_rx_cnt,
           (unsigned long)s_err_cnt, (unsigned)I2C1->SR1,
           (unsigned)I2C1->CR1, (unsigned)I2C1->CR2);
}

/* 闪烁（Toggle 极性无关，亮灭各 80ms） */
static void blink(uint32_t times)
{
    for (uint32_t i = 0; i < times; i++) {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        HAL_Delay(80);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        HAL_Delay(80);
    }
}

/* 主循环 WFI 唤醒后调用：错误/未连接指示 */
void ds3231_sim_poll(void)
{
    /* 应用待写入的 RTC 时间（中断外调用——HAL_RTC_SetDate/SetTime 等待 RTOFF，
     * 在中断里会阻塞 I2C 从机，设置页高频写时卡死） */
    if (s_reg_pending) {
        s_reg_pending = 0;
        RTC_TimeTypeDef t = {0};
        RTC_DateTypeDef d = {0};
        t.Hours = bin(s_regs_tmp[2] & 0x3F);
        t.Minutes = bin(s_regs_tmp[1]);
        t.Seconds = bin(s_regs_tmp[0] & 0x7F);
        d.Year = bin(s_regs_tmp[6] & 0xFF);
        d.Month = bin(s_regs_tmp[5] & 0x1F);
        d.Date = bin(s_regs_tmp[4] & 0x3F);
        uint8_t ds_wday = s_regs_tmp[3] & 0x07;
        if (ds_wday == 0) ds_wday = 1;
        d.WeekDay = (ds_wday == 1) ? 7 : (uint8_t)(ds_wday - 1);   /* DS3231 1=Sun → HAL 7=Sun */
        if (d.Month < 1 || d.Month > 12) d.Month = 1;
        if (d.Date < 1 || d.Date > 31) d.Date = 1;
        /* F103 经典坑（2026-08-16 实测根因）：写 CNT（HAL 进入 CNF 初始化模式）期间
         * 秒中断触发 → SECF 挂起且清不掉（RTOFF=0 时 CRL 写无效）→ RTOFF 永久卡 0
         * → HAL_RTC_SetTime 超时、时间写不进（CNT 恒 0）。
         * 修复：写前禁用秒中断 + 清 SECF，写后恢复。 */
        __HAL_RTC_SECOND_DISABLE_IT(&hrtc, RTC_IT_SEC);
        __HAL_RTC_SECOND_CLEAR_FLAG(&hrtc, RTC_FLAG_SEC);
        HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
        HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
        __HAL_RTC_SECOND_ENABLE_IT(&hrtc, RTC_IT_SEC);
        s_paused = 0;
        /* 更新校准基准（SetTime 后 RTC_CNT 重置，校准从新基准起算） */
        struct tm cal = {0};
        cal.tm_year = d.Year + 100;
        cal.tm_mon = d.Month - 1;
        cal.tm_mday = d.Date;
        cal.tm_hour = t.Hours;
        cal.tm_min = t.Minutes;
        cal.tm_sec = t.Seconds;
        cal.tm_isdst = -1;
        rtc_set_base(&cal);
        rtc_update_cal_cache();   /* time set 后立即更新校准缓存 */
    }

    /* 周期校准缓存更新（1s 间隔）：I2C 中断 reg_read 只读缓存（中断安全） */
    static uint32_t last_cal_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_cal_ms >= 1000) {
        last_cal_ms = now_ms;
        rtc_update_cal_cache();
    }

    static uint32_t last_err = 0;
    static uint32_t last_idle = 0;
    uint32_t now = HAL_GetTick();

    if (s_i2c_err != last_err) {
        last_err = s_i2c_err;
        blink(3);
    } else if (now - s_last_tx_tick > 30000 && now - last_idle > 30000) {
        last_idle = now;
        blink(1);
    }
}
