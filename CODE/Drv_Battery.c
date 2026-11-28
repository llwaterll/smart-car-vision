/*
 * ============================================================================
 * 文件名称：Drv_Battery.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 电池电压的一路慢速采样：
 *   1. Battery_Init() ：ADC1_IN10（PC0）设为模拟输入；
 *   2. Battery_Poll() ：主循环每 60 s 采样 8 次求平均，按分压比换算；
 *   3. 快照          ：只保留电池端 mV 和“已采样”标志，供菜单读取。
 *
 *   为什么放在主循环：
 *     电压是慢变量，控制中断里没有任何逻辑依赖它；每分钟一次放在主循环，
 *     既不拖慢 5 ms 控制节拍，也不会和电机/编码器争用外设。
 *
 *   换算：
 *     pin_mv  = raw × VREF / 4095
 *     batt_mv = pin_mv × 分压比
 *     分压比用 ×100 的整数表示，避免在 MCU 上引入浮点运算。
 *
 * 函数列表
 *   void     Battery_Init(void)
 *   void     Battery_Poll(uint32_t now)
 *   uint16_t Battery_Millivolts(void)
 *   uint8_t  Battery_Valid(void)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Battery.h"

/* 最近一次电池端电压，mV；尚未采样时为 0。 */
static uint16_t s_mv;

/* 1 表示 s_mv 已由一次完整采样写入。 */
static uint8_t s_valid;

/* 1 表示已经采样过，s_last_ms 有效。 */
static uint8_t s_sampled;

/* 上一次采样时间，HAL_GetTick() 毫秒。 */
static uint32_t s_last_ms;

/*
 * 函数名称：Battery_Init
 * 函数功能：初始化电池采样引脚与模块状态。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：adc_init 会把 PC0 切为模拟输入并完成 ADC1 基本配置；
 *           失败时库内部调用 Error_Handler()，与其它外设初始化一致。
 */
void Battery_Init(void)
{
    s_mv = 0U;
    s_valid = 0U;
    s_sampled = 0U;
    s_last_ms = 0U;

    adc_init(BOARD_BATTERY_ADC_CH, ADC_12BIT);
}

/*
 * 函数名称：Battery_Poll
 * 函数功能：到采样周期时读一次电池电压，更新快照。
 * 参数说明：now —— HAL_GetTick() 的毫秒时间。
 * 返回参数：无。
 * 备注信息：首次调用立即采样，便于上电后尽快显示；
 *           之后每 BATTERY_POLL_MS 一次，无符号差值兼容 tick 回绕。
 */
void Battery_Poll(uint32_t now)
{
    uint16_t raw;
    uint32_t pin_mv;
    uint32_t batt_mv;

    if (s_sampled && (uint32_t)(now - s_last_ms) < BATTERY_POLL_MS)
    {
        return;
    }
    s_last_ms = now;
    s_sampled = 1U;

    raw = adc_mean_filter_convert(BOARD_BATTERY_ADC_CH, BATTERY_FILTER_COUNT);
    pin_mv = ((uint32_t)raw * BOARD_BATTERY_VREF_MV + 2047U) / 4095U;
    batt_mv = pin_mv * (uint32_t)BOARD_BATTERY_DIVIDER_X100 / 100U;

    /* 分压比填错时防止显示成离谱的大数；60 V 已远超车载电池范围。 */
    if (batt_mv > 60000U)
    {
        batt_mv = 60000U;
    }

    s_mv = (uint16_t)batt_mv;
    s_valid = 1U;
}

/*
 * 函数名称：Battery_Millivolts
 * 函数功能：读取最近一次电池端电压。
 * 参数说明：无。
 * 返回参数：电池电压，单位 mV；未采样时返回 0。
 */
uint16_t Battery_Millivolts(void)
{
    return s_mv;
}

/*
 * 函数名称：Battery_Valid
 * 函数功能：查询是否已得到有效的电池电压。
 * 参数说明：无。
 * 返回参数：1 有效；0 尚未采样。
 */
uint8_t Battery_Valid(void)
{
    return s_valid;
}
