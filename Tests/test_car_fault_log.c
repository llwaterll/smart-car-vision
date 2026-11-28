#include "Task_Fsm.h"
#include <assert.h>
#include <stdio.h>

/* 记录模式（CAR_FAULT_STOP_ENABLE=0）回归：
 *   - 故障只记录，不停车、不清目标/输出；
 *   - 每个故障码独立计数；
 *   - 持续条件不重复计次，消失后再次出现才累加；
 *   - Car_LocalStop 清最近故障码但保留历史；
 *   - Car_ClearFaultLog 清历史且不停车。 */
int main(void)
{
    Car_Core c;

    Car_Init(&c);
    assert(c.state == CAR_STOP);
    assert(c.fault_log[CAR_STALL_FAULT].count == 0U);

    /* 故障只记录：状态、目标、输出都不变。 */
    c.output[0] = 123.0f;
    c.target[0] = 80.0f;
    Car_Fault(&c, CAR_STALL_FAULT);
    assert(c.state == CAR_STOP);
    assert(c.fault == CAR_STALL_FAULT);
    assert(c.output[0] == 123.0f);
    assert(c.target[0] == 80.0f);
    assert(c.fault_log[CAR_STALL_FAULT].count == 1U);

    /* 同一条件持续：每拍调用 Car_Fault 不能把次数刷爆。 */
    {
        int i;
        for (i = 0; i < 100; ++i)
        {
            Car_Fault(&c, CAR_STALL_FAULT);
            Car_Tick(&c, 0, 0);
        }
    }
    assert(c.fault_log[CAR_STALL_FAULT].count == 1U);

    /* 条件消失超过保持时间后再次出现，累计第二次。 */
    {
        int i;
        for (i = 0; i < (int)CAR_FAULT_HOLD_TICKS + 2; ++i)
        {
            Car_Tick(&c, 0, 0);
        }
    }
    Car_Fault(&c, CAR_STALL_FAULT);
    assert(c.fault_log[CAR_STALL_FAULT].count == 2U);
    assert(c.fault_log[CAR_STALL_FAULT].first_ms <= c.fault_log[CAR_STALL_FAULT].last_ms);

    /* 不同故障码互不影响。 */
    Car_Fault(&c, CAR_OVERSPEED_FAULT);
    assert(c.fault_log[CAR_STALL_FAULT].count == 2U);
    assert(c.fault_log[CAR_OVERSPEED_FAULT].count == 1U);
    assert(c.fault == CAR_OVERSPEED_FAULT);

    /* PE10 停车：清最近故障码和保持计数，历史记录保留。 */
    Car_LocalStop(&c);
    assert(c.state == CAR_STOP);
    assert(c.fault == CAR_NO_FAULT);
    assert(c.fault_log[CAR_STALL_FAULT].count == 2U);

    /* 故障页 OK：清空历史，但不停车、不改目标/输出。 */
    c.output[1] = 55.0f;
    c.target[1] = 40.0f;
    Car_ClearFaultLog(&c);
    assert(c.fault == CAR_NO_FAULT);
    assert(c.fault_log[CAR_STALL_FAULT].count == 0U);
    assert(c.fault_log[CAR_OVERSPEED_FAULT].count == 0U);
    assert(c.output[1] == 55.0f);
    assert(c.target[1] == 40.0f);

#if CAR_FAULT_DERATE_PERCENT > 0U
    /* 可选故障降额：参考轮速最多下调配置百分比，保持期结束后恢复 100%。 */
    Car_Init(&c);
    c.state = CAR_RUN;
    c.target[0] = 100.0f;
    c.target[1] = 100.0f;
    Car_Fault(&c, CAR_STALL_FAULT);
    assert(c.fault_derate_us == CAR_FAULT_DERATE_HOLD_MS * 1000U);
    Car_TickDt(&c, 0, 0, 5000);
    {
        float expected = 100.0f * (1.0f - (float)CAR_FAULT_DERATE_PERCENT / 100.0f);
        assert(c.reference[0] > expected - 0.01f && c.reference[0] < expected + 0.01f);
        assert(c.reference[1] > expected - 0.01f && c.reference[1] < expected + 0.01f);
    }
    /* 多个故障不叠加。 */
    Car_Fault(&c, CAR_OVERSPEED_FAULT);
    Car_TickDt(&c, 0, 0, 5000);
    {
        float expected = 100.0f * (1.0f - (float)CAR_FAULT_DERATE_PERCENT / 100.0f);
        assert(c.reference[0] > expected - 0.01f && c.reference[0] < expected + 0.01f);
    }
    /* 停车状态排空降额计时，再回到 RUN 应恢复 100%。 */
    c.state = CAR_STOP;
    {
        int i;
        for (i = 0; i < (int)(CAR_FAULT_DERATE_HOLD_MS / 5U) + 2; ++i)
        {
            Car_TickDt(&c, 0, 0, 5000);
        }
    }
    assert(c.fault_derate_us == 0U);
    c.state = CAR_RUN;
    Car_TickDt(&c, 0, 0, 5000);
    assert(c.reference[0] == 100.0f && c.reference[1] == 100.0f);
    puts("fault log: record-only, per-code counts, clear without stopping, derate enabled");
#else
    /* 当前配置：故障不降速，完全按目标轮速运行。 */
    Car_Init(&c);
    c.state = CAR_RUN;
    c.target[0] = 100.0f;
    c.target[1] = 100.0f;
    Car_Fault(&c, CAR_STALL_FAULT);
    assert(c.fault_derate_us == 0U);
    Car_TickDt(&c, 0, 0, 5000);
    assert(c.reference[0] == 100.0f && c.reference[1] == 100.0f);
    assert(c.fault_log[CAR_STALL_FAULT].count == 1U);
    puts("fault log: record-only, per-code counts, clear without stopping, no speed derate");
#endif
    return 0;
}
