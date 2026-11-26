#include <assert.h>
#include "NEUQ_Delay.h"
#include "NEUQ_Interrupts.h"
#include "Board_Config.h"
#include "Platform_Main.h"

TestCoreDebug test_debug;
uint32_t SystemCoreClock = 1000000u;
static TestDwt counter;
static unsigned reads, bt_count, vision_count;
static uint8_t bt_byte, vision_byte;

TestDwt *test_dwt_access(void)
{
    /* A disabled counter stops advancing, as on a board without a debugger.
       Bound the wait so a regression fails the test instead of hanging CI. */
    assert(++reads < 5000u);
    if ((test_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
        (counter.CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        ++counter.CYCCNT;
    }
    return &counter;
}

void Car_UART_RX_Handler(uint8_t byte) { ++bt_count; bt_byte = byte; }
void Car_Vision_UART_RX_Handler(uint8_t byte) { ++vision_count; vision_byte = byte; }
void Car_PIT_Handler(void) {}

int main(void)
{
    uint32_t before;
    /* First peripheral access with both gates closed, then each gate alone. */
    NEUQ_Delay_ms(1);
    assert(counter.CYCCNT >= 1000u);
    test_debug.DEMCR = 0;
    before = counter.CYCCNT;
    NEUQ_Delay_us(8);
    assert(counter.CYCCNT >= before + 8u);
    counter.CTRL = 0;
    counter.CYCCNT = UINT32_MAX - 3u;
    NEUQ_Delay_us(8);
    assert(counter.CYCCNT < 32u);
    counter.CYCCNT = 500u;
    NEUQ_Delay_init();
    assert(counter.CYCCNT >= 500u);

    TS_UART_0_RX_CALLBACK(0xa1);
    TS_UART_1_RX_CALLBACK(0xb2);
    TS_UART_2_RX_CALLBACK(0xc3);
    TS_UART_3_RX_CALLBACK(0xd4);
    assert(bt_count == 1u && bt_byte == 0xa1);
    assert(vision_count == BOARD_VISION_ENABLE);
    assert(!vision_count || vision_byte == 0xd4);
    return 0;
}
