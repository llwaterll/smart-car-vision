/*
 * ============================================================================
 * 文件名称：Drv_Uart.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 两路串口的接收缓冲层：
 *   1. 中断回调只写 FIFO（几微秒返回，不解析、不等待）；
 *   2. 控制中断按字节预算取出，交给协议层；
 *   3. FIFO 溢出或 HAL 串口错误 -> 置通道故障标志 + 重新挂接收。
 *
 * 故障后果不同：
 *   蓝牙故障 -> 控制中断判 CAR_RX_FAULT 停车；
 *   视觉故障 -> 只让视觉缓存失效，车继续按最后曲率降速行驶。
 *
 * 函数列表
 *   void    Uart_Init(void)
 *   void    Uart_Gate(uint8_t on)
 *   void    Uart_Reset(void)
 *   uint8_t Uart_TakeBt(uint8_t *byte)
 *   uint8_t Uart_TakeVision(uint8_t *byte)
 *   void    Uart_DrainBt(void)
 *   void    Uart_DrainVision(void)
 *   uint8_t Uart_BtFault(void) / Uart_VisionFault(void)
 *   void    Uart_ClearBtFault(void) / Uart_ClearVisionFault(void)
 *   void    Car_UART_RX_Handler(uint8_t byte)
 *   void    Car_Vision_UART_RX_Handler(uint8_t byte)
 *   void    HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include "Board_Config.h"
#include "Drv_Uart.h"

/* NEUQ 驱动持有单字节接收缓冲，错误恢复须复用同一缓冲。 */
extern UART_HandleTypeDef UART_0_handle;
extern uint8_t UART_0_RX_dat;
#if BOARD_VISION_ENABLE
extern UART_HandleTypeDef UART_3_handle;
extern uint8_t UART_3_RX_dat;
#endif

/* 蓝牙接收 FIFO 及其缓冲区：中断写入，控制中断读取。 */
static NEUQ_FIFO_t s_bt_fifo;
static uint8_t s_bt_buffer[UART_BT_FIFO_SIZE];

/* 视觉接收 FIFO 及其缓冲区。 */
static NEUQ_FIFO_t s_vis_fifo;
static uint8_t s_vis_buffer[UART_VISION_FIFO_SIZE];

/* 通道故障标志：FIFO 溢出或串口错误时置 1，由控制中断消费。 */
static volatile uint8_t s_bt_fault;
static volatile uint8_t s_vis_fault;

/* 1 表示写 Flash 期间暂停入队（此时 CPU 被阻塞，收到的字节不可信）。 */
static volatile uint8_t s_gate;

/*
 * 函数名称：Uart_Init
 * 函数功能：初始化两个接收 FIFO，清故障与门控。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_Init(void)
{
    NEUQ_FIFO_init(&s_bt_fifo, s_bt_buffer, sizeof s_bt_buffer);
    NEUQ_FIFO_init(&s_vis_fifo, s_vis_buffer, sizeof s_vis_buffer);
    s_bt_fault = 0;
    s_vis_fault = 0;
    s_gate = 0;
}

/*
 * 函数名称：Uart_Gate
 * 函数功能：开关接收门控。
 * 参数说明：on —— 1 暂停入队；0 恢复。
 * 返回参数：无。
 */
void Uart_Gate(uint8_t on)
{
    s_gate = on ? 1U : 0U;
}

/*
 * 函数名称：Uart_Reset
 * 函数功能：重建两个 FIFO、清故障、清串口溢出标志。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：供写参数返回后调用，丢弃 Flash 阻塞期间堆积的残留字节。
 */
void Uart_Reset(void)
{
    Uart_Init();
    __HAL_UART_CLEAR_OREFLAG(&UART_0_handle);
#if BOARD_VISION_ENABLE
    __HAL_UART_CLEAR_OREFLAG(&UART_3_handle);
#endif
}

/*
 * 函数名称：Uart_TakeBt
 * 函数功能：从蓝牙 FIFO 取一个字节。
 * 参数说明：byte —— 取到的字节。
 * 返回参数：1 取到；0 队列空。
 */
uint8_t Uart_TakeBt(uint8_t *byte)
{
    return (NEUQ_FIFO_read_byte(&s_bt_fifo, byte) == NEUQ_FIFO_OK) ? 1U : 0U;
}

/*
 * 函数名称：Uart_TakeVision
 * 函数功能：从视觉 FIFO 取一个字节。
 * 参数说明：byte —— 取到的字节。
 * 返回参数：1 取到；0 队列空。
 */
uint8_t Uart_TakeVision(uint8_t *byte)
{
    return (NEUQ_FIFO_read_byte(&s_vis_fifo, byte) == NEUQ_FIFO_OK) ? 1U : 0U;
}

/*
 * 函数名称：Uart_DrainBt
 * 函数功能：丢弃蓝牙通道全部残留字节。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_DrainBt(void)
{
    uint8_t byte;

    while (Uart_TakeBt(&byte))
    {
    }
}

/*
 * 函数名称：Uart_DrainVision
 * 函数功能：丢弃视觉通道全部残留字节。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_DrainVision(void)
{
    uint8_t byte;

    while (Uart_TakeVision(&byte))
    {
    }
}

/*
 * 函数名称：Uart_BtFault
 * 函数功能：查询蓝牙通道故障标志。
 * 参数说明：无。
 * 返回参数：1 有故障；0 正常。
 */
uint8_t Uart_BtFault(void)
{
    return s_bt_fault;
}

/*
 * 函数名称：Uart_VisionFault
 * 函数功能：查询视觉通道故障标志。
 * 参数说明：无。
 * 返回参数：1 有故障；0 正常。
 */
uint8_t Uart_VisionFault(void)
{
    return s_vis_fault;
}

/*
 * 函数名称：Uart_ClearBtFault
 * 函数功能：清蓝牙通道故障标志。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_ClearBtFault(void)
{
    s_bt_fault = 0;
}

/*
 * 函数名称：Uart_ClearVisionFault
 * 函数功能：清视觉通道故障标志。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_ClearVisionFault(void)
{
    s_vis_fault = 0;
}

/*
 * 函数名称：Car_UART_RX_Handler
 * 函数功能：蓝牙串口单字节接收回调，只入 FIFO。
 * 参数说明：byte —— 收到的字节。
 * 返回参数：无。
 * 备注信息：FIFO 写满只置故障标志，由控制中断统一停车并清队列；
 *           写 Flash 门控期间直接丢弃。
 */
void Car_UART_RX_Handler(uint8_t byte)
{
    if (s_gate)
    {
        return;
    }
    if (NEUQ_FIFO_write_byte(&s_bt_fifo, byte) != NEUQ_FIFO_OK)
    {
        s_bt_fault = 1;
    }
}

/*
 * 函数名称：Car_Vision_UART_RX_Handler
 * 函数功能：视觉串口单字节接收回调，只入 FIFO。
 * 参数说明：byte —— 收到的字节。
 * 返回参数：无。
 * 备注信息：视觉数据不参与实时控制，FIFO 写满只标记异常，不会停车。
 */
void Car_Vision_UART_RX_Handler(uint8_t byte)
{
    if (s_gate)
    {
        return;
    }
    if (NEUQ_FIFO_write_byte(&s_vis_fifo, byte) != NEUQ_FIFO_OK)
    {
        s_vis_fault = 1;
    }
}

/*
 * 函数名称：HAL_UART_ErrorCallback
 * 函数功能：HAL 串口错误回调，标记对应通道并重新挂接单字节接收。
 * 参数说明：huart —— 发生错误的串口句柄。
 * 返回参数：无。
 * 备注信息：不在这里停车；由控制中断根据故障标志决定动作。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &UART_0_handle)
    {
        s_bt_fault = 1;
        HAL_UART_Receive_IT(&UART_0_handle, &UART_0_RX_dat, 1);
    }
#if BOARD_VISION_ENABLE
    else if (huart == &UART_3_handle)
    {
        s_vis_fault = 1;
        HAL_UART_Receive_IT(&UART_3_handle, &UART_3_RX_dat, 1);
    }
#endif
}
