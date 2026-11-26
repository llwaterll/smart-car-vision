#ifndef DRV_UART_H
#define DRV_UART_H

/*
 * ============================================================================
 * 文件名称：Drv_Uart.h
 * ============================================================================
 * 模块功能：两个串口通道的接收侧：中断里只入 FIFO，控制中断里逐字节取出。
 *
 *   外设资源
 *     USART1（PA9/PA10）-> 蓝牙 JDY-31：命令下行 + VOFA+ 遥测上行
 *     UART4（PC10/PC11）-> 香橙派视觉：只下行（STM32 侧只收不发）
 *
 *   运行视角
 *     RX 中断 -> Car_UART_RX_Handler / Car_Vision_UART_RX_Handler 只写 FIFO；
 *     控制中断 -> Uart_TakeBt / Uart_TakeVision 取出交给协议解析；
 *     FIFO 溢出或串口错误 -> 置故障标志，由控制中断决定停车或使缓存失效。
 * ============================================================================
 */

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 外设：串口接收 FIFO
 * 功能：中断写、控制中断读的单生产者单消费者队列长度，单位字节。
 * 取值：蓝牙 128 B（一帧最长 12 B，足以吸收一整批命令）；视觉 512 B
 *       （香橙派可能连续甩出多帧，视觉数据不参与实时控制，允许排队）。
 * ------------------------------------------------------------------------- */
#define UART_BT_FIFO_SIZE        128U     /* 蓝牙接收 FIFO 深度，字节 */
#define UART_VISION_FIFO_SIZE    512U     /* 视觉接收 FIFO 深度，字节 */

/*
 * 函数名称：Uart_Init
 * 函数功能：初始化两个接收 FIFO 并清故障标志。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：必须在打开串口接收中断之前调用，否则中断里会写未初始化的 FIFO。
 */
void Uart_Init(void);

/*
 * 函数名称：Uart_Gate
 * 函数功能：写 Flash 期间禁止中断入队，避免把阻塞期间堆积的字节当成命令。
 * 参数说明：on —— 1 关闭入队；0 恢复。
 * 返回参数：无。
 */
void Uart_Gate(uint8_t on);

/*
 * 函数名称：Uart_Reset
 * 函数功能：清空两个 FIFO、清故障标志、清串口溢出标志。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：写参数阻塞 CPU 之后调用，丢弃这段时间里不可信的残留字节。
 */
void Uart_Reset(void);

/*
 * 函数名称：Uart_TakeBt
 * 函数功能：从蓝牙 FIFO 取一个字节。
 * 参数说明：byte —— 取到的字节。
 * 返回参数：1 取到；0 队列空。
 */
uint8_t Uart_TakeBt(uint8_t *byte);

/*
 * 函数名称：Uart_TakeVision
 * 函数功能：从视觉 FIFO 取一个字节。
 * 参数说明：byte —— 取到的字节。
 * 返回参数：1 取到；0 队列空。
 */
uint8_t Uart_TakeVision(uint8_t *byte);

/*
 * 函数名称：Uart_DrainBt / Uart_DrainVision
 * 函数功能：丢弃通道内所有残留字节。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_DrainBt(void);
void Uart_DrainVision(void);

/*
 * 函数名称：Uart_BtFault / Uart_VisionFault
 * 函数功能：查询本通道是否发生过溢出或串口错误。
 * 参数说明：无。
 * 返回参数：1 有故障；0 正常。
 */
uint8_t Uart_BtFault(void);
uint8_t Uart_VisionFault(void);

/*
 * 函数名称：Uart_ClearBtFault / Uart_ClearVisionFault
 * 函数功能：清除故障标志（调用方通常已经完成停车或缓存失效）。
 * 参数说明：无。
 * 返回参数：无。
 */
void Uart_ClearBtFault(void);
void Uart_ClearVisionFault(void);

/*
 * 函数名称：Car_UART_RX_Handler
 * 函数功能：蓝牙串口单字节接收回调，只把字节写入 FIFO。
 * 参数说明：byte —— 刚收到的原始字节。
 * 返回参数：无。
 * 备注信息：由 NEUQ 库 TS_UART_x_RX_CALLBACK 调用；回调里不做任何解析。
 */
void Car_UART_RX_Handler(uint8_t byte);

/*
 * 函数名称：Car_Vision_UART_RX_Handler
 * 函数功能：视觉串口单字节接收回调，只把字节写入视觉 FIFO。
 * 参数说明：byte —— 刚收到的原始字节。
 * 返回参数：无。
 */
void Car_Vision_UART_RX_Handler(uint8_t byte);

#endif
