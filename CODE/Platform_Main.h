#ifndef PLATFORM_MAIN_H
#define PLATFORM_MAIN_H

/*
 * ============================================================================
 * 文件名称：Platform_Main.h
 * ============================================================================
 * 模块功能：工程入口与中断调度层。把 driver / protocol / pid / task / track /
 *           menu 六个模块装配成一台车。
 *
 *   对外接口（只给启动文件与中断向量用）
 *     Car_Main()                   初始化全部外设与共享对象，进入主循环
 *     Car_PIT_Handler()            5 ms 控制中断入口
 *     Car_HoldMotorsOff()          上电第一件事：EN 脚拉低
 *     Car_GetVisionObservation()   临界区内复制道路观测快照
 *     Car_GetVisionTile()          临界区内复制数独子块快照
 *
 *   运行视角
 *     控制中断：Clock_PeriodUs -> Encoder_Delta -> 蓝牙解析 -> Track_Step
 *               -> Car_TickDt -> Motor_Apply -> 视觉解析 -> 遥测入队
 *     主循环  ：写参数（阻塞 Flash）、VOFA 发送、陀螺采样、电池采样、
 *               菜单逐行刷新
 * ============================================================================
 */

#include <stdint.h>

#include "Proto_Vision.h"

/*
 * 函数名称：Car_Main
 * 函数功能：初始化全部外设和共享对象，启动 PIT，进入主循环。
 * 参数说明：无。
 * 返回参数：正常不会返回。
 */
void Car_Main(void);

/*
 * 函数名称：Car_PIT_Handler
 * 函数功能：PIT_CH1 控制中断入口，完成采样、解析、控制、输出和遥测。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：只能由周期回调调用一次；主循环重复调用会让速度与里程累计两次。
 */
void Car_PIT_Handler(void);

/*
 * 函数名称：Car_HoldMotorsOff
 * 函数功能：上电最早期把两路 EN 拉低，保证初始化过程电机不转。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：由 HAL 的 main() 在 HAL_Init() 之前调用。
 */
void Car_HoldMotorsOff(void);

/*
 * 函数名称：Car_GetVisionObservation
 * 函数功能：在临界区内复制最新道路观测快照。
 * 参数说明：out —— 接收副本的对象。
 * 返回参数：1 有可用快照；0 无数据、过期或参数无效。
 */
uint8_t Car_GetVisionObservation(CarVisionObservation *out);

/*
 * 函数名称：Car_GetVisionTile
 * 函数功能：在临界区内复制指定索引的数独子块。
 * 参数说明：index —— 0 左上、1 右上、2 左下、3 右下；out —— 接收副本。
 * 返回参数：1 成功；0 参数无效或数据不可用。
 */
uint8_t Car_GetVisionTile(uint8_t index, CarVisionTile *out);

#endif
