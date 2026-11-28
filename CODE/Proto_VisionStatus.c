/*
 * ============================================================================
 * 文件名称：Proto_VisionStatus.c
 * ============================================================================
 * 模块功能：F407 -> 香橙派 STATUS 帧编码实现。字段布局与 CRC 参数冻结，
 *           黄金向量见控制端/图像端提示词的 2.6 节。
 *
 * 注意：本文件是纯编码器，不访问硬件、不发送。发送由 Drv_Uart 负责。
 * ============================================================================
 */
#include "Proto_VisionStatus.h"

#include <stddef.h>

/*
 * 函数名称：put_u16
 * 函数功能：按大端写入 16 位整数。
 * 参数说明：p     —— 目标缓冲；
 *           value —— 待写入值。
 * 返回参数：无。
 */
static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

/*
 * 函数名称：put_u32
 * 函数功能：按大端写入 32 位整数。
 * 参数说明：p     —— 目标缓冲；
 *           value —— 待写入值。
 * 返回参数：无。
 */
static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

/*
 * 函数名称：VisionStatus_Encode
 * 函数功能：把状态结构编码成 36 字节 STATUS 帧。
 * 参数说明：s   —— 待编码状态，不能为空；
 *           out —— 至少 36 字节的输出缓冲。
 * 返回参数：写入的字节数（固定 36）；参数无效返回 0。
 * 备注信息：CRC 覆盖 version 到 uptime_ms（偏移 2..33），高字节在前。
 */
uint16_t VisionStatus_Encode(const VisionStatus *s, uint8_t out[VISION_STATUS_FRAME_SIZE])
{
    uint16_t crc;                 /* CRC 累计值 */
    unsigned i;                   /* CRC 循环下标 */

    if (s == NULL || out == NULL)
    {
        return 0U;
    }

    out[0] = 0x5AU;               /* 方向帧头：F407 -> Pi */
    out[1] = 0xA5U;
    out[2] = 1U;                  /* version */
    out[3] = 0x01U;               /* type = STATUS */
    put_u16(out + 4, VISION_STATUS_PAYLOAD_SIZE);
    put_u16(out + 6, s->seq);

    out[8] = s->state;
    out[9] = s->fault;
    out[10] = s->flags;
    out[11] = s->reject;

    put_u32(out + 12, s->bound_session);
    put_u32(out + 16, s->accepted);
    put_u32(out + 20, s->rejected);
    put_u32(out + 24, s->bad);
    put_u16(out + 28, s->obs_age_ms);
    put_u32(out + 30, s->uptime_ms);

    crc = 0xFFFFU;
    for (i = 2U; i < (VISION_STATUS_FRAME_SIZE - 2U); i++)
    {
        crc = CarVision_Crc16(crc, out[i]);
    }
    put_u16(out + (VISION_STATUS_FRAME_SIZE - 2U), crc);

    return VISION_STATUS_FRAME_SIZE;
}
