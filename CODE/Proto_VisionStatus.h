#ifndef PROTO_VISION_STATUS_H
#define PROTO_VISION_STATUS_H

/*
 * ============================================================================
 * 文件名称：Proto_VisionStatus.h
 * ============================================================================
 * 模块功能：F407 -> 香橙派 STATUS 帧编码。协议版本 1，冻结字段布局。
 *
 *   帧格式（大端）：
 *     5A A5 | ver=1 | type=0x01 | len=26 | seq:u16
 *           | state:u8 | fault:u8 | flags:u8 | reject:u8
 *           | bound_session:u32 | accepted:u32 | rejected:u32 | bad:u32
 *           | obs_age_ms:u16 | uptime_ms:u32
 *           | CRC16-CCITT-FALSE
 *
 *   总长 36 字节；CRC 覆盖 ver 到 uptime_ms。
 *   任何字段、长度、枚举、时序改动必须同步图像端文档与实现。
 * ============================================================================
 */

#include <stdint.h>

#include "Proto_Vision.h"

#define VISION_STATUS_FRAME_SIZE        36U   /* STATUS 整帧长度，字节 */
#define VISION_STATUS_PAYLOAD_SIZE      26U   /* STATUS 负载长度，字节 */
#define VISION_STATUS_PERIOD_MS         100U  /* 周期发送间隔，毫秒 */
#define VISION_STATUS_MIN_GAP_MS        20U   /* 变化立即发送的最小间隔，毫秒 */

/* flags 位定义，与图像端逐位一致。 */
#define VISION_STATUS_FLAG_ALLOW_HELLO      (1U << 0)
#define VISION_STATUS_FLAG_SESSION_VALID    (1U << 1)
#define VISION_STATUS_FLAG_FINISH_PENDING   (1U << 2)
#define VISION_STATUS_FLAG_UART_FAULT       (1U << 3)
#define VISION_STATUS_FLAG_OBS_FRESH        (1U << 4)
#define VISION_STATUS_FLAG_AUTONOMOUS       (1U << 5)
#define VISION_STATUS_FLAG_LINK_ALIVE       (1U << 6)

/* STATUS 帧负载内容。字段顺序与协议文档 3.2 节一致。 */
typedef struct
{
    uint8_t state;            /* 0=STOP，1=RUN，2=FAULT */
    uint8_t fault;            /* CAR_*_FAULT */
    uint8_t flags;            /* VISION_STATUS_FLAG_* */
    uint8_t reject;           /* VISION_REJECT_* */
    uint32_t bound_session;   /* 0 表示未绑定 */
    uint32_t accepted;        /* accepted_frames */
    uint32_t rejected;        /* rejected_frames */
    uint32_t bad;             /* bad_frames */
    uint16_t obs_age_ms;      /* 0xFFFF 表示无观测 */
    uint32_t uptime_ms;       /* F407 上电毫秒 */
    uint16_t seq;             /* 每帧递增，允许回绕 */
} VisionStatus;

/*
 * 函数名称：VisionStatus_Encode
 * 函数功能：把状态结构编码成 36 字节 STATUS 帧。
 * 参数说明：s   —— 待编码状态，不能为空；
 *           out —— 至少 36 字节的输出缓冲。
 * 返回参数：写入的字节数（固定 36）；参数无效返回 0。
 * 备注信息：纯函数，不访问硬件；CRC 复用 CarVision_Crc16。
 */
uint16_t VisionStatus_Encode(const VisionStatus *s, uint8_t out[VISION_STATUS_FRAME_SIZE]);

#endif
