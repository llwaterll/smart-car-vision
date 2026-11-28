/*
 * ============================================================================
 * 文件名称：Proto_Vision.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 视觉帧解析器：把串口字节流还原成“道路观测”和“数独子块”两类数据。
 *   1. CarVision_Crc16()        ：CRC-16/CCITT-FALSE 逐字节累计；
 *   2. CarVision_Init()         ：清会话、序号、统计和全部缓存；
 *   3. CarVision_Invalidate()   ：接收异常时作废半帧与道路观测（保留数独块）；
 *   4. CarVision_CloseRaceSession()：一轮结束后作废会话，允许同会话重新 HELLO；
 *   5. CarVision_GetCameraStatus()：取摄像头状态（独立于比赛会话）；
 *   6. CarVision_Feed()         ：同步 + 长度校验 + CRC + 业务字段校验 + 入缓存；
 *   7. CarVision_GetObservation()：按 200 ms 时效取道路观测快照；
 *   8. CarVision_GetTile()      ：按索引取 3×3 数独子块。
 *
 * 设计要点
 *   - 只认已验证会话的业务帧，HELLO 只允许在停车时绑定；
 *   - 道路观测有硬时效（VIS_MAX_AGE_MS），数独块在本会话内一直保留；
 *   - 解析错误分三类统计：bad（长度/CRC）、rejected（业务字段不合法）、
 *     accepted（真正入缓存），便于现场判断是线缆问题还是视觉端问题。
 *
 * 函数列表
 *   uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte)
 *   void     CarVision_Init(CarVision *v)
 *   void     CarVision_Invalidate(CarVision *v)
 *   void     CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello)
 *   uint8_t  CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out)
 *   uint8_t  CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out)
 * ============================================================================
 */
#include "Proto_Vision.h"

#include <string.h>

/* ========================================================================== */
/*                              字节序辅助函数                                 */
/* ========================================================================== */

/*
 * 函数名称：u16
 * 函数功能：按大端顺序还原无符号 16 位整数。
 * 参数说明：p —— 指向两个连续字节的指针。
 * 返回参数：高字节在前组成的 uint16_t 数值。
 * 使用示例：
 *     uint16_t len = u16(v->buffer + 4);
 * 备注信息：视觉协议所有多字节整数均为大端。
 */
static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] * 256U + p[1]);
}

/*
 * 函数名称：u32
 * 函数功能：按大端顺序还原无符号 32 位整数。
 * 参数说明：p —— 指向四个连续字节的指针。
 * 返回参数：高字在前组成的 uint32_t 数值。
 * 使用示例：
 *     uint32_t session = u32(f + 8);
 * 备注信息：先还原高 16 位，再左移 16 位并与低 16 位拼接。
 */
static uint32_t u32(const uint8_t *p)
{
    return ((uint32_t)u16(p) << 16) | u16(p + 2);
}

/*
 * 函数名称：i16
 * 函数功能：按大端顺序还原有符号 16 位整数。
 * 参数说明：p —— 指向两个连续字节的指针。
 * 返回参数：补码表示的 int32_t 数值。
 * 使用示例：
 *     int32_t lateral = i16(p + 4);
 * 备注信息：最高位为 1 时减去 65536 还原负值。
 */
static int32_t i16(const uint8_t *p)
{
    uint16_t n = u16(p);    /* 原始无符号 16 位值 */

    if (n & 0x8000U)        /* 最高位为 1 表示负数 */
    {
        return (int32_t)n - 65536;
    }

    return (int32_t)n;
}

/*
 * 函数名称：CarVision_Crc16
 * 函数功能：对一个字节更新 CRC-16 校验值。
 * 参数说明：crc  —— 当前累计 CRC；整帧计算时初值为 0xFFFF；
 *           byte —— 本次参与计算的字节。
 * 返回参数：更新后的 CRC-16 值。
 * 使用示例：
 *     uint16_t crc = 0xFFFFU;
 *     for (i = 2; i < total - 2; i++)
 *     {
 *         crc = CarVision_Crc16(crc, v->buffer[i]);
 *     }
 * 备注信息：CRC-16/CCITT，多项式 0x1021，不反射。
 */
uint16_t CarVision_Crc16(uint16_t crc, uint8_t byte)
{
    unsigned i;                            /* 位循环下标，每个字节 8 位 */

    crc ^= (uint16_t)byte << 8;            /* 当前字节并入 CRC 高 8 位 */

    for (i = 0; i < 8; i++)
    {
        if (crc & 0x8000U)                 /* 最高位为 1 时异或多项式 */
        {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        }
        else
        {
            crc = (uint16_t)(crc << 1);    /* 否则仅左移 */
        }
    }

    return crc;
}

/* ========================================================================== */
/*                              解析器生命周期                                 */
/* ========================================================================== */

/*
 * 函数名称：CarVision_Init
 * 函数功能：清空视觉解析器和所有缓存。
 * 参数说明：v —— 视觉解析器对象。
 * 返回参数：无。
 * 使用示例：
 *     CarVision vision;
 *     CarVision_Init(&vision);
 * 备注信息：会话、序号、统计计数和接收缓冲区全部归零。
 */
void CarVision_Init(CarVision *v)
{
    memset(v, 0, sizeof(*v));
}

/*
 * 函数名称：CarVision_Invalidate
 * 函数功能：接收异常时丢弃半帧和道路观测，保留当前会话已校验的数独块。
 * 参数说明：v —— 视觉解析器对象。
 * 返回参数：无。
 * 使用示例：
 *     if (vision_rx_fault)
 *     {
 *         CarVision_Invalidate(&vision);
 *     }
 * 备注信息：保留会话、序号、链路和摄像头状态：
 *           一次 UART 错误不等于“链路断开/相机无画面”。链路和相机状态
 *           分别由 VIS_LINK_TIMEOUT_MS / VIS_STATUS_TIMEOUT_MS 超时淘汰，
 *           这样单次 ORE/噪声不会让首页两行同时闪成未连接。
 *           Collected tiles are persistent evidence for this session. Only a new
 *           HELLO clears them; a later UART fault must not erase an earlier tile.
 */
void CarVision_Invalidate(CarVision *v)
{
    v->used = 0;                  /* 只丢弃可能被截断的半帧 */
    v->have_observation = 0;      /* 道路观测需要下一帧重新确认 */
    /* link_seen/link_frame_ms 和 camera_status/camera_status_ms 保留，
     * 由各自的超时决定是否过期。 */
}

/*
 * 函数名称：CarVision_CloseRaceSession
 * 函数功能：结束一轮比赛，作废会话、道路观测和数独块。
 * 参数说明：v —— 视觉解析器对象。
 * 返回参数：无。
 * 备注信息：保留诊断统计和最近序号。停车并完成收尾后调用；发送端不需要
 *           重启，下一次停车状态下的 HELLO（即使会话号相同）会重新绑定，
 *           但必须等新的道路帧才能用于发车。
 */
void CarVision_CloseRaceSession(CarVision *v)
{
    v->used = 0;
    v->have_observation = 0;
    v->session_valid = 0;
    memset(v->tiles, 0, sizeof v->tiles);
    /* Retain diagnostics and the sequence watermark for this sender. */
}

/*
 * 函数名称：CarVision_GetCameraStatus
 * 函数功能：取最近的摄像头状态，超过 1 s 没收到状态帧就按未知处理。
 * 参数说明：v      —— 视觉解析器对象；
 *           now_ms —— 当前毫秒时间。
 * 返回参数：VISION_CAMERA_UNKNOWN / NO_FRAME / STREAMING。
 * 备注信息：摄像头状态帧不要求绑定比赛会话，视觉程序在相机打不开时仍会
 *           持续发送故障状态，便于和“整条视觉链路断了”区分开。
 */
CarVisionCameraStatus CarVision_GetCameraStatus(const CarVision *v, uint32_t now_ms)
{
    return (uint32_t)(now_ms - v->camera_status_ms) <= VIS_STATUS_TIMEOUT_MS ?
           v->camera_status : VISION_CAMERA_UNKNOWN;
}

/*
 * 函数名称：CarVision_LinkAlive
 * 函数功能：判断香橙派链路是否在线。
 * 参数说明：v      —— 视觉解析器对象；
 *           now_ms —— 当前毫秒时间。
 * 返回参数：1 表示超时时间内收到过任意 CRC 正确的帧；0 表示无数据或已超时。
 * 备注信息：只认 CRC 已经通过的帧，因此能排除纯串口噪声；业务字段被拒
 *           （旧会话/旧序号等）也算链路在线，便于和“协议不匹配”区分。
 */
uint8_t CarVision_LinkAlive(const CarVision *v, uint32_t now_ms)
{
    if (v == NULL || !v->link_seen)
    {
        return 0;
    }
    return (uint32_t)(now_ms - v->link_frame_ms) <= VIS_LINK_TIMEOUT_MS;
}

/*
 * 函数名称：discard
 * 函数功能：从接收缓冲区头部丢弃 n 个字节，剩余字节整体前移。
 * 参数说明：v —— 视觉解析器对象；
 *           n —— 要丢弃的字节数，不得超过 v->used。
 * 返回参数：无。
 * 使用示例：
 *     discard(v, 1);              // 跳过一个非法字节
 *     discard(v, total);          // 移出一整帧
 * 备注信息：保持缓冲区内容连续，避免留下空洞。
 */
static void discard(CarVision *v, unsigned n)
{
    v->used = (uint8_t)(v->used - n);                  /* 更新剩余字节数 */

    if (v->used != 0U)
    {
        memmove(v->buffer, v->buffer + n, v->used);    /* 剩余字节前移 */
    }
}

/* ========================================================================== */
/*                              帧业务校验                                     */
/* ========================================================================== */

/*
 * 函数名称：accept
 * 函数功能：对 CRC 已通过的完整帧做业务校验，并提交道路观测或数独子块。
 * 参数说明：v           —— 视觉解析器对象；
 *           now         —— 本机收到该帧的毫秒时间；
 *           allow_hello —— 1 表示当前允许接受 HELLO 会话声明。
 * 返回参数：无。
 * 使用示例：
 *     // 仅在 CRC 校验通过后由 CarVision_Feed 调用
 *     accept(v, now_ms, allow_hello);
 * 备注信息：校验顺序为：版本/会话 -> HELLO -> 会话匹配/年龄 -> 序号 ->
 *           类型/长度 -> 业务字段。任何一步失败都只增加 rejected_frames。
 */
static void accept(CarVision *v, uint32_t now, uint8_t allow_hello)
{
    const uint8_t *f = v->buffer;                   /* 完整帧首字节 */
    const uint8_t *p = f + 14;                      /* 负载首字节，跳过 14 字节帧头 */
    uint8_t type = f[3];                            /* 帧类型 */
    uint16_t len = u16(f + 4);                      /* 负载长度 */
    uint16_t seq = u16(f + 6);                      /* 帧序号 */
    uint16_t age = u16(f + 12);                     /* 发送端处理年龄 */
    uint16_t delta;                                 /* 与上一序号的差值，用于回绕比较 */
    uint32_t session = u32(f + 8);                  /* 会话编号 */
    uint8_t link_down;                              /* 本帧之前链路是否已超时 */
    unsigned i;                                     /* 数独子块循环下标 */

    /* 先记录“本帧到达前”的链路状态，再刷新时间戳：同一会话的周期 HELLO
     * 平时忽略，只有链路刚恢复时才当作重连。 */
    link_down = (uint8_t)(!v->link_seen ||
        (uint32_t)(now - v->link_frame_ms) > VIS_LINK_TIMEOUT_MS);

    /* CRC 已通过：无论业务字段是否合法，都说明香橙派和串口在工作。
     * 放在版本/会话校验之前，旧会话、旧序号等被拒帧也能维持链路在线。 */
    v->link_frame_ms = now;
    v->link_seen = 1;

    /* 版本必须为 1，会话编号不能为 0。 */
    if (f[2] != 1 || session == 0)
    {
        v->last_reject = VISION_REJECT_VERSION;
        v->rejected_frames++;
        return;
    }

    if (type == 0 && len == 0 && age == 0)
    {
        /* 周期重发的同会话 HELLO 正常忽略；只有链路刚恢复且允许 HELLO 时，
         * 才清掉旧观测并重置序号水位，避免调试暂停后一直拒绝旧序号。 */
        if (v->session_valid && v->session == session)
        {
            if (allow_hello && link_down)
            {
                v->have_observation = 0;
                v->have_sequence = 0;
                v->last_reject = VISION_REJECT_NONE;  /* 链路恢复后的合法 HELLO 清除旧拒绝 */
                v->accepted_frames++;
            }
            return;
        }

        /* 只有停车且未在收尾时才允许绑定新会话。 */
        if (!allow_hello)
        {
            v->last_reject = VISION_REJECT_HELLO;
            v->rejected_frames++;
            return;
        }

        v->have_observation = 0;
        if (v->session != session) v->have_sequence = 0;
        for (i = 0; i < 4; i++)
        {
            v->tiles[i].valid = 0;
        }
        v->session = session;
        v->session_valid = 1;
        v->last_reject = VISION_REJECT_NONE;  /* 绑定成功后的首帧 STATUS 不携带旧拒绝 */
        v->accepted_frames++;
        return;
    }

    /* STATUS 仅用于显示。即使未绑定比赛会话，也可报告相机故障。 */
    if (type == 3)
    {
        if (len != 1 || seq != 0 || age != 0)
        {
            v->last_reject = VISION_REJECT_TYPE_LENGTH;
            v->rejected_frames++;
            return;
        }
        if (p[0] > 1U)
        {
            v->last_reject = VISION_REJECT_PAYLOAD;
            v->rejected_frames++;
            return;
        }
        v->camera_status = p[0] ? VISION_CAMERA_STREAMING : VISION_CAMERA_NO_FRAME;
        v->camera_status_ms = now;
        return;
    }

    /* 非 HELLO 帧必须属于当前会话，且发送端处理年龄不能超限。 */
    if (!v->session_valid || session != v->session)
    {
        v->last_reject = VISION_REJECT_SESSION;
        v->rejected_frames++;
        return;
    }
    if (age > VIS_MAX_AGE_MS)
    {
        v->last_reject = VISION_REJECT_AGE;
        v->rejected_frames++;
        return;
    }

    /*
     * 序号按 16 位环比较，允许回绕；
     * 差值 0 或大于等于半圈视为重复帧/旧帧。
     */
    delta = (uint16_t)(seq - v->last_sequence);

    if (v->have_sequence && (delta == 0U || delta >= 0x8000U))
    {
        v->last_reject = VISION_REJECT_SEQUENCE;
        v->rejected_frames++;
        return;
    }

    if (type == 1 && len == 16)
    {
        /* 道路观测帧：解析临时对象，全部校验通过后才覆盖缓存。 */
        CarVisionObservation o;                     /* 临时观测对象 */

        int32_t lateral = i16(p + 4);               /* 横向偏差，线上单位 mm */
        int32_t heading = i16(p + 6);               /* 航向角，线上单位 mrad */
        int32_t curvature = i16(p + 8);             /* 曲率，线上单位 1/km */
        uint16_t flags = u16(p);                    /* 有效位掩码 */
        uint16_t confidence = u16(p + 2);           /* 置信度千分数 */

        /*
         * 即使某有效位未置位，对应线上数值仍须在协议范围内；
         * 保留字段必须为 0，防止协议扩展时旧程序误读。
         */
        if (flags > 31U ||
            ((flags & VISION_CURVATURE_ESTIMATED) && !(flags & VISION_CURVATURE_VALID)) ||
            confidence > 1000U ||
            lateral < -1000 || lateral > 1000 ||
            heading < -3142 || heading > 3142 ||
            curvature < -20000 || curvature > 20000 ||
            p[10] > VISION_ELEMENT_TAG ||
            (p[11] > 7U && p[11] != 255U) ||
            u16(p + 14) != 0U)
        {
            v->last_reject = VISION_REJECT_PAYLOAD;
            v->rejected_frames++;
            return;
        }

        memset(&o, 0, sizeof(o));                   /* 先清零临时对象 */

        o.received_ms = now;                        /* 本机接收时间 */
        o.session = session;                        /* 会话编号 */
        o.sequence = seq;                           /* 业务序号 */
        o.processing_age_ms = age;                  /* 发送端处理年龄 */

        /* 线上定点值转换为米、弧度和 1/米；置信度从千分数变为 0~1。 */
        o.valid_mask = flags;                       /* 有效位掩码 */
        o.confidence = confidence / 1000.0f;        /* 置信度 0~1 */
        o.lateral_m = lateral / 1000.0f;            /* 横向偏差，米 */
        o.heading_rad = heading / 1000.0f;          /* 航向角，弧度 */
        o.curvature_inv_m = curvature / 1000.0f;    /* 曲率，1/米 */
        o.distance_m = u16(p + 12) / 1000.0f;       /* 距离，米 */
        o.element = p[10];                          /* 元素类型 */
        o.tag_id = p[11];                           /* Tag 编号 */

        v->observation = o;                         /* 提交新观测 */
        v->have_observation = 1;
        v->last_reject = VISION_REJECT_NONE;        /* 有效 TYPE=1 清除最近拒绝原因 */
    }
    else if (type == 2 && len == 10)
    {
        /* 数独子块帧：负载第 1 字节为块号 0~3，后 9 字节为 3×3 格值。 */
        CarVisionTile tile;                         /* 临时子块对象 */

        if (p[0] > 3U)                              /* 块号只能是 0~3 */
        {
            v->last_reject = VISION_REJECT_TILE;
            v->rejected_frames++;
            return;
        }

        for (i = 1; i < 10; i++)
        {
            /* 格值允许 0~6；其他值视为协议错误。 */
            if (p[i] > 6U)
            {
                v->last_reject = VISION_REJECT_TILE;
                v->rejected_frames++;
                return;
            }
        }

        memset(&tile, 0, sizeof(tile));             /* 先清零临时对象 */

        tile.received_ms = now;                     /* 本机接收时间 */
        tile.session = session;                     /* 会话编号 */
        tile.sequence = seq;                        /* 业务序号 */
        tile.valid = 1;                             /* 标记该块有效 */
        memcpy(tile.cells, p + 1, 9);               /* 复制 9 个格值 */

        v->tiles[p[0]] = tile;                      /* 按块号覆盖旧数据 */
    }
    else
    {
        /* 未知类型或长度不符。 */
        v->last_reject = VISION_REJECT_TYPE_LENGTH;
        v->rejected_frames++;
        return;
    }

    /*
     * 只有完整业务校验通过才推进序号；
     * 拒绝帧不能挤掉最后一份有效观测。
     */
    v->last_sequence = seq;                         /* 记录最新业务序号 */
    v->have_sequence = 1;
    v->accepted_frames++;                           /* 统计成功帧 */
}

/* ========================================================================== */
/*                              逐字节组帧                                     */
/* ========================================================================== */

/*
 * 函数名称：CarVision_Feed
 * 函数功能：输入一个字节，并循环提取缓冲区中所有完整、CRC 正确的帧。
 * 参数说明：v           —— 视觉解析器对象；
 *           byte        —— 刚收到的原始字节；
 *           now_ms      —— 当前毫秒时间，用于帧间隔和时效判断；
 *           allow_hello —— 1 表示停车状态，允许接受 HELLO 新会话。
 * 返回参数：无。
 * 使用示例：
 *     CarVision_Feed(&vision, byte, HAL_GetTick(),
 *                    car.state == CAR_STOP);
 * 备注信息：长度或 CRC 异常时只滑动一个字节，保留缓冲区内潜在的下一帧帧头。
 */
void CarVision_Feed(CarVision *v, uint8_t byte, uint32_t now_ms, uint8_t allow_hello)
{
    unsigned i;                      /* CRC 计算循环下标 */
    unsigned total;                  /* 整帧总长度，字节 */
    uint16_t len;                    /* 负载长度 */
    uint16_t crc;                    /* CRC 累计值 */

    /* 字节间隔超时后丢弃半帧，重新寻找帧头。 */
    if (v->used != 0U &&
        (uint32_t)(now_ms - v->last_byte_ms) > VIS_FRAME_GAP_MS)
    {
        v->used = 0;                 /* 丢弃半帧 */
    }

    v->last_byte_ms = now_ms;        /* 刷新最近收字节时间 */

    /* 缓冲区满时丢掉最旧的一个字节，为本次新字节腾出空间。 */
    if (v->used >= VIS_MAX_FRAME)
    {
        discard(v, 1);
    }

    v->buffer[v->used++] = byte;     /* 新字节追加到缓冲区 */

    while (v->used != 0U)
    {
        /* 帧头第一字节不是 0xA5：继续向后寻找。 */
        if (v->buffer[0] != 0xA5)
        {
            discard(v, 1);
            continue;
        }

        if (v->used < 2U)            /* 至少要有两个字节才能确认帧头 */
        {
            return;
        }

        /* 帧头第二字节不是 0x5A：丢弃第一个字节后重新寻找。 */
        if (v->buffer[1] != 0x5A)
        {
            discard(v, 1);
            continue;
        }

        /* 至少要有 6 字节才能读到长度字段。 */
        if (v->used < 6U)
        {
            return;
        }

        len = u16(v->buffer + 4);    /* 读取负载长度 */

        if (len > VIS_MAX_PAYLOAD)
        {
            v->bad_frames++;
            discard(v, 1);           /* 长度越界，滑动一字节 */
            continue;
        }

        /* 帧头 14 字节 + 负载 len + 帧尾 CRC 2 字节。 */
        total = 16U + len;

        if (v->used < total)         /* 完整帧尚未收齐 */
        {
            return;
        }

        /* CRC 覆盖从版本字节（下标 2）到负载末尾，不含帧头和 CRC 本身。 */
        crc = 0xFFFFU;

        for (i = 2; i < total - 2; i++)
        {
            crc = CarVision_Crc16(crc, v->buffer[i]);
        }

        if (crc != u16(v->buffer + total - 2))
        {
            v->bad_frames++;
            discard(v, 1);           /* CRC 错误，滑动一字节继续找 */
            continue;
        }

        /* CRC 正确：进行业务校验，然后整帧移出缓冲区。 */
        accept(v, now_ms, allow_hello);
        discard(v, total);
    }
}

/* ========================================================================== */
/*                              数据读取接口                                   */
/* ========================================================================== */

/*
 * 函数名称：CarVision_GetObservation
 * 函数功能：复制道路观测快照，并检查数据是否过期。
 * 参数说明：v      —— 视觉解析器对象，只读；
 *           now_ms —— 当前毫秒时间；
 *           out    —— 接收副本的对象，不能为空。
 * 返回参数：1 有可用且未过期的观测；0 无数据或已过期，失败时不改写 out。
 * 使用示例：
 *     CarVisionObservation obs;
 *     if (CarVision_GetObservation(&vision, HAL_GetTick(), &obs))
 *     {
 *         // obs 可用，仍需检查 valid_mask
 *     }
 * 备注信息：总年龄 = 发送端处理年龄 + 本机接收后经过时间；
 *           拆成两个比较避免加法溢出。
 */
uint8_t CarVision_GetObservation(const CarVision *v, uint32_t now_ms, CarVisionObservation *out)
{
    uint32_t age;             /* 本机收到观测后经过的时间，毫秒 */

    if (out == NULL || !v->have_observation)
    {
        return 0;
    }

    age = (uint32_t)(now_ms - v->observation.received_ms);

    if (age > VIS_MAX_AGE_MS ||
        age > VIS_MAX_AGE_MS - v->observation.processing_age_ms)
    {
        return 0;             /* 数据已过期 */
    }

    *out = v->observation;    /* 复制一致快照 */

    return 1;
}

/*
 * 函数名称：CarVision_GetTile
 * 函数功能：复制指定数独子块快照。
 * 参数说明：v      —— 视觉解析器对象，只读；
 *           index  —— 子块编号 0~3；
 *           now_ms —— 当前毫秒时间；
 *           out    —— 接收副本的对象，不能为空。
 * 返回参数：1 成功；0 参数无效或数据不可用，失败时不改写 out。
 * 使用示例：
 *     CarVisionTile tile;
 *     if (CarVision_GetTile(&vision, 0, HAL_GetTick(), &tile))
 *     {
 *         // tile.cells[0] ~ tile.cells[8] 可用
 *     }
 * 备注信息：数独块在当前会话内保留，不按观测的 200 ms 时效自动淘汰；
 *           这里只排除时间戳明显来自“未来”的异常数据。
 */
uint8_t CarVision_GetTile(const CarVision *v, uint8_t index, uint32_t now_ms, CarVisionTile *out)
{
    /*
     * 半范围比较排除看似来自“未来”的时间戳；
     * 这是无符号回绕约定，不是短期过期策略。
     */
    if (out == NULL || index > 3U || !v->session_valid || !v->tiles[index].valid ||
        v->tiles[index].session != v->session ||
        (uint32_t)(now_ms - v->tiles[index].received_ms) >= 0x80000000U)
    {
        return 0;
    }

    *out = v->tiles[index];    /* 复制一致快照 */

    return 1;
}
