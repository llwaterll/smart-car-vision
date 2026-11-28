#include "Proto_Vision.h"
#include "Proto_VisionStatus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 主机测试只编译 Tools/check_car.ps1 列出的源文件；把纯编码器直接包含进来，
 * 让 STATUS 黄金向量在现有视觉测试里执行，不改变测试构建脚本。 */
#include "Proto_VisionStatus.c"

static unsigned make_frame(uint8_t *f, uint8_t type, uint16_t seq, uint32_t session,
                           uint16_t age, const uint8_t *data, unsigned len)
{
    unsigned i; uint16_t crc=0xffff;
    f[0]=0xa5;f[1]=0x5a;f[2]=1;f[3]=type;f[4]=0;f[5]=(uint8_t)len;
    f[6]=(uint8_t)(seq>>8);f[7]=(uint8_t)seq;
    f[8]=(uint8_t)(session>>24);f[9]=(uint8_t)(session>>16);
    f[10]=(uint8_t)(session>>8);f[11]=(uint8_t)session;
    f[12]=(uint8_t)(age>>8);f[13]=(uint8_t)age;
    if(len)memcpy(f+14,data,len);
    for(i=2;i<14+len;i++)crc=CarVision_Crc16(crc,f[i]);
    f[14+len]=(uint8_t)(crc>>8);f[15+len]=(uint8_t)crc;
    return len+16;
}
static void feed(CarVision *v, const uint8_t *f, unsigned n, uint32_t t, uint8_t stopped)
{unsigned i;for(i=0;i<n;i++)CarVision_Feed(v,f[i],t,stopped);}

static void test_hello_recovery(void)
{
    const uint32_t session = 123;
    const uint8_t data[16] = {0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0};
    const uint8_t cells[10] = {0,1,2,3,4,5,6,1,2,3};
    uint8_t hello[VIS_MAX_FRAME], track[VIS_MAX_FRAME], tile[VIS_MAX_FRAME];
    unsigned hello_len = make_frame(hello,0,0,session,0,NULL,0);
    unsigned track_len = make_frame(track,1,45,session,0,data,sizeof data);
    unsigned tile_len = make_frame(tile,2,46,session,0,cells,sizeof cells);
    CarVision vision;
    CarVisionObservation observation;
    CarVisionTile saved_tile;

    CarVision_Init(&vision);
    feed(&vision,hello,hello_len,100,1);
    feed(&vision,track,track_len,120,1);
    feed(&vision,tile,tile_len,130,1);
    feed(&vision,hello,hello_len,140,1);
    feed(&vision,hello,hello_len,150,0);
    assert(vision.accepted_frames == 3 && vision.rejected_frames == 0);
    assert(vision.last_sequence == 46 && CarVision_GetObservation(&vision,150,&observation));
    assert(observation.received_ms == 120 && CarVision_GetTile(&vision,0,150,&saved_tile));

    /* The sender keeps its session and sequence when only the STM32 resets. */
    CarVision_Init(&vision);
    feed(&vision,track,track_len,200,1);
    assert(!CarVision_GetObservation(&vision,200,&observation));
    feed(&vision,hello,hello_len,210,0);
    assert(!vision.session_valid);
    feed(&vision,hello,hello_len,220,1);
    feed(&vision,track,track_len,230,1);
    assert(CarVision_GetObservation(&vision,230,&observation));
    assert(observation.session == session && observation.sequence == 45);
}

static void test_link_alive(void)
{
    const uint8_t data[16] = {0};
    const uint8_t camera[1] = {1};
    uint8_t hello[VIS_MAX_FRAME], status[VIS_MAX_FRAME], track[VIS_MAX_FRAME];
    CarVision v;
    unsigned n;

    CarVision_Init(&v);
    assert(!CarVision_LinkAlive(&v, 0U));

    /* HELLO 也是合法帧：即使还没收到状态/道路帧，也能证明香橙派在线。 */
    n = make_frame(hello,0,0,42,0,NULL,0);
    feed(&v,hello,n,100U,1);
    assert(CarVision_LinkAlive(&v,100U));
    assert(CarVision_LinkAlive(&v,100U+VIS_LINK_TIMEOUT_MS));
    assert(!CarVision_LinkAlive(&v,101U+VIS_LINK_TIMEOUT_MS));

    /* 状态帧不要求比赛会话，同样刷新链路时间戳。 */
    n = make_frame(status,3,0,42,0,camera,1);
    feed(&v,status,n,3000U,0);
    assert(CarVision_LinkAlive(&v,3000U));
    assert(!CarVision_LinkAlive(&v,3001U+VIS_LINK_TIMEOUT_MS));

    /* 会话不符被业务层拒绝，但 CRC 正确仍证明链路在线。 */
    n = make_frame(track,1,9,43,0,data,16);
    feed(&v,track,n,5000U,0);
    assert(CarVision_LinkAlive(&v,5000U));

    /* 单次串口异常只丢半帧；链路保留到 1.5 s 超时才判离线。 */
    CarVision_Invalidate(&v);
    assert(CarVision_LinkAlive(&v,5000U));
    assert(!CarVision_LinkAlive(&v,5001U+VIS_LINK_TIMEOUT_MS));
    n = make_frame(status,3,0,42,0,camera,1);
    feed(&v,status,n,5100U,0);
    assert(CarVision_LinkAlive(&v,5100U));
}

static void test_hello_golden(void)
{
    static const uint8_t golden[16] = {
        0xA5,0x5A,0x01,0x00,0x00,0x00,0x00,0x00,
        0x11,0x22,0x33,0x44,0x00,0x00,0xE2,0xC7
    };
    uint8_t f[VIS_MAX_FRAME];
    unsigned n = make_frame(f, 0, 0, 0x11223344U, 0, NULL, 0);

    assert(n == sizeof golden);
    assert(memcmp(f, golden, sizeof golden) == 0);
}

static void test_status_golden(void)
{
    static const uint8_t golden1[VISION_STATUS_FRAME_SIZE] = {
        0x5A,0xA5,0x01,0x01,0x00,0x1A,0x12,0x34,
        0x00,0x00,0x03,0x00,0x11,0x22,0x33,0x44,
        0x00,0x00,0x00,0x64,0x00,0x00,0x00,0x05,
        0x00,0x00,0x00,0x02,0x00,0x20,0x00,0x00,
        0xAB,0xCD,0x31,0xD2
    };
    static const uint8_t golden2[VISION_STATUS_FRAME_SIZE] = {
        0x5A,0xA5,0x01,0x01,0x00,0x1A,0x00,0x01,
        0x01,0x00,0x20,0x03,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,
        0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,
        0x00,0x10,0x7C,0xB6
    };
    VisionStatus s;
    uint8_t out[VISION_STATUS_FRAME_SIZE];

    memset(&s, 0, sizeof s);
    s.seq = 0x1234;
    s.state = 0;
    s.fault = 0;
    s.flags = VISION_STATUS_FLAG_ALLOW_HELLO | VISION_STATUS_FLAG_SESSION_VALID;
    s.reject = VISION_REJECT_NONE;
    s.bound_session = 0x11223344U;
    s.accepted = 100U;
    s.rejected = 5U;
    s.bad = 2U;
    s.obs_age_ms = 32U;
    s.uptime_ms = 0x0000ABCDU;
    assert(VisionStatus_Encode(&s, out) == VISION_STATUS_FRAME_SIZE);
    assert(memcmp(out, golden1, sizeof golden1) == 0);

    memset(&s, 0, sizeof s);
    s.seq = 1U;
    s.state = 1U;
    s.flags = VISION_STATUS_FLAG_AUTONOMOUS;
    s.reject = VISION_REJECT_SESSION;
    s.rejected = 7U;
    s.obs_age_ms = 0xFFFFU;
    s.uptime_ms = 0x10U;
    assert(VisionStatus_Encode(&s, out) == VISION_STATUS_FRAME_SIZE);
    assert(memcmp(out, golden2, sizeof golden2) == 0);
}

static void test_last_reject(void)
{
    const uint8_t data[16] = {0};
    uint8_t f[VIS_MAX_FRAME];
    CarVision v;
    unsigned n;

    CarVision_Init(&v);

    /* 版本/会话为 0 -> 1 */
    n = make_frame(f, 1, 1, 0, 0, data, 16);
    feed(&v, f, n, 10U, 1);
    assert(v.last_reject == VISION_REJECT_VERSION);

    /* HELLO 不被允许 -> 2 */
    n = make_frame(f, 0, 0, 42, 0, NULL, 0);
    feed(&v, f, n, 20U, 0);
    assert(v.last_reject == VISION_REJECT_HELLO);

    /* 绑定后会话不匹配 -> 3；绑定成功本身必须清除旧拒绝 */
    feed(&v, f, n, 30U, 1);
    assert(v.session_valid && v.session == 42U);
    assert(v.last_reject == VISION_REJECT_NONE);
    n = make_frame(f, 1, 1, 43, 0, data, 16);
    feed(&v, f, n, 40U, 1);
    assert(v.last_reject == VISION_REJECT_SESSION);

    /* 年龄超限 -> 4 */
    n = make_frame(f, 1, 2, 42, 201, data, 16);
    feed(&v, f, n, 50U, 1);
    assert(v.last_reject == VISION_REJECT_AGE);

    /* 有效 TYPE=1 清除 -> 0 */
    n = make_frame(f, 1, 3, 42, 0, data, 16);
    feed(&v, f, n, 60U, 1);
    assert(v.last_reject == VISION_REJECT_NONE);

    /* 序号重复 -> 5 */
    feed(&v, f, n, 70U, 1);
    assert(v.last_reject == VISION_REJECT_SEQUENCE);

    /* 字段非法 -> 6 */
    {
        uint8_t bad[16] = {0};
        bad[0] = 0U;
        bad[1] = 32U;                 /* flags=32 超出协议范围 */
        n = make_frame(f, 1, 4, 42, 0, bad, 16);
        feed(&v, f, n, 80U, 1);
        assert(v.last_reject == VISION_REJECT_PAYLOAD);
    }

    /* 未知类型 -> 7 */
    n = make_frame(f, 9, 5, 42, 0, data, 16);
    feed(&v, f, n, 90U, 1);
    assert(v.last_reject == VISION_REJECT_TYPE_LENGTH);

    /* 数独块非法 -> 8 */
    {
        uint8_t cells[10] = {0,1,2,3,4,5,6,1,2,7};
        n = make_frame(f, 2, 6, 42, 0, cells, 10);
        feed(&v, f, n, 100U, 1);
        assert(v.last_reject == VISION_REJECT_TILE);
    }
}

int main(void)
{
    CarVision v; CarVisionObservation o; CarVisionTile tile;
    test_hello_recovery();
    test_link_alive();
    test_hello_golden();
    test_status_golden();
    test_last_reject();
    uint8_t f[64],data[16]={0,15,3,132,0,50,0,100,0,20,6,255,0,80,0,0};
    unsigned n; uint16_t crc=0xffff; const uint8_t *s=(const uint8_t *)"123456789";
    while(*s)crc=CarVision_Crc16(crc,*s++);
    assert(crc==0x29b1);
    CarVision_Init(&v);
    n=make_frame(f,1,1,123,0,data,16);feed(&v,f,n,10,1);
    assert(!CarVision_GetObservation(&v,10,&o)); /* Explicit HELLO required. */
    n=make_frame(f,0,0,123,0,0,0);feed(&v,f,n,20,0);
    assert(!v.session_valid); /* No session takeover while driving. */
    feed(&v,f,n,20,1);assert(v.session_valid);
    n=make_frame(f,1,65535,123,30,data,16);feed(&v,f,n,100,0);
    assert(CarVision_GetObservation(&v,100,&o));
    assert(o.lateral_m>0.049f && o.lateral_m<0.051f && o.element==6);
    assert(!CarVision_GetObservation(&v,271,&o)); /* 30ms processing + 171ms transport age. */
    n=make_frame(f,1,0,123,0,data,16);feed(&v,f,n,280,0); /* Sequence wrap. */
    assert(CarVision_GetObservation(&v,280,&o) && o.sequence==0);
    feed(&v,f,n,350,0);assert(CarVision_GetObservation(&v,450,&o));
    assert(!CarVision_GetObservation(&v,481,&o)); /* Duplicate does not refresh age. */
    n=make_frame(f,1,65534,123,0,data,16);feed(&v,f,n,490,0);
    assert(!CarVision_GetObservation(&v,490,&o)); /* Old frame rejected. */
    n=make_frame(f,1,1,123,0,data,16);f[n-1]^=1;feed(&v,f,n,500,0);
    assert(v.bad_frames>0 && !CarVision_GetObservation(&v,500,&o));
    n=make_frame(f,1,1,123,0,data,16);feed(&v,f,9,510,0);
    feed(&v,f,n,600,0);assert(CarVision_GetObservation(&v,600,&o)); /* Fragment timeout. */
    data[0]=0;data[1]=0;
    n=make_frame(f,1,2,123,0,data,16);feed(&v,f,n,650,0);
    assert(CarVision_GetObservation(&v,650,&o) && o.valid_mask==0); /* Alive, no detection. */
    data[2]=0xff;data[3]=0xff;
    n=make_frame(f,1,3,123,0,data,16);feed(&v,f,n,660,0);
    assert(CarVision_GetObservation(&v,660,&o) && o.sequence==2); /* Invalid confidence ignored. */
    n=make_frame(f,0,0,456,0,0,0);feed(&v,f,n,700,0);
    assert(v.session==123);
    feed(&v,f,n,700,1);assert(v.session==456 && !CarVision_GetObservation(&v,700,&o));
    {uint8_t cells[10]={2,1,2,3,4,5,6,0,0,0};
     n=make_frame(f,2,1,456,0,cells,10);feed(&v,f,n,710,0);
     assert(CarVision_GetTile(&v,2,710,&tile) && tile.cells[5]==6);}
    {
        uint8_t cells[10]={0,1,0,3,0,5,0,2,0,1};
        unsigned i;
        for (i=0;i<4;++i) {
            cells[0]=(uint8_t)(3-i);
            n=make_frame(f,2,(uint16_t)(2+i),456,10,cells,10);
            feed(&v,f,n,720+i,0);
        }
        for (i=0;i<4;++i) assert(CarVision_GetTile(&v,(uint8_t)i,730,&tile));
        cells[0]=0; cells[1]=7;
        n=make_frame(f,2,6,456,0,cells,10);feed(&v,f,n,740,0);
        assert(v.last_sequence==5 && CarVision_GetTile(&v,0,740,&tile) && tile.cells[0]==1);
        cells[1]=2; cells[0]=4;
        n=make_frame(f,2,6,456,0,cells,10);feed(&v,f,n,740,0);
        assert(v.last_sequence==5);
        cells[0]=0;
        n=make_frame(f,2,6,456,0,cells,10);f[n-1]^=1;feed(&v,f,n,740,0);
        assert(v.last_sequence==5 && v.tiles[0].cells[0]==1);
        n=make_frame(f,2,5,456,0,cells,10);feed(&v,f,n,740,0);
        assert(v.tiles[0].cells[0]==1);
        CarVision_Invalidate(&v);
        assert(CarVision_GetTile(&v,0,750,&tile) && tile.cells[0]==1);
        n=make_frame(f,0,0,456,0,0,0);feed(&v,f,n,750,1);
        assert(CarVision_GetTile(&v,0,750,&tile));
        n=make_frame(f,0,0,789,0,0,0);feed(&v,f,n,760,1);
        for (i=0;i<4;++i) assert(!CarVision_GetTile(&v,(uint8_t)i,760,&tile));
    }
    data[2]=0;data[3]=0;
    n=make_frame(f,1,2,789,0,data,16);feed(&v,f,n,0xfffffff0U,0);
    assert(CarVision_GetObservation(&v,0x10U,&o));
    assert(!CarVision_GetObservation(&v,0x100U,&o));
    CarVision_Invalidate(&v);assert(!CarVision_GetObservation(&v,0x100U,&o));

    /* A diagnostic sender can report camera failure without claiming a race session. */
    data[0] = 0;
    n = make_frame(f,3,0,456,0,data,1);
    feed(&v,f,n,0xfffffff0U,0);
    assert(CarVision_GetCameraStatus(&v,0x10U) == VISION_CAMERA_NO_FRAME);
    assert(v.session==789 && v.last_sequence==2 && !CarVision_GetObservation(&v,0x10U,&o));
    data[0] = 1;
    n = make_frame(f,3,0,456,0,data,1);
    feed(&v,f,n,100U,0);
    assert(CarVision_GetCameraStatus(&v,100U) == VISION_CAMERA_STREAMING);
    f[n-1] ^= 1;
    feed(&v,f,n,1000U,0);
    assert(CarVision_GetCameraStatus(&v,1101U) == VISION_CAMERA_UNKNOWN);
    CarVision_Invalidate(&v);
    /* 单次串口异常不清相机状态；超过 1 s 没新状态帧才变 UNKNOWN。 */
    assert(CarVision_GetCameraStatus(&v,100U) == VISION_CAMERA_STREAMING);
    assert(CarVision_GetCameraStatus(&v,1101U) == VISION_CAMERA_UNKNOWN);

    /* Estimated curvature can steer, but cannot launch or rejoin on its own. */
    data[0] = 0;
    data[1] = VISION_CURVATURE_ESTIMATED;
    n = make_frame(f,1,3,789,0,data,16);
    feed(&v,f,n,2000U,0);
    assert(!CarVision_GetObservation(&v,2000U,&o));
    data[1] |= VISION_CURVATURE_VALID;
    n = make_frame(f,1,3,789,0,data,16);
    feed(&v,f,n,2005U,0);
    assert(CarVision_GetObservation(&v,2005U,&o) && o.valid_mask == data[1]);

    /* A stopped race closes locally; the unchanged sender can start a clean round. */
    {
        uint8_t cells[10] = {0,1,2,3,4,5,6,1,2,3};
        n=make_frame(f,2,4,789,0,cells,10); feed(&v,f,n,2100,1);
        assert(CarVision_GetTile(&v,0,2100,&tile));
        CarVision_CloseRaceSession(&v);
        assert(!v.session_valid && !CarVision_GetObservation(&v,2100,&o));
        assert(!CarVision_GetTile(&v,0,2100,&tile));
        n=make_frame(f,0,0,789,0,NULL,0); feed(&v,f,n,2110,0);
        assert(!v.session_valid);
        feed(&v,f,n,2120,1);
        assert(v.session_valid && !CarVision_GetObservation(&v,2120,&o));
        n=make_frame(f,1,3,789,0,data,16); feed(&v,f,n,2130,1);
        assert(!CarVision_GetObservation(&v,2130,&o));
        n=make_frame(f,1,5,789,0,data,16); feed(&v,f,n,2140,1);
        assert(CarVision_GetObservation(&v,2140,&o) && o.sequence==5);
        assert(!CarVision_GetTile(&v,0,2140,&tile));
    }
    puts("vision interface: all checks passed");
    return 0;
}
