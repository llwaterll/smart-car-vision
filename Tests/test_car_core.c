#include "Task_Fsm.h"
#include "Proto_Bluetooth.h"
#include "Board_Config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void command(Car_Core *c, unsigned char cmd, const unsigned char *data, unsigned char len)
{
    unsigned char frame[16], i, crc=0;
    frame[0]=0xc3; frame[1]=0x3c; frame[2]=cmd; frame[3]=len;
    if(len) memcpy(frame+4,data,len);
    for(i=2;i<4+len;i++) crc=Car_Crc8(crc,frame[i]);
    frame[4+len]=crc;
    for(i=0;i<5+len;i++) Car_FeedByte(c,frame[i]);
}
static void wheels(Car_Core *c, int l, int r)
{
    unsigned char p[4]={(unsigned char)(l>>8),(unsigned char)l,(unsigned char)(r>>8),(unsigned char)r};
    command(c,3,p,4);
}
static void openloop(Car_Core *c, unsigned wheel, int pwm)
{
    unsigned char p[3]={(unsigned char)wheel,(unsigned char)(pwm>>8),(unsigned char)pwm};
    command(c,6,p,3);
}
static void ticks(Car_Core *c, int n, int l, int r)
{ int i; for(i=0;i<n;i++) Car_Tick(c,l,r); }

static void test_right_reverse_protection(void)
{
    Car_Core c;
    int i;
    Car_Init(&c);
    command(&c,1,0,0);
    wheels(&c,-120,-120);
    /* Left wheel reverses; right wheel keeps moving forward at about 218 rpm.
     * Keep the link alive and stay below the overspeed threshold so only the
     * direction protection can stop this reproduction of the reported fault. */
    for(i=0;i<600 && c.state==CAR_RUN;i++) {
        if(i%20==0) command(&c,5,0,0);
        Car_Tick(&c,-4,8);
    }
    assert(c.state==CAR_FAULT && c.fault==CAR_DIRECTION_FAULT);
    assert(c.output[0]==0 && c.output[1]==0);

    /* Correct reverse feedback must remain drivable, not trip the same guard. */
    Car_Init(&c);
    command(&c,1,0,0);
    wheels(&c,-120,-120);
    for(i=0;i<600;i++) {
        if(i%20==0) command(&c,5,0,0);
        Car_Tick(&c,-4,-4);
        assert(c.state==CAR_RUN);
    }
}
static void test_open_loop(void)
{
    Car_Core c;
    int i;

    /* 开环测试只在 RUN 接受，进入后直接覆盖 PID 输出。 */
    Car_Init(&c);
    openloop(&c, 0, 1500);
    assert(c.result == 1 && c.open_loop[0] == 0 && c.output[0] == 0);
    command(&c, 1, 0, 0);
    openloop(&c, 0, 1500);
    assert(c.result == 0 && c.open_loop[0] == 1 && c.open_loop_pwm[0] == 1500);
    Car_Tick(&c, 0, 0);
    assert(c.output[0] == 1500 && c.output[1] == 0);

    /* 轮号和幅值越界必须拒绝，且保留上一帧的有效命令。 */
    openloop(&c, 2, 1500);
    assert(c.result == 2);
    openloop(&c, 0, 3001);
    assert(c.result == 2 && c.open_loop_pwm[0] == 1500);

    /* 刷新命令会更新输出；轮速命令立即退出开环。 */
    for (i = 0; i < 20; i++) {
        if (i == 10) openloop(&c, 0, 2000);
        Car_Tick(&c, 0, 0);
    }
    assert(c.state == CAR_RUN && c.open_loop[0] == 1 && c.output[0] == 2000);
    wheels(&c, 80, 0);
    assert(c.open_loop[0] == 0);
    Car_Tick(&c, 0, 0);
    assert(c.target[0] > 0);

    /* 行驶目标未清零时，首次进入开环必须被拒绝。 */
    openloop(&c, 1, -1200);
    assert(c.result == 2 && c.open_loop[1] == 0);
    wheels(&c, 0, 0);

    /* 心跳只续租链路，不能刷新开环命令；500 ms 后开环必须自撤输出。 */
    openloop(&c, 1, -1200);
    assert(c.open_loop[1] == 1 && c.open_loop_pwm[1] == -1200);
    Car_Tick(&c, 0, 0);
    assert(c.output[1] == -1200 && c.output[0] == 0);
    for (i = 0; i < 101; i++) {
        if (i % 20 == 0) command(&c, 5, 0, 0);
        Car_Tick(&c, 0, 0);
    }
    assert(c.state == CAR_RUN && c.open_loop[1] == 0 && c.output[1] == 0);

    /* STOP 必须清除两轮的开环状态。 */
    openloop(&c, 0, 1000);
    command(&c, 2, 0, 0);
    assert(c.open_loop[0] == 0 && c.open_loop[1] == 0);

    /* 开环没有参考转速，但堵转保护仍须工作。 */
    command(&c, 1, 0, 0);
    openloop(&c, 0, 1500);
    for (i = 0; i < 320 && c.state == CAR_RUN; i++) {
        if (i % 20 == 0) openloop(&c, 0, 1500);
        Car_Tick(&c, 0, 0);
    }
    assert(c.state == CAR_FAULT && c.fault == CAR_STALL_FAULT);

    /* 开环方向反馈相反时，方向保护仍须工作。 */
    Car_Init(&c);
    command(&c, 1, 0, 0);
    openloop(&c, 0, 1500);
    for (i = 0; i < 400 && c.state == CAR_RUN; i++) {
        if (i % 20 == 0) openloop(&c, 0, 1500);
        Car_Tick(&c, -4, 0);
    }
    assert(c.state == CAR_FAULT && c.fault == CAR_DIRECTION_FAULT);
}

int main(void)
{
    Car_Core c;
    test_right_reverse_protection();
    test_open_loop();
    { const unsigned char *p=(const unsigned char *)"123456789"; unsigned char crc=0;
      while(*p) { crc=Car_Crc8(crc,*p++); }
      assert(crc==0xf4); }
    Car_Init(&c);
    assert(c.state==CAR_STOP && c.output[0]==0);
    wheels(&c,80,80); assert(c.target[0]==0 && c.result==1);
    command(&c,1,0,0); wheels(&c,1200,-1200);
    assert(c.target[0]==BOARD_RPM_LIMIT && c.target[1]==-BOARD_RPM_LIMIT);
    /* 参考规划器已删除：reference 在同一拍直接等于 target，不再有渐升过程。 */
    Car_Tick(&c,0,0); assert(c.reference[0]==c.target[0] && c.reference[1]==c.target[1]);
    ticks(&c,99,0,0); assert(c.state==CAR_FAULT && c.fault==CAR_LINK_FAULT);
    assert(c.output[0]==0 && c.output[1]==0);
    command(&c,5,0,0); command(&c,1,0,0); assert(c.state==CAR_FAULT);
    command(&c,2,0,0); command(&c,1,0,0);
    assert(c.state==CAR_RUN && c.target[0]==0 && c.target[1]==0);
    wheels(&c,80,80); ticks(&c,10,0,0);
    wheels(&c,0,0); assert(c.output[0]==0 && c.reference[0]==0 && c.output[1]==0);
    wheels(&c,80,80); ticks(&c,10,0,0);
    { unsigned char bad[]={0xc3,0x3c,2,0,0xff}; unsigned i;
      for(i=0;i<sizeof(bad);i++) Car_FeedByte(&c,bad[i]); }
    assert(c.state==CAR_RUN);
    /* A truncated header immediately before STOP must not swallow STOP. */
    Car_FeedByte(&c,0xc3); Car_FeedByte(&c,0x3c);
    command(&c,2,0,0); assert(c.state==CAR_STOP && c.output[0]==0 && c.output[1]==0);
    command(&c,2,0,0); assert(c.output[0]==0 && c.target[0]==0);
    { unsigned char pid[]={0,0,200,0,12,0,0};
      command(&c,4,pid,7); assert(c.pid_speed[0].kp==2);
      command(&c,1,0,0); pid[2]=250; command(&c,4,pid,7); assert(c.pid_speed[0].kp==2 && c.result==1); }
    /* A truncated frame must expire, allowing the next complete STOP. */
    Car_FeedByte(&c,0xc3); Car_FeedByte(&c,0x3c); Car_FeedByte(&c,4);
    ticks(&c,21,0,0); command(&c,2,0,0); assert(c.state==CAR_STOP);
    /* Heartbeats keep the link alive but cannot hide a disconnected encoder. */
    command(&c,1,0,0); wheels(&c,180,180);
    { int i; for(i=0;i<1400 && c.state==CAR_RUN;i++) { if(i%20==0) command(&c,5,0,0); Car_Tick(&c,0,0); } }
    assert(c.state==CAR_FAULT && c.fault==CAR_STALL_FAULT);
    command(&c,2,0,0); command(&c,1,0,0); wheels(&c,180,180);
    { int i; for(i=0;i<500 && c.state==CAR_RUN;i++) { if(i%20==0) command(&c,5,0,0); Car_Tick(&c,-8,-8); } }
    assert(c.state==CAR_FAULT && c.fault==CAR_DIRECTION_FAULT);
    /* Persistent reverse motion must fault even before PWM saturates. */
    Car_Init(&c);
    assert(Car_SetPid(&c, 1, 0.1f, 0.0f, 0.0f) == 0);
    command(&c,1,0,0); wheels(&c,180,180);
    { int i; for(i=0;i<300 && c.state==CAR_RUN;i++) {
        if(i%20==0) command(&c,5,0,0);
        Car_Tick(&c,7,-8);
    } }
    assert(c.state==CAR_FAULT && c.fault==CAR_DIRECTION_FAULT);
    assert(c.output[0]==0 && c.output[1]==0);
    /* Corrupted heartbeats must not keep a moving vehicle enabled. */
    command(&c,2,0,0); command(&c,1,0,0); wheels(&c,80,80);
    { int i; for(i=0;i<100;i++) {
        Car_FeedByte(&c,0xc3); Car_FeedByte(&c,0x3c);
        Car_FeedByte(&c,5); Car_FeedByte(&c,0); Car_FeedByte(&c,0xff);
        Car_Tick(&c,0,0);
    } }
    assert(c.state==CAR_FAULT && c.fault==CAR_LINK_FAULT);
    command(&c,2,0,0); command(&c,1,0,0); wheels(&c,80,80);
    Car_TickDt(&c,0,0,20000);
    assert(c.state==CAR_FAULT && c.fault==CAR_TIMING_FAULT && c.output[0]==0);
    assert(c.timing_fault_us==20000U);
    command(&c,2,0,0); command(&c,1,0,0); wheels(&c,80,80);
    { int i; for(i=0;i<50;i++)Car_TickDt(&c,0,0,10000); }
    assert(c.state==CAR_FAULT && c.fault==CAR_LINK_FAULT);
    Car_Init(&c);
    { int i; for(i=0;i<100;i++)Car_TickDt(&c,2,2,10000); }
    assert(c.speed[0]>27.2f && c.speed[0]<27.4f); /* Two counts /10ms, not /5ms. */
    /* Sliding speed window: one isolated burst of N counts is 218 rpm in a
     * single tick, but averaged over the full N-tick window it is 27.3 rpm.
     * After one 5 ms filter step the reading is alpha * 27.3 rpm, not the
     * 54.5 rpm a single-tick measurement with a 5 ms filter would produce.
     * Express the band symbolically so changing the window or filter does
     * not silently invalidate the test. */
    Car_Init(&c);
    {
        int i;
        float alpha;
        for(i=0;i<(int)TASK_WINDOW_TICKS;i++)Car_Tick(&c,0,0);
        Car_Tick(&c,(int)TASK_WINDOW_TICKS,(int)TASK_WINDOW_TICKS);
        alpha=0.005f/(TASK_SPEED_FILTER_TAU_S+0.005f);
        assert(c.speed[0]>alpha*27.0f*0.95f && c.speed[0]<alpha*27.5f*1.05f);
    }
    /* Identical encoder input and targets must produce identical speed and PWM
     * on both wheels, so a straight command cannot introduce a speed bias.
     * Production defaults use wheel-specific dead-zone feedforward; disable
     * both the new dead-zone model and the legacy Ks to isolate the symmetric
     * control law. */
    Car_Init(&c);
    c.pid_speed[0].ks = 0.0f;
    c.pid_speed[1].ks = 0.0f;
    c.pid_speed[0].ff_model = 0;
    c.pid_speed[1].ff_model = 0;
    command(&c,1,0,0); wheels(&c,90,90);
    { int i; for(i=0;i<100;i++) {
        if(i%20==0) command(&c,5,0,0);
        Car_Tick(&c,4,4);
    } }
    assert(c.speed[0]==c.speed[1]);
    assert(c.output[0]==c.output[1]);
    assert(c.state==CAR_RUN);
    Car_Init(&c);
    command(&c,1,0,0); wheels(&c,-180,180);
    { int i; for(i=0;i<60 && c.state==CAR_RUN;i++) {
        if(i%20==0) command(&c,5,0,0);
        Car_Tick(&c,-40,0);
    } }
    assert(c.state==CAR_FAULT && c.fault==CAR_OVERSPEED_FAULT);
    assert(c.output[0]==0 && c.output[1]==0);
    puts("car core: all checks passed");
    return 0;
}
