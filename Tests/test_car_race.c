#include "Track_Fsm.h"
#include "Sudoku.h"
#include "Proto_Bluetooth.h"
#include "Task_Fsm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* A 200 Hz differential-drive plant with 25 Hz independent camera updates.
 * Generated course elements exercise real wheel targets and encoder feedback. */
static void two_laps(void) {
    TrackState r;
    CarVisionObservation o = {0};
    TrackDrive wheels = {{0,0}};
    unsigned ticks, gaps=0, stops=0, bypasses=0;
    unsigned stop_ms=0;
    float dl=0, dr=0, clearance=0;
    Track_Start(&r);
    for (ticks=0; ticks<160000 && r.phase!=TRACK_SEGMENT_FINISHED; ++ticks) {
        float position = fmodf(r.distance,TRACK_LAP_M);
        TrackPhase before = r.phase;
        if (ticks%8==0) {
            ++o.sequence;
            o.confidence=0.9f;
            o.element=VISION_ELEMENT_NONE;
            o.valid_mask=VISION_CURVATURE_VALID;
            o.curvature_inv_m=0;
            if (position>=5 && position<6.60f) { o.valid_mask=0; o.element=VISION_ELEMENT_GAP; }
            if (position>=12 && position<12.6f) {
                o.element=VISION_ELEMENT_YELLOW; o.distance_m=12.6f-position;
                o.valid_mask |= VISION_DISTANCE_VALID;
            }
            if (position>=20 && position<20.5f) {
                o.element=VISION_ELEMENT_RED; o.distance_m=20.5f-position;
                o.valid_mask |= VISION_DISTANCE_VALID;
            }
            if (position>=30 && position<31) o.element=VISION_ELEMENT_GREEN;
        }
        wheels=Track_Step(&r,&o,dl,dr,0.005f,0,0,dl==0 && dr==0,ticks*5);
        assert(r.phase!=TRACK_SEGMENT_FAULT);
        if (r.phase!=before && r.phase==TRACK_SEGMENT_GAP) ++gaps;
        if (r.phase!=before && r.phase==TRACK_SEGMENT_BYPASS) ++bypasses;
        if (r.phase==TRACK_SEGMENT_BYPASS && fabsf(r.goal_m-0.42f)<0.005f) clearance=fabsf(r.offset);
        if (r.phase!=before && r.phase==TRACK_SEGMENT_RED) { ++stops; stop_ms=ticks*5; }
        if (before==TRACK_SEGMENT_RED && r.phase!=before) assert(ticks*5-stop_ms>=5000);
        dl=wheels.rpm[0]*0.005f*6.2831853f*TRACK_RADIUS_M/60;
        dr=wheels.rpm[1]*0.005f*6.2831853f*TRACK_RADIUS_M/60;
    }
    assert(r.phase==TRACK_SEGMENT_FINISHED && gaps==2 && bypasses==2 && stops==1);
    assert(clearance>0.12f && wheels.rpm[0]==0 && wheels.rpm[1]==0);
}

/* Single-side estimated curvature may steer at hold speed, but must not refresh
 * the measured-road timestamp or the straight-distance accumulator. */
static void estimated_curvature_hold(void) {
    TrackState r;
    CarVisionObservation o = {0};
    TrackDrive cmd;
    Track_Start(&r);
    o.valid_mask = VISION_CURVATURE_VALID;
    o.confidence = 0.9f;
    o.curvature_inv_m = 0;
    o.sequence = 1;
    cmd = Track_Step(&r,&o,0.01f,0.01f,0.005f,0,0,0,100);
    assert(r.phase == TRACK_SEGMENT_TRACK && !r.degraded);
    assert(r.last_measured_curve == 0 && r.last_road_ms == 100);

    o.valid_mask |= VISION_CURVATURE_ESTIMATED;
    o.curvature_inv_m = 2.0f;
    ++o.sequence;
    r.straight_m = 0.12f;
    cmd = Track_Step(&r,&o,0.01f,0.01f,0.005f,0,0,0,200);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.degraded);
    assert(r.last_curve > 1.9f);           /* estimated curve still steers */
    assert(r.last_measured_curve == 0);    /* measured curve unchanged */
    assert(r.last_road_ms == 100);         /* line-loss timer not refreshed */
    assert(fabsf(r.straight_m - 0.13f) < 0.0001f); /* measured straight only */
    {
        float mps = 6.2831853f * TRACK_RADIUS_M / 60.0f * 0.5f * (cmd.rpm[0] + cmd.rpm[1]);
        assert(mps <= TRACK_HOLD_SPEED_MPS + 0.001f);
    }
}

/* 黄灯必须连续 3 个新帧确认才进绕障；中间任一帧不满足立即清零。 */
static void yellow_confirmation(void) {
    TrackState r;
    CarVisionObservation o = {0};
    TrackDrive cmd;
    Track_Start(&r);
    o.valid_mask = VISION_CURVATURE_VALID | VISION_DISTANCE_VALID;
    o.confidence = 0.9f;
    o.element = VISION_ELEMENT_YELLOW;
    o.distance_m = 0.50f;
    o.sequence = 1;
    o.received_ms = 100;

    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,100);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.yellow_frames == 1);
    assert(cmd.rpm[0] > 0.0f && cmd.rpm[1] > 0.0f);
    ++o.sequence; o.received_ms = 140;
    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,140);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.yellow_frames == 2);

    /* 单帧元素异常：计数清零，重新开始。 */
    o.element = VISION_ELEMENT_NONE;
    ++o.sequence; o.received_ms = 180;
    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,180);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.yellow_frames == 0);

    o.element = VISION_ELEMENT_YELLOW;
    ++o.sequence; o.received_ms = 220;
    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,220);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.yellow_frames == 1);
    ++o.sequence; o.received_ms = 260;
    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,260);
    assert(r.phase == TRACK_SEGMENT_TRACK && r.yellow_frames == 2);
    ++o.sequence; o.received_ms = 300;
    cmd = Track_Step(&r,&o,0,0,0.005f,0,0,1,300);
    assert(r.phase == TRACK_SEGMENT_BYPASS);
}

int main(void) {
    TrackState r;
    TrackDrive cmd;
    Car_Core car;
    CarVisionObservation o = {0};
    uint8_t tiles[4][9]={{1,0,3,0,5,0,2,0,1},{4,0,0,0,2,3,5,0,4},
                         {5,6,0,0,1,2,6,0,0},{0,3,1,6,0,5,0,1,0}};
    const uint8_t solution[36]={1,2,3,4,5,6,4,5,6,1,2,3,2,3,1,5,6,4,
                                5,6,4,2,3,1,3,1,2,6,4,5,6,4,5,3,1,2};
    uint8_t answer[36];
    assert(Sudoku_Solve(tiles,answer) && !memcmp(answer,solution,36));
    tiles[0][1]=1;
    assert(!Sudoku_Solve(tiles,answer));
    memset(tiles,0,sizeof(tiles));
    assert(!Sudoku_Solve(tiles,answer));
    estimated_curvature_hold();
    yellow_confirmation();
    Track_Start(&r);
    o.valid_mask=VISION_CURVATURE_VALID; o.curvature_inv_m=3; o.confidence=1;
    cmd=Track_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[1]>cmd.rpm[0] && cmd.rpm[0]>0);
    Car_Init(&car);
    assert(Car_StartRace(&car));
    Car_RaceWheels(&car,cmd.rpm);
    Car_Tick(&car,0,0);
    assert(car.reference[0]>0 && car.reference[1]>car.reference[0]);
    Car_LocalStop(&car);
    assert(car.state==CAR_STOP && !(car.motion == CAR_MOTION_RACE) && !car.output[1]);
    /* A curvature spike beyond the chassis turning limit must not command the
     * inner wheel to reverse while the car is moving forward: it may stop, but
     * its target stays on the commanded side instead of being lost. */
    o.curvature_inv_m=15;
    cmd=Track_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[0]>=0 && cmd.rpm[1]>0);
    o.curvature_inv_m=-15;
    cmd=Track_Step(&r,&o,0,0,0.005f,0,0,1,0);
    assert(cmd.rpm[0]>0 && cmd.rpm[1]>=0);
    /* A missing/stale link must hold the last curvature and keep driving. */
    cmd=Track_Step(&r,NULL,0,0,0.005f,0,0,1,5);
    assert(r.phase==TRACK_SEGMENT_TRACK && r.degraded && cmd.rpm[0]>0 && cmd.rpm[1]>=0);
    /* Even a prolonged outage must not fault the race; only the warning stays. */
    for (unsigned i=0;i<400;++i) {
        cmd=Track_Step(&r,NULL,0,0,0.005f,0,0,1,20U+i*5U);
        assert(r.phase==TRACK_SEGMENT_TRACK && r.degraded);
    }
    /* A fresh frame that found no road is the same hold, not a fault. */
    o.valid_mask=0;
    ++o.sequence;
    cmd=Track_Step(&r,&o,0,0,0.005f,0,0,1,2500);
    assert(r.phase==TRACK_SEGMENT_TRACK && r.degraded && cmd.rpm[0]>0);
    /* The next valid curvature frame clears the warning. */
    o.valid_mask=VISION_CURVATURE_VALID;
    o.curvature_inv_m=0;
    o.confidence=1;
    ++o.sequence;
    cmd=Track_Step(&r,&o,0,0,0.005f,0,0,1,2505);
    assert(r.phase==TRACK_SEGMENT_TRACK && !r.degraded && cmd.rpm[0]>0);
    two_laps();
    puts("PASS race authority, geometry, vision hold, two laps, 1.60 m gaps, bypass clearance, red dwell, finish and Sudoku");
    return 0;
}
