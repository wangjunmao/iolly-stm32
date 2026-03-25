#ifndef MINIKAME_H
#define MINIKAME_H

#include <stdint.h>
#include <stdbool.h>
#include "oscillator.h"
#include "pwm_servo.h"

#define NUM_SERVOS 9

typedef struct {
    // 硬件引脚
    uint8_t board_pins[NUM_SERVOS];
    // 校准值
    float trim[NUM_SERVOS];
    // 反转标志
    bool reverse[NUM_SERVOS];
    // 当前伺服位置（角度）
    float servo_position[NUM_SERVOS];
    // 振荡器数组
    Oscillator_t oscillator[NUM_SERVOS];
    // 辅助变量（用于moveServos线性插值）
    float increment[NUM_SERVOS];
    uint32_t final_time;
    uint32_t init_time;
    uint32_t partial_time;
} MiniKame_t;

// 初始化
void MiniKame_Init(MiniKame_t *robot);
// 设置伺服角度
void MiniKame_SetServo(MiniKame_t *robot, int id, float target);
// 获取伺服角度
float MiniKame_GetServo(MiniKame_t *robot, int id);
// 反转伺服方向
void MiniKame_ReverseServo(MiniKame_t *robot, int id);
// 移动伺服到目标（线性插值）
void MiniKame_MoveServos(MiniKame_t *robot, int time_ms, float target[NUM_SERVOS]);
// 执行周期性动作（基于振荡器）
void MiniKame_Execute(MiniKame_t *robot, float steps, float period[NUM_SERVOS], int amplitude[NUM_SERVOS], int offset[NUM_SERVOS], int phase[NUM_SERVOS]);

// 预定义动作
void MiniKame_TurnR(MiniKame_t *robot, float steps, float T);
void MiniKame_TurnL(MiniKame_t *robot, float steps, float T);
void MiniKame_Dance(MiniKame_t *robot, float steps, float T);
void MiniKame_FrontBack(MiniKame_t *robot, float steps, float T);
void MiniKame_Run(MiniKame_t *robot, float steps, float T);
void MiniKame_Runback(MiniKame_t *robot, float steps, float T);
void MiniKame_Walk(MiniKame_t *robot, float steps, float T);
void MiniKame_Walkback(MiniKame_t *robot, float steps, float T);
void MiniKame_MoonwalkL(MiniKame_t *robot, float steps, float T);
void MiniKame_UpDown(MiniKame_t *robot, float steps, float T);
void MiniKame_PushUp(MiniKame_t *robot, float steps, float T);
void MiniKame_Hello(MiniKame_t *robot);
void MiniKame_Bye(MiniKame_t *robot);
void MiniKame_Afraid(MiniKame_t *robot);
void MiniKame_Confused(MiniKame_t *robot);
void MiniKame_ShakeYes(MiniKame_t *robot, float steps, float T);
void MiniKame_ShakeNo(MiniKame_t *robot);
void MiniKame_Headbang(MiniKame_t *robot, float steps, float T);
void MiniKame_LookLeft(MiniKame_t *robot);
void MiniKame_LookRight(MiniKame_t *robot);
void MiniKame_LookUp(MiniKame_t *robot);
void MiniKame_LookDown(MiniKame_t *robot);
void MiniKame_LookUpLeft(MiniKame_t *robot);
void MiniKame_LookUpRight(MiniKame_t *robot);
void MiniKame_LookDownLeft(MiniKame_t *robot);
void MiniKame_LookDownRight(MiniKame_t *robot);
void MiniKame_Center(MiniKame_t *robot);
void MiniKame_Rest(MiniKame_t *robot);
void MiniKame_Home(MiniKame_t *robot);
void MiniKame_Zero(MiniKame_t *robot);
void MiniKame_Calibrate(MiniKame_t *robot);
void MiniKame_Pie(MiniKame_t *robot);
void MiniKame_Imitate(MiniKame_t *robot, int s0, int s1, int s2, int s3, int s4, int s5, int s6, int s7, int s8);

#endif
