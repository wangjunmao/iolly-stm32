#ifndef MINIKAME_H
#define MINIKAME_H

#include <stdint.h>
#include <stdbool.h>
#include "oscillator.h"

#define NUM_SERVOS 9

typedef struct {
    int16_t trim[NUM_SERVOS];          // 校准值（角度×10）
    bool reverse[NUM_SERVOS];          // 反转标志
    int16_t servo_position[NUM_SERVOS]; // 当前角度（×10）
    Oscillator_t oscillator[NUM_SERVOS];
    // 线性插值辅助变量
    int16_t increment[NUM_SERVOS];
    uint32_t final_time;
    uint32_t init_time;
} MiniKame_t;

// 初始化和基础控制
void MiniKame_Init(MiniKame_t *robot);
void MiniKame_SetServo(MiniKame_t *robot, int id, int16_t target_x10);
float MiniKame_GetServo(MiniKame_t *robot, int id);
void MiniKame_ReverseServo(MiniKame_t *robot, int id);
void MiniKame_MoveServos(MiniKame_t *robot, uint32_t time_ms, const int16_t target_x10[NUM_SERVOS]);
void MiniKame_Execute(MiniKame_t *robot, uint16_t steps, 
                      const uint16_t period[NUM_SERVOS], 
                      const int16_t amplitude[NUM_SERVOS], 
                      const int16_t offset[NUM_SERVOS], 
                      const uint16_t phase[NUM_SERVOS]);

// 基本动作
void MiniKame_TurnR(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_TurnL(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_Dance(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_FrontBack(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_Run(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_Runback(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_Walk(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_Walkback(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_MoonwalkL(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_UpDown(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_PushUp(MiniKame_t *robot, uint16_t steps, uint16_t T);

// 社交动作
void MiniKame_Hello(MiniKame_t *robot);
void MiniKame_Bye(MiniKame_t *robot);
void MiniKame_Afraid(MiniKame_t *robot);
void MiniKame_Confused(MiniKame_t *robot);
void MiniKame_ShakeYes(MiniKame_t *robot, uint16_t steps, uint16_t T);
void MiniKame_ShakeNo(MiniKame_t *robot);
void MiniKame_Headbang(MiniKame_t *robot, uint16_t steps, uint16_t T);

// 头部动作
void MiniKame_LookLeft(MiniKame_t *robot);
void MiniKame_LookRight(MiniKame_t *robot);
void MiniKame_LookUp(MiniKame_t *robot);
void MiniKame_LookDown(MiniKame_t *robot);
void MiniKame_LookUpLeft(MiniKame_t *robot);
void MiniKame_LookUpRight(MiniKame_t *robot);
void MiniKame_LookDownLeft(MiniKame_t *robot);
void MiniKame_LookDownRight(MiniKame_t *robot);
void MiniKame_Center(MiniKame_t *robot);

// 姿态控制
void MiniKame_Rest(MiniKame_t *robot);
void MiniKame_Home(MiniKame_t *robot);
void MiniKame_Zero(MiniKame_t *robot);
void MiniKame_Calibrate(MiniKame_t *robot);
void MiniKame_Pie(MiniKame_t *robot);
void MiniKame_Imitate(MiniKame_t *robot, int16_t s0, int16_t s1, int16_t s2, int16_t s3, 
                      int16_t s4, int16_t s5, int16_t s6, int16_t s7, int16_t s8);

#endif
