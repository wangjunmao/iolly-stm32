#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include "stm32f1xx.h"
#include <stdint.h>

#define SERVO_COUNT 9
#define MIN_PULSE_WIDTH 500   // 舵机最小脉冲宽度（对应0°）
#define MAX_PULSE_WIDTH 2500  // 舵机最大脉冲宽度（对应180°）

// 舵机配置结构体
typedef struct {
    uint16_t min_pulse;   // 最小脉冲宽度（微秒）
    uint16_t max_pulse;   // 最大脉冲宽度（微秒）
    int16_t offset;       // 脉冲宽度偏移（微秒）
} ServoConfig_t;

// 函数声明
void PWM_Servo_Init(void);
void PWM_Servo_SetPulse(int id, uint16_t pulse_us);
uint16_t AngToUsec(uint16_t angle_x10);
void PWM_Servo_SetConfig(int id, uint16_t min_pulse, uint16_t max_pulse, int16_t offset);
void PWM_Servo_GetConfig(int id, ServoConfig_t *config);
void PWM_Servo_ConfigAll(uint16_t min_pulse, uint16_t max_pulse, int16_t offset);

#endif
