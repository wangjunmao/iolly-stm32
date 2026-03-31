#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include "stm32f1xx.h"
#include <stdint.h>

#define SERVO_COUNT 10  // 扩展至10个舵机（索引0~9，最后一个默认不动）

// 脉冲宽度范围（对应0°~180°）
#define MIN_PULSE_WIDTH 544   // 舵机最小脉冲宽度
#define MAX_PULSE_WIDTH 2400  // 舵机最大脉冲宽度

// 舵机CCR寄存器宏定义（根据最新引脚定义，索引0~9）
#define SERVO0  (&TIM4->CCR3)  // NO.0  TIM4_CHN3 PB8
#define SERVO1  (&TIM3->CCR2)  // NO.1  TIM3_CHN2 PA7
#define SERVO2  (&TIM4->CCR2)  // NO.2  TIM4_CHN2 PB7
#define SERVO3  (&TIM3->CCR3)  // NO.3  TIM3_CHN3 PB0
#define SERVO4  (&TIM4->CCR1)  // NO.4  TIM4_CHN1 PB6
#define SERVO5  (&TIM2->CCR4)  // NO.5  TIM2_CHN4 PB11
#define SERVO6  (&TIM4->CCR4)  // NO.6  TIM4_CHN4 PB9
#define SERVO7  (&TIM3->CCR1)  // NO.7  TIM3_CHN1 PA6
#define SERVO8  (&TIM2->CCR3)  // NO.8  TIM2_CHN3 PB10
#define SERVO9  (&TIM3->CCR4)  // NO.9  TIM3_CHN4 PB1 (备用)

// 函数声明
void PWM_Servo_Init(void);
uint16_t AngToUsec(uint16_t angle_x10);
void PWM_Servo_SetPulse(int id, uint16_t pulse_us);

#endif
