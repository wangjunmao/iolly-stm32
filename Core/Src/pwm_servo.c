#include "pwm_servo.h"

// 舵机CCR寄存器地址映射表（直接存储寄存器地址指针）
volatile uint32_t* const servo_ccr_addr[SERVO_COUNT] = {
    &TIM3->CCR1, // 舵机0: PA6  TIM3_CH1
    &TIM3->CCR2, // 舵机1: PA7  TIM3_CH2
    &TIM3->CCR3, // 舵机2: PB0  TIM3_CH3
    &TIM3->CCR4, // 舵机3: PB1  TIM3_CH4
    &TIM2->CCR3, // 舵机4: PB10 TIM2_CH3
    &TIM2->CCR4, // 舵机5: PB11 TIM2_CH4
    &TIM4->CCR4, // 舵机6: PB9  TIM4_CH4
    &TIM4->CCR3, // 舵机7: PB8  TIM4_CH3
    &TIM4->CCR2  // 舵机8: PB7  TIM4_CH2
};

uint16_t AngToUsec(float angle) {
    return MIN_PULSE_WIDTH + (uint16_t)((angle / 180.0f) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH));
}

void PWM_Servo_SetPulse(int id, uint16_t pulse_us) {
    if (id < 0 || id >= SERVO_COUNT) return;
    if (pulse_us < MIN_PULSE_WIDTH) pulse_us = MIN_PULSE_WIDTH;
    if (pulse_us > MAX_PULSE_WIDTH) pulse_us = MAX_PULSE_WIDTH;
    // 直接写入CCR寄存器
    *servo_ccr_addr[id] = pulse_us;
}
