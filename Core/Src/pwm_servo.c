#include "pwm_servo.h"

// 静态舵机CCR寄存器地址映射表（仅在文件内使用）
static volatile uint32_t* const servo_ccr_addr[SERVO_COUNT] = {
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

// 静态舵机配置数组（仅在文件内使用）
static ServoConfig_t servo_config[SERVO_COUNT] = {
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机0
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机1
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机2
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机3
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机4
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机5
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机6
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0},   // 舵机7
    {MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0}    // 舵机8
};

// 初始化（CubeMX已完成，此函数可空或用于额外设置）
void PWM_Servo_Init(void) {
    // CubeMX已初始化定时器和PWM输出，此函数保留以备需要
}

// 设置舵机配置
void PWM_Servo_SetConfig(int id, uint16_t min_pulse, uint16_t max_pulse, int16_t offset) {
    if (id < 0 || id >= SERVO_COUNT) return;
    
    // 验证参数有效性
    if (min_pulse < MIN_PULSE_WIDTH) min_pulse = MIN_PULSE_WIDTH;
    if (max_pulse > MAX_PULSE_WIDTH) max_pulse = MAX_PULSE_WIDTH;
    if (min_pulse >= max_pulse) {
        min_pulse = MIN_PULSE_WIDTH;
        max_pulse = MAX_PULSE_WIDTH;
    }
    
    servo_config[id].min_pulse = min_pulse;
    servo_config[id].max_pulse = max_pulse;
    servo_config[id].offset = offset;
}

// 获取舵机配置
void PWM_Servo_GetConfig(int id, ServoConfig_t *config) {
    if (id < 0 || id >= SERVO_COUNT) return;
    config->min_pulse = servo_config[id].min_pulse;
    config->max_pulse = servo_config[id].max_pulse;
    config->offset = servo_config[id].offset;
}

// 批量配置所有舵机
void PWM_Servo_ConfigAll(uint16_t min_pulse, uint16_t max_pulse, int16_t offset) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        PWM_Servo_SetConfig(i, min_pulse, max_pulse, offset);
    }
}

// 角度转换（0~180度，脉冲500~2500）
uint16_t AngToUsec(uint16_t angle_x10) {
    uint16_t angle = angle_x10 / 10;
    return MIN_PULSE_WIDTH + (uint16_t)((uint32_t)(angle) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180);
}

// 设置舵机脉冲宽度（自动应用限制和偏移）
void PWM_Servo_SetPulse(int id, uint16_t pulse_us) {
    if (id < 0 || id >= SERVO_COUNT) return;
    
    // 应用脉冲宽度限制
    if (pulse_us < servo_config[id].min_pulse) {
        pulse_us = servo_config[id].min_pulse;
    }
    if (pulse_us > servo_config[id].max_pulse) {
        pulse_us = servo_config[id].max_pulse;
    }
    
    // 应用偏移
    int32_t final_pulse = (int32_t)pulse_us + servo_config[id].offset;
    
    // 最终限制（防止偏移后超出硬件范围）
    if (final_pulse < MIN_PULSE_WIDTH) final_pulse = MIN_PULSE_WIDTH;
    if (final_pulse > MAX_PULSE_WIDTH) final_pulse = MAX_PULSE_WIDTH;
    
    // 写入CCR寄存器
    *servo_ccr_addr[id] = (uint32_t)final_pulse;
}
