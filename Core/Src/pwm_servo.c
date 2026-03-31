#include "pwm_servo.h"
#include "stm32f1xx_ll_tim.h"

// 静态舵机CCR寄存器地址映射表（使用宏定义）
static volatile uint32_t* const servo_ccr_addr[SERVO_COUNT] = {
    SERVO0, SERVO1, SERVO2, SERVO3, SERVO4,
    SERVO5, SERVO6, SERVO7, SERVO8, SERVO9
};

/**
 * @brief 初始化PWM输出（仅启用计数器和通道，时钟由CubeMX配置）
 */
void PWM_Servo_Init(void) {
    // 启用定时器计数器（若CubeMX已启动，此操作无效但安全）
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableCounter(TIM4);

    // 启用所有可能用到的PWM通道（根据引脚定义，使用到的通道）
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3); // SERVO0
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2); // SERVO1
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2); // SERVO2
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3); // SERVO3
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1); // SERVO4
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4); // SERVO5
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4); // SERVO6
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); // SERVO7
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3); // SERVO8
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); // SERVO9 (备用)
}

// 角度（×10）转脉冲宽度（微秒）
uint16_t AngToUsec(uint16_t angle_x10) {
    uint16_t angle = angle_x10 / 10;                     // 0~180
    return MIN_PULSE_WIDTH + (uint16_t)((uint32_t)(angle) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / 180);
}

// 设置指定舵机的脉冲宽度
void PWM_Servo_SetPulse(int id, uint16_t pulse_us) {
    if (id < 0 || id >= SERVO_COUNT) return;

    // 限制脉冲宽度
    if (pulse_us < MIN_PULSE_WIDTH) pulse_us = MIN_PULSE_WIDTH;
    if (pulse_us > MAX_PULSE_WIDTH) pulse_us = MAX_PULSE_WIDTH;

    // 写入CCR寄存器
    *servo_ccr_addr[id] = pulse_us;
}
