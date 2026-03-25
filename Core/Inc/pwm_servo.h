#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include "stm32f1xx.h"

#define SERVO_COUNT 9
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2400

extern volatile uint32_t* const servo_ccr_addr[SERVO_COUNT];

void PWM_Servo_SetPulse(int id, uint16_t pulse_us);
uint16_t AngToUsec(float angle);

#endif
