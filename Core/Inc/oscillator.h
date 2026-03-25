#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include <stdint.h>

typedef struct {
    float period;      // ms
    float amplitude;   // 度
    float phase;       // 度
    float offset;      // 度
    float trim;        // 校准值
    uint32_t last_time; // 上次计算时间（ms）
    float current_angle;
} Oscillator_t;

void Oscillator_Init(Oscillator_t *osc);
void Oscillator_SetPeriod(Oscillator_t *osc, float period);
void Oscillator_SetAmplitude(Oscillator_t *osc, float amplitude);
void Oscillator_SetPhase(Oscillator_t *osc, float phase);
void Oscillator_SetOffset(Oscillator_t *osc, float offset);
void Oscillator_SetTrim(Oscillator_t *osc, float trim);
void Oscillator_SetTime(Oscillator_t *osc, uint32_t time_ms);
float Oscillator_Refresh(Oscillator_t *osc, uint32_t current_time_ms);
void Oscillator_Reset(Oscillator_t *osc);

#endif
