#ifndef OSCILLATOR_H
#define OSCILLATOR_H

#include <stdint.h>

typedef struct {
    uint16_t period;       // 周期（毫秒）
    int16_t amplitude;     // 振幅（角度 × 10）
    uint16_t phase;        // 相位（0~359度）
    int16_t offset;        // 偏移（角度 × 10）
    int16_t trim;          // 校准值（角度 × 10）
    uint32_t start_time;   // 起始时间（毫秒）
    int16_t current_angle; // 当前角度（角度 × 10）
} Oscillator_t;

void Oscillator_Init(Oscillator_t *osc);
void Oscillator_SetPeriod(Oscillator_t *osc, uint16_t period);
void Oscillator_SetAmplitude(Oscillator_t *osc, int16_t amplitude);
void Oscillator_SetPhase(Oscillator_t *osc, uint16_t phase);
void Oscillator_SetOffset(Oscillator_t *osc, int16_t offset);
void Oscillator_SetTrim(Oscillator_t *osc, int16_t trim);
void Oscillator_SetStartTime(Oscillator_t *osc, uint32_t time_ms);
int16_t Oscillator_Refresh(Oscillator_t *osc, uint32_t current_time_ms);

#endif
