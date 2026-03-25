#include "oscillator.h"
#include <math.h>

#define PI 3.14159265358979323846f

void Oscillator_Init(Oscillator_t *osc) {
    osc->period = 1000.0f;
    osc->amplitude = 0.0f;
    osc->phase = 0.0f;
    osc->offset = 90.0f;
    osc->trim = 0.0f;
    osc->last_time = 0;
    osc->current_angle = 90.0f;
}

void Oscillator_SetPeriod(Oscillator_t *osc, float period) { osc->period = period; }
void Oscillator_SetAmplitude(Oscillator_t *osc, float amplitude) { osc->amplitude = amplitude; }
void Oscillator_SetPhase(Oscillator_t *osc, float phase) { osc->phase = phase; }
void Oscillator_SetOffset(Oscillator_t *osc, float offset) { osc->offset = offset; }
void Oscillator_SetTrim(Oscillator_t *osc, float trim) { osc->trim = trim; }
void Oscillator_SetTime(Oscillator_t *osc, uint32_t time_ms) { osc->last_time = time_ms; }
void Oscillator_Reset(Oscillator_t *osc) { /* 重置状态，可留空 */ }

float Oscillator_Refresh(Oscillator_t *osc, uint32_t current_time_ms) {
    // 计算角度：angle = offset + amplitude * sin(2*pi*t/period + phase)
    float t = (float)(current_time_ms - osc->last_time);
    float rad = 2.0f * PI * t / osc->period + osc->phase * PI / 180.0f;
    float angle = osc->offset + osc->amplitude * sinf(rad);
    osc->current_angle = angle;
    return angle;
}
