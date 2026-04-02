#include "oscillator.h"
#include "sin_table.h"

void Oscillator_Init(Oscillator_t *osc) {
    osc->period = 1000;
    osc->amplitude = 0;
    osc->phase = 0;
    osc->offset = 900;   // 90.0度
    osc->trim = 0;
    osc->start_time = 0;
    osc->current_angle = 900;
}

void Oscillator_SetPeriod(Oscillator_t *osc, uint16_t period) { 
    osc->period = period; 
}

void Oscillator_SetAmplitude(Oscillator_t *osc, int16_t amplitude) { 
    osc->amplitude = amplitude; 
}

void Oscillator_SetPhase(Oscillator_t *osc, uint16_t phase) { 
    osc->phase = phase; 
}

void Oscillator_SetOffset(Oscillator_t *osc, int16_t offset) { 
    osc->offset = offset; 
}

void Oscillator_SetTrim(Oscillator_t *osc, int16_t trim) { 
    osc->trim = trim; 
}

void Oscillator_SetStartTime(Oscillator_t *osc, uint32_t time_ms) { 
    osc->start_time = time_ms; 
}

int16_t Oscillator_Refresh(Oscillator_t *osc, uint32_t current_time_ms) {
    uint32_t dt = current_time_ms - osc->start_time;
    
    // 计算当前相位角（0~3600，对应0~360°，精度0.1°）
    uint32_t phase_angle = (osc->phase * 10) + (3600UL * dt) / osc->period;
    int16_t angle_out = value_x_sindeg(osc->amplitude, phase_angle);
    angle_out += osc->offset;
    
    // 限制范围 0~1800（0~180度）
    if (angle_out < 0) angle_out = 0;
    if (angle_out > 1800) angle_out = 1800;
    
    osc->current_angle = angle_out;
    return angle_out;
}
