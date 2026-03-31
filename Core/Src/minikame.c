#include "minikame.h"
#include "pwm_servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>

// ==================== 默认校准值（ROM） ====================
const int16_t default_trim[NUM_SERVOS] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

const bool default_reverse[NUM_SERVOS] = {
    false, false, false, false, false, false, false, false, false, false
};

// ==================== 初始化和基础控制 ====================

void MiniKame_Init(MiniKame_t *robot) {
    // 从常量数组复制校准值和反转标志
    memcpy(robot->trim, default_trim, sizeof(robot->trim));
    memcpy(robot->reverse, default_reverse, sizeof(robot->reverse));
    
    // 初始化振荡器
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_Init(&robot->oscillator[i]);
        Oscillator_SetTrim(&robot->oscillator[i], robot->trim[i]);
    }
    
    // 逐个设置舵机到初始位置（90°），间隔5ms，避免电流尖峰
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, 900); // 90.0度
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void MiniKame_SetServo(MiniKame_t *robot, int id, int16_t target_x10) {
    if (id < 0 || id >= NUM_SERVOS) return;
    
    int16_t angle = target_x10 + robot->trim[id];
    if (robot->reverse[id]) angle = 1800 - angle;
    if (angle < 0) angle = 0;
    if (angle > 1800) angle = 1800;
    
    uint16_t pulse = AngToUsec(angle);
    PWM_Servo_SetPulse(id, pulse);
    robot->servo_position[id] = target_x10;
}

float MiniKame_GetServo(MiniKame_t *robot, int id) {
    if (id < 0 || id >= NUM_SERVOS) return 0;
    return robot->servo_position[id] / 10.0f;
}

void MiniKame_ReverseServo(MiniKame_t *robot, int id) {
    if (id >= 0 && id < NUM_SERVOS) {
        robot->reverse[id] = !robot->reverse[id];
    }
}

// ==================== 线性插值移动 ====================

void MiniKame_MoveServos(MiniKame_t *robot, uint32_t time_ms, const int16_t target_x10[NUM_SERVOS]) {
    if (time_ms <= 10) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            MiniKame_SetServo(robot, i, target_x10[i]);
        }
        return;
    }
    
    // 计算每毫秒变化量（角度×10）
    for (int i = 0; i < NUM_SERVOS; i++) {
        int32_t diff = target_x10[i] - robot->servo_position[i];
        robot->increment[i] = (int16_t)((diff * 10) / (int32_t)time_ms);
    }
    
    uint32_t start = xTaskGetTickCount();
    uint32_t end = start + time_ms;
    uint32_t now;
    
    do {
        now = xTaskGetTickCount();
        if (now >= end) break;
        
        uint32_t elapsed = now - start;
        for (int i = 0; i < NUM_SERVOS; i++) {
            int16_t new_angle = robot->servo_position[i] + (int16_t)(robot->increment[i] * elapsed / 10);
            MiniKame_SetServo(robot, i, new_angle);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (now < end);
    
    // 最终精确到达目标
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, target_x10[i]);
    }
}

// ==================== 周期性动作执行 ====================

void MiniKame_Execute(MiniKame_t *robot, uint16_t steps, 
                      const uint16_t period[NUM_SERVOS], 
                      const int16_t amplitude[NUM_SERVOS], 
                      const int16_t offset[NUM_SERVOS], 
                      const uint16_t phase[NUM_SERVOS]) {
    uint32_t start_time = xTaskGetTickCount();
    
    // 设置所有振荡器参数
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetPeriod(&robot->oscillator[i], period[i]);
        Oscillator_SetAmplitude(&robot->oscillator[i], amplitude[i]);
        Oscillator_SetPhase(&robot->oscillator[i], phase[i]);
        Oscillator_SetOffset(&robot->oscillator[i], offset[i]);
        Oscillator_SetStartTime(&robot->oscillator[i], start_time);
    }
    
    uint32_t duration = period[0] * steps;
    uint32_t end_time = start_time + duration;
    uint32_t now;
    
    do {
        now = xTaskGetTickCount();
        for (int i = 0; i < NUM_SERVOS; i++) {
            int16_t angle = Oscillator_Refresh(&robot->oscillator[i], now);
            MiniKame_SetServo(robot, i, angle);
        }
        taskYIELD();
    } while (now < end_time);
}

// ==================== 基本动作（整数化，数组扩展至10） ====================

void MiniKame_TurnR(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 150;
    int16_t z_amp = 150;
    int16_t ap = -150;
    int16_t hi = 230;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 450, 900 };
    uint16_t phase[] = { 0, 180, 90, 90, 180, 0, 90, 90, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_TurnL(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 150;
    int16_t z_amp = 150;
    int16_t ap = -150;
    int16_t hi = 230;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 1350, 900 };
    uint16_t phase[] = { 180, 0, 90, 90, 0, 180, 90, 90, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_Dance(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 0;
    int16_t z_amp = 400;
    int16_t ap = -300;
    int16_t hi = 200;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 90, 90, 0, 270, 270, 270, 90, 180, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_FrontBack(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 300;
    int16_t z_amp = 250;
    int16_t ap = -200;
    int16_t hi = 300;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 0, 180, 270, 90, 0, 180, 90, 270, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_Walk(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 200;
    int16_t z_amp = 350;
    int16_t ap = 200;
    int16_t hi = 150;
    int16_t front_x = 120;
    uint16_t period[] = { T, T, T/2, T/2, T, T, T/2, T/2, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 - ap - front_x, 900 + ap + front_x, 900 - hi + 100, 900 + hi - 100,
                         900 + ap - front_x, 900 - ap + front_x, 900 + hi - 50, 900 - hi + 50, 0, 900 };
    uint16_t phase[] = { 90, 90, 270, 90, 270, 270, 90, 270, 0, 0 };
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetPeriod(&robot->oscillator[i], period[i]);
        Oscillator_SetAmplitude(&robot->oscillator[i], amplitude[i]);
        Oscillator_SetPhase(&robot->oscillator[i], phase[i]);
        Oscillator_SetOffset(&robot->oscillator[i], offset[i]);
    }
    
    uint32_t start_time = xTaskGetTickCount();
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetStartTime(&robot->oscillator[i], start_time);
    }
    
    uint32_t final_time = start_time + period[0] * steps;
    uint32_t init_time = start_time;
    uint32_t now;
    bool side;
    
    while (1) {
        now = xTaskGetTickCount();
        if (now >= final_time) break;
        
        side = ((now - init_time) / (period[0] / 2)) % 2;
        
        MiniKame_SetServo(robot, 0, Oscillator_Refresh(&robot->oscillator[0], now));
        MiniKame_SetServo(robot, 1, Oscillator_Refresh(&robot->oscillator[1], now));
        MiniKame_SetServo(robot, 4, Oscillator_Refresh(&robot->oscillator[4], now));
        MiniKame_SetServo(robot, 5, Oscillator_Refresh(&robot->oscillator[5], now));
        
        if (side == 0) {
            MiniKame_SetServo(robot, 3, Oscillator_Refresh(&robot->oscillator[3], now));
            MiniKame_SetServo(robot, 6, Oscillator_Refresh(&robot->oscillator[6], now));
        } else {
            MiniKame_SetServo(robot, 2, Oscillator_Refresh(&robot->oscillator[2], now));
            MiniKame_SetServo(robot, 7, Oscillator_Refresh(&robot->oscillator[7], now));
        }
        
        // 第8、9个舵机（头部和备用）刷新
        MiniKame_SetServo(robot, 8, Oscillator_Refresh(&robot->oscillator[8], now));
        MiniKame_SetServo(robot, 9, Oscillator_Refresh(&robot->oscillator[9], now));
        
        taskYIELD();
    }
}

void MiniKame_Walkback(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = -200;
    int16_t z_amp = 350;
    int16_t ap = -200;
    int16_t hi = 150;
    int16_t back_x = 120;
    uint16_t period[] = { T, T, T/2, T/2, T, T, T/2, T/2, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap + back_x, 900 - ap - back_x, 900 - hi, 900 + hi,
                         900 - ap + back_x, 900 + ap - back_x, 900 + hi, 900 - hi, 0, 900 };
    uint16_t phase[] = { 90, 90, 270, 90, 270, 270, 90, 270, 0, 0 };
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetPeriod(&robot->oscillator[i], period[i]);
        Oscillator_SetAmplitude(&robot->oscillator[i], amplitude[i]);
        Oscillator_SetPhase(&robot->oscillator[i], phase[i]);
        Oscillator_SetOffset(&robot->oscillator[i], offset[i]);
    }
    
    uint32_t start_time = xTaskGetTickCount();
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetStartTime(&robot->oscillator[i], start_time);
    }
    
    uint32_t final_time = start_time + period[0] * steps;
    uint32_t init_time = start_time;
    uint32_t now;
    bool side;
    
    while (1) {
        now = xTaskGetTickCount();
        if (now >= final_time) break;
        
        side = ((now - init_time) / (period[0] / 2)) % 2;
        
        MiniKame_SetServo(robot, 1, Oscillator_Refresh(&robot->oscillator[1], now));
        MiniKame_SetServo(robot, 0, Oscillator_Refresh(&robot->oscillator[0], now));
        MiniKame_SetServo(robot, 5, Oscillator_Refresh(&robot->oscillator[5], now));
        MiniKame_SetServo(robot, 4, Oscillator_Refresh(&robot->oscillator[4], now));
        
        if (side == 0) {
            MiniKame_SetServo(robot, 3, Oscillator_Refresh(&robot->oscillator[3], now));
            MiniKame_SetServo(robot, 6, Oscillator_Refresh(&robot->oscillator[6], now));
        } else {
            MiniKame_SetServo(robot, 2, Oscillator_Refresh(&robot->oscillator[2], now));
            MiniKame_SetServo(robot, 7, Oscillator_Refresh(&robot->oscillator[7], now));
        }
        
        MiniKame_SetServo(robot, 8, Oscillator_Refresh(&robot->oscillator[8], now));
        MiniKame_SetServo(robot, 9, Oscillator_Refresh(&robot->oscillator[9], now));
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void MiniKame_Run(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t x_amp = 200;
    int16_t z_amp = 450;
    int16_t ap = 200;
    int16_t hi = -150;
    int16_t front_x = 120;
    uint16_t period[] = { T, T, T/2, T/2, T, T, T/2, T/2, T, T };
    int16_t amplitude[] = { x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 - ap - front_x, 900 + ap + front_x, 900 - hi + 100, 900 + hi - 100,
                         900 + ap - front_x, 900 - ap + front_x, 900 + hi - 50, 900 - hi + 50, 0, 900 };
    uint16_t phase[] = { 90, 90, 270, 90, 270, 270, 90, 270, 0, 0 };
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetPeriod(&robot->oscillator[i], period[i]);
        Oscillator_SetAmplitude(&robot->oscillator[i], amplitude[i]);
        Oscillator_SetPhase(&robot->oscillator[i], phase[i]);
        Oscillator_SetOffset(&robot->oscillator[i], offset[i]);
    }
    
    uint32_t start_time = xTaskGetTickCount();
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetStartTime(&robot->oscillator[i], start_time);
    }
    
    uint32_t final_time = start_time + period[0] * steps;
    uint32_t init_time = start_time;
    uint32_t now;
    bool side;
    
    while (1) {
        now = xTaskGetTickCount();
        if (now >= final_time) break;
        
        side = ((now - init_time) / (period[0] / 2)) % 2;
        
        MiniKame_SetServo(robot, 0, Oscillator_Refresh(&robot->oscillator[0], now));
        MiniKame_SetServo(robot, 1, Oscillator_Refresh(&robot->oscillator[1], now));
        MiniKame_SetServo(robot, 4, Oscillator_Refresh(&robot->oscillator[4], now));
        MiniKame_SetServo(robot, 5, Oscillator_Refresh(&robot->oscillator[5], now));
        
        if (side == 0) {
            MiniKame_SetServo(robot, 3, Oscillator_Refresh(&robot->oscillator[3], now));
            MiniKame_SetServo(robot, 6, Oscillator_Refresh(&robot->oscillator[6], now));
        } else {
            MiniKame_SetServo(robot, 2, Oscillator_Refresh(&robot->oscillator[2], now));
            MiniKame_SetServo(robot, 7, Oscillator_Refresh(&robot->oscillator[7], now));
        }
        
        MiniKame_SetServo(robot, 8, Oscillator_Refresh(&robot->oscillator[8], now));
        MiniKame_SetServo(robot, 9, Oscillator_Refresh(&robot->oscillator[9], now));
        
        taskYIELD();
    }
}

void MiniKame_Runback(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    MiniKame_Walkback(robot, steps, T);
}

void MiniKame_MoonwalkL(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t z_amp = 450;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900, 900, 900, 900, 900, 900, 900, 900, 900, 900 };
    uint16_t phase[] = { 0, 0, 0, 120, 0, 0, 180, 290, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_UpDown(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t z_amp = 1200;
    int16_t hi = -300;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900, 900, 900 - hi, 900 + hi, 900, 900, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 90, 270, 270, 90, 270, 90, 90, 270, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_PushUp(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t z_amp = 400;
    int16_t x_amp = 650;
    int16_t hi = 300;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, 0, 0, 0, 0 };
    int16_t offset[] = { 900, 900, 900 - hi, 900 + hi, 900 - x_amp, 900 + x_amp, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 0, 0, 0, 180, 0, 0, 0, 0, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

// ==================== 社交动作（数组扩展至10） ====================

void MiniKame_Hello(MiniKame_t *robot) {
    uint16_t T = 800;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 400, 0, 400, 0, 0, 0, 0, 0, 0 };
    int16_t offset[] = { 900 + 150, 400, 900 - 650, 900, 900 + 100, 900 - 100, 900 + 200, 900 - 200, 900, 900 };
    uint16_t phase[] = { 0, 0, 0, 90, 0, 0, 0, 0, 0, 0 };
    MiniKame_Execute(robot, 4, period, amplitude, offset, phase);
}

void MiniKame_Bye(MiniKame_t *robot) {
    uint16_t T = 800;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 400, 400, 900, 900, 0, 0, 0, 20, 0, 0 };
    int16_t offset[] = { 900 + 500, 900 - 500, 900 - 150, 900 + 150, 900 + 200, 900 - 200, 900 + 100, 900 - 100, 900, 900 };
    uint16_t phase[] = { 0, 0, 90, 90, 0, 0, 0, 0, 0, 0 };
    MiniKame_Execute(robot, 4, period, amplitude, offset, phase);
}

void MiniKame_Afraid(MiniKame_t *robot) {
    int16_t afraidPositionLeft[] = { 900 + 150, 900 - 150, 900 - 650, 900 + 650, 900 + 200, 900 - 200, 900 + 100, 900 - 100, 900 - 50, 900 };
    MiniKame_MoveServos(robot, 150, afraidPositionLeft);
    int16_t afraidPositionRight[] = { 900 + 150, 900 - 150, 900 - 650, 900 + 650, 900 + 200, 900 - 200, 900 + 100, 900 - 100, 900 + 50, 900 };
    MiniKame_MoveServos(robot, 150, afraidPositionRight);
}

void MiniKame_Confused(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t up = 300 + (rand() % 150);
    int16_t down = 50 + (rand() % 150);
    int16_t confusedPositionA[] = { 900 + ap, 900 - ap, 900 - hi + up, 900 + hi + down, 900 - ap, 900 + ap, 900 + hi - up, 900 - hi - down, 900 + 50, 900 };
    MiniKame_MoveServos(robot, rand() % 500 + 200, confusedPositionA);
    vTaskDelay(pdMS_TO_TICKS(rand() % 600 + 800));
    int16_t confusedPositionB[] = { 900 + ap, 900 - ap, 900 - hi - down, 900 + hi - up, 900 - ap, 900 + ap, 900 + hi + down, 900 - hi + up, 900 - 50, 900 };
    MiniKame_MoveServos(robot, rand() % 500 + 200, confusedPositionB);
    vTaskDelay(pdMS_TO_TICKS(rand() % 600 + 800));
}

void MiniKame_ShakeYes(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t z_amp = 300;
    int16_t hi = 300;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, 0, 0, 0, 0 };
    int16_t offset[] = { 900, 900, 900 - hi, 900 + hi, 900, 900, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

void MiniKame_ShakeNo(MiniKame_t *robot) {
    int16_t ap = 200;
    int16_t hi = 350;
    int16_t shakeNoPositionLeft[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900 - 450, 900 };
    MiniKame_MoveServos(robot, 100, shakeNoPositionLeft);
    int16_t shakeNoPositionRight[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900 + 450, 900 };
    MiniKame_MoveServos(robot, 100, shakeNoPositionRight);
}

void MiniKame_Headbang(MiniKame_t *robot, uint16_t steps, uint16_t T) {
    int16_t z_amp = 300;
    int16_t ap = -100;
    int16_t hi = 300;
    uint16_t period[] = { T, T, T, T, T, T, T, T, T, T };
    int16_t amplitude[] = { 0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp, 0, 0 };
    int16_t offset[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900, 900 };
    uint16_t phase[] = { 0, 0, 0, 180, 0, 0, 0, 180, 0, 0 };
    MiniKame_Execute(robot, steps, period, amplitude, offset, phase);
}

// ==================== 头部动作（数组扩展至10） ====================

void MiniKame_LookLeft(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookLeftPosition[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900 + 450, 900 };
    MiniKame_MoveServos(robot, 150, lookLeftPosition);
}

void MiniKame_LookRight(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookRightPosition[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900 - 450, 900 };
    MiniKame_MoveServos(robot, 150, lookRightPosition);
}

void MiniKame_LookUp(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookUpPosition[] = { 900 + ap, 900 - ap, 900 - hi - 350, 900 + hi + 350, 900 - ap, 900 + ap, 900 + hi - 350, 900 - hi + 350, 900, 900 };
    MiniKame_MoveServos(robot, 150, lookUpPosition);
}

void MiniKame_LookDown(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookDownPosition[] = { 900 + ap, 900 - ap, 900 - hi + 350, 900 + hi - 350, 900 - ap, 900 + ap, 900 + hi + 350, 900 - hi - 350, 900, 900 };
    MiniKame_MoveServos(robot, 150, lookDownPosition);
}

void MiniKame_LookUpLeft(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookUpLeftPosition[] = { 900 + ap, 900 - ap, 900 - hi - 350, 900 + hi + 350, 900 - ap, 900 + ap, 900 + hi - 350, 900 - hi + 350, 900 - 450, 900 };
    MiniKame_MoveServos(robot, 150, lookUpLeftPosition);
}

void MiniKame_LookUpRight(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookUpRightPosition[] = { 900 + ap, 900 - ap, 900 - hi - 350, 900 + hi + 350, 900 - ap, 900 + ap, 900 + hi - 350, 900 - hi + 350, 900 + 450, 900 };
    MiniKame_MoveServos(robot, 150, lookUpRightPosition);
}

void MiniKame_LookDownLeft(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookDownLeftPosition[] = { 900 + ap, 900 - ap, 900 - hi + 350, 900 + hi - 350, 900 - ap, 900 + ap, 900 + hi + 350, 900 - hi - 350, 900 - 450, 900 };
    MiniKame_MoveServos(robot, 150, lookDownLeftPosition);
}

void MiniKame_LookDownRight(MiniKame_t *robot) {
    int16_t ap = -200;
    int16_t hi = 350;
    int16_t lookDownRightPosition[] = { 900 + ap, 900 - ap, 900 - hi + 350, 900 + hi - 350, 900 - ap, 900 + ap, 900 + hi + 350, 900 - hi - 350, 900 + 450, 900 };
    MiniKame_MoveServos(robot, 150, lookDownRightPosition);
}

void MiniKame_Center(MiniKame_t *robot) {
    int16_t ap = 200;
    int16_t hi = 350;
    int16_t centerPosition[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900, 900 };
    MiniKame_MoveServos(robot, 150, centerPosition);
}

// ==================== 姿态控制（数组扩展至10） ====================

void MiniKame_Rest(MiniKame_t *robot) {
    int16_t ap = 200;
    int16_t hi = -150;
    int16_t restPosition[] = { 900 + ap, 900 - ap, 900 - hi, 900 + hi, 900 - ap, 900 + ap, 900 + hi, 900 - hi, 900, 900 };
    MiniKame_MoveServos(robot, 150, restPosition);
}

void MiniKame_Home(MiniKame_t *robot) {
    int16_t position[] = { 900, 900, 900, 900, 900, 900, 900, 900, 900, 900 };
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, position[i]);
    }
}

void MiniKame_Zero(MiniKame_t *robot) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, 900);
    }
}

void MiniKame_Calibrate(MiniKame_t *robot) {
    int16_t hi = -150;
    int16_t position[] = { 900, 900, 900 - hi, 900 + hi, 900, 900, 900 + hi, 900 - hi, 900, 900 };
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, position[i]);
    }
}

void MiniKame_Pie(MiniKame_t *robot) {
    int16_t ap = 200;
    int16_t hi = 350;
    int16_t piePositionA[] = { 900 + ap, 900 - ap + 150, 900 - hi + 600, 900 + hi + 300, 900 - ap - 450, 900 + ap - 300, 900 + hi + 100, 900 - hi + 900, 900 - 50, 900 };
    MiniKame_MoveServos(robot, 150, piePositionA);
    int16_t piePositionB[] = { 900 + ap, 900 - ap + 150, 900 - hi + 600, 900 + hi + 300, 900 - ap - 450, 900 + ap - 150, 900 + hi + 100, 900 - hi + 900, 900 - 50, 900 };
    MiniKame_MoveServos(robot, 150, piePositionB);
}

void MiniKame_Imitate(MiniKame_t *robot, int16_t s0, int16_t s1, int16_t s2, int16_t s3, 
                      int16_t s4, int16_t s5, int16_t s6, int16_t s7, int16_t s8) {
    int16_t position[] = { s0, s1, s2, s3, s4, s5, s6, s7, s8, 900 };
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, position[i]);
    }
}
