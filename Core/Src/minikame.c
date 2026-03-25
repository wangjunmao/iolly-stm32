#include "minikame.h"
#include "pwm_servo.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h> // for rand()

// 初始化
void MiniKame_Init(MiniKame_t *robot) {
    // 引脚映射（与之前定义一致，但这里仅存储逻辑）
    robot->board_pins[0] = 0; // 索引对应D1等，实际在PWM中已固定
    // ... 省略，因为PWM已固定，此处不需要。

    // 校准值初始为0
    for (int i = 0; i < NUM_SERVOS; i++) {
        robot->trim[i] = 0.0f;
        robot->reverse[i] = false;
    }

    // 初始化振荡器
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_Init(&robot->oscillator[i]);
        Oscillator_SetTrim(&robot->oscillator[i], robot->trim[i]);
    }

    // 初始位置：所有舵机90度
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// 设置单个舵机
void MiniKame_SetServo(MiniKame_t *robot, int id, float target) {
    float angle = target;
    // 应用trim
    angle += robot->trim[id];
    if (robot->reverse[id]) {
        angle = 180.0f - angle;
    }
    // 限制角度范围0~180
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint16_t pulse = AngToUsec(angle);
    PWM_Servo_SetPulse(id, pulse);
    robot->servo_position[id] = target;
}

float MiniKame_GetServo(MiniKame_t *robot, int id) {
    return robot->servo_position[id];
}

void MiniKame_ReverseServo(MiniKame_t *robot, int id) {
    robot->reverse[id] = !robot->reverse[id];
}

// 线性插值移动
void MiniKame_MoveServos(MiniKame_t *robot, int time_ms, float target[NUM_SERVOS]) {
    if (time_ms <= 10) {
        // 立即移动
        for (int i = 0; i < NUM_SERVOS; i++) {
            MiniKame_SetServo(robot, i, target[i]);
        }
        return;
    }
    // 计算步进增量
    for (int i = 0; i < NUM_SERVOS; i++) {
        robot->increment[i] = (target[i] - robot->servo_position[i]) / (time_ms / 10.0f);
    }
    uint32_t start = xTaskGetTickCount(); // ms
    uint32_t end = start + time_ms;
    uint32_t now;
    do {
        now = xTaskGetTickCount();
        if (now >= end) break;
        // 每10ms更新一次
        for (int i = 0; i < NUM_SERVOS; i++) {
            float new_angle = robot->servo_position[i] + robot->increment[i] * 10.0f;
            MiniKame_SetServo(robot, i, new_angle);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (now < end);
    // 最终精确到目标位置
    for (int i = 0; i < NUM_SERVOS; i++) {
        MiniKame_SetServo(robot, i, target[i]);
    }
}

// 执行周期性动作
void MiniKame_Execute(MiniKame_t *robot, float steps, float period[NUM_SERVOS], int amplitude[NUM_SERVOS], int offset[NUM_SERVOS], int phase[NUM_SERVOS]) {
    // 设置所有振荡器参数
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetPeriod(&robot->oscillator[i], period[i]);
        Oscillator_SetAmplitude(&robot->oscillator[i], (float)amplitude[i]);
        Oscillator_SetPhase(&robot->oscillator[i], (float)phase[i]);
        Oscillator_SetOffset(&robot->oscillator[i], (float)offset[i]);
    }
    uint32_t global_time = xTaskGetTickCount();
    for (int i = 0; i < NUM_SERVOS; i++) {
        Oscillator_SetTime(&robot->oscillator[i], global_time);
    }
    uint32_t duration = (uint32_t)(period[0] * steps);
    uint32_t final_time = global_time + duration;
    uint32_t now;
    do {
        now = xTaskGetTickCount();
        for (int i = 0; i < NUM_SERVOS; i++) {
            float angle = Oscillator_Refresh(&robot->oscillator[i], now);
            MiniKame_SetServo(robot, i, angle);
        }
        taskYIELD(); // 让出CPU，相当于yield
    } while (now < final_time);
}
