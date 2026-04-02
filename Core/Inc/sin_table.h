#ifndef SIN_TABLE_H
#define SIN_TABLE_H

#include <stdint.h>

#define CIRCLE_DIV_1 8192u   // 8196对应360°
#define CIRCLE_DIV_1_MSK 13u // 8196的位宽是13位
#define CIRCLE_DIV_2 4096u   // 4096对应180°
#define CIRCLE_DIV_2_MSK 12u // 4096的位宽是12位
#define CIRCLE_DIV_4 2048u   // 2048对应90°
#define CIRCLE_DIV_4_MSK 11u // 2048的位宽是11位

#define ANGLE_MASK  (CIRCLE_DIV_1 - 1)   // 2^13 - 1
#define QUADRANT    CIRCLE_DIV_4   // 8192 / 4

// 获取sin值（角度×10，范围0~3600，对应0~360°）
// 返回值：sin值放大1000倍（范围-1000~1000）
int16_t sin_fast_q14_8192(uint16_t angle);
int16_t SinTable_Get(int16_t angle_x10);
int16_t value_x_sindeg(int16_t value, uint16_t angle_x10);

inline uint16_t angle_map_deg2bit(uint16_t angle)
{
    return (uint16_t)(((uint32_t)angle * 18641) >> CIRCLE_DIV_1_MSK);
};

inline uint16_t angle_map_bit2deg(uint16_t angle)
{
    return (uint16_t)(((uint32_t)angle * 3600) >> CIRCLE_DIV_1_MSK);
};

#endif
