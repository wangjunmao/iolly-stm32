#ifndef SIN_TABLE_H
#define SIN_TABLE_H

#include <stdint.h>

// 获取sin值（角度×10，范围0~3600，对应0~360°）
// 返回值：sin值放大1000倍（范围-1000~1000）
int16_t SinTable_Get(int16_t angle_x10);

#endif
