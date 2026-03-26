#ifndef SIN_TABLE_H
#define SIN_TABLE_H

#include <stdint.h>

#define CIRCLE_DIV_1 4096u   // 4096对应360°
#define CIRCLE_DIV_1_MSK 12u // 4096的位宽是12位
#define CIRCLE_DIV_2 2048u   // 2048对应180°
#define CIRCLE_DIV_2_MSK 11u // 2048的位宽是11位
#define CIRCLE_DIV_4 1024u   // 1024对应90°
#define CIRCLE_DIV_4_MSK 10u // 1024的位宽是10位

// 获取sin值（角度×10，范围0~3600，对应0~360°）
// 返回值：sin值放大1000倍（范围-1000~1000）
int16_t SinTable_Get(int16_t angle_x10);

#endif
