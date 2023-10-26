//
// Created by nino on 23-10-26.
//

#ifndef FOC_OPENLOOP_LOWPASSFILTER_H
#define FOC_OPENLOOP_LOWPASSFILTER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
  * 低通滤波器类
  */

typedef struct {
    float Tf;                  // 低通滤波时间常数
    uint32_t timestamp_prev;    // 最后执行时间戳
    float y_prev;               // 上一个循环中的过滤后的值
} LowPassFilter;

LowPassFilter* LowPassFilter_create(float Tf);
void LowPassFilter_destroy(LowPassFilter* filter);
float LowPassFilter_process(LowPassFilter* filter, float x);

#ifdef __cplusplus
}
#endif
#endif //FOC_OPENLOOP_LOWPASSFILTER_H
