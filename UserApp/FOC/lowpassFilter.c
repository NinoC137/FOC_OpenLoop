//
// Created by nino on 23-10-26.
//

#include "lowpassFilter.h"
#include "cmsis_os.h"

LowPassFilter* LowPassFilter_create(float Tf) {
    LowPassFilter* filter = (LowPassFilter*)malloc(sizeof(LowPassFilter));
    if (filter == NULL) {
        return NULL;  // Error: Failed to allocate memory
    }

    filter->Tf = Tf;
    filter->y_prev = 0.0f;
    filter->timestamp_prev = osKernelSysTick();
    return filter;
}

void LowPassFilter_destroy(LowPassFilter* filter) {
    free(filter);
}

float LowPassFilter_process(LowPassFilter* filter, float x) {
    uint32_t timestamp = osKernelSysTick();
    float dt = (timestamp - filter->timestamp_prev) * 1e-3f;

    if (dt < 0.0f) {
        dt = 1e-2f;
    } else if (dt > 0.3f) {
        filter->y_prev = x;
        filter->timestamp_prev = timestamp;
        return x;
    }

    float alpha = filter->Tf / (filter->Tf + dt);
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    filter->y_prev = y;
    filter->timestamp_prev = timestamp;
    return y;
}
