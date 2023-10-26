//
// Created by nino on 23-10-26.
//

#include "pid.h"
#include "cmsis_os.h"
#include "main.h"

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

PIDController* PIDController_create(float P, float I, float D, float ramp, float limit) {
    PIDController* controller = (PIDController*)malloc(sizeof(PIDController));
    if (controller == NULL) {
        return NULL;  // Error: Failed to allocate memory
    }

    controller->P = P;
    controller->I = I;
    controller->D = D;
    controller->output_ramp = ramp;
    controller->limit = limit;
    controller->error_prev = 0.0f;
    controller->output_prev = 0.0f;
    controller->integral_prev = 0.0f;
    controller->timestamp_prev = osKernelSysTick();

    return controller;
}

void PIDController_destroy(PIDController* controller) {
    free(controller);
}

float PIDController_process(PIDController* controller, float error) {
    unsigned long timestamp_now = osKernelSysTick();
    // 计算两次循环中间的间隔时间
    float Ts = (timestamp_now - controller->timestamp_prev) * 1e-3f;
    if (Ts <= 0 || Ts > 0.5f) {
        Ts = 1e-2f;
    }

    float proportional = controller->P * error;
    // Tustin 散点积分（I环）
    float integral = controller->integral_prev + controller->I * Ts * 0.5f * (error + controller->error_prev);
    integral = _constrain(integral, -controller->limit, controller->limit);
    float derivative = controller->D * (error - controller->error_prev) / Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -controller->limit, controller->limit);

    if (controller->output_ramp > 0) {
        // 对PID的变化速率进行限制
        float output_rate = (output - controller->output_prev) / Ts;
        if (output_rate > controller->output_ramp) {
            output = controller->output_prev + controller->output_ramp * Ts;
        } else if (output_rate < -controller->output_ramp) {
            output = controller->output_prev - controller->output_ramp * Ts;
        }
    }
    // 保存值（为了下一次循环）
    controller->integral_prev = integral;
    controller->output_prev = output;
    controller->error_prev = error;
    controller->timestamp_prev = timestamp_now;

    return output;
}