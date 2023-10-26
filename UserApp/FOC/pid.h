//
// Created by nino on 23-10-26.
//

#ifndef FOC_OPENLOOP_PID_H
#define FOC_OPENLOOP_PID_H

typedef struct {
    float P;          // 比例增益(P环增益)
    float I;          // 积分增益（I环增益）
    float D;          // 微分增益（D环增益）
    float output_ramp; // PID控制器加速度限幅
    float limit;      // PID控制器输出限幅
    float error_prev; // 最后的跟踪误差值
    float output_prev;  // 最后一个 pid 输出值
    float integral_prev; // 最后一个积分分量值
    unsigned long timestamp_prev; // 上次执行时间戳
} PIDController;

PIDController* PIDController_create(float P, float I, float D, float ramp, float limit);
void PIDController_destroy(PIDController* controller);
float PIDController_process(PIDController* controller, float error);

#endif //FOC_OPENLOOP_PID_H
