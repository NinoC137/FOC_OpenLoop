#ifndef _FOC_H__
#define _FOC_H__

#include "math.h"
#include "main.h"

#include "MT6701.h"


#define PI 3.1415926
#define _3PI_2 4.71238898038f

#include "pid.h"
#include "lowpassFilter.h"

//传感器读取
float DFOC_M0_Velocity();
float DFOC_M0_Angle();
//PID
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);
float DFOC_M0_VEL_PID(float error);
float DFOC_M0_ANGLE_PID(float error);

//电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs);
float _electricalAngle_FeedBack();
//角度归一化
float _normalizeAngle(float angle);
//输出PWM
void setPWM(float Ua, float Ub, float Uc);
//设置相电压
void setPhaseVoltage(float Uq, float Ud, float angle_elec);

//开环速度接口函数
float velocityOpenLoop(float target_velocity);

//闭环部分
void setTorque(float Uq, float angle_el);
void FOC_Vbus(float _Vbus);
void FOC_alignSensor(int _PP, int _DIR);

//闭环控制接口函数
void FOC_M0_set_Velocity_Angle(float Target);
void FOC_M0_setVelocity(float Target);
void FOC_M0_set_Force_Angle(float Target);
void FOC_M0_setTorque(float Target);

#endif