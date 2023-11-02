#include "FOC.h"
#include "cmsis_os.h"

#define _constrain(amt, low, high) ((amt)<(low)?:(amt)>(high)?(high):(amt))

float shaft_angle = 0.0f, open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;
float voltage_power_supply = 12.0f;
float Ualpha = 0.0f, Ubeta = 0.0f;
float Ua, Ub, Uc, dc_a, dc_b, dc_c;

float PP, DIR = 1;

//传感器数值
int16_t angle;
float angle_f;

//低通滤波初始化
LowPassFilter* M0_Vel_Flt; // Tf = 10ms   //M0速度环

//PID
PIDController* vel_loop_M0;
PIDController* angle_loop_M0;

//===================================PID 设置函数=========================================
//速度PID
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp)   //M0角度环PID设置
{
    vel_loop_M0->P = P;
    vel_loop_M0->I = I;
    vel_loop_M0->D = D;
    vel_loop_M0->output_ramp = ramp;
}

//角度PID
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp)   //M0角度环PID设置
{
    angle_loop_M0->P = P;
    angle_loop_M0->I = I;
    angle_loop_M0->D = D;
    angle_loop_M0->output_ramp = ramp;
}

//M0速度PID接口
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
    return PIDController_process(vel_loop_M0, error);
}

//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error) {
    return PIDController_process(angle_loop_M0, error);
}
//=====================================================================================

void FOC_Vbus(float _Vbus) {
    voltage_power_supply = _Vbus;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    //PID 加载
    vel_loop_M0 = PIDController_create(2, 0, 0, 100000, voltage_power_supply / 2);
    //低通滤波初始化
    M0_Vel_Flt = LowPassFilter_create(0.01); // Tf = 10ms   //M0速度环
    //PID
    vel_loop_M0 = PIDController_create(2, 0, 0, 100000, voltage_power_supply / 2);
    angle_loop_M0 = PIDController_create(2, 0, 0, 100000, 100);
}

void FOC_alignSensor(int _PP, int _DIR) {
    PP = _PP;
    DIR = _DIR;

    setTorque(3, _3PI_2);  //起劲
    HAL_Delay(1000);
    i2c_mt6701_get_angle(&angle, &angle_f); //更新传感器数值
    zero_electric_angle = _electricalAngle_FeedBack();
    setTorque(0, _3PI_2);  //松劲（解除校准）

    uart_printf("0电角度：%f\r\n", zero_electric_angle);
}

float DFOC_M0_Velocity() {
    static float angle_now, angle_old;
    static long sampleTimeStamp;
    i2c_mt6701_get_angle(&angle, &angle_f); //更新传感器数值
    sampleTimeStamp = osKernelSysTick();
    angle_now = angle_f;

    float vel_speed_ori = (angle_now - angle_old) / sampleTimeStamp * 1e3;  //采样时间以ms为单位, 乘1e3后为每秒的角速度

    angle_old = angle_now;

    float vel_M0_flit = LowPassFilter_process(M0_Vel_Flt, DIR * vel_speed_ori);

    return vel_M0_flit;
}

float DFOC_M0_Angle() {
    i2c_mt6701_get_angle(&angle, &angle_f);
    return DIR * angle_f;
}

//电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
    return (shaft_angle * pole_pairs);
}

float _electricalAngle_FeedBack() {
    i2c_mt6701_get_angle(&angle, &angle_f); //更新传感器数值
    return _normalizeAngle((float) (DIR * PP) * angle_f - zero_electric_angle);
}

//角度归一化
float _normalizeAngle(float angle) {
    float a = (float) fmod(angle, 2 * PI);
    return a > 0 ? a : (float) (a + 2 * PI);
}

//输出PWM
void setPWM(float Ua, float Ub, float Uc) {
    //计算占空比, 并限制其在0~1
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    //写入PWM通道
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, dc_a * htim2.Init.Period);
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, dc_b * htim3.Init.Period);
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, dc_c * htim4.Init.Period);
}

void setTorque(float Uq, float angle_el) {
    i2c_mt6701_get_angle(&angle, &angle_f); //更新传感器数值

    Uq = _constrain(Uq, -(voltage_power_supply) / 2, (voltage_power_supply) / 2);

    angle_el = _normalizeAngle(angle_el);

    // 帕克逆变换
    Ualpha = -Uq * sin(angle_el);
    Ubeta = Uq * cos(angle_el);

    //克拉克逆变换
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (float) (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (float) (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

//设置相电压
void setPhaseVoltage(float Uq, float Ud, float angle_elec) {
    angle_elec = _normalizeAngle(angle_elec + zero_electric_angle);
    //帕克逆变换
    Ualpha = -Uq * (float) sin(angle_elec);
    Ubeta = Uq * cos(angle_elec);

    //克拉克逆变换
    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (float) (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (float) (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

//开环速度函数
float velocityOpenLoop(float target_velocity) {
    unsigned long now_us = HAL_GetTick();  //获取从开启芯片以来的微秒数，它的精度是 1ms

    //计算当前每个Loop的运行时间间隔
    float Ts = (now_us - open_loop_timestamp) * 1e-3f;

    //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
//    if (Ts <= 0 || Ts > 0.5f) Ts = 5 * 1e-2f;


    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = voltage_power_supply / 10.0f;

    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));

    open_loop_timestamp = now_us;  //用于计算下一个时间间隔

    return Uq;
}

//================简易接口函数================
void FOC_M0_set_Velocity_Angle(float Target) {
    setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI)),
              _electricalAngle_FeedBack());   //角度闭环
}

void FOC_M0_setVelocity(float Target) {
    setTorque(DFOC_M0_VEL_PID((Target * DFOC_M0_Velocity()) * 180 / PI), _electricalAngle_FeedBack());   //速度闭环
}

void FOC_M0_set_Force_Angle(float Target)   //力位
{
    setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI), _electricalAngle_FeedBack());
}

void FOC_M0_setTorque(float Target)
{
    setTorque(Target, _electricalAngle_FeedBack());
}
