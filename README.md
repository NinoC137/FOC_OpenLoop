# FOC Open Loop Control

实现了开环FOC控制, 将STM32的PWM输出封装于setPWM函数之中, 


注意事项:

1. 电机颤抖, 噪声大, 电流大:  

检查PWM输出频率, 至少应在10k以上, 推荐20k+
