# FOC Open Loop Control

实现了开环FOC控制, 将STM32的PWM输出封装于setPWM函数之中, 

可以由用户自行更新定时器上限数值,而无需修改函数内部的占空比转换.