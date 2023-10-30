# FOC Open Loop Control

实现了开环FOC控制, 将STM32的PWM输出封装于setPWM函数之中, 


注意事项:

1. 电机颤抖, 噪声大, 电流大:  

检查PWM输出频率, 至少应在10k以上, 推荐20k+

2. 速度加大之后电机开始颤动:

与力矩Uq有关, KV值大的电机, 想要提高速度, 需保证力矩充足

3. 实现非线性拖拽的效果:

在开环控制部分使用for循环进行值的设定, 如:
```c
static float i;
for (i = 1.0f; i <= 30.0f; i += 0.5f){
    velocityOpenLoop(i);
    osDelay(5);
}
for (i = 30.0f; i >= 1.0f; i -= 0.5f){
    velocityOpenLoop(i);
    osDelay(5);
}
```

即可实现非线性拖拽的效果