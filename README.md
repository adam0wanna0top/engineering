# HKU HerKules 步兵代码教学/解释文档


## 核心Tasks
核心的三个tasks为：Gimbal_Task, Chassis_Task, Shoot_Task

### Gimbal Task:
顾名思义负责云台的相关任务，主要为电机的控制。

**Gimbal_Task.h**
1. 宏定义 #define
   
   宏定义中定义了云台两电机的ID、CAN口、初始位置的电机ecd值（用于上电回正）

2. 数据结构 gimbal_motor_t

   该数据结构包括了电机的反馈数据（经处理）和控制数据以及pid。大致来说INS表示与IMU相关的数据，ENC表示与编码器相关的数据，_set后缀表示pid控制中的设定期望值。其他具体含义见注释。

3. gimbal_m6020[2] 数组
   
   这个数组存放云台两电机的数据，0为yaw，1为pitch。在Gimbal_Task.c中申明，而头文件extern是方便别的程序需要该数组时，包含Gimbal_Task.h即可。

**Gimbal_Task.c**

_如果想按照程序运行顺序阅读，请先看主函数`void Gimbal_Task()`_
1. 宏定义
   
   定义PID参数，包括两个电机的速度环、角度环和自瞄控制环。其中KP KI KD的含义不多说，MAX_OUT是该环输出结果的最大值，比如：**速度环**的输出是**电流值**，MAX_OUT则会限制这个**电流**输出值的最大值。MAX_IOUT是积分过程的一个参数的最大值。

2. CAN口发送数据变量和数组

    定义一个CAN协议的message header和一个uint8_t数组用于存放发送数据，发送电机命令时会使用到
    ```c
    CAN_TxHeaderTypeDef  gimbal_tx_message;
    uint8_t              gimbal_can_send_data[8];
    ```

3. gimbal_m6020[2] 数组
   
   见头文件部分的解释

4. **TODO**
   
   ```c
   uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
   uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle
   int16_t auto_aim_err_yaw=0,auto_aim_err_pitch=0;
   ```

5. 主函数/任务入口 `void Gimbal_Task(void const * argument)`
   
   这个函数内容简洁明了，按照如下顺序执行
   1. `Gimbal_Motor_Init()` 电机初始化：将gimbal_m6020数组赋初值
   2. `Gimbal_Pos_Init()` 云台位置初始化：我们希望车上电时云台能够自动回正，也就是枪口水平对准正前方，那要怎么做到呢？逻辑是对于每台车读取枪口水平朝前时两个电机的编码器值（Keil debug模式），把头文件宏定义里的 `#define YAW_MOTOR_INIT_POS` 和 `#define PITCH_MOTOR_INIT_POS`改成读到的这个值，然后控制电机转到这个位置，误差小于一定范围就认为完成回正。
   3. `vTaskDelay(200)` 暂停一会，应该是等待其他东西的初始化
   4. 进入`while(1)`循环，不断地更新电机数据、执行控制逻辑（根据遥控器指令计算出要发给电机的电流值）以及把指令通过`CAN_Gimbal_CMD`发给电机。**最后，非常重要！用`vTaskDelay(1)`暂停一毫秒以防发送频率过高，导致无法接受电机回传报文**


垂直c板 bmi088_real_data.gyro[0]

短轴 bmi088_real_data.gyro[1]

长轴 bmi088_real_data.gyro[2]

## 步兵硬件连接

C板置于云台弹舱附近

CAN1 走滑环连接底盘电机、云台Yaw电机

CAN2 连接云台Pitch电机、发射机构所有电机