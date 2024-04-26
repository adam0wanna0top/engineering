# HKU HerKules ���������ѧ/�����ĵ�


## ����Tasks
���ĵ�����tasksΪ��Gimbal_Task, Chassis_Task, Shoot_Task

### Gimbal Task:
����˼�帺����̨�����������ҪΪ����Ŀ��ơ�

**Gimbal_Task.h**
1. �궨�� #define
   
   �궨���ж�������̨�������ID��CAN�ڡ���ʼλ�õĵ��ecdֵ�������ϵ������

2. ���ݽṹ gimbal_motor_t

   �����ݽṹ�����˵���ķ������ݣ��������Ϳ��������Լ�pid��������˵INS��ʾ��IMU��ص����ݣ�ENC��ʾ���������ص����ݣ�_set��׺��ʾpid�����е��趨����ֵ���������庬���ע�͡�

3. gimbal_m6020[2] ����
   
   �����������̨����������ݣ�0Ϊyaw��1Ϊpitch����Gimbal_Task.c����������ͷ�ļ�extern�Ƿ����ĳ�����Ҫ������ʱ������Gimbal_Task.h���ɡ�

**Gimbal_Task.c**

_����밴�ճ�������˳���Ķ������ȿ�������`void Gimbal_Task()`_
1. �궨��
   
   ����PID��������������������ٶȻ����ǶȻ���������ƻ�������KP KI KD�ĺ��岻��˵��MAX_OUT�Ǹû������������ֵ�����磺**�ٶȻ�**�������**����ֵ**��MAX_OUT����������**����**���ֵ�����ֵ��MAX_IOUT�ǻ��ֹ��̵�һ�����������ֵ��

2. CAN�ڷ������ݱ���������

    ����һ��CANЭ���message header��һ��uint8_t�������ڴ�ŷ������ݣ����͵������ʱ��ʹ�õ�
    ```c
    CAN_TxHeaderTypeDef  gimbal_tx_message;
    uint8_t              gimbal_can_send_data[8];
    ```

3. gimbal_m6020[2] ����
   
   ��ͷ�ļ����ֵĽ���

4. **TODO**
   
   ```c
   uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
   uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle
   int16_t auto_aim_err_yaw=0,auto_aim_err_pitch=0;
   ```

5. ������/������� `void Gimbal_Task(void const * argument)`
   
   ����������ݼ�����ˣ���������˳��ִ��
   1. `Gimbal_Motor_Init()` �����ʼ������gimbal_m6020���鸳��ֵ
   2. `Gimbal_Pos_Init()` ��̨λ�ó�ʼ��������ϣ�����ϵ�ʱ��̨�ܹ��Զ�������Ҳ����ǹ��ˮƽ��׼��ǰ������Ҫ��ô�����أ��߼��Ƕ���ÿ̨����ȡǹ��ˮƽ��ǰʱ��������ı�����ֵ��Keil debugģʽ������ͷ�ļ��궨����� `#define YAW_MOTOR_INIT_POS` �� `#define PITCH_MOTOR_INIT_POS`�ĳɶ��������ֵ��Ȼ����Ƶ��ת�����λ�ã����С��һ����Χ����Ϊ��ɻ�����
   3. `vTaskDelay(200)` ��ͣһ�ᣬӦ���ǵȴ����������ĳ�ʼ��
   4. ����`while(1)`ѭ�������ϵظ��µ�����ݡ�ִ�п����߼�������ң����ָ������Ҫ��������ĵ���ֵ���Լ���ָ��ͨ��`CAN_Gimbal_CMD`���������**��󣬷ǳ���Ҫ����`vTaskDelay(1)`��ͣһ�����Է�����Ƶ�ʹ��ߣ������޷����ܵ���ش�����**


��ֱc�� bmi088_real_data.gyro[0]

���� bmi088_real_data.gyro[1]

���� bmi088_real_data.gyro[2]

## ����Ӳ������

C��������̨���ո���

CAN1 �߻������ӵ��̵������̨Yaw���

CAN2 ������̨Pitch���������������е��