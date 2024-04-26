#include "Chassis_Task.h"
#include "Gantry_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"

#define M3505_MOTOR_SPEED_PID_KP 180.0f
#define M3505_MOTOR_SPEED_PID_KI 0.05f
#define M3505_MOTOR_SPEED_PID_KD 50.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.00f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 2.0f


extern uint8_t current_mode;
extern uint8_t last_mode;

extern uint8_t Mining_mode;
extern uint8_t Debug_mode;
extern uint8_t Redemption_mode;

extern uint8_t control_flag;
extern uint8_t control_open;
extern uint8_t control_close;
extern void RC_Mode_Set(void);

CAN_TxHeaderTypeDef  chassis_tx_message;

uint8_t              chassis_can_send_data[8];

chassis_motor_t chassis_m3508[4];
chassis_control_t chassis_control;

static void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-16384,+16384
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = 0x200;//1到4号电机
	chassis_tx_message.IDE = M3508_MOTOR_RPM_TO_VECTOR;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Chassis_Motor_Init(void)
{
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed=0;
		chassis_m3508[i].speed_set=0;
		chassis_m3508[i].give_current=0;
		
		PID_init(&chassis_m3508[i].pid,PID_POSITION,motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}

}

void Chassis_Motor_Data_Update(void)
{
	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed= M3508_MOTOR_RPM_TO_VECTOR * motor_measure_chassis[i].speed_rpm;
	}
}

static void Chassis_Vector_to_Mecanum_Wheel_Speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{

	wheel_speed[0] = -(-vx_set*sqrt(2) - vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[1] = -(vx_set*sqrt(2) - vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[2] = -(vx_set*sqrt(2) + vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
	wheel_speed[3] = -(-vx_set*sqrt(2) + vy_set*sqrt(2) + wz_set * 0.4)/0.0763;
}	


void Chassis_PID_Calculator(void)
{
		for( uint8_t i = 0; i < 4; i++ )
		{
			PID_calc(&chassis_m3508[i].pid,chassis_m3508[i].speed,chassis_m3508[i].speed_set);
			chassis_m3508[i].give_current=chassis_m3508[i].pid.out;	
		}
	
}

//如果按键按下，变量为1，如果没有按下，变量为0
static uint16_t Chassis_Byte_Calculator(uint16_t input) {
    return (input > 0) ? 1 : 0;
}


static void Chassis_Motor_Control(void)
{

	fp32 vx,vy,wz;

	if(control_flag == control_open)//enable模式，地盘控制打开
	{
////////////////////////////////////////摇杆控制///////////////////////////////////////////////////
		 vx=(fp32)rc_ctrl.rc.ch[3]/660.0f;//左摇杆上下
		 vy=(fp32)rc_ctrl.rc.ch[2]/660.0f;//左摇杆左右
         wz=(fp32)rc_ctrl.rc.ch[0]/(-660.0f);//右摇杆左右

		 if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)//快速移动_x和y方向上
		 {
		 	chassis_control.vx = 5 * vx;
		 	chassis_control.vy = 5 * vy;
		 	chassis_control.wz = 5 * wz;
		 }
		 else//正常移动_x和y方向上
		 {
		 	chassis_control.vx = vx;
		 	chassis_control.vy = vy;
		 	chassis_control.wz = wz;
		 }
////////////////////////////////////////键鼠控制地盘/////////////////////////////////////////////
//		vx = 5 * Chassis_Byte_Calculator(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)
//		     - 5 * Chassis_Byte_Calculator(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S);//W与S方向速度

//		vy = 5 * Chassis_Byte_Calculator(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)
//		     - 5 * Chassis_Byte_Calculator(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A);//A与D方向速度

		// if(current_mode == Debug_mode)//判断当前是否处于调试模式
		// {
		// 	wz = 5 * rc_ctrl.mouse.press_l - 5 * rc_ctrl.mouse.press_r;//鼠标控制车身旋转
		// }
		// else
		// {
		// 	wz = 0;
		// }

		// if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)//按下shift加速
		// {
		// 	chassis_control.vx = 5 * vx;
		// 	chassis_control.vy = 5 * vy;
		// 	chassis_control.wz = 5 * wz;
		// }
		// else
		// {
		// 	chassis_control.vx = vx;
		// 	chassis_control.vy = vy;
		// 	chassis_control.wz = wz;
		// }
//////////////////////////////////////////////////////////////////////////////////////////////////		
	}

	else if ( control_flag == control_close )//disable模式，地盘控制关闭
	{
		chassis_control.vx = 0;
		chassis_control.vy = 0;
		chassis_control.wz = 0;
	}

	fp32 motor_speed[4];
	Chassis_Vector_to_Mecanum_Wheel_Speed(chassis_control.vx,chassis_control.vy,chassis_control.wz,motor_speed);

	for(uint8_t i=0;i<4;i++)
	{
		chassis_m3508[i].speed_set = motor_speed[i];
	}

	Chassis_PID_Calculator();
}

void Chassis_Task(void const * argument)
{
	Chassis_Motor_Init();
	
	vTaskDelay(200);	
	
	while(1)
	{
		Chassis_Motor_Data_Update();
		RC_Mode_Set();
		Chassis_Motor_Control();		
		CAN_Chassis_CMD(chassis_m3508[0].give_current,chassis_m3508[1].give_current,chassis_m3508[2].give_current,chassis_m3508[3].give_current);
		vTaskDelay(2);
	}
}
