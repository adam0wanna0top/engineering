#include "Gantry_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "USB_Task.h"
#include "arm_math.h"

////////////////////////////////////增加的龙门电机宏定义(左边5号右边6号)/////////////////////////////////////

//龙门左5两个电机的位置环pid
#define GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KP 30.0f
#define GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KD 1.0f
#define GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_MAX_OUT 6000.0f
#define GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_MAX_IOUT 2000.0f

//龙门左5两个电机的速度环pid
#define GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KP 15.9f //15.9f 
#define GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KI 0.02f //0.02f
#define GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KD 6.6f  //6.6f
#define GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_MAX_OUT 6000.0f
#define GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_MAX_IOUT 2000.0f

//龙门右6两个电机的位置环pid
#define GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KP 30.0f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KD 1.0f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_MAX_OUT 6000.0f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_MAX_IOUT 2000.0f

//龙门右6两个电机的速度环pid
#define GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KP 15.9f //15.9f 
#define GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KI 0.02f //0.02f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KD 6.6f  //6.6f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_MAX_OUT 6000.0f
#define GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_MAX_IOUT 2000.0f


__IO uint32_t uwTick_Key_Set_Point = 0;

extern uint8_t current_mode;
extern uint8_t last_mode;

extern uint8_t Mining_mode;
extern uint8_t Debug_mode;
extern uint8_t Redemption_mode;

extern uint8_t control_flag;
extern uint8_t control_open;
extern uint8_t control_close;
extern void RC_Mode_Set(void);

////////////////////////////////
gimbal_control_t gimbal_control;//这段代码不能删，一删就报错，不知道为什么
////////////////////////////////

//////////////////////////////////////////增加的龙门(左边5号和右边6号)电机can发送代码///////////////////////////////////////////////////////////////
CAN_TxHeaderTypeDef  gantry_tx_message;
uint8_t              gantry_can_send_data[8];
gantry_control_t gantry_control;

int8_t Count_number = 0;
uint32_t i=0;

static void CAN_Gantry_CMD(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box;
	gantry_tx_message.StdId = 0x1FF;
	gantry_tx_message.IDE = CAN_ID_STD;
	gantry_tx_message.RTR = CAN_RTR_DATA;
	gantry_tx_message.DLC = 0x08;
	gantry_can_send_data[0] = motor5 >> 8;
	gantry_can_send_data[1] = motor5;
	gantry_can_send_data[2] = motor6 >> 8;
	gantry_can_send_data[3] = motor6;
	gantry_can_send_data[4] = motor7 >> 8;
	gantry_can_send_data[5] = motor7;
	gantry_can_send_data[6] = motor8 >> 8;
	gantry_can_send_data[7] = motor8;

	HAL_CAN_AddTxMessage(&GANTRY_CAN, &gantry_tx_message, gantry_can_send_data, &send_mail_box);
}


void Gantry_Motor_Init(void)
{
	////////////////////////////////////////////////////龙门5号和6号电机pid参数数组////////////////////////////////////////////////////////////////////
	//左边5号位置环pid数组
	const static fp32 gantry_left_motor_enc_angle_pid[3] = {GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KP, GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KI, GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_KD};
	//左边5号速度环pid数组
	const static fp32 gantry_left_motor_enc_speed_pid[3] = {GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KP, GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KI, GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_KD};

	//右边6号位置环pid数组
	const static fp32 gantry_right_motor_enc_angle_pid[3] = {GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KP, GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KI, GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_KD};
	//右边6号速度环pid数组
	const static fp32 gantry_right_motor_enc_speed_pid[3] = {GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KP, GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KI, GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_KD};
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	for(uint8_t i=0;i<2;i++)
	{
		//3508编码器的参数
		gantry_control.gantry_m3508[i].ENC_speed=0;
		gantry_control.gantry_m3508[i].ENC_angle=0;
		gantry_control.gantry_m3508[i].ENC_last_ecd=0;
		gantry_control.gantry_m3508[i].ENC_ecd=0;
		gantry_control.gantry_m3508[i].ENC_round_cnt=0;
		gantry_control.gantry_m3508[i].ENC_beginning=0;//test


		//3508输出端段参数
		gantry_control.gantry_m3508[i].out_speed=0;
		gantry_control.gantry_m3508[i].out_speed_set=0;
		gantry_control.gantry_m3508[i].out_angle=0;
		gantry_control.gantry_m3508[i].out_angle_set=0;
		gantry_control.gantry_m3508[i].out_round_cnt=0;//输出端零点检测
		gantry_control.gantry_m3508[i].out_relative_angle=0;

		//3508的电流
		gantry_control.gantry_m3508[i].give_current=0;
	}
	//左边5号电机pid初始化
	PID_init(&gantry_control.gantry_m3508[0].ENC_angle_pid,PID_POSITION,gantry_left_motor_enc_angle_pid,GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_MAX_OUT,GANTRY_LEFT_M3505_MOTOR_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&gantry_control.gantry_m3508[0].ENC_speed_pid,PID_POSITION,gantry_left_motor_enc_speed_pid,GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_MAX_OUT,GANTRY_LEFT_M3505_MOTOR_ENC_SPEED_PID_MAX_IOUT);

	//右边6号电机pid初始化
	PID_init(&gantry_control.gantry_m3508[1].ENC_angle_pid,PID_POSITION,gantry_right_motor_enc_angle_pid,GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_MAX_OUT,GANTRY_RIGHT_M3505_MOTOR_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&gantry_control.gantry_m3508[1].ENC_speed_pid,PID_POSITION,gantry_right_motor_enc_speed_pid,GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_MAX_OUT,GANTRY_RIGHT_M3505_MOTOR_ENC_SPEED_PID_MAX_IOUT);
}



//龙门电机位置限位初始化，需要每次上电的时候都标记一个origin值
void Gantry_Pos_Init(void)
{
	gantry_control.gantry_m3508[0].ENC_beginning = motor_measure_gantry[0].ecd / 8192.0f * 360.0f;//左边5号电机把上电时候的位置标记成origin值
	gantry_control.gantry_m3508[1].ENC_beginning = motor_measure_gantry[1].ecd / 8192.0f * 360.0f;//右边6号电机把上电时候的位置标记成origin值
}


void Gantry_Motor_Data_Update(void)
{

	for(uint8_t i=0;i<2;i++)
	{
		//编码器端参数
		gantry_control.gantry_m3508[i].ENC_last_ecd = gantry_control.gantry_m3508[i].ENC_ecd;//把ecd转换为last_ecd
		gantry_control.gantry_m3508[i].ENC_speed = motor_measure_gantry[i].speed_rpm/60.0f * 360.0f;//把电机RPM多少转每分钟换成多少°每秒。
		gantry_control.gantry_m3508[i].ENC_ecd = motor_measure_gantry[i].ecd;//测出当前位置并且赋给ecd
	 	gantry_control.gantry_m3508[i].ENC_angle = (motor_measure_gantry[i].ecd / 8192.0f * 360.0f + gantry_control.gantry_m3508[i].ENC_round_cnt * 360.0f );//当前角度+圈数
		
		
		//输出端参数，传动比19:1
		gantry_control.gantry_m3508[i].out_speed = 0.052631578947368f * gantry_control.gantry_m3508[i].ENC_speed;
		gantry_control.gantry_m3508[i].out_angle = 0.052631578947368f * gantry_control.gantry_m3508[i].ENC_angle;	
		gantry_control.gantry_m3508[i].out_origin = 0.052631578947368f * gantry_control.gantry_m3508[i].ENC_beginning;
		if((uwTick -  uwTick_Key_Set_Point)<1680)//2400限制的时间
		{
			gantry_control.gantry_m3508[i].out_angle = gantry_control.gantry_m3508[i].out_origin;
			gantry_control.gantry_m3508[i].ENC_beginning = gantry_control.gantry_m3508[i].ENC_angle;
//			CAN_Gantry_CMD(3000,3000,0,0);//5号电机和6号电机发送函数
//			vTaskDelay(1800);
		}
		else
		{
			gantry_control.gantry_m3508[i].out_relative_angle = gantry_control.gantry_m3508[i].out_angle - gantry_control.gantry_m3508[i].out_origin;
		}
		

		//零点检测，严格来说是计算跨过了几次零点，out_round_cnt是输出的圈数累计
		if (gantry_control.gantry_m3508[i].ENC_ecd - gantry_control.gantry_m3508[i].ENC_last_ecd > 6000)//逆时针转自减
		{
			gantry_control.gantry_m3508[i].ENC_round_cnt--;
			Count_number--;
			if(Count_number <= -19)
			{
				gantry_control.gantry_m3508[i].out_round_cnt--;
				Count_number = 0;
			}
		}
		if (gantry_control.gantry_m3508[i].ENC_ecd - gantry_control.gantry_m3508[i].ENC_last_ecd < -6000)//顺时针转自减
		{
			gantry_control.gantry_m3508[i].ENC_round_cnt++;
			Count_number++;
			if(Count_number >= 19)
			{
				gantry_control.gantry_m3508[i].out_round_cnt++;
				Count_number = 0;
			}
		}
	}
}

void Gantry_PID_Calculator( uint8_t motor_ID )//龙门电机位置闭环
{	
	//pid位置环
	PID_calc( &gantry_control.gantry_m3508[motor_ID].ENC_angle_pid, gantry_control.gantry_m3508[motor_ID].out_relative_angle, gantry_control.gantry_m3508[motor_ID].out_angle_set );
	gantry_control.gantry_m3508[motor_ID].out_speed_set = gantry_control.gantry_m3508[motor_ID].ENC_angle_pid.out;

	//pid速度环
	PID_calc( &gantry_control.gantry_m3508[motor_ID].ENC_speed_pid, gantry_control.gantry_m3508[motor_ID].out_speed, gantry_control.gantry_m3508[motor_ID].out_speed_set );
	gantry_control.gantry_m3508[motor_ID].give_current = gantry_control.gantry_m3508[motor_ID].ENC_speed_pid.out;
	
}


void Gantry_Motor_Control(void)
{
	if( control_flag == control_close )
	{
		for(uint8_t i=0;i<2;i++)
		{
			gantry_control.gantry_m3508[i].give_current = 0.0f;
		}
	}

	else if( control_flag == control_open )
	{
		if( rc_ctrl.rc.ch[1] > 10 || rc_ctrl.rc.ch[1] < -10 )//右竖摇杆
		{
			gantry_control.gantry_m3508[0].out_angle_set = gantry_control. gantry_m3508[0].out_angle_set - (float)rc_ctrl.rc.ch[1]/3300.0f;//这个方向和大小可以后面调
			gantry_control.gantry_m3508[1].out_angle_set = gantry_control.gantry_m3508[1].out_angle_set + (float)rc_ctrl.rc.ch[1]/3300.0f;//这个方向和大小可以后面调
		}
		Gantry_PID_Calculator(Gantry_left_motor);
		Gantry_PID_Calculator(Gantry_right_motor);
	}
	else
	{
		for(uint8_t i=0;i<2;i++)
		{
			gantry_control.gantry_m3508[i].give_current = 0.0f;
		}
	}
}


void Gantry_Task(void const * argument)
{

	Gantry_Motor_Init();//电机初始化
	Gantry_Pos_Init();//电机位置初始化
	
//	DM_YAW_Enable_Motor();
//	DM_Pitch_Enable_Motor();
	
	
	// vTaskDelay(200);
	uwTick_Key_Set_Point = uwTick;

	while(1)
	{
		Gantry_Motor_Data_Update();		
		Gantry_Motor_Control();

  	CAN_Gantry_CMD(gantry_control.gantry_m3508[0].give_current,gantry_control.gantry_m3508[1].give_current,0,0);//5号电机和6号电机发送函数
		
		vTaskDelay(2);
	}
}
