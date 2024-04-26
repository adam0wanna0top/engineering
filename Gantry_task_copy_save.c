#include "Gantry_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
// #include "auto_aim.h"
#include "USB_Task.h"

#include "arm_math.h"

#define PITCH_UP_BOUND 39.3f
#define PITCH_DOWN_BOUND -32.4f

#define YAW_MOTOR_ENC_SPEED_PID_KP 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_KI 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_KD 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_MAX_OUT 0.0f
#define YAW_MOTOR_ENC_SPEED_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_ENC_ANGLE_PID_KP 10.5f
#define YAW_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ENC_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_ENC_ANGLE_PID_MAX_OUT 70.0f
#define YAW_MOTOR_ENC_ANGLE_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_INS_SPEED_PID_KP 70.0f
#define YAW_MOTOR_INS_SPEED_PID_KI 1.1f
#define YAW_MOTOR_INS_SPEED_PID_KD 0.1f
#define YAW_MOTOR_INS_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_INS_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_INS_ANGLE_PID_KP 15.0f
// #define YAW_MOTOR_INS_ANGLE_PID_KP 10.5f
#define YAW_MOTOR_INS_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_INS_ANGLE_PID_KD 0.0f
#define YAW_MOTOR_INS_ANGLE_PID_MAX_OUT 70.0f
#define YAW_MOTOR_INS_ANGLE_PID_MAX_IOUT 0.0f

#define YAW_MOTOR_AUTO_AIM_PID_KP 0.05f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_KD 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

#define PITCH_MOTOR_ENC_SPEED_PID_KP 100.0f
#define PITCH_MOTOR_ENC_SPEED_PID_KI 1.0f
#define PITCH_MOTOR_ENC_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_ENC_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_MOTOR_ENC_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_MOTOR_ENC_ANGLE_PID_KP 7.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_MAX_OUT 100.0f
#define PITCH_MOTOR_ENC_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_INS_SPEED_PID_KP 100.0f
#define PITCH_MOTOR_INS_SPEED_PID_KI 1.0f
#define PITCH_MOTOR_INS_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_INS_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_MOTOR_INS_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_MOTOR_INS_ANGLE_PID_KP 7.0f
#define PITCH_MOTOR_INS_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_INS_ANGLE_PID_KD 0.0f
#define PITCH_MOTOR_INS_ANGLE_PID_MAX_OUT 100.0f
#define PITCH_MOTOR_INS_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.01f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 0.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 3.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

CAN_TxHeaderTypeDef  gimbal_tx_message;
uint8_t              gimbal_can_send_data[8];
fp32 temp_angle;

gimbal_control_t gimbal_control;
uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle

int16_t auto_aim_err_yaw=0,auto_aim_err_pitch=0;

int status_1 = 0;

fp32 rc_acc;

extern vision_rx_t vision_rx;

fp32 fabs_2( fp32 temp_val )
{
	if( temp_val < 0.0f )
		return -temp_val;
	return temp_val;
}

static void CAN_YAW_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = motor1 >> 8;
	gimbal_can_send_data[1] = motor1;
	gimbal_can_send_data[2] = motor2 >> 8;
	gimbal_can_send_data[3] = motor2;
	gimbal_can_send_data[4] = motor3 >> 8;
	gimbal_can_send_data[5] = motor3;
	gimbal_can_send_data[6] = motor4 >> 8;
	gimbal_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

static void CAN_PITCH_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = motor1 >> 8;
	gimbal_can_send_data[1] = motor1;
	gimbal_can_send_data[2] = motor2 >> 8;
	gimbal_can_send_data[3] = motor2;
	gimbal_can_send_data[4] = motor3 >> 8;
	gimbal_can_send_data[5] = motor3;
	gimbal_can_send_data[6] = motor4 >> 8;
	gimbal_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_enc_speed_pid[3] = {YAW_MOTOR_ENC_SPEED_PID_KP, YAW_MOTOR_ENC_SPEED_PID_KI, YAW_MOTOR_ENC_SPEED_PID_KD};
	const static fp32 yaw_motor_enc_angle_pid[3] = {YAW_MOTOR_ENC_ANGLE_PID_KP, YAW_MOTOR_ENC_ANGLE_PID_KI, YAW_MOTOR_ENC_ANGLE_PID_KD};
	const static fp32 yaw_motor_ins_speed_pid[3] = {YAW_MOTOR_INS_SPEED_PID_KP, YAW_MOTOR_INS_SPEED_PID_KI, YAW_MOTOR_INS_SPEED_PID_KD};
	const static fp32 yaw_motor_ins_angle_pid[3] = {YAW_MOTOR_INS_ANGLE_PID_KP, YAW_MOTOR_INS_ANGLE_PID_KI, YAW_MOTOR_INS_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	
	const static fp32 pitch_motor_enc_speed_pid[3] = {PITCH_MOTOR_ENC_SPEED_PID_KP, PITCH_MOTOR_ENC_SPEED_PID_KI, PITCH_MOTOR_ENC_SPEED_PID_KD};
	const static fp32 pitch_motor_enc_angle_pid[3] = {PITCH_MOTOR_ENC_ANGLE_PID_KP, PITCH_MOTOR_ENC_ANGLE_PID_KI, PITCH_MOTOR_ENC_ANGLE_PID_KD};
	const static fp32 pitch_motor_ins_speed_pid[3] = {PITCH_MOTOR_INS_SPEED_PID_KP, PITCH_MOTOR_INS_SPEED_PID_KI, PITCH_MOTOR_INS_SPEED_PID_KD};
	const static fp32 pitch_motor_ins_angle_pid[3] = {PITCH_MOTOR_INS_ANGLE_PID_KP, PITCH_MOTOR_INS_ANGLE_PID_KI, PITCH_MOTOR_INS_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_control.gimbal_m6020[i].INS_speed=0;
		gimbal_control.gimbal_m6020[i].INS_speed_set=0;
		gimbal_control.gimbal_m6020[i].INS_angle=0;
		gimbal_control.gimbal_m6020[i].angle_set=0;
		gimbal_control.gimbal_m6020[i].ENC_angle=0;
		gimbal_control.gimbal_m6020[i].angle_set=0;
		gimbal_control.gimbal_m6020[i].give_current=0;
		gimbal_control.mode = Gimbal_No_Force;
		gimbal_control.last_mode = Gimbal_No_Force;
	}

	PID_init(&gimbal_control.gimbal_m6020[0].ENC_speed_pid,PID_POSITION,yaw_motor_enc_speed_pid,YAW_MOTOR_ENC_SPEED_PID_MAX_OUT,YAW_MOTOR_ENC_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[0].ENC_angle_pid,PID_POSITION,yaw_motor_enc_angle_pid,YAW_MOTOR_ENC_ANGLE_PID_MAX_OUT,YAW_MOTOR_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[0].INS_speed_pid,PID_POSITION,yaw_motor_ins_speed_pid,YAW_MOTOR_INS_SPEED_PID_MAX_OUT,YAW_MOTOR_INS_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[0].INS_angle_pid,PID_POSITION,yaw_motor_ins_angle_pid,YAW_MOTOR_INS_ANGLE_PID_MAX_OUT,YAW_MOTOR_INS_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[0].auto_aim_pid,PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	
	PID_init(&gimbal_control.gimbal_m6020[1].ENC_speed_pid,PID_POSITION,pitch_motor_enc_speed_pid,PITCH_MOTOR_ENC_SPEED_PID_MAX_OUT,PITCH_MOTOR_ENC_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[1].ENC_angle_pid,PID_POSITION,pitch_motor_enc_angle_pid,PITCH_MOTOR_ENC_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ENC_ANGLE_PID_MAX_IOUT);	
	PID_init(&gimbal_control.gimbal_m6020[1].INS_speed_pid,PID_POSITION,pitch_motor_ins_speed_pid,PITCH_MOTOR_INS_SPEED_PID_MAX_OUT,PITCH_MOTOR_INS_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_control.gimbal_m6020[1].INS_angle_pid,PID_POSITION,pitch_motor_ins_angle_pid,PITCH_MOTOR_INS_ANGLE_PID_MAX_OUT,PITCH_MOTOR_INS_ANGLE_PID_MAX_IOUT);	
	PID_init(&gimbal_control.gimbal_m6020[1].auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);	
}


void Gimbal_PID_Calculator( uint8_t motor_ID )
{
	if( gimbal_control.mode == Gimbal_No_Force )
	{
		gimbal_control.gimbal_m6020[motor_ID].give_current = 0.0f;
	}
	else if( gimbal_control.mode == Gimbal_Normal_Mode )
	{
		if( gimbal_control.gimbal_m6020[motor_ID].motor_mode == Gimbal_Motor_Encoder )
		{
			PID_calc(&gimbal_control.gimbal_m6020[motor_ID].ENC_angle_pid, gimbal_control.gimbal_m6020[motor_ID].ENC_relative_angle, gimbal_control.gimbal_m6020[motor_ID].angle_set );
			// gimbal_control.gimbal_m6020[motor_ID].INS_speed_set = gimbal_control.gimbal_m6020[motor_ID].ENC_angle_pid.out;
			// PID_calc( &gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid, gimbal_control.gimbal_m6020[motor_ID].INS_speed, gimbal_control.gimbal_m6020[motor_ID].INS_speed_set );
			// gimbal_control.gimbal_m6020[motor_ID].give_current = gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid.out;
			PID_calc(&gimbal_control.gimbal_m6020[motor_ID].ENC_speed_pid, gimbal_control.gimbal_m6020[motor_ID].ENC_speed, gimbal_control.gimbal_m6020[motor_ID].ENC_speed_set );
			gimbal_control.gimbal_m6020[motor_ID].give_current = gimbal_control.gimbal_m6020[motor_ID].ENC_speed_pid.out;
		}
		else if( gimbal_control.gimbal_m6020[motor_ID].motor_mode == Gimbal_Motor_INS )
		{
		// 	PID_calc( &gimbal_control.gimbal_m6020[motor_ID].INS_angle_pid, gimbal_control.gimbal_m6020[motor_ID].INS_angle, gimbal_control.gimbal_m6020[motor_ID].angle_set );
		// 	gimbal_control.gimbal_m6020[motor_ID].INS_speed_set = gimbal_control.gimbal_m6020[motor_ID].INS_angle_pid.out;
		// 	PID_calc( &gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid, gimbal_control.gimbal_m6020[motor_ID].INS_speed, gimbal_control.gimbal_m6020[motor_ID].INS_speed_set );
		// 	gimbal_control.gimbal_m6020[motor_ID].give_current = gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid.out;
		}
	}
	else
	{
		PID_calc( &gimbal_control.gimbal_m6020[motor_ID].auto_aim_pid, gimbal_control.gimbal_m6020[motor_ID].INS_angle, gimbal_control.gimbal_m6020[motor_ID].angle_set );
		gimbal_control.gimbal_m6020[motor_ID].INS_speed_set = gimbal_control.gimbal_m6020[motor_ID].auto_aim_pid.out;
		PID_calc( &gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid, gimbal_control.gimbal_m6020[motor_ID].INS_speed, gimbal_control.gimbal_m6020[motor_ID].INS_speed_set );
		gimbal_control.gimbal_m6020[motor_ID].give_current = gimbal_control.gimbal_m6020[motor_ID].INS_speed_pid.out;
	}

}

void Gantry_Motor_Data_Update(void)
{
	//pitch
	gimbal_control.gimbal_m6020[1].ENC_speed = motor_measure_gimbal[1].speed_rpm/60.0f * 360.0f;//把电机1RPM转换成角度。
	gimbal_control.gimbal_m6020[1].ENC_last_ecd = gimbal_control.gimbal_m6020[1].ENC_ecd;//把ecd转换为last_ecd
	gimbal_control.gimbal_m6020[1].ENC_ecd = motor_measure_gimbal[1].ecd;//测出当前位置并且赋给ecd
	gimbal_control.gimbal_m6020[1].ENC_angle = (gimbal_control.gimbal_m6020[1].ENC_ecd / 8192.0f * 360.0f);//把当前位置转换成角度
	gimbal_control.gimbal_m6020[1].origin = PITCH_MOTOR_INIT_POS / 8192.0f * 360.0f;//位置初始化
	gimbal_control.gimbal_m6020[1].ENC_relative_angle = gimbal_control.gimbal_m6020[1].ENC_angle - gimbal_control.gimbal_m6020[1].origin;//当前位置和初始化位置的差值
	
	//Yaw
	gimbal_control.gimbal_m6020[0].ENC_last_ecd = gimbal_control.gimbal_m6020[0].ENC_ecd;//把ecd转换为last_ecd
									//gimbal_control.gimbal_m6020[0].INS_angle = INS_angle_deg[0] - gimbal_control.gimbal_m6020[0].INS_angle_offset; // -180<angle_deg<180
									
									//if ( gimbal_control.gimbal_m6020[0].INS_angle > 180.0f)
									//	gimbal_control.gimbal_m6020[0].INS_angle = gimbal_control.gimbal_m6020[0].INS_angle - 360.0f;
									//if ( gimbal_control.gimbal_m6020[0].INS_angle < -180.0f)
									//	gimbal_control.gimbal_m6020[0].INS_angle = gimbal_control.gimbal_m6020[0].INS_angle + 360.0f;
			
	gimbal_control.gimbal_m6020[0].ENC_speed = motor_measure_gimbal[0].speed_rpm / 60.0f * 360.0f;//转每分转换成°每秒
	gimbal_control.gimbal_m6020[0].ENC_ecd = motor_measure_gimbal[0].ecd;//获取当前位置ecd



	//零点检测
	if (gimbal_control.gimbal_m6020[0].ENC_ecd - gimbal_control.gimbal_m6020[0].ENC_last_ecd > 4000)//逆时针转自减
		gimbal_control.gimbal_m6020[0].ENC_round_cnt--;
	if (gimbal_control.gimbal_m6020[0].ENC_ecd - gimbal_control.gimbal_m6020[0].ENC_last_ecd < -4000)//顺时针转自加
		gimbal_control.gimbal_m6020[0].ENC_round_cnt++;


	gimbal_control.gimbal_m6020[0].ENC_angle = ( gimbal_control.gimbal_m6020[0].ENC_ecd / 8192.0f * 360.0f + gimbal_control.gimbal_m6020[0].ENC_round_cnt * 360.0f );//当前角度+圈数
	gimbal_control.gimbal_m6020[0].origin = YAW_MOTOR_INIT_POS / 8192.0f * 360.0f + ( gimbal_control.gimbal_m6020[0].ENC_round_cnt * 360.0f );//初始位置+圈数
	gimbal_control.gimbal_m6020[0].ENC_relative_angle = gimbal_control.gimbal_m6020[0].ENC_angle - gimbal_control.gimbal_m6020[0].origin;//当前角度-初始位置

	//origin位置顺时针180°是正，逆时针180°是负，用于判断ecd相对于origin的位置.
	if (gimbal_control.gimbal_m6020[0].ENC_relative_angle < -180.0f)
		gimbal_control.gimbal_m6020[0].ENC_relative_angle = gimbal_control.gimbal_m6020[0].ENC_relative_angle + 360.0f;
	if (gimbal_control.gimbal_m6020[0].ENC_relative_angle > 180.0f )
		gimbal_control.gimbal_m6020[0].ENC_relative_angle = gimbal_control.gimbal_m6020[0].ENC_relative_angle - 360.0f;


									//gimbal_control.gimbal_m6020[0].INS_speed = arm_cos_f32(gimbal_control.gimbal_m6020[1].ENC_relative_angle * 3.14159f /180.0f) * bmi088_real_data.gyro[2] - arm_sin_f32(gimbal_control.gimbal_m6020[1].ENC_relative_angle * 3.14159f /180.0f) * bmi088_real_data.gyro[0];
									//gimbal_control.gimbal_m6020[0].INS_speed = gimbal_control.gimbal_m6020[0].INS_speed / 3.1415f * 180.0f ;
}

// use encoder initial value as set value, initialise the position of gimbal
void Gimbal_Pos_Init(void)
{
	float init_range = 3.0f;//the initial bearing angle

	gimbal_control.gimbal_m6020[0].motor_mode = Gimbal_Motor_Encoder;
	gimbal_control.gimbal_m6020[1].motor_mode = Gimbal_Motor_Encoder;

	// Yaw
	while( fabs_2(gimbal_control.gimbal_m6020[0].ENC_relative_angle) > init_range ){
		Gantry_Motor_Data_Update();
		gimbal_control.gimbal_m6020[0].angle_set = 0.0f;
		Gimbal_PID_Calculator(Yaw);
		CAN_YAW_CMD(gimbal_control.gimbal_m6020[0].give_current,0,0,0);
		vTaskDelay(1);
	}
	gimbal_control.gimbal_m6020[0].INS_angle_offset = INS_angle_deg[0];

	// Pitch		
	while( fabs_2(gimbal_control.gimbal_m6020[1].ENC_relative_angle) > init_range ){
		Gantry_Motor_Data_Update();
		gimbal_control.gimbal_m6020[1].angle_set = 0.0f;
		Gimbal_PID_Calculator(Pitch);
		CAN_PITCH_CMD(0, gimbal_control.gimbal_m6020[1].give_current,0,0);
		vTaskDelay(1);
	}

	gimbal_control.gimbal_m6020[0].motor_mode = gimbal_control.gimbal_m6020[0].last_motor_mode;
	gimbal_control.gimbal_m6020[1].motor_mode = gimbal_control.gimbal_m6020[1].last_motor_mode;
}

void Yaw_Motor_Control(void)
{
	fp32 err;
	if( gimbal_control.mode == Gimbal_Normal_Mode )
	{
		if( rc_ctrl.rc.ch[2] > 10 || rc_ctrl.rc.ch[2] < -10 )
			// gimbal_control.gimbal_m6020[0].angle_set = gimbal_control.gimbal_m6020[0].INS_angle;
			gimbal_control.gimbal_m6020[0].angle_set = gimbal_control.gimbal_m6020[0].angle_set - (float)rc_ctrl.rc.ch[2]/6600.0f;
		if ( gimbal_control.gimbal_m6020[0].angle_set > 180.0f )
			gimbal_control.gimbal_m6020[0].angle_set = gimbal_control.gimbal_m6020[0].angle_set - 360.0f;
		if ( gimbal_control.gimbal_m6020[0].angle_set < -180.0f )
			gimbal_control.gimbal_m6020[0].angle_set = gimbal_control.gimbal_m6020[0].angle_set + 360.0f;

		err = gimbal_control.gimbal_m6020[0].INS_angle - gimbal_control.gimbal_m6020[0].angle_set;

		if ( err > 180.0f )
			err = err - 360.0f;
		if ( err < -180.0f )
			err = err + 360.0f;
		PID_calc( &gimbal_control.gimbal_m6020[0].INS_angle_pid, err, 0.0f );
		gimbal_control.gimbal_m6020[0].INS_speed_set = gimbal_control.gimbal_m6020[0].INS_angle_pid.out;
		PID_calc( &gimbal_control.gimbal_m6020[0].INS_speed_pid, gimbal_control.gimbal_m6020[0].INS_speed, gimbal_control.gimbal_m6020[0].INS_speed_set );
		gimbal_control.gimbal_m6020[0].give_current = gimbal_control.gimbal_m6020[0].INS_speed_pid.out;
		return;
	}
	else if( gimbal_control.mode == Gimbal_Auto_Aiming )
	{
		gimbal_control.gimbal_m6020[0].angle_set = vision_rx.yaw_angle;
	}

	Gimbal_PID_Calculator(Yaw);
}

void Pitch_Motor_Control(void)
{
	if( gimbal_control.mode == Gimbal_Normal_Mode )
	{
		if( rc_ctrl.rc.ch[3] > 10 || rc_ctrl.rc.ch[3] < -10 )
			gimbal_control.gimbal_m6020[1].angle_set = gimbal_control.gimbal_m6020[1].angle_set + (float)rc_ctrl.rc.ch[3]/6600.0f;
	}
	else if( gimbal_control.mode == Gimbal_Auto_Aiming )
	{
		gimbal_control.gimbal_m6020[1].angle_set = vision_rx.pitch_angle;
	}
	if (gimbal_control.gimbal_m6020[1].angle_set<PITCH_DOWN_BOUND)
		gimbal_control.gimbal_m6020[1].angle_set = PITCH_DOWN_BOUND;
	if (gimbal_control.gimbal_m6020[1].angle_set>PITCH_UP_BOUND)
		gimbal_control.gimbal_m6020[1].angle_set = PITCH_UP_BOUND;
	Gimbal_PID_Calculator(Pitch);
}

void Gantry_RC_Mode_Set(void)
{

	gimbal_control.last_mode = gimbal_control.mode;
	switch (rc_ctrl.rc.s[0]) // right switch
	{
	case RC_SW_DOWN:
		gimbal_control.mode = Gimbal_No_Force;
		break;
	case RC_SW_UP:
		//gimbal_control.mode = Gimbal_Auto_Aiming;
		gimbal_control.mode = Gimbal_Normal_Mode;
		break;
	case RC_SW_MID:
		gimbal_control.mode = Gimbal_Normal_Mode;
		break;
	default:
		break;
	}
	
}

void Gimbal_Motor_Mode_Set(void)
{
	gimbal_control.gimbal_m6020[0].last_motor_mode = gimbal_control.gimbal_m6020[0].motor_mode;
	gimbal_control.gimbal_m6020[1].last_motor_mode = gimbal_control.gimbal_m6020[1].motor_mode;

	gimbal_control.gimbal_m6020[0].motor_mode = Gimbal_Motor_Encoder;
	gimbal_control.gimbal_m6020[1].motor_mode = Gimbal_Motor_Encoder;
}

void Gimbal_Mode_Change_Control_Transit(void)
{
	if( gimbal_control.last_mode == Gimbal_No_Force && gimbal_control.mode == Gimbal_Normal_Mode )
	{
		Gimbal_Pos_Init();
	}
}

void Gantry_Task(void const * argument)
{

	// Gimbal_Motor_Init();
	// vTaskDelay(200);

	while(1)
	{
		// Gantry_Motor_Data_Update();

		// Gantry_RC_Mode_Set();
		// Gimbal_Motor_Mode_Set();

		// Gimbal_Mode_Change_Control_Transit();
		
		// // Yaw_Motor_Control();
		// Pitch_Motor_Control();
		
		// CAN_YAW_CMD(gimbal_control.gimbal_m6020[0].give_current,gimbal_control.gimbal_m6020[0].give_current,gimbal_control.gimbal_m6020[0].give_current,gimbal_control.gimbal_m6020[0].give_current);
		//CAN_YAW_CMD(rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10);

		// CAN_PITCH_CMD(gimbal_control.gimbal_m6020[1].give_current,gimbal_control.gimbal_m6020[1].give_current,gimbal_control.gimbal_m6020[1].give_current,gimbal_control.gimbal_m6020[1].give_current);
		// CAN_PITCH_CMD(0,0,0,0);
		//CAN_PITCH_CMD(rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10,rc_ctrl.rc.ch[2]*10);
		//CAN_YAW_CMD(0,rc_ctrl.rc.ch[3]*10,rc_ctrl.rc.ch[3]*10,rc_ctrl.rc.ch[3]*10);
		
		vTaskDelay(1);
	}
}
