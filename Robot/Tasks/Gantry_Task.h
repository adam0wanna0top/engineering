#ifndef _GANTRY_TASK
#define _GANTRY_TASK

#include "struct_typedef.h"
#include "pid.h"


#define CAN_6020_M2_ID 0x206//这段不能删，在别的文件里被引用


#define Gantry_left_motor (uint8_t)0
#define Gantry_right_motor (uint8_t)1
 
////////////////////////////////////增加的龙门电机宏定义//////////////////////////////////////////////////////

#define GANTRY_CAN hcan2
#define CAN_3508_M5_ID 0x205
#define CAN_3508_M6_ID 0x206


typedef struct
{
	//没用但是不能删的定义
	fp32 INS_angle;


	//3508编码器的参数
	fp32 ENC_speed; // Encoder speed (rpm -> deg/s)
	fp32 ENC_speed_set; // Set goal speed according to encoder speed (degree/s)
	fp32 ENC_angle; // Encoder angle [0,8191)
	fp32 ENC_angle_actual; // Relative angle to the initial angle (degree)
	fp32 ENC_angle_set; // Set goal angle for the encoder
	fp32 ENC_last_ecd;
	fp32 ENC_ecd;
	fp32 ENC_round_cnt; // circle count
	fp32 ENC_origin;
	fp32 ENC_relative_angle;
	fp32 ENC_beginning;


	//3508输出端段参数
	fp32 out_speed;
	fp32 out_speed_set;
	fp32 out_round_cnt;
	fp32 out_angle;
	fp32 out_angle_set;
	fp32 out_origin;
	fp32 out_relative_angle;

	
	//3508的电流
	int16_t give_current; // Set current to the motor



	uint8_t motor_mode;
	uint8_t last_motor_mode;
	


	pid_type_def ENC_speed_pid; // pid for speed loop
	pid_type_def INS_speed_pid;
	pid_type_def ENC_angle_pid; // pid for angle loop
	pid_type_def INS_angle_pid;
	pid_type_def auto_aim_pid;
} gantry_motor_t;

//龙门结构体
typedef struct
{
	gantry_motor_t gantry_m3508[2];
	uint8_t mode;
	uint8_t last_mode;
} gantry_control_t;


typedef struct
{
	gantry_motor_t gimbal_m6020[2];
	uint8_t mode;
	uint8_t last_mode;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

extern gantry_control_t gantry_control;


void Gantry_Task(void const * argument);

#endif
