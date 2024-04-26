#ifndef _GANTRY_TASK
#define _GANTRY_TASK

#include "struct_typedef.h"
#include "pid.h"

#define CAN_GIMBAL_ALL_ID 0x1FF
#define CAN_6020_M1_ID 0x205
#define CAN_6020_M2_ID 0x206
#define GIMBAL_YAW_CAN hcan2 // gimbal yaw is hcan2 in current design
#define GIMBAL_PITCH_CAN hcan1 // gimbal yaw is hcan2 in current design
#define YAW_MOTOR_INIT_POS 4950 // WARNING: REMEBER TO MODIFY
#define PITCH_MOTOR_INIT_POS 4016 // WARNING: REMEBER TO MODIFY
#define Gimbal_Normal_Mode (uint8_t)1
#define Gimbal_No_Force (uint8_t)0
#define Gimbal_Auto_Aiming (uint8_t)2
#define Gimbal_Motor_Encoder (uint8_t)1
#define Gimbal_Motor_INS (uint8_t)2
#define Yaw (uint8_t)0
#define Pitch (uint8_t)1

typedef struct
{
	fp32 angle_set;
	fp32 INS_speed; // Actual speed from IMU
	fp32 INS_speed_set; // Set goal speed fot IMU data
	fp32 INS_angle; // Actual angle from IMU (degree) (yaw: relative, pitch: absolute)
	fp32 INS_angle_offset;
	fp32 ENC_speed; // Encoder speed (rpm -> deg/s)
	fp32 ENC_speed_set; // Set goal speed according to encoder speed (degree/s)
	fp32 ENC_angle; // Encoder angle [0,8191)
	fp32 ENC_angle_actual; // Relative angle to the initial angle (degree)
	// fp32 ENC_angle_set; // Set goal angle for the encoder
	fp32 ENC_last_ecd;
	fp32 ENC_ecd;
	fp32 ENC_round_cnt; // circle count
	fp32 origin;
	fp32 ENC_relative_angle;
	int16_t give_current; // Set current to the motor

	uint8_t motor_mode;
	uint8_t last_motor_mode;
	
	pid_type_def ENC_speed_pid; // pid for speed loop
	pid_type_def INS_speed_pid;
	pid_type_def ENC_angle_pid; // pid for angle loop
	pid_type_def INS_angle_pid;
	pid_type_def auto_aim_pid;
} gimbal_motor_t;




typedef struct
{
	gimbal_motor_t gimbal_m6020[2];
	uint8_t mode;
	uint8_t last_mode;
} gimbal_control_t;

extern gimbal_control_t gimbal_control;

void Gantry_Task(void const * argument);

#endif
