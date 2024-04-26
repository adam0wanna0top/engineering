#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "main.h"
#include "Gantry_Task.h"
#include "struct_typedef.h"
#include "pid.h"
#define CAN_FRIC_LIFT_MOTOR_ID 0x201
#define CAN_FRIC_RIGHT_MOTOR_ID 0x202
#define CAN_2006_M1_ID 0x203

#define CAN_SHOOT_ALL_ID 0x200
#define FRIC_CAN hcan2
#define SHOOT_CAN hcan1
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

////////////////////////////////////////////////////////////////////修改
#define Y_move_3508_motor (uint8_t)3
#define X_move_2006__motor (uint8_t)4


#define up_move_can hcan1
#define Y_move_3508_motor_ID 0x203
#define X_move_2006_motor_ID 0x204

typedef struct
{
	fp32 pos;//电机位置
    fp32 last_pos;
	fp32 vel;//电机速度
	fp32 tor;//电机扭矩

	fp32 pos_set;
	fp32 vel_set;
	fp32 tor_give;


	pid_type_def ENC_angle_pid; // pid for angle loop
	pid_type_def ENC_speed_pid; // pid for speed loop

}DM_motor_control_t;



typedef struct
{

	//3508和2006编码器的参数
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


	//3508和2006输出端段参数
	fp32 out_speed;
	fp32 out_speed_set;
	fp32 out_round_cnt;
	fp32 out_angle;
	fp32 out_angle_set;
	fp32 out_origin;
	fp32 out_relative_angle;

	
	//3508和2006的电流
	int16_t give_current; // Set current to the motor


	uint8_t motor_mode;
	uint8_t last_motor_mode;
	

	pid_type_def ENC_speed_pid; // pid for speed loop
	pid_type_def INS_speed_pid;
	pid_type_def ENC_angle_pid; // pid for angle loop
	pid_type_def INS_angle_pid;
	pid_type_def auto_aim_pid;
} up_platform_motor_t;
////////////////////////////////////////////////////////////////////




// Trigger motor (Bo Dan Lun Dian Ji)
typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	int block_time;
	int reverse_time;
	
	pid_type_def speed_pid;
	
	pid_type_def angle_pid;
} trigger_motor_t;

// Friction motor
typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	int block_time;
	int reverse_time;
	
	pid_type_def speed_pid;
	
	pid_type_def angle_pid;
} friction_motor_t;

//////////////////////////////修改的////////////////////
extern up_platform_motor_t x_move_m2006;
extern up_platform_motor_t y_move_m3508;
extern DM_motor_control_t yaw_DM;
extern DM_motor_control_t pitch_DM;
/////////////////////////////////////////////////////////


extern trigger_motor_t shoot_m2006[1];
extern friction_motor_t fric_m3508[2];

void Shoot_Task(void const * argument);

#endif
