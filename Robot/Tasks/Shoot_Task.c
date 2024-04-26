#include "Shoot_Task.h"
#include "Gantry_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "can.h"
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



//Y轴3508位置pid
#define Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KP 30.0f
#define Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KI 0.0f
#define Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KD 1.0f
#define Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_MAX_OUT 6000.0f
#define Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_MAX_IOUT 2000.0f

//Y轴3508速度pid
#define Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KP 15.9f //15.9f 
#define Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KI 0.02f //0.02f
#define Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KD 6.6f  //6.6f
#define Y_MOVE_MOTOR_3508_ENC_SPEED_PID_MAX_OUT 6000.0f
#define Y_MOVE_MOTOR_3508_ENC_SPEED_PID_MAX_IOUT 2000.0f

//X轴2006位置pid
#define X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KP 18.0f
#define X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KI 0.0f
#define X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KD 2.0f
#define X_MOVE_MOTOR_2006_ENC_ANGLE_PID_MAX_OUT 6000.0f
#define X_MOVE_MOTOR_2006_ENC_ANGLE_PID_MAX_IOUT 2000.0f

//Y轴2006速度pid
#define X_MOVE_MOTOR_2006_ENC_SPEED_PID_KP 18.0f
#define X_MOVE_MOTOR_2006_ENC_SPEED_PID_KI 0.4f
#define X_MOVE_MOTOR_2006_ENC_SPEED_PID_KD 1.4f
#define X_MOVE_MOTOR_2006_ENC_SPEED_PID_MAX_OUT 6000.0f
#define X_MOVE_MOTOR_2006_ENC_SPEED_PID_MAX_IOUT 2000.0f


///////////////////////////////////////////////////////////////////////////////
//yaw轴DM位置pid
#define YAW_DM_ENC_ANGLE_PID_KP 18.3f
#define YAW_DM_ENC_ANGLE_PID_KI 0.02f
#define YAW_DM_ENC_ANGLE_PID_KD 7.5f
#define YAW_DM_ENC_ANGLE_PID_MAX_OUT 5.0f
#define YAW_DM_ENC_ANGLE_PID_MAX_IOUT 5.0f

//yaw轴DM速度pid
#define YAW_DM_ENC_SPEED_PID_KP 3.0f
#define YAW_DM_ENC_SPEED_PID_KI 0.00001f
#define YAW_DM_ENC_SPEED_PID_KD 0.1f
#define YAW_DM_ENC_SPEED_PID_MAX_OUT 5.0f
#define YAW_DM_ENC_SPEED_PID_MAX_IOUT 5.0f

//pitch轴DM位置pid
#define PITCH_DM_ENC_ANGLE_PID_KP 4.4f
#define PITCH_DM_ENC_ANGLE_PID_KI 0.01f
#define PITCH_DM_ENC_ANGLE_PID_KD 0.1f
#define PITCH_DM_ENC_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_DM_ENC_ANGLE_PID_MAX_IOUT 10.0f

//pitch轴DM速度pid
#define PITCH_DM_ENC_SPEED_PID_KP 0.4f
#define PITCH_DM_ENC_SPEED_PID_KI 0.000001f
#define PITCH_DM_ENC_SPEED_PID_KD 0.01f
#define PITCH_DM_ENC_SPEED_PID_MAX_OUT 10.0f
#define PITCH_DM_ENC_SPEED_PID_MAX_IOUT 10.0f

//pitch轴DM位置pid
#define pos_kp 18.0f
#define pos_ki 0.0f
#define pos_kp 2.0f
#define pos_max 6000.0f
#define pos_imax 2000.0f

//pitch轴DM速度pid
#define speed_kp 18.0f
#define speed_ki 0.4f
#define speed_kd 1.4f
#define speed_max 6000.0f
#define speed_imax 2000.0f

///////////////////////////////////////////////////////////////////////////////



__IO uint32_t uwTick_Up_Platform_Set_Point = 0;
int8_t Y_move_3508_Count_number = 0;
int8_t X_move_2006_Count_number = 0;
int8_t count_number = 0;

CAN_TxHeaderTypeDef  X_and_Y_move_tx_message;
uint8_t              X_and_Y_move_can_send_data[8];

CAN_TxHeaderTypeDef  up_robot_tx_message;
uint8_t              up_robot_can_send_data[8];

up_platform_motor_t x_move_m2006;
up_platform_motor_t y_move_m3508;
up_platform_motor_t roll_2006;
DM_motor_control_t yaw_DM;
DM_motor_control_t pitch_DM;
// DM_motor_measure_t motor_measure_DM[2];//达妙电机：用hcan1，yaw轴1号，pitch轴2号




extern uint8_t current_mode;
extern uint8_t last_mode;

extern uint8_t Mining_mode;
extern uint8_t Debug_mode;
extern uint8_t Redemption_mode;

extern uint8_t control_flag;
extern uint8_t control_open;
extern uint8_t control_close;
extern void RC_Mode_Set(void);


static void CAN_X_and_Y_CMD( int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4 )//1号达妙yaw轴，2号达妙pitch轴，3号Y轴电机3508,4号X轴电机2006
{
	uint32_t send_mail_box;
	X_and_Y_move_tx_message.StdId = 0x200;//1到4号电机
	X_and_Y_move_tx_message.IDE = CAN_ID_STD;
	X_and_Y_move_tx_message.RTR = CAN_RTR_DATA;
	X_and_Y_move_tx_message.DLC = 0x08;
	X_and_Y_move_can_send_data[0] = motor1 >> 8;
	X_and_Y_move_can_send_data[1] = motor1;
	X_and_Y_move_can_send_data[2] = motor2 >> 8;
	X_and_Y_move_can_send_data[3] = motor2;
	X_and_Y_move_can_send_data[4] = motor3 >> 8;
	X_and_Y_move_can_send_data[5] = motor3;
	X_and_Y_move_can_send_data[6] = motor4 >> 8;
	X_and_Y_move_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &X_and_Y_move_tx_message, X_and_Y_move_can_send_data, &send_mail_box);
}

static void CAN_test_CMD(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box;
	up_robot_tx_message.StdId = 0x1FF;
	up_robot_tx_message.IDE = CAN_ID_STD;
	up_robot_tx_message.RTR = CAN_RTR_DATA;
	up_robot_tx_message.DLC = 0x08;
	up_robot_can_send_data[0] = motor5 >> 8;
	up_robot_can_send_data[1] = motor5;
	up_robot_can_send_data[2] = motor6 >> 8;
	up_robot_can_send_data[3] = motor6;
	up_robot_can_send_data[4] = motor7 >> 8;
	up_robot_can_send_data[5] = motor7;
	up_robot_can_send_data[6] = motor8 >> 8;
	up_robot_can_send_data[7] = motor8;

	HAL_CAN_AddTxMessage(&SHOOT_CAN, &up_robot_tx_message, up_robot_can_send_data, &send_mail_box);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Up_Platform_Motor_Init(void)
{
	////////////////////////////////////////////////////平台X轴Y轴还有机械臂电机pid参数数组////////////////////////////////////////////////////////////////////
	//Y轴3508位置环pid数组
	const static fp32 Y_move_3508_enc_angle_pid[3] = {Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KP, Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KI, Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_KD};
	//Y轴3508速度环pid数组
	const static fp32 Y_move_3508_enc_speed_pid[3] = {Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KP, Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KI, Y_MOVE_MOTOR_3508_ENC_SPEED_PID_KD};

	//X轴2006位置环pid数组
	const static fp32 X_move_2006_enc_angle_pid[3] = {X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KP, X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KI, X_MOVE_MOTOR_2006_ENC_ANGLE_PID_KD};
	//X轴2006速度环pid数组
	const static fp32 X_move_2006_enc_speed_pid[3] = {X_MOVE_MOTOR_2006_ENC_SPEED_PID_KP, X_MOVE_MOTOR_2006_ENC_SPEED_PID_KI, X_MOVE_MOTOR_2006_ENC_SPEED_PID_KD};

	//YAW轴DM位置环pid数组
	const static fp32 yaw_DM_enc_angle_pid[3] = {YAW_DM_ENC_ANGLE_PID_KP, YAW_DM_ENC_ANGLE_PID_KI, YAW_DM_ENC_ANGLE_PID_KD};
	//YAW轴DM速度环pid数组
	const static fp32 yaw_DM_enc_speed_pid[3] = {YAW_DM_ENC_SPEED_PID_KP, YAW_DM_ENC_SPEED_PID_KI, YAW_DM_ENC_SPEED_PID_KD};

	//PITCH轴DM位置环pid数组
	const static fp32 pitch_DM_enc_angle_pid[3] = {PITCH_DM_ENC_ANGLE_PID_KP, PITCH_DM_ENC_ANGLE_PID_KI, PITCH_DM_ENC_ANGLE_PID_KD};
	//PITCH轴DM速度环pid数组
	const static fp32 pitch_DM_enc_speed_pid[3] = {PITCH_DM_ENC_SPEED_PID_KP, PITCH_DM_ENC_SPEED_PID_KI, PITCH_DM_ENC_SPEED_PID_KD};


	//PITCH轴DM位置环pid数组
	const static fp32 pos_pid[3] = {pos_kp, pos_ki, pos_kp};
	//PITCH轴DM速度环pid数组
	const static fp32 speed_pid[3] = {speed_kp, speed_ki, speed_kd};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//3508编码器的参数
	y_move_m3508.ENC_speed=0;
	y_move_m3508.ENC_angle=0;
	y_move_m3508.ENC_last_ecd=0;
	y_move_m3508.ENC_ecd=0;
	y_move_m3508.ENC_round_cnt=0;
	y_move_m3508.ENC_beginning=0;//test
	
	//3508输出端段参数
	y_move_m3508.out_speed=0;
	y_move_m3508.out_speed_set=0;
	y_move_m3508.out_angle=0;
	y_move_m3508.out_angle_set=0;
	y_move_m3508.out_round_cnt=0;
	y_move_m3508.ENC_beginning=0;//test
	y_move_m3508.out_relative_angle=0;

	//2006编码器的参数
	x_move_m2006.ENC_speed=0;
	x_move_m2006.ENC_angle=0;
	x_move_m2006.ENC_last_ecd=0;
	x_move_m2006.ENC_ecd=0;
	x_move_m2006.ENC_round_cnt=0;
	x_move_m2006.ENC_beginning=0;//test
	
	//2006输出端段参数
	x_move_m2006.out_speed=0;
	x_move_m2006.out_speed_set=0;
	x_move_m2006.out_angle=0;
	x_move_m2006.out_angle_set=0;
	x_move_m2006.out_round_cnt=0;
	x_move_m2006.ENC_beginning=0;//test
	x_move_m2006.out_relative_angle=0;

	//yaw轴0x201号达妙
	yaw_DM.pos=0;
	yaw_DM.tor=0;
	yaw_DM.vel=0;

	//pitch轴0x202号达妙
	pitch_DM.pos=0;
	pitch_DM.tor=0;
	pitch_DM.vel=0;

	//2006编码器的参数
	roll_2006.ENC_speed=0;
	roll_2006.ENC_angle=0;
	roll_2006.ENC_last_ecd=0;
	roll_2006.ENC_ecd=0;
	roll_2006.ENC_round_cnt=0;
	roll_2006.ENC_beginning=0;//test
	
	//2006输出端段参数
	roll_2006.out_speed=0;
	roll_2006.out_speed_set=0;
	roll_2006.out_angle=0;
	roll_2006.out_angle_set=0;
	roll_2006.out_round_cnt=0;
	roll_2006.ENC_beginning=0;//test
	roll_2006.out_relative_angle=0;


	//Y轴3508电机pid初始化
	PID_init(&y_move_m3508.ENC_angle_pid,PID_POSITION,Y_move_3508_enc_angle_pid,Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_MAX_OUT,Y_MOVE_MOTOR_3508_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&y_move_m3508.ENC_speed_pid,PID_POSITION,Y_move_3508_enc_speed_pid,Y_MOVE_MOTOR_3508_ENC_SPEED_PID_MAX_OUT,Y_MOVE_MOTOR_3508_ENC_SPEED_PID_MAX_IOUT);

	//X轴2006电机pid初始化
	PID_init(&x_move_m2006.ENC_angle_pid,PID_POSITION,X_move_2006_enc_angle_pid,X_MOVE_MOTOR_2006_ENC_ANGLE_PID_MAX_OUT,X_MOVE_MOTOR_2006_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&x_move_m2006.ENC_speed_pid,PID_POSITION,X_move_2006_enc_speed_pid,X_MOVE_MOTOR_2006_ENC_SPEED_PID_MAX_OUT,X_MOVE_MOTOR_2006_ENC_SPEED_PID_MAX_IOUT);

	//yaw轴DM电机pid初始化
	PID_init(&yaw_DM.ENC_angle_pid,PID_POSITION,yaw_DM_enc_angle_pid,YAW_DM_ENC_ANGLE_PID_MAX_OUT,YAW_DM_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&yaw_DM.ENC_speed_pid,PID_POSITION,yaw_DM_enc_speed_pid,YAW_DM_ENC_SPEED_PID_MAX_OUT,YAW_DM_ENC_SPEED_PID_MAX_IOUT);

	//pitch轴DM电机pid初始化
	PID_init(&pitch_DM.ENC_angle_pid,PID_POSITION,pitch_DM_enc_angle_pid,PITCH_DM_ENC_ANGLE_PID_MAX_OUT,PITCH_DM_ENC_ANGLE_PID_MAX_IOUT);
	PID_init(&pitch_DM.ENC_speed_pid,PID_POSITION,pitch_DM_enc_speed_pid,PITCH_DM_ENC_SPEED_PID_MAX_OUT,PITCH_DM_ENC_SPEED_PID_MAX_IOUT);

	
	//pitch轴DM电机pid初始化
	PID_init(&roll_2006.ENC_angle_pid,PID_POSITION,pos_pid,pos_max,pos_imax);
	PID_init(&roll_2006.ENC_speed_pid,PID_POSITION,speed_pid,speed_max,speed_imax);


}


//龙门电机位置限位初始化，需要每次上电的时候都标记一个origin值
void Up_Platform_Pos_Init(void)
{
	y_move_m3508.ENC_beginning = motor_measure_Y_move_3508.ecd / 8192.0f * 360.0f;//左边5号电机把上电时候的位置标记成origin值
	x_move_m2006.ENC_beginning = motor_measure_X_move_2006.ecd / 8192.0f * 360.0f;//右边6号电机把上电时候的位置标记成origin值
	roll_2006.ENC_beginning = motor_measure_roll_2006.ecd / 8192.0f * 360.0f;//右边6号电机把上电时候的位置标记成origin值
}


void Up_Platform_Pos_Init_Motor_Data_Update(void)
{

	//3508编码器端参数
	y_move_m3508.ENC_last_ecd = y_move_m3508.ENC_ecd;
	y_move_m3508.ENC_speed = motor_measure_Y_move_3508.speed_rpm/60.0f * 360.0f;//把电机RPM多少转每分钟换成多少°每秒。
	y_move_m3508.ENC_ecd = motor_measure_Y_move_3508.ecd;//测出当前位置并且赋给ecd
	y_move_m3508.ENC_angle = (motor_measure_Y_move_3508.ecd / 8192.0f * 360.0f + y_move_m3508.ENC_round_cnt * 360.0f);

	//3508输出端参数，传动比19:1
	y_move_m3508.out_speed = 0.052631578947368f * y_move_m3508.ENC_speed;
	y_move_m3508.out_angle = 0.052631578947368f * y_move_m3508.ENC_angle;	
	y_move_m3508.out_origin = 0.052631578947368f * y_move_m3508.ENC_beginning;

	//2006编码器端参数
	x_move_m2006.ENC_last_ecd = x_move_m2006.ENC_ecd;
	x_move_m2006.ENC_speed = motor_measure_X_move_2006.speed_rpm/60.0f * 360.0f;//把电机RPM多少转每分钟换成多少°每秒。
	x_move_m2006.ENC_ecd = motor_measure_X_move_2006.ecd;//测出当前位置并且赋给ecd
	x_move_m2006.ENC_angle = (motor_measure_X_move_2006.ecd / 8192.0f * 360.0f + x_move_m2006.ENC_round_cnt * 360.0f);

	//2006输出端参数，传动比36:1
	x_move_m2006.out_speed = 0.027777777777778f * x_move_m2006.ENC_speed;
	x_move_m2006.out_angle = 0.027777777777778f * x_move_m2006.ENC_angle;	
	x_move_m2006.out_origin = 0.027777777777778f * x_move_m2006.ENC_beginning;

	//yaw轴DM位置和速度
	yaw_DM.pos = motor_measure_DM[0].pos;
	yaw_DM.vel = motor_measure_DM[0].vel;

	//pitch轴DM位置和速度
	pitch_DM.pos = motor_measure_DM[1].pos;
	pitch_DM.vel = motor_measure_DM[1].vel;

	//2006编码器端参数
	roll_2006.ENC_last_ecd = roll_2006.ENC_ecd;
	roll_2006.ENC_speed = motor_measure_roll_2006.speed_rpm/60.0f * 360.0f;//把电机RPM多少转每分钟换成多少°每秒。
	roll_2006.ENC_ecd = motor_measure_roll_2006.ecd;//测出当前位置并且赋给ecd
	roll_2006.ENC_angle = (motor_measure_roll_2006.ecd / 8192.0f * 360.0f + roll_2006.ENC_round_cnt * 360.0f);

	//2006输出端参数，传动比36:1
	roll_2006.out_speed = 0.027777777777778f * roll_2006.ENC_speed;
	roll_2006.out_angle = 0.027777777777778f * roll_2006.ENC_angle;	
	roll_2006.out_origin = 0.027777777777778f * roll_2006.ENC_beginning;




	if((uwTick -  uwTick_Up_Platform_Set_Point)<2800)//1680限制的时间
	{
		CAN_DM_CMD_PITCH(0);
		CAN_DM_CMD_YAW(0);
		yaw_DM.pos_set = yaw_DM.pos;
		pitch_DM.pos_set = pitch_DM.pos;


		
		y_move_m3508.out_angle = y_move_m3508.out_origin;
		y_move_m3508.ENC_beginning = y_move_m3508.ENC_angle;

		x_move_m2006.out_angle = x_move_m2006.out_origin;
		x_move_m2006.ENC_beginning = x_move_m2006.ENC_angle;

		roll_2006.out_angle = roll_2006.out_origin;
		roll_2006.ENC_beginning = roll_2006.ENC_angle;

		//CAN_Gantry_CMD(3000,-3000,0,0);//5号电机和6号电机发送函数
		//vTaskDelay(2400);
	}
	else
	{
		y_move_m3508.out_relative_angle = y_move_m3508.out_angle - y_move_m3508.out_origin;

		x_move_m2006.out_relative_angle = x_move_m2006.out_angle - x_move_m2006.out_origin;

		roll_2006.out_relative_angle = roll_2006.out_angle - roll_2006.out_origin;
	}
	

	//3508零点检测，严格来说是计算跨过了几次零点，out_round_cnt是输出的圈数累计
	if (y_move_m3508.ENC_ecd - y_move_m3508.ENC_last_ecd > 6000)//逆时针转自减
	{
		y_move_m3508.ENC_round_cnt--;
		Y_move_3508_Count_number--;
		if(Y_move_3508_Count_number <= -19)
		{
			y_move_m3508.out_round_cnt--;
			Y_move_3508_Count_number = 0;
		}
	}
	if (y_move_m3508.ENC_ecd - y_move_m3508.ENC_last_ecd < -6000)//顺时针转自减
	{
		y_move_m3508.ENC_round_cnt++;
		Y_move_3508_Count_number++;
		if(Y_move_3508_Count_number >= 19)
		{
			y_move_m3508.out_round_cnt++;
			Y_move_3508_Count_number = 0;
		}
	}


	//2006零点检测，严格来说是计算跨过了几次零点，out_round_cnt是输出的圈数累计
	if (x_move_m2006.ENC_ecd - x_move_m2006.ENC_last_ecd > 6000)//逆时针转自减
	{
		x_move_m2006.ENC_round_cnt--;
		X_move_2006_Count_number--;
		if(X_move_2006_Count_number <= -19)
		{
			x_move_m2006.out_round_cnt--;
			X_move_2006_Count_number = 0;
		}
	}
	if (x_move_m2006.ENC_ecd - x_move_m2006.ENC_last_ecd < -6000)//顺时针转自减
	{
		x_move_m2006.ENC_round_cnt++;
		X_move_2006_Count_number++;
		if(X_move_2006_Count_number >= 19)
		{
			x_move_m2006.out_round_cnt++;
			X_move_2006_Count_number = 0;
		}
	}

		//2006零点检测，严格来说是计算跨过了几次零点，out_round_cnt是输出的圈数累计
	if (roll_2006.ENC_ecd - roll_2006.ENC_last_ecd > 6000)//逆时针转自减
	{
		roll_2006.ENC_round_cnt--;
		count_number--;
		if(count_number <= -19)
		{
			roll_2006.out_round_cnt--;
			count_number = 0;
		}
	}
	if (roll_2006.ENC_ecd - roll_2006.ENC_last_ecd < -6000)//顺时针转自减
	{
		roll_2006.ENC_round_cnt++;
		count_number++;
		if(count_number >= 19)
		{
			roll_2006.out_round_cnt++;
			count_number = 0;
		}
	}

}


void Up_Platform_PID_Calculator(void)//上平台位置闭环
{	
	//Y轴3508位置环和速度环
	PID_calc( &y_move_m3508.ENC_angle_pid, y_move_m3508.out_relative_angle, y_move_m3508.out_angle_set );
	y_move_m3508.out_speed_set = y_move_m3508.ENC_angle_pid.out;
	PID_calc( &y_move_m3508.ENC_speed_pid, y_move_m3508.out_speed, y_move_m3508.out_speed_set );
	y_move_m3508.give_current = y_move_m3508.ENC_speed_pid.out;

	//X轴2006位置环和速度环
	PID_calc( &x_move_m2006.ENC_angle_pid, x_move_m2006.out_relative_angle, x_move_m2006.out_angle_set );
	x_move_m2006.out_speed_set = x_move_m2006.ENC_angle_pid.out;
	PID_calc( &x_move_m2006.ENC_speed_pid, x_move_m2006.out_speed, x_move_m2006.out_speed_set );
	x_move_m2006.give_current = x_move_m2006.ENC_speed_pid.out;

	//yaw轴DM
	PID_calc( &yaw_DM.ENC_angle_pid, yaw_DM.pos, yaw_DM.pos_set );
	yaw_DM.vel_set = yaw_DM.ENC_angle_pid.out;
	PID_calc( &yaw_DM.ENC_speed_pid, yaw_DM.vel, yaw_DM.vel_set );
	yaw_DM.tor_give = yaw_DM.ENC_speed_pid.out;

	//pitch轴DM
	PID_calc( &pitch_DM.ENC_angle_pid, pitch_DM.pos, pitch_DM.pos_set );
	pitch_DM.vel_set = pitch_DM.ENC_angle_pid.out;
	PID_calc( &pitch_DM.ENC_speed_pid, pitch_DM.vel, pitch_DM.vel_set );
	pitch_DM.tor_give = pitch_DM.ENC_speed_pid.out;

	//X轴2006位置环和速度环
	PID_calc( &roll_2006.ENC_angle_pid, roll_2006.out_relative_angle, roll_2006.out_angle_set );
	roll_2006.out_speed_set = roll_2006.ENC_angle_pid.out;
	PID_calc( &roll_2006.ENC_speed_pid, roll_2006.out_speed, roll_2006.out_speed_set );
	roll_2006.give_current = roll_2006.ENC_speed_pid.out;

}


void Up_Platform_Motor_Control(void)
{
	if( control_flag == control_close )
	{
		y_move_m3508.give_current = 0.0f;
		x_move_m2006.give_current = 0.0f;
		yaw_DM.tor_give = 0.0f;
		pitch_DM.tor_give = 0.0f;
		roll_2006.give_current = 0.0f;
	}

	else if( control_flag == control_open && current_mode == Debug_mode)
	{
		if(yaw_DM.pos>=-4.7f)
		{
			yaw_DM.pos_set = -4.7f;
		}
		if(yaw_DM.pos<=-5.7f)
		{
			yaw_DM.pos_set = -5.7f;
		}
		if(pitch_DM.pos>=-0.1f)
		{
			pitch_DM.pos_set = -0.1f;
		}
		if(pitch_DM.pos<=-3.3f)
		{
			pitch_DM.pos_set = -3.3f;
		}


		//yaw轴DM电机
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
		{
			yaw_DM.pos_set += 0.001f;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
		{
			yaw_DM.pos_set -= 0.001f;
		}

		//pitch轴DM电机
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
		{
			pitch_DM.pos_set += 0.001f;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
		{
			pitch_DM.pos_set -= 0.001f;
		}

		//roll轴DM电机
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
		{
			roll_2006.out_angle_set += 1.0f;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
		{
			roll_2006.out_angle_set -= 1.0f;
		}


		//Y轴3508电机
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
		{
			y_move_m3508.out_angle_set += 0.1f;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)
		{
			y_move_m3508.out_angle_set -= 0.1f;
		}


		//X轴2006电机
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)
		{
			x_move_m2006.out_angle_set -= 0.1f;
		}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)
		{
			x_move_m2006.out_angle_set += 0.1f;
		}
		Up_Platform_PID_Calculator();
	}
	else
	{
		y_move_m3508.give_current = 0.0f;
		x_move_m2006.give_current = 0.0f;
		yaw_DM.tor_give = 0.0f;
		pitch_DM.tor_give = 0.0f;
		roll_2006.out_angle_set = 0.0f;

	}
}

void Gas_Control(void)
{
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
	{
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_13);
	}
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
	{
		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_14);
	}
	
}



void Shoot_Task(void const * argument)
{
		
	DM_YAW_Enable_Motor();
	DM_Pitch_Enable_Motor();
	
	Up_Platform_Motor_Init();
	Up_Platform_Pos_Init();
	vTaskDelay(200);
	uwTick_Up_Platform_Set_Point = uwTick;
	
	while(1)
	{
		Gas_Control();
		Up_Platform_Pos_Init_Motor_Data_Update();	
		Up_Platform_Motor_Control();
		CAN_X_and_Y_CMD( roll_2006.give_current, 0,y_move_m3508.give_current,x_move_m2006.give_current);
		CAN_DM_CMD_YAW(yaw_DM.tor_give);
		CAN_DM_CMD_PITCH(pitch_DM.tor_give);

		vTaskDelay(2);
	}
}















































// CAN_TxHeaderTypeDef  shoot_tx_message;
// uint8_t              shoot_can_send_data[8];

// trigger_motor_t shoot_m2006[1];
// friction_motor_t fric_m3508[2];

// uint8_t fric_mode=0;//0:stop,1:start,2:on

// fp32 fabs_1( fp32 temp_val )  // abs
// {
// 	if( temp_val < 0.0f )
// 		return -temp_val;
// 	return temp_val;
// }

// static void CAN_Fric_CMD( int16_t motor1, int16_t motor2, int16_t motor3 )
// {
// 	uint32_t send_mail_box;
// 	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
// 	shoot_tx_message.IDE = CAN_ID_STD;
// 	shoot_tx_message.RTR = CAN_RTR_DATA;
// 	shoot_tx_message.DLC = 0x08;
// 	shoot_can_send_data[0] = motor1 >> 8;
// 	shoot_can_send_data[1] = motor1;
// 	shoot_can_send_data[2] = motor2 >> 8;
// 	shoot_can_send_data[3] = motor2;
// 	shoot_can_send_data[4] = motor3 >> 8;
// 	shoot_can_send_data[5] = motor3;

// 	HAL_CAN_AddTxMessage(&FRIC_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
// }

// static void CAN_Shoot_CMD( int16_t motor1, int16_t motor2, int16_t motor3 )
// {
// 	uint32_t send_mail_box;
// 	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
// 	shoot_tx_message.IDE = CAN_ID_STD;
// 	shoot_tx_message.RTR = CAN_RTR_DATA;
// 	shoot_tx_message.DLC = 0x08;
// 	shoot_can_send_data[0] = motor1 >> 8;
// 	shoot_can_send_data[1] = motor1;
// 	shoot_can_send_data[2] = motor2 >> 8;
// 	shoot_can_send_data[3] = motor2;
// 	shoot_can_send_data[4] = motor3 >> 8;
// 	shoot_can_send_data[5] = motor3;

// 	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
// }

// void Shoot_Motor_Init(void)
// {
// 	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
// 	const static fp32 fric_motor_speed_pid[3] = {FRIC_MOTOR_SPEED_PID_KP, FRIC_MOTOR_SPEED_PID_KI, FRIC_MOTOR_SPEED_PID_KD};
	
// 	shoot_m2006[0].speed=0;
// 	shoot_m2006[0].speed_set=0;
// 	shoot_m2006[0].angle=0;
// 	shoot_m2006[0].angle_set=0;
// 	shoot_m2006[0].ENC_angle=0;
// 	shoot_m2006[0].give_current=0;
	
// 	PID_init(&shoot_m2006[0].speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	

// 	for(uint8_t i=0;i<2;i++)
// 	{
// 		fric_m3508[i].speed=0;
// 		fric_m3508[i].speed_set=0;
// 		fric_m3508[i].angle=0;
// 		fric_m3508[i].angle_set=0;
// 		fric_m3508[i].ENC_angle=0;
// 		fric_m3508[i].give_current=0;
// 		PID_init(&fric_m3508[i].speed_pid,PID_POSITION,fric_motor_speed_pid,FRIC_MOTOR_SPEED_PID_MAX_OUT,FRIC_MOTOR_SPEED_PID_MAX_IOUT);
// 	}		
// }

// void Shoot_Motor_Data_Update(void)
// {
// 	static fp32 speed_fliter_1 = 0.0f;
//   	static fp32 speed_fliter_2 = 0.0f;
//   	static fp32 speed_fliter_3 = 0.0f;
	
// 	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};


//   	speed_fliter_1 = speed_fliter_2;
//  	speed_fliter_2 = speed_fliter_3;
//   	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (motor_measure_shoot[2].speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
//   	shoot_m2006[0].speed = speed_fliter_3;
// 	shoot_m2006[0].ENC_angle = motor_measure_shoot[2].ecd;
// //	shoot_m2006[0].angle=motor_measure_gimbal[1].ecd;	

	
// 	fric_m3508[0].speed = motor_measure_shoot[0].speed_rpm;
// 	fric_m3508[0].ENC_angle = motor_measure_shoot[0].ecd;
	
// 	fric_m3508[1].speed = motor_measure_shoot[1].speed_rpm;
// 	fric_m3508[1].ENC_angle = motor_measure_shoot[1].ecd;
// }

// void Dial_Motor_Control(void)
// {
	
// 	if (shoot_m2006[0].block_time < BLOCK_TIME)
//     {
//         shoot_m2006[0].speed_set = -10;
//     }
//     else
//     {
//         shoot_m2006[0].speed_set = 10;
//     }

//     if (fabs_1(shoot_m2006[0].speed) < BLOCK_TRIGGER_SPEED && shoot_m2006[0].block_time < BLOCK_TIME)
//     {
//         shoot_m2006[0].block_time++;
//         shoot_m2006[0].reverse_time = 0;
//     }
//     else if (shoot_m2006[0].block_time == BLOCK_TIME && shoot_m2006[0].reverse_time < REVERSE_TIME)
//     {
//         shoot_m2006[0].reverse_time++;
//     }
//     else
//     {
//         shoot_m2006[0].block_time = 0;
//     }

// 	PID_calc(&shoot_m2006[0].speed_pid,shoot_m2006[0].speed,shoot_m2006[0].speed_set);
// 	shoot_m2006[0].give_current=-shoot_m2006[0].speed_pid.out;
// 	// if( rc_ctrl.rc.s[1] != RC_SW_UP )
// 		shoot_m2006[0].give_current = 0;
// }

// void Fric_Motor_Control(void)
// {
// 	if(rc_ctrl.rc.s[1]==RC_SW_UP)
// 	{
// 		//fric_m3508[0].give_current = -1000;
// 		//fric_m3508[1].give_current = 1000;
		
// 		PID_calc( &fric_m3508[1].speed_pid, fric_m3508[1].speed, 10000 );
// 		fric_m3508[1].give_current = fric_m3508[1].speed_pid.out;
// 		PID_calc( &fric_m3508[0].speed_pid, fric_m3508[0].speed, 10000 );
// 		fric_m3508[0].give_current = -fric_m3508[0].speed_pid.out;
// 	}
// 	else
// 	{
// 		PID_calc( &fric_m3508[1].speed_pid, fric_m3508[1].speed, 0 );
// 		fric_m3508[1].give_current = fric_m3508[1].speed_pid.out;
// 		PID_calc( &fric_m3508[0].speed_pid, fric_m3508[0].speed, 0 );
// 		fric_m3508[0].give_current = -fric_m3508[0].speed_pid.out;
// 	}
// }

// void Shoot_Task(void const * argument)
// {
// 	Shoot_Motor_Init();
// 	vTaskDelay(200);
	
// 	while(1)
// 	{
// 		Shoot_Motor_Data_Update();
				
// 		Fric_Motor_Control();
// 		Dial_Motor_Control();
// 		// CAN_Fric_CMD( fric_m3508[0].give_current, fric_m3508[1].give_current, 0 );
// 		// CAN_Shoot_CMD( 0, 0, shoot_m2006[0].give_current );
// 		vTaskDelay(2);
// 	}
// }
