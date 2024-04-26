#ifndef _USB_TASK
#define _USB_TASK

#include "struct_typedef.h"
#include "Chassis_Task.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "INS_task.h"
// #include "referee.h"
#include "Gantry_Task.h"
#include "task.h"


static void motion_frame_update(void);
static void sys_time_update();
static void pitch_angle_update();
static void yaw_angle_update();
static void velocity_update();
static void angular_speed_update();

extern uint8_t txbuf[26];
union fp32_buf_t
{
	float f;
	unsigned char buf[4];
};

typedef __packed struct
{
	
	uint8_t SOF;
	fp32 pitch_angle;
	fp32 yaw_angle;
	uint8_t shoot_or_not;
	
}vision_rx_t;

void USB_Task(void const * argument);

#endif
