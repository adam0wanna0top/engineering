#include "USB_Task.h"

#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "bsp_can.h"
#include "task.h"
#include "Gantry_Task.h"
#include "FreeRTOS.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#define SOF 0x3A
// #include "crc8_crc16.h"
// extern gimbal_motor_t gimbal_control.gimbal_m6020[2];
extern gimbal_control_t gimbal_control;

uint8_t txbuf[26];
static int txbuf_index;
uint32_t system_time;

vision_rx_t vision_rx;
void USB_Task(void const *argument)
{
	TIM3->CCR1=1000;
  osDelay(1000);
  while (1)
  {
		motion_frame_update();
    CDC_Transmit_FS(txbuf, sizeof(txbuf));
		// CDC_Transmit_FS("ylj\r\n",sizeof("ylj\r\n"));   
    vTaskDelay(1000);
   }
}

static void motion_frame_update(void)
{
  txbuf[0] = SOF;
  txbuf_index = 1;
	sys_time_update();
	pitch_angle_update();
	yaw_angle_update();
  velocity_update();
  angular_speed_update();
	// append_CRC8_check_sum(txbuf,sizeof(txbuf));
}

static void sys_time_update()
{
    uint32_t system_time = xTaskGetTickCount();
    txbuf[txbuf_index + 3] = system_time >> 24;
    txbuf[txbuf_index + 2] = system_time >> 16;
    txbuf[txbuf_index + 1] = system_time >> 8;
    txbuf[txbuf_index] = system_time;
    txbuf_index += 4;
}

static void pitch_angle_update()
{
    union fp32_buf_t pitch_angle_data;
    pitch_angle_data.f = gimbal_control.gimbal_m6020[1].INS_angle;

    txbuf[txbuf_index] = pitch_angle_data.buf[0];
    txbuf[txbuf_index + 1] = pitch_angle_data.buf[1];
    txbuf[txbuf_index + 2] = pitch_angle_data.buf[2];
    txbuf[txbuf_index + 3] = pitch_angle_data.buf[3];
    txbuf_index += 4;
}

static void yaw_angle_update()
{
    union fp32_buf_t yaw_angle_data;
    yaw_angle_data.f = gimbal_control.gimbal_m6020[0].INS_angle;
	
    txbuf[txbuf_index] = yaw_angle_data.buf[0];
    txbuf[txbuf_index + 1] = yaw_angle_data.buf[1];
    txbuf[txbuf_index + 2] = yaw_angle_data.buf[2];
    txbuf[txbuf_index + 3] = yaw_angle_data.buf[3];
    txbuf_index += 4;
}

static void velocity_update()
{
    union fp32_buf_t velocity_data;
    // velocity_data.f = 1;

    txbuf[txbuf_index] = 0;
    txbuf[txbuf_index + 1] = 0;
    txbuf[txbuf_index + 2] = 0;
    txbuf[txbuf_index + 3] = 0;
	
		txbuf_index += 4;
		txbuf[txbuf_index] = 0;
    txbuf[txbuf_index + 1] = 0;
    txbuf[txbuf_index + 2] = 0;
    txbuf[txbuf_index + 3] = 0;
    txbuf_index += 4;
}

static void angular_speed_update()
{
    union fp32_buf_t angular_data;
    // angular_data.f = 1;
	  txbuf[txbuf_index] = 0;
    txbuf[txbuf_index + 1] = 0;
    txbuf[txbuf_index + 2] = 0;
    txbuf[txbuf_index + 3] = 0;

    // txbuf[txbuf_index] = angular_data.buf[0];
    // txbuf[txbuf_index + 1] = angular_data.buf[1];
    // txbuf[txbuf_index + 2] = angular_data.buf[2];
    // txbuf[txbuf_index + 3] = angular_data.buf[3];
    txbuf_index += 4;
}