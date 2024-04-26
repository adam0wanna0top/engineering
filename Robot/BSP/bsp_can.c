#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gantry_Task.h"
#include "detect_task.h"
#include "Shoot_Task.h"

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}


#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


#define get_DM_motor_measure_1(ptr, data)                         \
{																  \
	(ptr)->id = ((data)[0])&0x0F;  									\
	(ptr)->state = ((data)[0])>>4;\
	(ptr)->p_int=((data)[1])<<8|((data)[2]);\
	(ptr)->v_int=(((data))[3]<<4)|((data)[4]>>4);\
	(ptr)->t_int=(((data)[4]&0xF)<<8)|(data)[5];\
	(ptr)->last_pos = (ptr)->pos;\
	(ptr)->pos = uint_to_float((ptr)->p_int, DM_P_MIN, DM_P_MAX, 16); \
	(ptr)->vel = uint_to_float((ptr)->v_int, DM_V_MIN, DM_V_MAX, 12); \
	(ptr)->tor = uint_to_float((ptr)->t_int, DM_T_MIN, DM_T_MAX, 12); \
	(ptr)->Tmos = (fp32)((data)[6]); \
	(ptr)->Tcoil = (fp32)((data)[7]); \
}


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t rx_data[8];
motor_measure_t motor_measure_chassis[4]; // four chassis motors
motor_measure_t motor_measure_gimbal[2]; // 0: gimbal yaw, 1: pitch
motor_measure_t motor_measure_shoot[3]; // 0: fric left, 1: fric right, 2: trigger

motor_measure_t motor_measure_gantry[2]; //龙门的左边5号和右边6号电机
motor_measure_t motor_measure_Y_move_3508;
motor_measure_t motor_measure_X_move_2006;

motor_measure_t motor_measure_roll_2006;

DM_motor_measure_t motor_measure_DM[2];//达妙电机：用hcan1，yaw轴1号，pitch轴2号



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &hcan1){
		switch (rx_header.StdId){
////////////////////////////////////////////////////////////////////////////////	
			//1号达妙电机，yaw轴
			case CAN_DM_YAW_M1_RECIVE_ID:{
				get_DM_motor_measure_1(&motor_measure_DM[0], rx_data);
				break;
			}

			//2号达妙电机，pitch轴
			case CAN_DM_PITCH_M2_RECIVE_ID:{
				get_DM_motor_measure_1(&motor_measure_DM[1], rx_data);
				break;
			}
			case CAN_2006_ROLL_REVIVE_ID:{
				get_motor_measure(&motor_measure_roll_2006, rx_data);
				break;
			}

////////////////////////////////////////////////////////////////////////////////	
			//Y轴的3508是0x203号电机
			case Y_move_3508_motor_ID:{
				get_motor_measure(&motor_measure_Y_move_3508, rx_data);
				detect_hook(Y_3508_MOTOR_TOE);
				break;
			}
			//X轴的2006是0x204号电机
			case X_move_2006_motor_ID:{
				get_motor_measure(&motor_measure_X_move_2006, rx_data);
				detect_hook(X_2006_MOTOR_TOE);
				break;
			}

			default:{
				break;
			}
		}
	}
	else if (hcan == &hcan2){
		switch (rx_header.StdId){
			//龙门左边5号电机can接收反馈
			case CAN_3508_M5_ID:{
				get_motor_measure(&motor_measure_gantry[0], rx_data);
				detect_hook(GANTRY_LEFT_MOTOR_TOE);
				break;
			}
			// 龙门右边6号电机can接收反馈		
			case CAN_3508_M6_ID:{
				get_motor_measure(&motor_measure_gantry[1], rx_data);
				detect_hook(GANTRY_RIGHT_MOTOR_TOE);
				break;
			}
			//地盘的4个3508电机
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID: {
				static uint8_t i = 0;
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_measure_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}
			default:{
				break;
			}
		}
	}
}


//////////////////////////////////////////达妙电机函数/////////////////////////////////////////////
void DM_YAW_Enable_Motor(void)//1号yaw轴电机初始化
{
	CAN_TxHeaderTypeDef test_msg;
	uint32_t send_mail_box;
	test_msg.StdId = CAN_DM_YAW_SEND_ID;
	test_msg.ExtId = 0;
	test_msg.IDE = 0;
	test_msg.RTR = 0;
	test_msg.DLC = 8;
		
	uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	HAL_CAN_AddTxMessage(&DM_CAN, &test_msg, Data_Enable, &send_mail_box);
}

void DM_Pitch_Enable_Motor(void)//2号pitch轴电机初始化
{
	CAN_TxHeaderTypeDef test_msg;
	uint32_t send_mail_box;
	test_msg.StdId = CAN_DM_PITCH_SEND_ID;
	test_msg.ExtId = 0;
	test_msg.IDE = 0;
	test_msg.RTR = 0;
	test_msg.DLC = 8;
		
	uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	HAL_CAN_AddTxMessage(&DM_CAN, &test_msg, Data_Enable, &send_mail_box);
}








/////////////////////////////////////////////////////////////////////DM发送函数//////////////////////////////////////////////////////
void CAN_DM_CMD_YAW(float _torq)//1号YAW轴发送
//MIT mode
{
	CAN_TxHeaderTypeDef test_msg;
	uint32_t send_mail_box;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint8_t            	DM_yaw_can_send_data[8];

	test_msg.StdId = CAN_DM_YAW_SEND_ID;
	test_msg.IDE = CAN_ID_STD;
	test_msg.RTR = CAN_RTR_DATA;
	test_msg.DLC = 0x08;
	DM_yaw_can_send_data[0] = (0 >> 8);
	DM_yaw_can_send_data[1] = 0;
	DM_yaw_can_send_data[2] = (0 >> 4);
	DM_yaw_can_send_data[3] = ((0&0xF)<<4)|(0>>8);
	DM_yaw_can_send_data[4] = 0;
	DM_yaw_can_send_data[5] = (0 >> 4);
	DM_yaw_can_send_data[6] = ((0&0xF)<<4)|(tor_tmp>>8);
	DM_yaw_can_send_data[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan1, &test_msg, DM_yaw_can_send_data, &send_mail_box);
}
void CAN_DM_CMD_PITCH(float _torq)//2号PITCH轴发送
//MIT mode
{
	CAN_TxHeaderTypeDef test_msg;
	uint32_t send_mail_box;
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint8_t            	DM_pitch_can_send_data[8];

	test_msg.StdId = CAN_DM_PITCH_SEND_ID;
	test_msg.IDE = CAN_ID_STD;
	test_msg.RTR = CAN_RTR_DATA;
	test_msg.DLC = 0x08;
	DM_pitch_can_send_data[0] = (0 >> 8);
	DM_pitch_can_send_data[1] = 0;
	DM_pitch_can_send_data[2] = (0 >> 4);
	DM_pitch_can_send_data[3] = ((0&0xF)<<4)|(0>>8);
	DM_pitch_can_send_data[4] = 0;
	DM_pitch_can_send_data[5] = (0 >> 4);
	DM_pitch_can_send_data[6] = ((0&0xF)<<4)|(tor_tmp>>8);
	DM_pitch_can_send_data[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan1, &test_msg, DM_pitch_can_send_data, &send_mail_box);
}
//////////////////////////////////////////////////////////////////////////////////////////////


