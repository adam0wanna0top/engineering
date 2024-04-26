#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include <stdio.h>
#include <math.h>


////////////////////////////////////达妙电机宏定义///////////////////////////////////////
#define CAN_DM_YAW_M1_RECIVE_ID 0x301 //yaw轴 master ID
#define CAN_DM_PITCH_M2_RECIVE_ID 0x302 //ptich轴 master ID

#define CAN_2006_ROLL_REVIVE_ID 0x201

#define CAN_DM_YAW_SEND_ID 0x300   //CAN ID
#define CAN_DM_PITCH_SEND_ID 0x2FF  //CAN ID

#define DM_CAN hcan1

#define DM_P_MIN -3.141593f //PMAX是一个绝对值
#define DM_P_MAX 3.141593f


#define DM_V_MIN -30.0f
#define DM_V_MAX 30.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f
#define DM_T_MIN -10.0f
#define DM_T_MAX 10.0f

#define T_MIN -18.0f
#define T_MAX 18.0f


typedef struct 
{
	int16_t id;
	int16_t state;
	int16_t p_int;
	int16_t v_int;
	int16_t t_int;
	int16_t kp_int;
	int16_t kd_int;
	fp32 pos;//电机位置
    fp32 last_pos;
	fp32 vel;//电机速度
	fp32 tor;//电机扭矩
	fp32 Kp;
	fp32 Kd;
	fp32 Tmos;
	fp32 Tcoil;
} DM_motor_measure_t;



typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void can_filter_init(void);

void DM_YAW_Enable_Motor(void);
void DM_Pitch_Enable_Motor(void);

void CAN_DM_CMD_YAW(float _torq);
void CAN_DM_CMD_PITCH(float _torq);

extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[2];
extern motor_measure_t motor_measure_shoot[3];
extern motor_measure_t motor_measure_Y_move_3508;
extern motor_measure_t motor_measure_X_move_2006;
extern motor_measure_t motor_measure_roll_2006;


extern motor_measure_t motor_measure_gantry[2];
extern DM_motor_measure_t motor_measure_DM[2];

#endif
