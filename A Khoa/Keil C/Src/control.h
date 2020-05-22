#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "PID.h"
#include "define.h"

#define DRIVER_PULSE_PER_REV 50000
#define DRIVER_DEGREE_TO_PULSE (float)(DRIVER_PULSE_PER_REV / 360.0f)
#define DRIVER_PULSE_TO_DEGREE (float) (360.0f / DRIVER_PULSE_PER_REV)

//**********************************************//
//**************Struct variable*****************//
//**********************************************//
typedef struct
{
//	SERVO_STATE_ENUM e_servo_state;
	PIDType st_pid_pos, st_pid_vel;
	bool servo_on_off_stt;
	int32_t	i32_pulse;
	int32_t i32_pulse_acc;
	int32_t i32_offset_pulse_forward;
	int32_t i32_offset_pulse_reverse;
	float f_offset_angle_home;
	int32_t i32_current_enc_pulse;
	float f_current_enc_angle;
	float f_current_enc_vel;
	float f_pulse;
	float f_pulse_acc;
	float f_current_angle;
	float f_current_vel;
	float f_setpoint_angle;
	float f_setpoint_vel;
	bool b_servo_home_done;
	bool is_above_home;
	float f_next_setpoint_angle;
	bool b_servo_home_reach;
}
SERVO_DATA_STRUCT;

typedef struct
{
	int32_t i32_offset_pulse_sv[6];
	uint32_t ui32_period_run_cnt;
	uint32_t ui32_period_run_total; //thoi gian di chuyen cua 6 dong co, tinh theo chu ki dieu khien
	uint32_t ui32_period_test_cnt;
	uint32_t ui32_period_test_total;
	float f_run_home_speed;
}
SERVO_RUN_DATA_STRUCT;

typedef struct
{
	float f_min_axis;
	float f_max_axis;
	float f_next_axis;
	float f_current_axis;
	float f_step_test_axis;
	float f_step_jog_axis;
	uint32_t ui32_speed_axis;
	bool b_is_fwd_move;
}
PLATFORM_COORDINATOR_DATA_STRUCT;

//**********************************************//
//**********Enumeration variable****************//
//**********************************************//
typedef enum
{
	SYSTEM_STATE_ERROR,
	SYSTEM_STATE_IDLE,
	SYSTEM_STATE_RUN,
}
SYSTEM_STATE_ENUM;

typedef enum
{
	RUN_MODE_NONE,
	RUN_MODE_HOME,
	RUN_MODE_TEST,
	RUN_MODE_JOG,
	RUN_MODE_TEACH,
	RUN_MODE_FOLLOW,
}
RUN_MODE_ENUM;

typedef enum
{
	HOME_MODE_NONE,
	HOME_MODE_SERVO_INIT,
	HOME_MODE_SERVO_POS,
	HOME_MODE_SERVO_POS_WAIT,
	HOME_MODE_SERVO,
	HOME_MODE_MODEL,
	HOME_MODE_MODEL_WAIT,
}
HOME_MODE_ENUM;

typedef enum	
{
	TEST_MODE_NONE = 0,
	TEST_MODE_PID,
	TEST_MODE_PULSE,
	TEST_MODE_X,
	TEST_MODE_X_Y_LEFT,
	TEST_MODE_X_Y_RIGHT,
	TEST_MODE_Y,
	TEST_MODE_Y_Z_LEFT,
	TEST_MODE_Y_Z_RIGHT,
	TEST_MODE_Z,
	TEST_MODE_X_Z_LEFT,
	TEST_MODE_X_Z_RIGHT,
	TEST_MODE_ROLL,
	TEST_MODE_PITCH,
	TEST_MODE_YAW,
	TEST_MODE_CIRCLE,
	TEST_MODE_SQUARE,
	TEST_MODE_ECLIPSE,
	TEST_MODE_RANDOM_1,
	TEST_MODE_RANDOM_2,
	TEST_MODE_MAX,	
}
TEST_MODE_ENUM;

typedef enum
{
	
	CONTROL_MODE_FEEDFORWARD_POS,
	CONTROL_MODE_FEEDFORWARD_POS_VEL,
	CONTROL_MODE_FEEDBACK_IMU, //Xuat xung dua tren gia tri IMU doc ve
	CONTROL_MODE_FEEDBACK_ENC, //Xuat xung dua tren gia tri ENC doc ve
	CONTROL_MODE_FEEDBACK_ENC_IMU, //Xuat xung dua tren gia tri IMU va ENC doc ve
	CONTROL_MODE_FEEDBACK_ENC_IMU_LIDAR,
	CONTROL_MODE_NONE,
	CONTROL_MODE_FEEDFORWARD, //Xuat xung thang ra drive
}
CONTROL_MODE_ENUM;

typedef enum
{
	JOG_MODE_UP_X,
	JOG_MODE_UP_Y,
	JOG_MODE_UP_Z,
	JOG_MODE_UP_LEFT_X_Y,
	JOG_MODE_UP_RIGHT_X_Y,
	JOG_MODE_UP_LEFT_X_Z,
	JOG_MODE_UP_RIGHT_X_Z,
	JOG_MODE_UP_LEFT_Y_Z,
	JOG_MODE_UP_RIGHT_Y_Z,
	JOG_MODE_UP_LEFT_X_Y_Z,
	JOG_MODE_UP_RIGHT_X_Y_Z,
	JOG_MODE_UP_ROLL,
	JOG_MODE_UP_PITCH,
	JOG_MODE_UP_YAW,
	
	JOG_MODE_DOWN_X,
	JOG_MODE_DOWN_Y,
	JOG_MODE_DOWN_Z,
	JOG_MODE_DOWN_LEFT_X_Y,
	JOG_MODE_DOWN_RIGHT_X_Y,
	JOG_MODE_DOWN_LEFT_X_Z,
	JOG_MODE_DOWN_RIGHT_X_Z,
	JOG_MODE_DOWN_LEFT_Y_Z,
	JOG_MODE_DOWN_RIGHT_Y_Z,
	JOG_MODE_DOWN_LEFT_X_Y_Z,
	JOG_MODE_DOWN_RIGHT_X_Y_Z,
	JOG_MODE_DOWN_ROLL,
	JOG_MODE_DOWN_PITCH,
	JOG_MODE_DOWN_YAW,
	
	JOG_MODE_FWD_SV_1,
	JOG_MODE_FWD_SV_2,
	JOG_MODE_FWD_SV_3,
	JOG_MODE_FWD_SV_4,
	JOG_MODE_FWD_SV_5,
	JOG_MODE_FWD_SV_6,
	
	JOG_MODE_REV_SV_1,
	JOG_MODE_REV_SV_2,
	JOG_MODE_REV_SV_3,
	JOG_MODE_REV_SV_4,
	JOG_MODE_REV_SV_5,
	JOG_MODE_REV_SV_6,
	
	JOG_MODE_NONE,
}
JOG_MODE_ENUM;

typedef enum
{
	TEACH_MODE_NONE,
	TEACH_MODE_RUN,
	TEACH_MODE_RUN_INIT,
	TEACH_MODE_HOME,
	TEACH_MODE_BACK,
	TEACH_MODE_NEXT,
	TEACH_MODE_GO,
}
TEACH_MODE_ENUM;

typedef enum
{
	CIRCLE_MODE_X_Y,
	CIRCLE_MODE_Y_Z,
	CIRCLE_MODE_X_Z,
}
CIRCLE_MODE_ENUM;

typedef enum
{
	CIRCLE_RADIUS,
	CIRCLE_SPEED,
	CIRCLE_MODE,
}
CIRCLE_MODE_DATA;

typedef enum
{
	SQUARE_MODE_X_Y,
	SQUARE_MODE_Y_Z,
	SQUARE_MODE_X_Z,
}
SQUARE_MODE_ENUM;

typedef enum
{
	SQUARE_X_FROM,
	SQUARE_Y_FROM,
	SQUARE_X_TO,
	SQUARE_Y_TO,
	SQUARE_SPEED,
	SQUARE_MODE,
}
SQUARE_MODE_DATA;

typedef enum
{
	ECLIPSE_MODE_X_Y,
	ECLIPSE_MODE_Y_Z,
	ECLIPSE_MODE_X_Z,
}
ECLIPSE_MODE_ENUM;

typedef enum
{
	ECLIPSE_A_RADIUS,
	ECLIPSE_B_RADIUS,
	ECLIPSE_SPEED,
	ECLIPSE_MODE,
}
ECLIPSE_MODE_DATA;

typedef enum
{
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS,
	ROLL_AXIS,
	PITCH_AXIS,
	YAW_AXIS,
	MAX_AXIS,
	ALL_AXIS,
	SERVO_1_AXIS,
	SERVO_2_AXIS,
	SERVO_3_AXIS,
	SERVO_4_AXIS,
	SERVO_5_AXIS,
	SERVO_6_AXIS,
	ALL_SERVO_AXIS,
	INVALID_AXIS,
}
AXIS_ENUM;

typedef enum
{
	STATE_FORWARD,
	STATE_REVERSE,
	STATE_STOP,
	STATE_HOMING_POS,
	STATE_CALCULATING,
	STATE_SERVO_RUN,
	STATE_SERVO_RUN_PULSE,
	STATE_ERROR,
	INVALID_SERVO_STATE,
}
SERVO_STATE_ENUM;

typedef enum
{
	SERVO1,
	SERVO2,
	SERVO3,
	SERVO4,
	SERVO5,
	SERVO6,
	MAX_SERVO
}
SERVO_ENUM;

typedef enum
{
	X_Y_AXIS = 0,
	X_Z_AXIS,
	Y_Z_AXIS,
	ROLL_PITCH_AXIS,
	ROLL_YAW_AXIS,
	PITCH_YAW_AXIS,
}
TEST_MODE_AXIS_ENUM;
//**********************************************//
//*********** Prototype Function ***************//
//**********************************************//
typedef void (*PULSE_HANDLER)(void);

/*****************************************************/
/***************** Init function *********************/
/*****************************************************/
void System_Init(void);
void Control_Init(void);
/*****************************************************/
/***************** Checking function *****************/
/*****************************************************/
bool Is_Axis_Home_Done(AXIS_ENUM e_axis);
void Check_Servo_Home_Pos(void);
/*****************************************************/
/*************** Update/Set/Reset function ***************/
/*****************************************************/
void Reset_Angle_Servo(SERVO_DATA_STRUCT *servo_data);
void Servo_Update_Current_Angle(SERVO_DATA_STRUCT *servo_data);
void Reset_Axis_Home_Done(SERVO_DATA_STRUCT *servo_data);
void Reset_Axis_Model(void);
void Update_Angle_Servo(SERVO_DATA_STRUCT *servo_data);
void Set_Axis_Home_Done(SERVO_DATA_STRUCT *servo_data, AXIS_ENUM e_axis);
/*****************************************************/
/**************** Calculate function *****************/
/*****************************************************/
void Calculate_Servo_Deg(SERVO_DATA_STRUCT *servo_data);
void Calculate_Servo_F_Pulse(SERVO_DATA_STRUCT *servo_data);
void Calculate_Servo_Pulse(SERVO_DATA_STRUCT *servo_data,int32_t *servo_pulse);
/*****************************************************/
/***************** Process function ******************/
/*****************************************************/
void Servo_Homing_Process(void);
void Model_Homing_Process(void);
void Servo_Run_Process(void);
void Test_Pulse_Process(void);
void Control_Feed_Forward_Process(void);
/*****************************************************/
/****************** Main function ********************/
/*****************************************************/
void System_Process(void);
void Control_Motor(void);
/*****************************************************/
/**************** Contractor function ****************/
/*****************************************************/
void Set_System_State(SYSTEM_STATE_ENUM state);
SYSTEM_STATE_ENUM Get_System_State(void);
void Set_Run_Mode(RUN_MODE_ENUM e_mode);
RUN_MODE_ENUM Get_Run_Mode(void);
void Set_Control_Mode(CONTROL_MODE_ENUM e_mode);
CONTROL_MODE_ENUM Get_Control_Mode(void);
void Set_Jog_Mode(JOG_MODE_ENUM e_mode);
JOG_MODE_ENUM Get_Jog_Mode(void);
void Set_Test_Mode(TEST_MODE_ENUM e_test_state);
TEST_MODE_ENUM Get_Test_Mode(void);
void Set_Home_Mode(HOME_MODE_ENUM e_mode);
HOME_MODE_ENUM Get_Home_Mode(void);
void Set_Servo_State(SERVO_STATE_ENUM e_servo_state);
SERVO_STATE_ENUM Get_Servo_State(void);
void Set_Teach_Mode(TEACH_MODE_ENUM e_teach_state);
TEACH_MODE_ENUM Get_Teach_Mode(void);
void Set_Pl_Coords_Data_Max_Axis(AXIS_ENUM e_axis, float value);
void Set_Pl_Coords_Data_Min_Axis(AXIS_ENUM e_axis, float value);
void Set_Pl_Coords_Data_Step_Test_Axis(AXIS_ENUM e_axis, float value);
void Set_Pl_Coords_Data_Step_Jog_Axis(AXIS_ENUM e_axis, float value);
void Set_Pl_Coords_Data_Pos(AXIS_ENUM e_axis, float value);
float Get_Pl_Coords_Data_Pos(AXIS_ENUM e_axis);
void Set_Servo_Data_Pos(SERVO_ENUM e_servo, float value);
float Get_Servo_Data_Pos(SERVO_ENUM e_servo);
void Set_Servo_Data_Vel(SERVO_ENUM e_servo, float value);
float Get_Servo_Data_Vel(SERVO_ENUM e_servo);
void Set_Servo_Data_Pos_Up(SERVO_ENUM e_servo);
void Set_Servo_Data_Pos_Down(SERVO_ENUM e_servo);
void Set_Servo_Data_STT_On_Off(SERVO_ENUM e_servo, bool value);
void Set_Speed(RUN_MODE_ENUM e_run_mode, float value);
void Set_Circle_Test_Data(CIRCLE_MODE_DATA c_data_type, float value, CIRCLE_MODE_ENUM c_mode);
void Set_Square_Test_Data(SQUARE_MODE_DATA s_data_type, float value, SQUARE_MODE_ENUM s_mode);
void Set_Eclipse_Test_Data(ECLIPSE_MODE_DATA e_data_type, float value, ECLIPSE_MODE_ENUM e_mode);
void Reset_Teach_Index(void);
/*****************************************************/
/****************** Handler function *****************/
/*****************************************************/

/*****************************************************/
/*************** Button handler function ***************/
/*****************************************************/
bool Start_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Stop_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Home_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Home_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Home_Servo_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Test_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Control_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Teach_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Follow_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Reset_Servo_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Reset_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Servo_On_Off_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
	
/*************** Button handler function in Test Mode ***************/
bool Set_Test_X_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Y_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Z_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_X_Y_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_X_Z_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Y_Z_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_X_Y_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_X_Z_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Y_Z_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Roll_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Pitch_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Yaw_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Circle_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Square_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Random_1_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Random_2_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Pulse_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Test_Eclipse_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
/*************** Button handler function in Jog Mode ***************/
bool Set_Model_Jog_Mode_X_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Z_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Z_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Roll_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Roll_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Pitch_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Pitch_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Yaw_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Yaw_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Y_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Y_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Y_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Y_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Z_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Z_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Z_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_X_Z_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Z_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Z_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Z_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Set_Model_Jog_Mode_Y_Z_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);

/*****************************************************/
/*************** Data handler function ***************/
/*****************************************************/
bool Servo_Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Servo_Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Update_Model_Parametter_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
#endif
