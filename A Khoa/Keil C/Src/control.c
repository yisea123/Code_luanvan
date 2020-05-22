#include <math.h>
#include "stm32f4xx.h"
#include "define.h"
#include "control.h"
#include "utils.h"
#include "system_timetick.h"
#include "FSMC.h"
#include "LED.h"
#include "GPIO.h"
#include "UART.h"
#include "CALCULATOR.h"
#include <string.h>
#include "stdlib.h"
#include "PATH_PLANNING.h"
#include "hmi_handler.h"
#include "params.h"

// Enumeration variable
SYSTEM_STATE_ENUM e_current_system_state = SYSTEM_STATE_IDLE;
RUN_MODE_ENUM e_current_run_mode = RUN_MODE_NONE;
HOME_MODE_ENUM e_current_home_mode = HOME_MODE_NONE;
TEST_MODE_ENUM e_current_test_mode = TEST_MODE_NONE;
JOG_MODE_ENUM e_current_jog_mode = JOG_MODE_NONE;
SERVO_STATE_ENUM e_current_servo_state = STATE_STOP;
CONTROL_MODE_ENUM e_current_control_mode = CONTROL_MODE_NONE;
TEACH_MODE_ENUM e_current_teach_mode = TEACH_MODE_NONE;

CIRCLE_MODE_ENUM e_current_circle_mode = CIRCLE_MODE_X_Y;
SQUARE_MODE_ENUM e_current_square_mode = SQUARE_MODE_X_Y;
ECLIPSE_MODE_ENUM e_current_eclipse_mode = ECLIPSE_MODE_X_Y;

// Struct variable
SERVO_DATA_STRUCT st_servo_data[MAX_SERVO];
PLATFORM_COORDINATOR_DATA_STRUCT st_pl_coords_data[MAX_AXIS];
SERVO_RUN_DATA_STRUCT st_servo_run_data;

int32_t servo_pulse[MAX_SERVO];
float pl_coords_buffer_teach[MAX_AXIS][100] = {{0}};
float pl_coords_buffer_follow[MAX_AXIS][5] = {{0}};
uint32_t pl_coords_buffer_teach_index = 0;
uint32_t pl_coords_buffer_teach_index_max = 100;
uint32_t pl_coords_buffer_teach_current_index = 0;
uint32_t pl_coords_buffer_present_teach_index = 0;
uint32_t pl_coords_buffer_follow_index = 0;
bool b_is_teach_run_lap = false;

bool b_is_new_pl_coords_data = false;

float T_Circle_Speed = F_CTRL;
float T_Eclipse_Speed = F_CTRL;
float T_Square_Speed = F_CTRL;

uint32_t Circle_index = 0; 
uint32_t Eclipse_index = 0;
uint32_t Square_index = 0;

float Circle_Test_Radius = 10.0f;
float Eclipse_Test_A_Radius = 10.0f;
float Eclipse_Test_B_Radius = 10.0f;

float Square_Test_X_From = -10.0f;
float Square_Test_Y_From = -10.0f;
float Square_Test_X_To = 10.0f;
float Square_Test_Y_To = 10.0f;

bool first_circle_test_move = true;
bool first_square_test_move = true;
bool first_eclipse_test_move = true;
bool first_one_test_move = true;

bool bt_back_teach_press = false;
bool bt_teach_press = false;
bool bt_next_teach_press = false;
bool bt_home_teach_press = false;
bool bt_stop_teach_press = false;

extern bool b_first_cycle; 
extern bool b_first_cycle_one_axis_trajectory; 
extern bool b_first_cycle_two_axis_trajectory;

uint32_t home_mode_speed = F_CTRL;
uint32_t jog_mode_speed = F_CTRL/2;
uint32_t test_mode_speed = F_CTRL/2;
uint32_t teach_mode_speed = F_CTRL;
uint32_t follow_mode_speed = F_CTRL;
/*****************************************************/
/****************** Init function ********************/
/*****************************************************/
void System_Init(void)
{
	Set_System_State(SYSTEM_STATE_IDLE);
}

void Control_Init(void)
{
	st_servo_run_data.i32_offset_pulse_sv[SERVO1] = 1;
	st_servo_run_data.i32_offset_pulse_sv[SERVO2] = 1;
	st_servo_run_data.i32_offset_pulse_sv[SERVO3] = 2;
	st_servo_run_data.i32_offset_pulse_sv[SERVO4] = 1;
	st_servo_run_data.i32_offset_pulse_sv[SERVO5] = 1;
	st_servo_run_data.i32_offset_pulse_sv[SERVO6] = 1;
	
	st_servo_run_data.ui32_period_run_total = F_CTRL;
	st_servo_run_data.ui32_period_test_total = 5000;
	st_servo_run_data.f_run_home_speed = 2;
	st_servo_data[SERVO1].f_offset_angle_home = -6.6;
	st_servo_data[SERVO2].f_offset_angle_home = 4.9;
	st_servo_data[SERVO3].f_offset_angle_home = 2.9;
	st_servo_data[SERVO4].f_offset_angle_home = -1.6;
	st_servo_data[SERVO5].f_offset_angle_home = -4;
	st_servo_data[SERVO6].f_offset_angle_home = 3.6;
	
	Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
}
/*****************************************************/
/**************** Checking function ******************/
/*****************************************************/
bool Is_Axis_Home_Done(AXIS_ENUM e_axis)
{
	switch((int)e_axis)
	{
		case SERVO_1_AXIS:
			return st_servo_data[SERVO1].b_servo_home_done;
			break;
		case SERVO_2_AXIS:
			return st_servo_data[SERVO2].b_servo_home_done;
			break;
		case SERVO_3_AXIS:
			return st_servo_data[SERVO3].b_servo_home_done;
			break;
		case SERVO_4_AXIS:
			return st_servo_data[SERVO4].b_servo_home_done;
			break;
		case SERVO_5_AXIS:
			return st_servo_data[SERVO5].b_servo_home_done;
			break;
		case SERVO_6_AXIS:
			return st_servo_data[SERVO6].b_servo_home_done;
			break;
		case ALL_SERVO_AXIS:
			return 	st_servo_data[SERVO1].b_servo_home_done &
							st_servo_data[SERVO2].b_servo_home_done &
							st_servo_data[SERVO3].b_servo_home_done &
							st_servo_data[SERVO4].b_servo_home_done &
							st_servo_data[SERVO5].b_servo_home_done &
							st_servo_data[SERVO6].b_servo_home_done;
			break;
	}
	return false;
}



void Check_Servo_Home_Pos(void)
{
	// Servo 1
	if(Get_Servo_Home_Status(SERVO1))
	{
		st_servo_data[SERVO1].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO1].is_above_home = false;
	}
	// Servo 2
	if(Get_Servo_Home_Status(SERVO2))
	{
		st_servo_data[SERVO2].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO2].is_above_home = false;
	}
	// Servo 3
	if(Get_Servo_Home_Status(SERVO3))
	{
		st_servo_data[SERVO3].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO3].is_above_home = false;
	}
	// Servo 4
	if(Get_Servo_Home_Status(SERVO4))
	{
		st_servo_data[SERVO4].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO4].is_above_home = false;
	}
	// Servo 5
	if(Get_Servo_Home_Status(SERVO5))
	{
		st_servo_data[SERVO5].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO5].is_above_home = false;
	}
	// Servo 6
	if(Get_Servo_Home_Status(SERVO6))
	{
		st_servo_data[SERVO6].is_above_home = true;
	}
	else 
	{
		st_servo_data[SERVO6].is_above_home = false;
	}
	
}
/*****************************************************/
/************* Update/Set/Reset function *************/
/*****************************************************/
/*-----------------------------------------------  
	Reset_Angle_Servo(): 
	Used to reset "f_current_angle, f_setpoint_angle" in "st_servo_data"
  -----------------------------------------------*/
void Reset_Angle_Servo(SERVO_DATA_STRUCT *servo_data)
{
	for(int i = 0; i < MAX_SERVO; i++)
	{
		servo_data[i].f_current_angle = 0;
		servo_data[i].f_setpoint_angle = 0;
		servo_data[i].f_pulse = 0;
		servo_data[i].f_pulse_acc = 0;
		servo_data[i].i32_pulse_acc = 0;
	}
	FSMC_ENC_Reset();
}
/*-----------------------------------------------  
	Reset_Axis_Home_Done(): 
	Used to reset "b_servo_home_done" in "st_servo_data"
  -----------------------------------------------*/
void Reset_Axis_Home_Done(SERVO_DATA_STRUCT *servo_data)
{
	int i;
	for(i = 0; i < MAX_SERVO; i++)
	{
		servo_data[i].b_servo_home_done = false;
	}
}
/*-----------------------------------------------  
	Reset_Servo_Home_Reach_Stt(): 
	Used to reset "b_servo_home_reach" in "st_servo_data"
  -----------------------------------------------*/
void Reset_Servo_Home_Reach_Stt(void)
{
	for(int i = 0; i < MAX_SERVO; i++)
	{
		st_servo_data[i].b_servo_home_reach = false;
	}
}
/*-----------------------------------------------  
	Reset_Axis_Model(): 
	Used to reset "f_next_?_axis, f_current_?_axis, b_is_fwd_move_?" in "st_pl_coords_data"
  -----------------------------------------------*/
void Reset_Axis_Model(void)
{
	for(int i = 0; i < MAX_AXIS; i++)
	{
		st_pl_coords_data[i].f_next_axis = 0;
		st_pl_coords_data[i].f_current_axis = 0;
		st_pl_coords_data[i].b_is_fwd_move = false;
	}
}
/*-----------------------------------------------  
	Update_Angle_Servo(): 
	Used to update "f_current_angle" in "st_servo_data"
  -----------------------------------------------*/
void Update_Angle_Servo(SERVO_DATA_STRUCT *servo_data)
{
	for(int i = 0; i < MAX_SERVO; i++)
	{
		servo_data[i].f_current_angle = (float)(servo_data[i].i32_pulse_acc * DRIVER_PULSE_TO_DEGREE);
	}
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_1, (int32_t)(servo_data[SERVO1].f_current_angle*10));
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_2, (int32_t)(servo_data[SERVO2].f_current_angle*10));
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_3, (int32_t)(servo_data[SERVO3].f_current_angle*10));
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_4, (int32_t)(servo_data[SERVO4].f_current_angle*10));
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_5, (int32_t)(servo_data[SERVO5].f_current_angle*10));
//	HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_6, (int32_t)(servo_data[SERVO6].f_current_angle*10));
	
}
void Update_Angle_Enc_Servo(SERVO_DATA_STRUCT *servo_data)
{
	
}
/*-----------------------------------------------  
	Set_Axis_Home_Done(): 
	Used to set "b_servo_home_done" in "st_servo_data"
  -----------------------------------------------*/
void Set_Axis_Home_Done(SERVO_DATA_STRUCT *servo_data, AXIS_ENUM e_axis)
{
	switch((int)e_axis)
	{
		case SERVO_1_AXIS:
			servo_data[SERVO1].b_servo_home_done = true;
			break;
		case SERVO_2_AXIS:
			servo_data[SERVO2].b_servo_home_done = true;
			break;
		case SERVO_3_AXIS:
			servo_data[SERVO3].b_servo_home_done = true;
			break;
		case SERVO_4_AXIS:
			servo_data[SERVO4].b_servo_home_done = true;
			break;
		case SERVO_5_AXIS:
			servo_data[SERVO5].b_servo_home_done = true;
			break;
		case SERVO_6_AXIS:
			servo_data[SERVO6].b_servo_home_done = true;
			break;
		case ALL_SERVO_AXIS:
			servo_data[SERVO1].b_servo_home_done = true;
			servo_data[SERVO2].b_servo_home_done = true;
			servo_data[SERVO3].b_servo_home_done = true;
			servo_data[SERVO4].b_servo_home_done = true;
			servo_data[SERVO5].b_servo_home_done = true;
			servo_data[SERVO6].b_servo_home_done = true;
			break;
	}
}
/*****************************************************/
/**************** Calculate function *****************/
/*****************************************************/
/*-----------------------------------------------  
	Calculate_Servo_Deg(): 
	Used to calculate "f_setpoint_angle" in "st_servo_data"
  -----------------------------------------------*/
void Calculate_Servo_Deg(SERVO_DATA_STRUCT *servo_data)
{
	Cal_Servo_Degree(	st_pl_coords_data[X_AXIS].f_next_axis,
										st_pl_coords_data[Y_AXIS].f_next_axis,
										st_pl_coords_data[Z_AXIS].f_next_axis,
										st_pl_coords_data[ROLL_AXIS].f_next_axis*DEG2RAG,
										st_pl_coords_data[PITCH_AXIS].f_next_axis*DEG2RAG,
										st_pl_coords_data[YAW_AXIS].f_next_axis*DEG2RAG,
										servo_data);
}
/*-----------------------------------------------  
	Calculate_Servo_F_Pulse(): 
	Used to calculate "f_pulse" in "st_servo_data"
  -----------------------------------------------*/
void Calculate_Servo_F_Pulse(SERVO_DATA_STRUCT *servo_data)
{
	for(int i = 0; i < MAX_SERVO; i++)
	{
		servo_data[i].f_pulse = (float)(( servo_data[i].f_setpoint_angle - 
																			servo_data[i].f_current_angle ) *
															DRIVER_DEGREE_TO_PULSE / st_servo_run_data.ui32_period_run_total);
	}
}
/*-----------------------------------------------  
	Calculate_Servo_Pulse(): 
	Used to calculate "i32_pulse" in "st_servo_data"
  -----------------------------------------------*/
void Calculate_Servo_Pulse(SERVO_DATA_STRUCT *servo_data,int32_t *servo_pulse)
{
	for (int i=0;i<MAX_SERVO;i++)
	{
		servo_data[i].f_pulse_acc += servo_data[i].f_pulse;															
		servo_data[i].i32_pulse = (int32_t)(servo_data[i].f_pulse_acc
											- servo_data[i].i32_pulse_acc) ;
		servo_data[i].i32_pulse_acc += servo_data[i].i32_pulse;
		if(st_servo_data[i].b_servo_home_reach)
		{
			servo_pulse[i] = 0;
		}
		else
		{
			servo_pulse[i] = servo_data[i].i32_pulse * st_servo_run_data.i32_offset_pulse_sv[i];
		}
	}
}

/*****************************************************/
/**************** Process function *******************/
/*****************************************************/
void Test_Pulse_Process(void)
{
	int32_t i_motor_pulse[6];
	st_servo_run_data.ui32_period_test_cnt++;
	if(st_servo_run_data.ui32_period_test_cnt < st_servo_run_data.ui32_period_test_total)
	{
		i_motor_pulse[0] = 1; //i_motor_1_pulse+10;
		i_motor_pulse[1] = -1; //i_motor_2_pulse-10;
		i_motor_pulse[2] = 2; //i_motor_3_pulse+20;
		i_motor_pulse[3] = -1; //i_motor_4_pulse-10;
		i_motor_pulse[4] = 1; //i_motor_5_pulse+10;
		i_motor_pulse[5] = -1; //i_motor_6_pulse-10;
	}
	else if(st_servo_run_data.ui32_period_test_cnt < st_servo_run_data.ui32_period_test_total * 3)
	{
		i_motor_pulse[0] = -1; //i_motor_1_pulse+10;
		i_motor_pulse[1] = 1; //i_motor_2_pulse-10;
		i_motor_pulse[2] = -2; //i_motor_3_pulse+20;
		i_motor_pulse[3] = 1; //i_motor_4_pulse-10;
		i_motor_pulse[4] = -1; //i_motor_5_pulse+10;
		i_motor_pulse[5] = 1; //i_motor_6_pulse-10;
	}
	else if(st_servo_run_data.ui32_period_test_cnt < st_servo_run_data.ui32_period_test_total * 4)
	{
		i_motor_pulse[0] = 1; //i_motor_1_pulse+10;
		i_motor_pulse[1] = -1; //i_motor_2_pulse-10;
		i_motor_pulse[2] = 2; //i_motor_3_pulse+20;
		i_motor_pulse[3] = -1; //i_motor_4_pulse-10;
		i_motor_pulse[4] = 1; //i_motor_5_pulse+10;
		i_motor_pulse[5] = -1; //i_motor_6_pulse-10;
	}
	else
	{
		st_servo_run_data.ui32_period_test_cnt = 0;
	}
	Servo_Move(i_motor_pulse);
}
/*****************************************************/
/***************** Main function *********************/
/*****************************************************/
void System_Process()
{
	switch((int)Get_System_State())
	{
		case SYSTEM_STATE_ERROR:
		{
			break;
		}	
		case SYSTEM_STATE_IDLE: 
		{
			break;
		}
		case SYSTEM_STATE_RUN: 
		{
			switch((int)Get_Run_Mode())
			{
				case RUN_MODE_NONE:
				{
					HMI_Update_InpParams_Param(HMI_CUR_X_POS, (int32_t)(st_pl_coords_data[X_AXIS].f_current_axis*100));
					HMI_Update_InpParams_Param(HMI_CUR_Y_POS, (int32_t)(st_pl_coords_data[Y_AXIS].f_current_axis*100));
					HMI_Update_InpParams_Param(HMI_CUR_Z_POS, (int32_t)(st_pl_coords_data[Z_AXIS].f_current_axis*100));
					HMI_Update_InpParams_Param(HMI_CUR_ROLL_POS, (int32_t)(st_pl_coords_data[ROLL_AXIS].f_current_axis*100));
					HMI_Update_InpParams_Param(HMI_CUR_PITCH_POS, (int32_t)(st_pl_coords_data[PITCH_AXIS].f_current_axis*100));
					HMI_Update_InpParams_Param(HMI_CUR_YAW_POS, (int32_t)(st_pl_coords_data[YAW_AXIS].f_current_axis*100));
					break;
				}			
				case RUN_MODE_HOME:
				{	
					switch((int)Get_Home_Mode())
					{
						case HOME_MODE_NONE:
						{
							HMI_Update_InpParams_Param(HMI_CUR_X_POS, (int32_t)(st_pl_coords_data[X_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_Y_POS, (int32_t)(st_pl_coords_data[Y_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_Z_POS, (int32_t)(st_pl_coords_data[Z_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_ROLL_POS, (int32_t)(st_pl_coords_data[ROLL_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_PITCH_POS, (int32_t)(st_pl_coords_data[PITCH_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_YAW_POS, (int32_t)(st_pl_coords_data[YAW_AXIS].f_current_axis*100));
							break;
						}
						case HOME_MODE_SERVO_INIT:
						{
							st_servo_run_data.ui32_period_run_total = home_mode_speed;
							Reset_Angle_Servo(st_servo_data);
							Reset_Axis_Home_Done(st_servo_data);
							Check_Servo_Home_Pos();
							Set_Home_Mode(HOME_MODE_SERVO);
							break;
						}
						case HOME_MODE_SERVO_POS:
						{	
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = home_mode_speed/5;
								Reset_Angle_Servo(st_servo_data);
								for(int index = 0; index < MAX_SERVO; index++)
								{
									st_servo_data[index].f_setpoint_angle += st_servo_data[index].f_offset_angle_home;
								}
								Set_Servo_State(STATE_CALCULATING);
								Set_Home_Mode(HOME_MODE_SERVO_POS_WAIT);
							}
							break;
						}
						case HOME_MODE_SERVO_POS_WAIT:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = home_mode_speed;
								Reset_Angle_Servo(st_servo_data);
								Set_Home_Mode(HOME_MODE_NONE);
							}
							break;
						}
						case HOME_MODE_SERVO:
						{
							if(Is_Axis_Home_Done(ALL_SERVO_AXIS))
							{
								Set_Home_Mode(HOME_MODE_SERVO_POS);
								Reset_Servo_Home_Reach_Stt();
							}
							else
							{
								if(Is_Axis_Home_Done(SERVO_1_AXIS))
								{
									st_servo_data[SERVO1].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO1].is_above_home)
											st_servo_data[SERVO1].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO1].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
									}
								}
								if(Is_Axis_Home_Done(SERVO_2_AXIS))
								{
									st_servo_data[SERVO2].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO2].is_above_home)
											st_servo_data[SERVO2].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO2].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
									}
								}
								if(Is_Axis_Home_Done(SERVO_3_AXIS))
								{
									st_servo_data[SERVO3].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO3].is_above_home)
											st_servo_data[SERVO3].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO3].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
									}
								}
								if(Is_Axis_Home_Done(SERVO_4_AXIS))
								{
									st_servo_data[SERVO4].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO4].is_above_home)
											st_servo_data[SERVO4].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO4].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
									}
								}
								if(Is_Axis_Home_Done(SERVO_5_AXIS))
								{
									st_servo_data[SERVO5].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO5].is_above_home)
											st_servo_data[SERVO5].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO5].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
									}
								}
								if(Is_Axis_Home_Done(SERVO_6_AXIS))
								{
									st_servo_data[SERVO6].b_servo_home_reach = true;
								}
								else
								{
									if(Get_Servo_State() == STATE_STOP)
									{
										if(st_servo_data[SERVO6].is_above_home)
											st_servo_data[SERVO6].f_setpoint_angle += st_servo_run_data.f_run_home_speed;
										else
											st_servo_data[SERVO6].f_setpoint_angle -= st_servo_run_data.f_run_home_speed;
									}
								}
								if(Get_Servo_State() == STATE_STOP)
								{
									st_servo_run_data.ui32_period_run_total = home_mode_speed;
									Set_Servo_State(STATE_CALCULATING);
								}
							}
							break;
						}	
						case HOME_MODE_MODEL:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = home_mode_speed;
								Reset_Axis_Model();
								Calculate_Servo_Deg(st_servo_data);
								Set_Servo_State(STATE_CALCULATING);
								Set_Home_Mode(HOME_MODE_MODEL_WAIT);
							}
							break;
						}
						case HOME_MODE_MODEL_WAIT:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = home_mode_speed;
								Set_Home_Mode(HOME_MODE_NONE);
								Set_Run_Mode(RUN_MODE_NONE);
							}
						}
					}
					break;
				}
				case RUN_MODE_TEST:
				{
					if(Get_Servo_State() == STATE_STOP)
					{
						st_servo_run_data.ui32_period_run_total = test_mode_speed;
						HMI_Update_InpParams_Param(HMI_CUR_X_POS, (int32_t)(st_pl_coords_data[X_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_Y_POS, (int32_t)(st_pl_coords_data[Y_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_Z_POS, (int32_t)(st_pl_coords_data[Z_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_ROLL_POS, (int32_t)(st_pl_coords_data[ROLL_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_PITCH_POS, (int32_t)(st_pl_coords_data[PITCH_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_YAW_POS, (int32_t)(st_pl_coords_data[YAW_AXIS].f_current_axis*100));
						switch((int)Get_Test_Mode())
						{
							case TEST_MODE_NONE:
							{
								break;
							}	
							case TEST_MODE_PULSE:
							{
								break;
							}
							case TEST_MODE_X:
							{
								if(st_pl_coords_data[X_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis <= st_pl_coords_data[X_AXIS].f_max_axis)
									{
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_max_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis >= st_pl_coords_data[X_AXIS].f_min_axis)
									{										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_min_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_X_Y_LEFT:
							{
								if(st_pl_coords_data[X_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis <= st_pl_coords_data[X_AXIS].f_max_axis)
									{
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_max_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis >= st_pl_coords_data[X_AXIS].f_min_axis)
									{										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_min_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Y_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis <= st_pl_coords_data[Y_AXIS].f_max_axis)
									{										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									if(st_pl_coords_data[Y_AXIS].f_current_axis >= st_pl_coords_data[Y_AXIS].f_min_axis)
									{
										st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_test_axis;
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_X_Y_RIGHT:
							{
								if(st_pl_coords_data[X_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis <= st_pl_coords_data[X_AXIS].f_max_axis)
									{										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_max_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis >= st_pl_coords_data[X_AXIS].f_min_axis)
									{										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_min_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Y_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis <= st_pl_coords_data[Y_AXIS].f_max_axis)
									{										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis >= st_pl_coords_data[Y_AXIS].f_min_axis)
									{										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_min_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_Y:
							{
								if(st_pl_coords_data[Y_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis <= st_pl_coords_data[Y_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis >= st_pl_coords_data[Y_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_min_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_Y_Z_LEFT:
							{
								if(st_pl_coords_data[Y_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis <= st_pl_coords_data[Y_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis >= st_pl_coords_data[Y_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_min_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Z_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis <= st_pl_coords_data[Z_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_max_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis >= st_pl_coords_data[Z_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_Y_Z_RIGHT:
							{
								if(st_pl_coords_data[Y_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis <= st_pl_coords_data[Y_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_max_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Y_AXIS].f_current_axis >= st_pl_coords_data[Y_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Y_AXIS].f_next_axis = 
																							st_pl_coords_data[Y_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_min_axis;
										st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
										st_pl_coords_data[Y_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Z_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis <= st_pl_coords_data[Z_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_max_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis >= st_pl_coords_data[Z_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_Z:
							{
								if(st_pl_coords_data[Z_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis <= st_pl_coords_data[Z_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_max_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis >= st_pl_coords_data[Z_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_X_Z_LEFT:
							{
								if(st_pl_coords_data[X_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis <= st_pl_coords_data[X_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_max_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis >= st_pl_coords_data[X_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_min_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Z_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis <= st_pl_coords_data[Z_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_max_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis >= st_pl_coords_data[Z_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_X_Z_RIGHT:
							{
								if(st_pl_coords_data[X_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis <= st_pl_coords_data[X_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_max_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_test_axis;
									if(st_pl_coords_data[X_AXIS].f_current_axis >= st_pl_coords_data[X_AXIS].f_min_axis)
									{										
										st_pl_coords_data[X_AXIS].f_next_axis = 
																							st_pl_coords_data[X_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_min_axis;
										st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
										st_pl_coords_data[X_AXIS].b_is_fwd_move = true;
									}
								}
								if(st_pl_coords_data[Z_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis <= st_pl_coords_data[Z_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_max_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_test_axis;
									if(st_pl_coords_data[Z_AXIS].f_current_axis >= st_pl_coords_data[Z_AXIS].f_min_axis)
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = 
																							st_pl_coords_data[Z_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_min_axis;
										st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
										st_pl_coords_data[Z_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_ROLL:
							{
								if(st_pl_coords_data[ROLL_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[ROLL_AXIS].f_current_axis += st_pl_coords_data[ROLL_AXIS].f_step_test_axis;
									if(st_pl_coords_data[ROLL_AXIS].f_current_axis <= st_pl_coords_data[ROLL_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[ROLL_AXIS].f_next_axis = 
																							st_pl_coords_data[ROLL_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[ROLL_AXIS].f_next_axis = st_pl_coords_data[ROLL_AXIS].f_max_axis;
										st_pl_coords_data[ROLL_AXIS].f_current_axis = st_pl_coords_data[ROLL_AXIS].f_next_axis;
										st_pl_coords_data[ROLL_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[ROLL_AXIS].f_current_axis -= st_pl_coords_data[ROLL_AXIS].f_step_test_axis;
									if(st_pl_coords_data[ROLL_AXIS].f_current_axis >= st_pl_coords_data[ROLL_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[ROLL_AXIS].f_next_axis = 
																							st_pl_coords_data[ROLL_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[ROLL_AXIS].f_next_axis = st_pl_coords_data[ROLL_AXIS].f_min_axis;
										st_pl_coords_data[ROLL_AXIS].f_current_axis = st_pl_coords_data[ROLL_AXIS].f_next_axis;
										st_pl_coords_data[ROLL_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_PITCH:
							{
								if(st_pl_coords_data[PITCH_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[PITCH_AXIS].f_current_axis += st_pl_coords_data[PITCH_AXIS].f_step_test_axis;
									if(st_pl_coords_data[PITCH_AXIS].f_current_axis <= st_pl_coords_data[PITCH_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[PITCH_AXIS].f_next_axis = 
																							st_pl_coords_data[PITCH_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[PITCH_AXIS].f_next_axis = st_pl_coords_data[PITCH_AXIS].f_max_axis;
										st_pl_coords_data[PITCH_AXIS].f_current_axis = st_pl_coords_data[PITCH_AXIS].f_next_axis;
										st_pl_coords_data[PITCH_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[PITCH_AXIS].f_current_axis -= st_pl_coords_data[PITCH_AXIS].f_step_test_axis;
									if(st_pl_coords_data[PITCH_AXIS].f_current_axis >= st_pl_coords_data[PITCH_AXIS].f_min_axis)
									{
										
										st_pl_coords_data[PITCH_AXIS].f_next_axis = 
																							st_pl_coords_data[PITCH_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[PITCH_AXIS].f_next_axis = st_pl_coords_data[PITCH_AXIS].f_min_axis;
										st_pl_coords_data[PITCH_AXIS].f_current_axis = st_pl_coords_data[PITCH_AXIS].f_next_axis;
										st_pl_coords_data[PITCH_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_YAW:
							{
								if(st_pl_coords_data[YAW_AXIS].b_is_fwd_move)
								{
									st_pl_coords_data[YAW_AXIS].f_current_axis += st_pl_coords_data[YAW_AXIS].f_step_test_axis;
									if(st_pl_coords_data[YAW_AXIS].f_current_axis <= st_pl_coords_data[YAW_AXIS].f_max_axis)
									{
										
										st_pl_coords_data[YAW_AXIS].f_next_axis = 
																							st_pl_coords_data[YAW_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[YAW_AXIS].f_next_axis = st_pl_coords_data[YAW_AXIS].f_max_axis;
										st_pl_coords_data[YAW_AXIS].f_current_axis = st_pl_coords_data[YAW_AXIS].f_next_axis;
										st_pl_coords_data[YAW_AXIS].b_is_fwd_move = false;
									}
								}
								else
								{
									st_pl_coords_data[YAW_AXIS].f_current_axis -= st_pl_coords_data[YAW_AXIS].f_step_test_axis;
									if(st_pl_coords_data[YAW_AXIS].f_current_axis >= st_pl_coords_data[YAW_AXIS].f_min_axis)
									{										
										st_pl_coords_data[YAW_AXIS].f_next_axis = 
																							st_pl_coords_data[YAW_AXIS].f_current_axis;
									}
									else
									{
										st_pl_coords_data[YAW_AXIS].f_next_axis = st_pl_coords_data[YAW_AXIS].f_min_axis;
										st_pl_coords_data[YAW_AXIS].f_current_axis = st_pl_coords_data[YAW_AXIS].f_next_axis;
										st_pl_coords_data[YAW_AXIS].b_is_fwd_move = true;
									}
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_CIRCLE:
							{
								Circle_index++;
								if(first_circle_test_move)
								{
									first_circle_test_move = false;
									st_servo_run_data.ui32_period_run_total = 7*F_CTRL;
								}
								circle_trajectory(Circle_Test_Radius, 1, T_Circle_Speed, Circle_index,e_current_circle_mode, st_pl_coords_data);
								if(Circle_index >=  T_Circle_Speed)
								{
									Circle_index = 0;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							case TEST_MODE_SQUARE:
							{	
								
								if(first_square_test_move)
								{
									b_first_cycle = true;
									first_square_test_move = false;
									st_servo_run_data.ui32_period_run_total = 7*F_CTRL;
								}
								rectangle_trajectory(Square_Test_X_From,Square_Test_Y_From,Square_Test_X_To,Square_Test_Y_To,true,T_Square_Speed,Square_index,e_current_square_mode, st_pl_coords_data);
								Square_index++;
								if(Square_index >= T_Square_Speed)
								{
									Square_index = 0;
									b_first_cycle = true;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_ECLIPSE:
							{
								if(first_eclipse_test_move)
								{
									first_eclipse_test_move = false;
									st_servo_run_data.ui32_period_run_total = 7*F_CTRL;
									Eclipse_index = 0;
								}
								ellipse_trajectory(Eclipse_Test_A_Radius,Eclipse_Test_B_Radius,1,T_Eclipse_Speed,Eclipse_index,e_current_eclipse_mode,st_pl_coords_data);
								Eclipse_index++;
								if(Eclipse_index == T_Eclipse_Speed)
								{
									Eclipse_index = 0;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Test_Mode(TEST_MODE_NONE);
								}
								break;
							}
							
							case TEST_MODE_RANDOM_1:
							{
								
								break;
							}
							
							case TEST_MODE_RANDOM_2:
							{
								
								break;
							}
						}
					}
					break;
				}
				case RUN_MODE_JOG:
				{
					if(Get_Servo_State() == STATE_STOP)
					{
						st_servo_run_data.ui32_period_run_total = jog_mode_speed;
						HMI_Update_InpParams_Param(HMI_CUR_X_POS, (int32_t)(st_pl_coords_data[X_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_Y_POS, (int32_t)(st_pl_coords_data[Y_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_Z_POS, (int32_t)(st_pl_coords_data[Z_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_ROLL_POS, (int32_t)(st_pl_coords_data[ROLL_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_PITCH_POS, (int32_t)(st_pl_coords_data[PITCH_AXIS].f_current_axis*100));
						HMI_Update_InpParams_Param(HMI_CUR_YAW_POS, (int32_t)(st_pl_coords_data[YAW_AXIS].f_current_axis*100));
						switch((int)Get_Jog_Mode())
						{
							case JOG_MODE_NONE:
							{
								if(Get_Servo_State() == STATE_STOP)
								{
									
								}
								break;
							}
							case JOG_MODE_UP_X:
							{
								st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);	
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}				
							case JOG_MODE_UP_Y:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}			
							case JOG_MODE_UP_Z:
							{
								st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_LEFT_X_Y:
							{
								st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = 20;
								}
								st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}			
							case JOG_MODE_UP_RIGHT_X_Y:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}				
							case JOG_MODE_UP_LEFT_X_Z:
							{
								st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > 20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = 20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_UP_RIGHT_X_Z:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}			
							case JOG_MODE_UP_LEFT_Y_Z:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = 20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_RIGHT_Y_Z:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < -20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis += st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_LEFT_X_Y_Z:
							{
								Calculate_Servo_Deg(st_servo_data);
								Set_Servo_State(STATE_CALCULATING);
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_ROLL:
							{
								st_pl_coords_data[ROLL_AXIS].f_current_axis += st_pl_coords_data[ROLL_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[ROLL_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[ROLL_AXIS].f_next_axis = st_pl_coords_data[ROLL_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[ROLL_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_PITCH:
							{
								st_pl_coords_data[PITCH_AXIS].f_current_axis += st_pl_coords_data[PITCH_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[PITCH_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[PITCH_AXIS].f_next_axis = st_pl_coords_data[PITCH_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[PITCH_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_UP_YAW:
							{
								st_pl_coords_data[YAW_AXIS].f_current_axis += st_pl_coords_data[YAW_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[YAW_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[YAW_AXIS].f_next_axis = st_pl_coords_data[YAW_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[YAW_AXIS].f_next_axis = 20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_X:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_Y:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_Z:
							{
								st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_LEFT_X_Y:
							{
								st_pl_coords_data[X_AXIS].f_current_axis += st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = 20;
								}
								st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_RIGHT_X_Y:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_LEFT_X_Z:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_RIGHT_X_Z:
							{
								st_pl_coords_data[X_AXIS].f_current_axis -= st_pl_coords_data[X_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[X_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[X_AXIS].f_next_axis = st_pl_coords_data[X_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[X_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_LEFT_Y_Z:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis += st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis < 20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = 20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}						
							case JOG_MODE_DOWN_RIGHT_Y_Z:
							{
								st_pl_coords_data[Y_AXIS].f_current_axis -= st_pl_coords_data[Y_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Y_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = st_pl_coords_data[Y_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Y_AXIS].f_next_axis = -20;
								}
								st_pl_coords_data[Z_AXIS].f_current_axis -= st_pl_coords_data[Z_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[Z_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = st_pl_coords_data[Z_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[Z_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_DOWN_LEFT_X_Y_Z:
							{
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_DOWN_RIGHT_X_Y_Z:
							{
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_DOWN_ROLL:
							{
								st_pl_coords_data[ROLL_AXIS].f_current_axis -= st_pl_coords_data[ROLL_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[ROLL_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[ROLL_AXIS].f_next_axis = st_pl_coords_data[ROLL_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[ROLL_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_DOWN_PITCH:
							{
								st_pl_coords_data[PITCH_AXIS].f_current_axis -= st_pl_coords_data[PITCH_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[PITCH_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[PITCH_AXIS].f_next_axis = st_pl_coords_data[PITCH_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[PITCH_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_DOWN_YAW:
							{
								st_pl_coords_data[YAW_AXIS].f_current_axis -= st_pl_coords_data[YAW_AXIS].f_step_jog_axis; 
								if(st_pl_coords_data[YAW_AXIS].f_current_axis > -20)
								{
									st_pl_coords_data[YAW_AXIS].f_next_axis = st_pl_coords_data[YAW_AXIS].f_current_axis;
								}
								else
								{
									st_pl_coords_data[YAW_AXIS].f_next_axis = -20;
								}
								Calculate_Servo_Deg(st_servo_data);
								if(Get_Servo_State() == STATE_ERROR)
								{
									Set_Servo_State(STATE_STOP);
									Set_Jog_Mode(JOG_MODE_NONE);
								}
								Set_Jog_Mode(JOG_MODE_NONE);
								break;
							}
							case JOG_MODE_FWD_SV_1:
							{
								
							}
							break;
							case JOG_MODE_FWD_SV_2:
							{
								
							}
							break;
							case JOG_MODE_FWD_SV_3:
							{
								
							}
							break;
							case JOG_MODE_FWD_SV_4:
							{
								
							}
							break;
							case JOG_MODE_FWD_SV_5:
							{
								
							}
							break;
							case JOG_MODE_FWD_SV_6:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_1:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_2:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_3:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_4:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_5:
							{
								
							}
							break;
							case JOG_MODE_REV_SV_6:
							{
								
							}
							break;
						}	
					}
					break;
				}
				case RUN_MODE_TEACH:
				{
					switch((int)Get_Teach_Mode())
					{
						HMI_Update_InpParams_Param(HMI_TEACH_MODE_STEP, (int32_t)pl_coords_buffer_teach_index);
						HMI_Update_InpParams_Param(HMI_TEACH_MODE_CURR_STEP, (int32_t)pl_coords_buffer_teach_current_index);
						case TEACH_MODE_NONE:
						{
							HMI_Update_InpParams_Param(HMI_CUR_X_POS, (int32_t)(st_pl_coords_data[X_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_Y_POS, (int32_t)(st_pl_coords_data[Y_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_Z_POS, (int32_t)(st_pl_coords_data[Z_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_ROLL_POS, (int32_t)(st_pl_coords_data[ROLL_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_PITCH_POS, (int32_t)(st_pl_coords_data[PITCH_AXIS].f_current_axis*100));
							HMI_Update_InpParams_Param(HMI_CUR_YAW_POS, (int32_t)(st_pl_coords_data[YAW_AXIS].f_current_axis*100));
							st_servo_run_data.ui32_period_run_total = teach_mode_speed;
							break;
						}
						case TEACH_MODE_RUN_INIT:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								pl_coords_buffer_teach_current_index = 0;
								Set_Teach_Mode(TEACH_MODE_RUN);
							}
							break;
						}
						
						case TEACH_MODE_RUN:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = teach_mode_speed;
								if(pl_coords_buffer_teach_current_index < pl_coords_buffer_teach_index)
								{
									for(int index = 0; index < MAX_AXIS; index++)
									{
										st_pl_coords_data[index].f_current_axis = pl_coords_buffer_teach[index][pl_coords_buffer_teach_current_index];
										st_pl_coords_data[index].f_next_axis = st_pl_coords_data[index].f_current_axis;
									}
									pl_coords_buffer_teach_current_index++;
									Calculate_Servo_Deg(st_servo_data);
									
								}
								else 
								{
									if(b_is_teach_run_lap)
									{
										pl_coords_buffer_teach_current_index = 0;
									}
									else
									{
										pl_coords_buffer_teach_current_index = 0;
										Set_Teach_Mode(TEACH_MODE_NONE);
									}
								}
							}
							break;
						}
						case TEACH_MODE_HOME:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								st_servo_run_data.ui32_period_run_total = 10*F_CTRL;
								Reset_Axis_Model();
								Calculate_Servo_Deg(st_servo_data);
								Set_Servo_State(STATE_CALCULATING);
								Set_Teach_Mode(TEACH_MODE_NONE);
							}
							break;
						}
						case TEACH_MODE_BACK:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								if(pl_coords_buffer_teach_current_index > 0)
								{
									pl_coords_buffer_teach_current_index--;
									for(int index = 0; index < MAX_AXIS; index++)
									{
										st_pl_coords_data[index].f_current_axis = pl_coords_buffer_teach[index][pl_coords_buffer_teach_current_index];
										st_pl_coords_data[index].f_next_axis = st_pl_coords_data[index].f_current_axis;
									}
									Calculate_Servo_Deg(st_servo_data);
									Set_Teach_Mode(TEACH_MODE_NONE);
								}
								else
								{
									Set_Teach_Mode(TEACH_MODE_NONE);
								}
							}
							else
							{
								Set_Teach_Mode(TEACH_MODE_NONE);
							}
							break;
						}
						case TEACH_MODE_NEXT:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
									if(pl_coords_buffer_teach_current_index < pl_coords_buffer_teach_index)
									{
										for(int index = 0; index < MAX_AXIS; index++)
										{
											st_pl_coords_data[index].f_current_axis = pl_coords_buffer_teach[index][pl_coords_buffer_teach_current_index];
											st_pl_coords_data[index].f_next_axis = st_pl_coords_data[index].f_current_axis;
										}
										pl_coords_buffer_teach_current_index++;
										Calculate_Servo_Deg(st_servo_data);
										if(Get_Servo_State() == STATE_ERROR)
										{
											pl_coords_buffer_teach_current_index--;
											Set_Teach_Mode(TEACH_MODE_NONE);
											Set_Servo_State(STATE_STOP);
										}	
									}
									else
									{
										Set_Teach_Mode(TEACH_MODE_NONE);
									}
							}
							else
							{
								Set_Teach_Mode(TEACH_MODE_NONE);
							}
							break;
						}
						case TEACH_MODE_GO:
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								if(pl_coords_buffer_teach_index < 30)
								{
									for(int index = 0; index < MAX_AXIS; index++)
									{
										pl_coords_buffer_teach[index][pl_coords_buffer_teach_current_index] = st_pl_coords_data[index].f_next_axis;
										st_pl_coords_data[index].f_current_axis = st_pl_coords_data[index].f_next_axis;
									}
									pl_coords_buffer_teach_current_index++;
									pl_coords_buffer_teach_index++;
									Calculate_Servo_Deg(st_servo_data);
									if(Get_Servo_State() == STATE_ERROR)
									{
										pl_coords_buffer_teach_index--;
										Set_Teach_Mode(TEACH_MODE_NONE);
										Set_Servo_State(STATE_STOP);
									}
								}
								else
								{
									pl_coords_buffer_teach_index = 30;
									Set_Teach_Mode(TEACH_MODE_NONE);
								}
							}
							else
							{
								Set_Teach_Mode(TEACH_MODE_NONE);
							}
							break;
						}
					}
					break;
				}
				case RUN_MODE_FOLLOW:
				{
					if(Get_Servo_State() == STATE_STOP)
					{
						st_servo_run_data.ui32_period_run_total = follow_mode_speed;
						if(b_is_new_pl_coords_data)
						{
							for(int i = 0; i < MAX_AXIS; i++)
							{
								st_pl_coords_data[i].f_current_axis = pl_coords_buffer_follow[i][pl_coords_buffer_follow_index++];
								st_pl_coords_data[i].f_next_axis = st_pl_coords_data[i].f_current_axis;
							}
							Calculate_Servo_Deg(st_servo_data);
							if(Get_Servo_State() == STATE_ERROR)
							{
								Set_Servo_State(STATE_STOP);
							}
							if(pl_coords_buffer_follow_index == 5)
							{
								pl_coords_buffer_follow_index = 0;
								b_is_new_pl_coords_data = false;
							}
						}
					}
					break;
				}
			}
			Control_Motor();
			break;
		}
	}
}
void Control_Motor(void)
{
	if(Get_Servo_State() == STATE_STOP)
	{
		return;
	}
	if(Get_Servo_State() == STATE_HOMING_POS)
	{
		Calculate_Servo_Deg(st_servo_data);
		Set_Servo_State(STATE_CALCULATING);
	}
	if(Get_Servo_State() == STATE_CALCULATING)
	{
		switch((int)Get_Control_Mode())
		{
			case CONTROL_MODE_NONE:
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
				break;
			case CONTROL_MODE_FEEDFORWARD_POS:
			{
				Calculate_Servo_F_Pulse(st_servo_data); //f_pulse = (f_setpoint_angle - f_current_angle)*DRIVER_DEGREE_TO_PULSE/period_total
			}
				break;
			case CONTROL_MODE_FEEDFORWARD_POS_VEL:
			{
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
			}
				break;
			default:
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
				break;
		}
		Set_Servo_State(STATE_SERVO_RUN);
	}
	if(Get_Servo_State() == STATE_SERVO_RUN)
	{
		switch((int)Get_Control_Mode())
		{
			case CONTROL_MODE_NONE:
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
				break;
			case CONTROL_MODE_FEEDFORWARD_POS:
			{
				Calculate_Servo_Pulse(st_servo_data,servo_pulse); //servo_pulse = f_pulse_acc - i32_pulse_acc
				Servo_Move(servo_pulse);
				Update_Angle_Servo(st_servo_data); //f_current_angle = i32_pulse_acc * DRIVER_PULSE_TO_DEGREE
				st_servo_run_data.ui32_period_run_cnt++;
				if(st_servo_run_data.ui32_period_run_cnt > st_servo_run_data.ui32_period_run_total)
				{
					st_servo_run_data.ui32_period_run_cnt = 0;
					Set_Servo_State(STATE_STOP);
					//UART_PC_Send_PL_Pos_Test();
				}
			}
				break;
			case CONTROL_MODE_FEEDFORWARD_POS_VEL:
			{
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
			}
				break;
			default:
				Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
				break;
		}
	}
	
}


/*****************************************************/
/*************** Contractor function ***************/
/*****************************************************/

//For Get / Set System status and Run mode
void Set_System_State(SYSTEM_STATE_ENUM e_state)
{
	e_current_system_state = e_state;
}
SYSTEM_STATE_ENUM Get_System_State()
{
	return e_current_system_state;
}

void Set_Run_Mode(RUN_MODE_ENUM e_mode)
{
	e_current_run_mode = e_mode;
}
RUN_MODE_ENUM Get_Run_Mode(void)
{
	return e_current_run_mode;
}
void Set_Control_Mode(CONTROL_MODE_ENUM e_mode)
{
	e_current_control_mode = e_mode;
}

CONTROL_MODE_ENUM Get_Control_Mode()
{
	return e_current_control_mode;
}
void Set_Jog_Mode(JOG_MODE_ENUM e_mode)
{
	e_current_jog_mode = e_mode;
}

JOG_MODE_ENUM Get_Jog_Mode(void)
{
	return e_current_jog_mode;
}
void Set_Test_Mode(TEST_MODE_ENUM e_test_mode)
{
	e_current_test_mode = e_test_mode;
}
TEST_MODE_ENUM Get_Test_Mode(void)
{
	return e_current_test_mode;
}
void Set_Home_Mode(HOME_MODE_ENUM e_mode)
{
	e_current_home_mode = e_mode;
}
HOME_MODE_ENUM Get_Home_Mode(void)
{
	return e_current_home_mode;
}
void Set_Teach_Mode(TEACH_MODE_ENUM e_mode)
{
	e_current_teach_mode = e_mode;
}
TEACH_MODE_ENUM Get_Teach_Mode(void)
{
	return e_current_teach_mode;
}
void Set_Servo_State(SERVO_STATE_ENUM e_servo_state)
{
	e_current_servo_state = e_servo_state;
}

SERVO_STATE_ENUM Get_Servo_State(void)
{
	return e_current_servo_state;
}




//For Get / Set Model and Servo data
void Set_Pl_Coords_Data_Max_Axis(AXIS_ENUM e_axis, float value)
{
	if(e_axis >= MAX_AXIS)
		return;
	switch((int)e_axis)
	{
		case X_AXIS:
			st_pl_coords_data[X_AXIS].f_max_axis = value;
			break;
		case Y_AXIS:
			st_pl_coords_data[Y_AXIS].f_max_axis = value;
			break;
		case Z_AXIS:
			st_pl_coords_data[Z_AXIS].f_max_axis = value;
			break;
		case ROLL_AXIS:
			st_pl_coords_data[ROLL_AXIS].f_max_axis = value;
			break;
		case PITCH_AXIS:
			st_pl_coords_data[PITCH_AXIS].f_max_axis = value;
			break;
		case YAW_AXIS:
			st_pl_coords_data[YAW_AXIS].f_max_axis = value;
			break;
	}
}

float Get_Pl_Coords_Data_Max_Axis(AXIS_ENUM e_axis)
{
	if(e_axis >= MAX_AXIS)
		return 0;
	float temp;
	switch((int)e_axis)
	{
		case X_AXIS:
			temp = st_pl_coords_data[X_AXIS].f_max_axis;
			break;
		case Y_AXIS:
			temp = st_pl_coords_data[Y_AXIS].f_max_axis;
			break;
		case Z_AXIS:
			temp = st_pl_coords_data[Z_AXIS].f_max_axis;
			break;
		case ROLL_AXIS:
			temp = st_pl_coords_data[ROLL_AXIS].f_max_axis;
			break;
		case PITCH_AXIS:
			temp = st_pl_coords_data[PITCH_AXIS].f_max_axis;
			break;
		case YAW_AXIS:
			temp = st_pl_coords_data[YAW_AXIS].f_max_axis;
			break;
	}
	return temp;
}
void Set_Pl_Coords_Data_Min_Axis(AXIS_ENUM e_axis, float value)
{
	if(e_axis >= MAX_AXIS)
		return;
	switch((int)e_axis)
	{
		case X_AXIS:
			st_pl_coords_data[X_AXIS].f_min_axis = value;
			break;
		case Y_AXIS:
			st_pl_coords_data[Y_AXIS].f_min_axis = value;
			break;
		case Z_AXIS:
			st_pl_coords_data[Z_AXIS].f_min_axis = value;
			break;
		case ROLL_AXIS:
			st_pl_coords_data[ROLL_AXIS].f_min_axis = value;
			break;
		case PITCH_AXIS:
			st_pl_coords_data[PITCH_AXIS].f_min_axis = value;
			break;
		case YAW_AXIS:
			st_pl_coords_data[YAW_AXIS].f_min_axis = value;
			break;
	}
}
float Get_Pl_Coords_Data_Min_Axis(AXIS_ENUM e_axis)
{
	if(e_axis >= MAX_AXIS)
		return 0;
	float temp;
	switch((int)e_axis)
	{
		case X_AXIS:
			temp = st_pl_coords_data[X_AXIS].f_min_axis;
			break;
		case Y_AXIS:
			temp = st_pl_coords_data[Y_AXIS].f_min_axis;
			break;
		case Z_AXIS:
			temp = st_pl_coords_data[Z_AXIS].f_min_axis;
			break;
		case ROLL_AXIS:
			temp = st_pl_coords_data[ROLL_AXIS].f_min_axis;
			break;
		case PITCH_AXIS:
			temp = st_pl_coords_data[PITCH_AXIS].f_min_axis;
			break;
		case YAW_AXIS:
			temp = st_pl_coords_data[YAW_AXIS].f_min_axis;
			break;
	}
	return temp;
}
void Set_Pl_Coords_Data_Step_Test_Axis(AXIS_ENUM e_axis, float value)
{
	if(e_axis >= MAX_AXIS)
		return;
	switch((int)e_axis)
	{
		case X_AXIS:
			st_pl_coords_data[X_AXIS].f_step_test_axis = value;
			break;
		case Y_AXIS:
			st_pl_coords_data[Y_AXIS].f_step_test_axis = value;
			break;
		case Z_AXIS:
			st_pl_coords_data[Z_AXIS].f_step_test_axis = value;
			break;
		case ROLL_AXIS:
			st_pl_coords_data[ROLL_AXIS].f_step_test_axis = value;
			break;
		case PITCH_AXIS:
			st_pl_coords_data[PITCH_AXIS].f_step_test_axis = value;
			break;
		case YAW_AXIS:
			st_pl_coords_data[YAW_AXIS].f_step_test_axis = value;
			break;
	}
}

float Get_Pl_Coords_Data_Step_Test_Axis(AXIS_ENUM e_axis)
{
	if(e_axis >= MAX_AXIS)
		return 0;
	float temp;
	switch((int)e_axis)
	{
		case X_AXIS:
			temp = st_pl_coords_data[X_AXIS].f_step_test_axis;
			break;
		case Y_AXIS:
			temp = st_pl_coords_data[Y_AXIS].f_step_test_axis;
			break;
		case Z_AXIS:
			temp = st_pl_coords_data[Z_AXIS].f_step_test_axis;
			break;
		case ROLL_AXIS:
			temp = st_pl_coords_data[ROLL_AXIS].f_step_test_axis;
			break;
		case PITCH_AXIS:
			temp = st_pl_coords_data[PITCH_AXIS].f_step_test_axis;
			break;
		case YAW_AXIS:
			temp = st_pl_coords_data[YAW_AXIS].f_step_test_axis;
			break;
	}
	return temp;
}
void Set_Pl_Coords_Data_Step_Jog_Axis(AXIS_ENUM e_axis, float value)
{
	if(e_axis >= MAX_AXIS)
		return;
	switch((int)e_axis)
	{
		case X_AXIS:
			st_pl_coords_data[X_AXIS].f_step_jog_axis = value;
			break;
		case Y_AXIS:
			st_pl_coords_data[Y_AXIS].f_step_jog_axis = value;
			break;
		case Z_AXIS:
			st_pl_coords_data[Z_AXIS].f_step_jog_axis = value;
			break;
		case ROLL_AXIS:
			st_pl_coords_data[ROLL_AXIS].f_step_jog_axis = value;
			break;
		case PITCH_AXIS:
			st_pl_coords_data[PITCH_AXIS].f_step_jog_axis = value;
			break;
		case YAW_AXIS:
			st_pl_coords_data[YAW_AXIS].f_step_jog_axis = value;
			break;
	}
}
float Get_Pl_Coords_Data_Step_Jog_Axis(AXIS_ENUM e_axis)
{
	if(e_axis >= MAX_AXIS)
		return 0;
	float temp;
	switch((int)e_axis)
	{
		case X_AXIS:
			temp = st_pl_coords_data[X_AXIS].f_step_jog_axis;
			break;
		case Y_AXIS:
			temp = st_pl_coords_data[Y_AXIS].f_step_jog_axis;
			break;
		case Z_AXIS:
			temp = st_pl_coords_data[Z_AXIS].f_step_jog_axis;
			break;
		case ROLL_AXIS:
			temp = st_pl_coords_data[ROLL_AXIS].f_step_jog_axis;
			break;
		case PITCH_AXIS:
			temp = st_pl_coords_data[PITCH_AXIS].f_step_jog_axis;
			break;
		case YAW_AXIS:
			temp = st_pl_coords_data[YAW_AXIS].f_step_jog_axis;
			break;
	}
	return temp;
}
void Set_Pl_Coords_Data_Pos(AXIS_ENUM e_axis, float value)
{
	if(e_axis >= MAX_AXIS)
		return;
	switch((int)e_axis)
	{
		case X_AXIS:
			st_pl_coords_data[X_AXIS].f_next_axis = value;
			break;
		case Y_AXIS:
			st_pl_coords_data[Y_AXIS].f_next_axis = value;
			break;
		case Z_AXIS:
			st_pl_coords_data[Z_AXIS].f_next_axis = value;
			break;
		case ROLL_AXIS:
			st_pl_coords_data[ROLL_AXIS].f_next_axis = value;
			break;
		case PITCH_AXIS:
			st_pl_coords_data[PITCH_AXIS].f_next_axis = value;
			break;
		case YAW_AXIS:
			st_pl_coords_data[YAW_AXIS].f_next_axis = value;
			break;
	}
}
float Get_Pl_Coords_Data_Pos(AXIS_ENUM e_axis)
{
	if(e_axis >= MAX_AXIS)
		return 0;
	float temp = 0;
	switch((int)e_axis)
	{
		case X_AXIS:
			temp = st_pl_coords_data[X_AXIS].f_current_axis;
			break;
		case Y_AXIS:
			temp = st_pl_coords_data[Y_AXIS].f_current_axis;
			break;
		case Z_AXIS:
			temp = st_pl_coords_data[Z_AXIS].f_current_axis;
			break;
		case ROLL_AXIS:
			temp = st_pl_coords_data[ROLL_AXIS].f_current_axis;
			break;
		case PITCH_AXIS:
			temp = st_pl_coords_data[PITCH_AXIS].f_current_axis;
			break;
		case YAW_AXIS:
			temp = st_pl_coords_data[YAW_AXIS].f_current_axis;
			break;
	}
	return temp;
}
void Set_Servo_Data_Pos(SERVO_ENUM e_servo, float value)
{
	if(e_servo >= MAX_SERVO)
		return;
	switch((int)e_servo)
	{
		case SERVO1:
			st_servo_data[SERVO1].f_next_setpoint_angle = value;
			break;
		case SERVO2:
			st_servo_data[SERVO2].f_next_setpoint_angle = value;
			break;
		case SERVO3:
			st_servo_data[SERVO3].f_next_setpoint_angle = value;
			break;
		case SERVO4:
			st_servo_data[SERVO4].f_next_setpoint_angle = value;
			break;
		case SERVO5:
			st_servo_data[SERVO5].f_next_setpoint_angle = value;
			break;
		case SERVO6:
			st_servo_data[SERVO6].f_next_setpoint_angle = value;
			break;
	}
}
float Get_Servo_Data_Pos(SERVO_ENUM e_servo)
{
	if(e_servo >= MAX_SERVO)
		return 0;
	float temp;
	switch((int)e_servo)
	{
		case SERVO1:
			temp = st_servo_data[SERVO1].f_current_angle;
			break;
		case SERVO2:
			temp = st_servo_data[SERVO2].f_current_angle;
			break;
		case SERVO3:
			temp = st_servo_data[SERVO3].f_current_angle;
			break;
		case SERVO4:
			temp = st_servo_data[SERVO4].f_current_angle;
			break;
		case SERVO5:
			temp = st_servo_data[SERVO5].f_current_angle;
			break;
		case SERVO6:
			temp = st_servo_data[SERVO6].f_current_angle;
			break;
	}
	return temp;
}

void Set_Servo_Data_Vel(SERVO_ENUM e_servo, float value)
{
	if(e_servo >= MAX_SERVO)
		return;
	switch((int)e_servo)
	{
		case SERVO1:
			st_servo_data[SERVO1].f_setpoint_vel = value;
			break;
		case SERVO2:
			st_servo_data[SERVO2].f_setpoint_vel = value;
			break;
		case SERVO3:
			st_servo_data[SERVO3].f_setpoint_vel = value;
			break;
		case SERVO4:
			st_servo_data[SERVO4].f_setpoint_vel = value;
			break;
		case SERVO5:
			st_servo_data[SERVO5].f_setpoint_vel = value;
			break;
		case SERVO6:
			st_servo_data[SERVO6].f_setpoint_vel = value;
			break;
	}
}
float Get_Servo_Data_Vel(SERVO_ENUM e_servo)
{
	if(e_servo >= MAX_SERVO)
		return 0;
	float temp;
	switch((int)e_servo)
	{
		case SERVO1:
			temp = st_servo_data[SERVO1].f_current_vel;
			break;
		case SERVO2:
			temp = st_servo_data[SERVO2].f_current_vel;
			break;
		case SERVO3:
			temp = st_servo_data[SERVO3].f_current_vel;
			break;
		case SERVO4:
			temp = st_servo_data[SERVO4].f_current_vel;
			break;
		case SERVO5:
			temp = st_servo_data[SERVO5].f_current_vel;
			break;
		case SERVO6:
			temp = st_servo_data[SERVO6].f_current_vel;
			break;
	}
	return temp;
}
void Set_Servo_Data_Pos_Up(SERVO_ENUM e_servo)
{
	if(e_servo >= MAX_SERVO)
		return;
	switch((int)e_servo)
	{
		case SERVO1:
			st_servo_data[SERVO1].f_setpoint_angle += st_servo_data[SERVO1].f_next_setpoint_angle;
			break;
		case SERVO2:
			st_servo_data[SERVO2].f_setpoint_angle -= st_servo_data[SERVO2].f_next_setpoint_angle;
			break;
		case SERVO3:
			st_servo_data[SERVO3].f_setpoint_angle += st_servo_data[SERVO3].f_next_setpoint_angle;
			break;
		case SERVO4:
			st_servo_data[SERVO4].f_setpoint_angle -= st_servo_data[SERVO4].f_next_setpoint_angle;
			break;
		case SERVO5:
			st_servo_data[SERVO5].f_setpoint_angle += st_servo_data[SERVO5].f_next_setpoint_angle;
			break;
		case SERVO6:
			st_servo_data[SERVO6].f_setpoint_angle -= st_servo_data[SERVO6].f_next_setpoint_angle;
			break;
	}
}
void Set_Servo_Data_Pos_Down(SERVO_ENUM e_servo)
{
	if(e_servo >= MAX_SERVO)
		return;
	switch((int)e_servo)
	{
		case SERVO1:
			st_servo_data[SERVO1].f_setpoint_angle -= st_servo_data[SERVO1].f_next_setpoint_angle;
			break;
		case SERVO2:
			st_servo_data[SERVO2].f_setpoint_angle += st_servo_data[SERVO2].f_next_setpoint_angle;
			break;
		case SERVO3:
			st_servo_data[SERVO3].f_setpoint_angle -= st_servo_data[SERVO3].f_next_setpoint_angle;
			break;
		case SERVO4:
			st_servo_data[SERVO4].f_setpoint_angle += st_servo_data[SERVO4].f_next_setpoint_angle;
			break;
		case SERVO5:
			st_servo_data[SERVO5].f_setpoint_angle -= st_servo_data[SERVO5].f_next_setpoint_angle;
			break;
		case SERVO6:
			st_servo_data[SERVO6].f_setpoint_angle += st_servo_data[SERVO6].f_next_setpoint_angle;
			break;
	}
}
void Set_Servo_Data_STT_On_Off(SERVO_ENUM e_servo, bool value)
{
	switch((int)e_servo)
	{
		case SERVO1:
			st_servo_data[SERVO1].servo_on_off_stt = value;
			break;
		case SERVO2:
			st_servo_data[SERVO2].servo_on_off_stt = value;
			break;
		case SERVO3:
			st_servo_data[SERVO3].servo_on_off_stt = value;
			break;
		case SERVO4:
			st_servo_data[SERVO4].servo_on_off_stt = value;
			break;
		case SERVO5:
			st_servo_data[SERVO5].servo_on_off_stt = value;
			break;
		case SERVO6:
			st_servo_data[SERVO6].servo_on_off_stt = value;
			break;
	}
}
void Set_Speed(RUN_MODE_ENUM e_run_mode, float value)
{
	switch((int)e_run_mode)
	{
		case RUN_MODE_HOME:
			home_mode_speed = (uint32_t)(value*F_CTRL);
			break;
		case RUN_MODE_JOG:
			jog_mode_speed = (uint32_t)(value*F_CTRL);
			break;
		case RUN_MODE_TEST:
			test_mode_speed = (uint32_t)(value*F_CTRL);
			break;
		case RUN_MODE_TEACH:
			teach_mode_speed = (uint32_t)(value*F_CTRL);
			break;
		case RUN_MODE_FOLLOW:
			follow_mode_speed = (uint32_t)(value*F_CTRL);
			break;
		default:
			break;
	}
}
void Set_Circle_Test_Data(CIRCLE_MODE_DATA c_data_type, float value, CIRCLE_MODE_ENUM c_mode)
{
	switch((int)c_data_type)
	{
		case CIRCLE_RADIUS:
			Circle_Test_Radius = value;
			break;
		case CIRCLE_SPEED:
			T_Circle_Speed = value*F_CTRL;
			break;
		case CIRCLE_MODE:
			e_current_circle_mode = c_mode;
			break;
	}
}

void Set_Eclipse_Test_Data(ECLIPSE_MODE_DATA e_data_type, float value, ECLIPSE_MODE_ENUM e_mode)
{
	switch((int)e_data_type)
	{
		case ECLIPSE_A_RADIUS:
			Eclipse_Test_A_Radius = value;
			break;
		case ECLIPSE_B_RADIUS:
			Eclipse_Test_B_Radius = value;
			break;
		case ECLIPSE_SPEED:
			T_Eclipse_Speed = value*F_CTRL;
			break;
		case ECLIPSE_MODE:
			e_current_eclipse_mode = e_mode;
			break;
	}
}

void Set_Square_Test_Data(SQUARE_MODE_DATA s_data_type, float value, SQUARE_MODE_ENUM s_mode)
{
	switch((int)s_data_type)
	{
		case SQUARE_X_FROM:
			Square_Test_X_From = value;
			break;
		case SQUARE_Y_FROM:
			Square_Test_Y_From = value;
			break;
		case SQUARE_X_TO:
			Square_Test_X_To = value;
			break;
		case SQUARE_Y_TO:
			Square_Test_Y_To = value;
			break;
		case SQUARE_SPEED:
			T_Square_Speed = value*F_CTRL;
			break;
		case SQUARE_MODE:
			e_current_square_mode = s_mode;
			break;
	}
}	
void Reset_Teach_Index(void)
{
	pl_coords_buffer_teach_current_index = 0;
	pl_coords_buffer_teach_index = 0;
}
/*****************************************************/
/*************** Handler function ***************/
/*****************************************************/

/*****************************************************/
/*************** Button handler function ***************/
/*****************************************************/
bool Start_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_ERROR)
	{
		Set_System_State(SYSTEM_STATE_RUN);
		Set_Run_Mode(RUN_MODE_HOME);
		Set_Test_Mode(TEST_MODE_NONE);
		Set_Jog_Mode(JOG_MODE_NONE);
		Set_Home_Mode(HOME_MODE_MODEL);
		Set_Servo_State(STATE_CALCULATING);
	}
	if(Get_System_State() == SYSTEM_STATE_IDLE) //State initial
	{
		Set_System_State(SYSTEM_STATE_RUN);
		Set_Run_Mode(RUN_MODE_NONE);
		Set_Servo_State(STATE_STOP);
	}
	return true;
}
bool Stop_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	switch((int)Get_System_State())
	{
		case SYSTEM_STATE_RUN: 
		{
			Set_Run_Mode(RUN_MODE_NONE);
			Set_Test_Mode(TEST_MODE_NONE);
			Set_Jog_Mode(JOG_MODE_NONE);
			Set_Home_Mode(HOME_MODE_NONE);
			Set_System_State(SYSTEM_STATE_IDLE);
			Set_Servo_State(STATE_STOP);
		}
		break;
		case SYSTEM_STATE_IDLE:
		{
			
		}
		break;
		case SYSTEM_STATE_ERROR:
		{
			Set_Run_Mode(RUN_MODE_NONE);
			Set_Test_Mode(TEST_MODE_NONE);
			Set_Jog_Mode(JOG_MODE_NONE);
			Set_Home_Mode(HOME_MODE_NONE);
			Set_System_State(SYSTEM_STATE_IDLE);
			Set_Servo_State(STATE_STOP);
		}
		break;
	}
	return true;
}
bool Home_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{	
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_NONE);
			Set_Servo_State(STATE_STOP);
			Reset_Angle_Servo(st_servo_data);
		}
		else if(Get_Run_Mode() == RUN_MODE_HOME)
		{
			Set_Run_Mode(RUN_MODE_NONE);
			Set_Servo_State(STATE_STOP);
			Reset_Angle_Servo(st_servo_data);
		}
	}
	else if(Get_System_State() == SYSTEM_STATE_IDLE)
	{
		
	}
	else if(Get_System_State() == SYSTEM_STATE_ERROR)
	{
		Set_Run_Mode(RUN_MODE_HOME);
		Set_Home_Mode(HOME_MODE_NONE);
		Set_Servo_State(STATE_STOP);
	}
	return true;
}
bool Home_Button_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{	
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_NONE);
			Set_Servo_State(STATE_STOP);
		}
	}
	else if(Get_System_State() == SYSTEM_STATE_IDLE)
	{
		
	}
	else if(Get_System_State() == SYSTEM_STATE_ERROR)
	{
		Set_Run_Mode(RUN_MODE_HOME);
		Set_Home_Mode(HOME_MODE_NONE);
		Set_Servo_State(STATE_STOP);
	}
	return true;
}
bool Home_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	switch((int)Get_System_State())
	{
		case SYSTEM_STATE_ERROR:
		{
			Set_System_State(SYSTEM_STATE_RUN);
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_MODEL);
			break;
		}
		case SYSTEM_STATE_RUN:
		{
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_MODEL);
			break;
		}
	}
	return true;
}

bool Home_Servo_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	switch((int)Get_System_State())
	{
		case SYSTEM_STATE_ERROR:
		{
			Set_System_State(SYSTEM_STATE_RUN);
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_SERVO_INIT);
			Set_Servo_State(STATE_STOP);
		}
			break;
		case SYSTEM_STATE_RUN:
		{
			Set_Run_Mode(RUN_MODE_HOME);
			Set_Home_Mode(HOME_MODE_SERVO_INIT);
			Set_Servo_State(STATE_STOP);
		}
			break;
	}
	return true;
}
bool Set_Model_Test_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{
			Set_Run_Mode(RUN_MODE_TEST);
			Set_Test_Mode(TEST_MODE_NONE);
		}
		else if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Run_Mode(RUN_MODE_NONE);
				Set_Test_Mode(TEST_MODE_NONE);
			}
		}
	}
	return true;
}
bool Set_Model_Control_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Control_Mode() == CONTROL_MODE_NONE)
	{
		Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
	}
	else if(Get_Control_Mode() == CONTROL_MODE_FEEDFORWARD_POS)
	{
		Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS_VEL);
	}
	else if(Get_Control_Mode() == CONTROL_MODE_FEEDFORWARD_POS_VEL)
	{
		Set_Control_Mode(CONTROL_MODE_FEEDBACK_ENC);
	}
	else if(Get_Control_Mode() == CONTROL_MODE_FEEDBACK_ENC)
	{
		Set_Control_Mode(CONTROL_MODE_FEEDBACK_ENC_IMU_LIDAR);
	}
	else if(Get_Control_Mode() == CONTROL_MODE_FEEDBACK_ENC_IMU_LIDAR)
	{
		Set_Control_Mode(CONTROL_MODE_FEEDFORWARD_POS);
	}
	return true;
}

bool Set_Model_Jog_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{
			Set_Run_Mode(RUN_MODE_JOG);
			Set_Jog_Mode(JOG_MODE_NONE);
		}
		else if(Get_Run_Mode() == RUN_MODE_JOG)
		{
			Set_Run_Mode(RUN_MODE_NONE);
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	return true;
}
bool Set_Model_Teach_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{
			Set_Run_Mode(RUN_MODE_TEACH);
		}
		else if(Get_Run_Mode() == RUN_MODE_TEACH)
		{
			Set_Run_Mode(RUN_MODE_NONE);
		}
	}
	return true;
}
bool Set_Follow_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{
			Set_Run_Mode(RUN_MODE_FOLLOW);
		}
		else if(Get_Run_Mode() == RUN_MODE_FOLLOW)
		{
			Set_Run_Mode(RUN_MODE_NONE);
		}
	}
	return true;
}
bool Reset_Servo_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Run_Mode() == RUN_MODE_HOME)
	{
		Reset_Angle_Servo(st_servo_data);
	}
	return true;
}
bool Servo_On_Off_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	switch(ui8_data[1])
	{
		case '1':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_1);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_1);
			}		
			break;
		case '2':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_2);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_2);
			}
			break;
		case '3':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_3);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_3);
			}
			break;
		case '4':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_4);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_4);
			}
			break;
		case '5':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_5);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_5);
			}
			break;
		case '6':
			if(ui8_data[3] == '1')
			{
				Relay_Off(RELAY_6);
			}
			else if(ui8_data[3] == '0')
			{
				Relay_On(RELAY_6);
			}
			break;
	}
	return true;
}
bool Reset_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_NONE)
		{
			Reset_Axis_Model();
		}
	}
	return true;
}


/*************** Button handler function in Test Mode ***************/
bool Set_Test_X_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_X)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_X);
			}
		}
	}
	return true;
}
bool Set_Test_Y_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_Y)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_Y);
			}
		}
	}
	return true;
}
bool Set_Test_Z_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_Z)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_Z);
			}
		}
	}
	return true;
}
bool Set_Test_X_Y_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_X_Y_LEFT)
			{
				Set_Test_Mode(TEST_MODE_NONE);	
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_X_Y_LEFT);
			}
		}
	}
	return true;
}
bool Set_Test_X_Y_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_X_Y_RIGHT)
			{
				Set_Test_Mode(TEST_MODE_NONE);	
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_X_Y_RIGHT);
			}
		}
	}
	return true;
}
bool Set_Test_X_Z_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_X_Z_LEFT)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_X_Z_LEFT);
			}
		}
	}
	return true;
}
bool Set_Test_X_Z_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_X_Z_RIGHT)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_X_Z_RIGHT);
			}
		}
	}
	return true;
}

bool Set_Test_Y_Z_Left_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_Y_Z_LEFT)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_Y_Z_LEFT);
			}
		}
	}
	return true;
}
bool Set_Test_Y_Z_Right_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_Y_Z_RIGHT)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_Y_Z_RIGHT);
			}
		}
	}
	return true;
}

bool Set_Test_Roll_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_ROLL)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_ROLL);
			}
		}
	}
	return true;
}
bool Set_Test_Pitch_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_PITCH)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_PITCH);
			}
		}
	}
	return true;
}
bool Set_Test_Yaw_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_YAW)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_YAW);
			}
		}
	}
	return true;
}
bool Set_Test_Circle_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_CIRCLE)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				first_circle_test_move = true;
				Set_Test_Mode(TEST_MODE_CIRCLE);
			}
		}
	}
	return true;
}
bool Set_Test_Square_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_SQUARE)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				first_square_test_move = true;
				Set_Test_Mode(TEST_MODE_SQUARE);
			}
		}
	}
	return true;
}
bool Set_Test_Random_1_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_RANDOM_1)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_RANDOM_1);
			}
		}
	}
	return true;
}
bool Set_Test_Random_2_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_RANDOM_2)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_RANDOM_2);
			}
		}
	}
	return true;
}
bool Set_Test_Pulse_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_PULSE)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				Set_Test_Mode(TEST_MODE_PULSE);
			}
		}
	}
	return true;
}

bool Set_Test_Eclipse_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEST)
		{
			if(Get_Test_Mode() == TEST_MODE_ECLIPSE)
			{
				Set_Test_Mode(TEST_MODE_NONE);
			}
			else if(Get_Test_Mode() == TEST_MODE_NONE)
			{
				first_eclipse_test_move = true;
				Set_Test_Mode(TEST_MODE_ECLIPSE);
			}
		}
	}
	return true;
}
/*************** Button handler function in Jog Mode ***************/
bool Set_Model_Jog_Mode_X_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_X);
		}
	}
	}
	return true;
}

bool Set_Model_Jog_Mode_X_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_X);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_X)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
}
	return true;
}
bool Set_Model_Jog_Mode_Y_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
}
	return true;
}

bool Set_Model_Jog_Mode_Y_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Z_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Z_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
}return true;
}
bool Set_Model_Jog_Mode_Roll_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_ROLL);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_ROLL)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Roll_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_ROLL);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_ROLL)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Pitch_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_PITCH);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_PITCH)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Pitch_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_PITCH);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_PITCH)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
}return true;
}

bool Set_Model_Jog_Mode_Yaw_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_YAW);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_YAW)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Yaw_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_YAW);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_YAW)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Y_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_LEFT_X_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_LEFT_X_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
}return true;
}

bool Set_Model_Jog_Mode_X_Y_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_LEFT_X_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_LEFT_X_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Y_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_RIGHT_X_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_RIGHT_X_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Y_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_RIGHT_X_Y);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_RIGHT_X_Y)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Z_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_LEFT_X_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_LEFT_X_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Z_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_LEFT_X_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_LEFT_X_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Z_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_RIGHT_X_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_RIGHT_X_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_X_Z_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_RIGHT_X_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_RIGHT_X_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Y_Z_Left_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_LEFT_Y_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_LEFT_Y_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Y_Z_Left_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_LEFT_Y_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_LEFT_Y_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Y_Z_Right_Up_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_UP_RIGHT_Y_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_UP_RIGHT_Y_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

bool Set_Model_Jog_Mode_Y_Z_Right_Down_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_Servo_State() == STATE_STOP)
	{
	if(Get_Run_Mode() == RUN_MODE_JOG)
	{
		if(Get_Jog_Mode() == JOG_MODE_NONE)
		{
			Set_Jog_Mode(JOG_MODE_DOWN_RIGHT_Y_Z);
		}
		else if(Get_Jog_Mode() == JOG_MODE_DOWN_RIGHT_Y_Z)
		{
			Set_Jog_Mode(JOG_MODE_NONE);
		}
	}
	
}return true;
}

/*****************************************************/
/*************** Data handler function ***************/
/*****************************************************/
bool Servo_Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if (Get_Run_Mode() == RUN_MODE_HOME)
		{
			if(Get_Home_Mode() == HOME_MODE_SERVO)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					switch((int)e_axis)
					{
					case SERVO_1_AXIS: //Frame: '1'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO1].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					case SERVO_2_AXIS: //Frame: '2'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO2].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					case SERVO_3_AXIS: //Frame: '3'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO3].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					case SERVO_4_AXIS: //Frame: '4'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO4].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					case SERVO_5_AXIS: //Frame: '5'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO5].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					case SERVO_6_AXIS: //Frame: '6'' ''Servo_Pos'' ''-xxxxx''\n'
						st_servo_data[SERVO6].f_setpoint_angle += (float)(atoi((char*)ui8_data)/100.0f);
						break;
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
		}
	}
	return true;
}
bool Servo_Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	
	return true;
}
bool Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_TEACH)
		{
			if(Get_Servo_State() == STATE_STOP)
			{
				switch((int)e_axis)
				{
					case X_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
					case Y_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
					case Z_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
					case ROLL_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
					case PITCH_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
					case YAW_AXIS:
						st_pl_coords_data[e_axis].f_next_axis = (float)(atoi(ui8_data)/1000.0f);
						break;
				}
				Calculate_Servo_Deg(st_servo_data);
				Set_Servo_State(STATE_CALCULATING);
			}
		}
	}
	return true;
}
bool Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	return true;
}
bool All_Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	float temp;
	if(Get_System_State() == SYSTEM_STATE_RUN)
	{
		if(Get_Run_Mode() == RUN_MODE_FOLLOW)
		{
			if(Get_Servo_State() == STATE_STOP)
			{
				for(int index = 0; index < 5; index++)
				{
					for(int i = 0; i < MAX_AXIS; i++)
					{
						temp = (float)(atoi((char*)ui8_data + 8*(i + index))*0.0001f);
						if(temp > -18.0f && temp < 18.0f)
						{
							pl_coords_buffer_follow[i][index]= temp;
						}
						else
						{
							if(temp > 0)
							{
								pl_coords_buffer_follow[i][index] = 18.0f;
							}
							else
							{
								pl_coords_buffer_follow[i][index] = -18.0f;
							}
						}
					}
				}
				b_is_new_pl_coords_data = true;
			}
		}
	}
	return true;
}
bool Update_Model_Parametter_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	uint8_t temp_data[8];
	memcpy(temp_data, ui8_data+3, 8);
	float value;
	value = (float)(atoi(temp_data))/10000.0f;
	char *pstr;
	pstr = (char*)(ui8_data+1);
	switch((char)pstr[0]) //'m Para''_''a''_''xxxxxxxx''\n' 
	{
		case 'a':
			Set_Model_Parametter(PLATFORM_JOINT_ANGLE,value);
			PARAMS_Save_Float(EEP_PL_JOINT_ANGLE,value);
			HMI_Update_RegParams_Param(HMI_PL_JOINT_ANGLE,(int32_t)(value*10000));
		break;
		case 'b':
			Set_Model_Parametter(BASE_JOINT_ANGLE,value);
			PARAMS_Save_Float(EEP_BASE_JOINT_ANGLE,value);
			HMI_Update_RegParams_Param(HMI_BASE_JOINT_ANGLE,(int32_t)(value*10000));
		break;
		case 'c':
			Set_Model_Parametter(PLATFORM_RADIUS,value);
			PARAMS_Save_Float(EEP_PL_RADIUS,value);
			HMI_Update_RegParams_Param(HMI_PL_RADIUS,(int32_t)(value*10000));
		break;
		case 'd':
			Set_Model_Parametter(BASE_RADIUS,value);
			PARAMS_Save_Float(EEP_BASE_RADIUS,value);
			HMI_Update_RegParams_Param(HMI_BASE_RADIUS,(int32_t)(value*10000));
		break;
		case 'e':
			Set_Model_Parametter(SERVO_ARM_LENGTH,value);
			PARAMS_Save_Float(EEP_SV_ARM_LENGTH,value);
			HMI_Update_RegParams_Param(HMI_SV_ARM_LENGTH,(int32_t)(value*10000));
		break;
		case 'f':
			Set_Model_Parametter(CONNECTING_ARM_LENGTH,value);
			PARAMS_Save_Float(EEP_CON_ARM_LENGTH,value);
			HMI_Update_RegParams_Param(HMI_CON_ARM_LENGTH,(int32_t)(value*10000));
		break;
		case 'g':
			Set_Model_Parametter(DEFAULT_Z_HEIGHT,value);
			PARAMS_Save_Float(EEP_DEFAUT_Z_HEIGHT,value);
			HMI_Update_RegParams_Param(HMI_DEFAUT_Z_HEIGHT,(int32_t)(value*10000));
		break;
		case 'h':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_1,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_1,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_1,(int32_t)(value*10000));
		break;
		case 'i':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_2,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_2,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_2,(int32_t)(value*10000));
		break;
		case 'j':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_3,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_3,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_3,(int32_t)(value*10000));
		break;
		case 'k':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_4,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_4,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_4,(int32_t)(value*10000));
		break;
		case 'l':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_5,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_5,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_5,(int32_t)(value*10000));
		break;
		case 'm':
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_6,value);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_6,value);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_6,(int32_t)(value*10000));
		break;
	}
	Cal_Permanant_Parameter();
	return true;
}