#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "misc.h"
#include "system_timetick.h"
#include "system_state.h"
#include "IMU_Quest.h"                  /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "GPIO.h"
#include "LED.h"
#include "control.h"

extern SERVO_DATA_STRUCT st_servo_data[MAX_SERVO];
bool b_is_button_press = false;

void Button_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
//NVIC_InitTypeDef NVIC_InitStructure;
//EXTI_InitTypeDef EXTI_InitStructure;
//	
	//Button start init
	RCC_AHB1PeriphClockCmd(BUTTON_START_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = BUTTON_START_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_START_PORT,&GPIO_InitStructure);
//	
//	SYSCFG_EXTILineConfig(BUTTON_START_EXTI_PORT_SOURCE,BUTTON_START_EXTI_PIN_SOURCE);
//	
//	EXTI_InitStructure.EXTI_Line = BUTTON_START_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = BUTTON_START_EXTI_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//Button stop init
	RCC_AHB1PeriphClockCmd(BUTTON_STOP_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = BUTTON_STOP_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_STOP_PORT,&GPIO_InitStructure);
//	
//	SYSCFG_EXTILineConfig(BUTTON_STOP_EXTI_PORT_SOURCE,BUTTON_STOP_EXTI_PIN_SOURCE);
//	EXTI_InitStructure.EXTI_Line = BUTTON_STOP_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = BUTTON_STOP_EXTI_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
	//Button home init
	RCC_AHB1PeriphClockCmd(BUTTON_HOME_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = BUTTON_HOME_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_HOME_PORT,&GPIO_InitStructure);
	
//	SYSCFG_EXTILineConfig(BUTTON_HOME_EXTI_PORT_SOURCE,BUTTON_HOME_EXTI_PIN_SOURCE);
//	EXTI_InitStructure.EXTI_Line = BUTTON_HOME_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = BUTTON_HOME_EXTI_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

void Servo_Home_Switch_Init(void)  // Config Servo Home Switch:  
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// For Servo 1 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO1_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO1_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO1_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO1_EXTI_PORT_SOURCE,HOME_SERVO1_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO1_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO1_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// For Servo 2 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO2_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO2_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO2_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO2_EXTI_PORT_SOURCE,HOME_SERVO2_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO2_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO2_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// For Servo 3 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO3_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO3_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO3_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO3_EXTI_PORT_SOURCE,HOME_SERVO3_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO3_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO3_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// For Servo 4 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO4_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO4_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO4_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO4_EXTI_PORT_SOURCE,HOME_SERVO4_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO4_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO4_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// For Servo 5 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO5_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO5_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO5_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO5_EXTI_PORT_SOURCE,HOME_SERVO5_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO5_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO5_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// For Servo 6 switch
	RCC_AHB1PeriphClockCmd(HOME_SERVO6_PORT_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = HOME_SERVO6_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(HOME_SERVO6_PORT, & GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(HOME_SERVO6_EXTI_PORT_SOURCE,HOME_SERVO6_EXTI_PIN_SOURCE);
	
	EXTI_InitStructure.EXTI_Line = HOME_SERVO6_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = HOME_SERVO6_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Relay_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RELAY_1_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_1_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RELAY_2_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_2_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RELAY_3_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_3_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RELAY_4_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_4_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_4_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RELAY_5_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_5_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_5_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RELAY_6_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = RELAY_6_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RELAY_6_PORT,&GPIO_InitStructure);
}

void Relay_On(RELAY_ENUM e_relay)
{
	switch(e_relay)
	{
		case RELAY_1:
			GPIO_SetBits(RELAY_1_PORT,RELAY_1_PIN);
			break;
		case RELAY_2:
			GPIO_SetBits(RELAY_2_PORT,RELAY_2_PIN);
			break;
		case RELAY_3:
			GPIO_SetBits(RELAY_3_PORT,RELAY_3_PIN);
			break;
		case RELAY_4:
			GPIO_SetBits(RELAY_4_PORT,RELAY_4_PIN);
			break;
		case RELAY_5:
			GPIO_SetBits(RELAY_5_PORT,RELAY_5_PIN);
			break;
		case RELAY_6:
			GPIO_SetBits(RELAY_6_PORT,RELAY_6_PIN);
			break;
	}
}

void Relay_Off(RELAY_ENUM e_relay)
{
	switch(e_relay)
	{
		case RELAY_1:
			GPIO_ResetBits(RELAY_1_PORT,RELAY_1_PIN);
			break;
		case RELAY_2:
			GPIO_ResetBits(RELAY_2_PORT,RELAY_2_PIN);
			break;
		case RELAY_3:
			GPIO_ResetBits(RELAY_3_PORT,RELAY_3_PIN);
			break;
		case RELAY_4:
			GPIO_ResetBits(RELAY_4_PORT,RELAY_4_PIN);
			break;
		case RELAY_5:
			GPIO_ResetBits(RELAY_5_PORT,RELAY_5_PIN);
			break;
		case RELAY_6:
			GPIO_ResetBits(RELAY_6_PORT,RELAY_6_PIN);
			break;
	}
}
//Function Interrupt For Home Switch
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(BUTTON_HOME_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(BUTTON_HOME_EXTI_LINE);
		Home_Handler(0,0,0);
	}
}

//Function Interrupt For Servo Home
void EXTI15_10_IRQHandler(void)
{
	//For Servo 1
	if(EXTI_GetITStatus(HOME_SERVO1_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO1_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_1_AXIS);
	}
	
	//For Servo 2
	if(EXTI_GetITStatus(HOME_SERVO2_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO2_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_2_AXIS);
	}
	
	//For Servo 3
	if(EXTI_GetITStatus(HOME_SERVO3_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO3_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_3_AXIS);
	}
	
	//For Servo 4
	if(EXTI_GetITStatus(HOME_SERVO4_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO4_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_4_AXIS);
	}
}
// Function Interrupt For Home Servo 6
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(HOME_SERVO6_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO6_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_6_AXIS);
	}
}
// Function Interrupt For Home Servo 5
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(HOME_SERVO5_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(HOME_SERVO5_EXTI_LINE);
		Set_Axis_Home_Done(st_servo_data, SERVO_5_AXIS);
	}
}
// Function Interrupt For Start Button
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(BUTTON_START_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(BUTTON_START_EXTI_LINE);
		Start_Handler(0,0,0);
	}
}
//Function Interrupt For Stop Button
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(BUTTON_STOP_EXTI_LINE) != RESET)
	{
		EXTI_ClearITPendingBit(BUTTON_STOP_EXTI_LINE);
		Stop_Handler(0,0,0);
	}
}

void Reset_Button_Press_Status(void)
{
	b_is_button_press = false;
}

uint8_t Get_Button_Start_Press_Status(void)
{
	return GPIO_ReadInputDataBit(BUTTON_START_PORT, BUTTON_START_PIN);
}

uint8_t Get_Button_Stop_Press_Status(void)
{
	return GPIO_ReadInputDataBit(BUTTON_STOP_PORT, BUTTON_STOP_PIN);
}

uint8_t Get_Button_Home_Press_Status(void)
{
	return GPIO_ReadInputDataBit(BUTTON_HOME_PORT, BUTTON_HOME_PIN);
}
uint8_t Get_Servo_Home_Status(SERVO_ENUM e_servo)
{
	uint8_t temp;
	switch((int)e_servo)
	{
		case SERVO1:
			temp = GPIO_ReadInputDataBit(HOME_SERVO1_PORT,HOME_SERVO1_PIN);
			break;
		case SERVO2:
			temp = GPIO_ReadInputDataBit(HOME_SERVO2_PORT,HOME_SERVO2_PIN);
			break;
		case SERVO3:
			temp = GPIO_ReadInputDataBit(HOME_SERVO3_PORT,HOME_SERVO3_PIN);
			break;
		case SERVO4:
			temp = GPIO_ReadInputDataBit(HOME_SERVO4_PORT,HOME_SERVO4_PIN);
			break;
		case SERVO5:
			temp = GPIO_ReadInputDataBit(HOME_SERVO5_PORT,HOME_SERVO5_PIN);
			break;
		case SERVO6:
			temp = GPIO_ReadInputDataBit(HOME_SERVO6_PORT,HOME_SERVO6_PIN);
			break;
	}
	return temp;
}
