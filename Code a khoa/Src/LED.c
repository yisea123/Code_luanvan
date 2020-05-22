#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "LED.h"
#include "define.h"
#include "control.h"

uint32_t ui_led_time_count = 0;

void LED_Display(void)
{
	ui_led_time_count++;
	if(ui_led_time_count == F_CTRL) //1s
	{
		ui_led_time_count = 0;
		switch((int)Get_System_State())
		{
			case SYSTEM_STATE_ERROR:
			{
				LED_Off(LED_STANDBY);
				LED_Off(LED_READY);
				LED_Toggle(LED_ERROR);
			}
			break;
			case SYSTEM_STATE_IDLE:
			{
				LED_Off(LED_ERROR);
				LED_Off(LED_READY);
				LED_Toggle(LED_STANDBY);
			}
			break;
			case SYSTEM_STATE_RUN:
			{
				switch((int)Get_Run_Mode())
				{
					case RUN_MODE_NONE:
					{
						LED_Toggle(LED_READY);
						LED_Off(LED_STANDBY);
						LED_Off(LED_ERROR);
					}
						break;
					case RUN_MODE_HOME:
					{
						LED_Off(LED_STANDBY);
						LED_Off(LED_ERROR);
						LED_On(LED_READY);
					}
						break;
					case RUN_MODE_TEST:
					{
						LED_On(LED_STANDBY);
						LED_Off(LED_ERROR);
						LED_On(LED_READY);
					}
						break;
					case RUN_MODE_JOG:
					{
						LED_Toggle(LED_STANDBY);
						LED_Off(LED_ERROR);
						LED_On(LED_READY);
					}
						break;
					case RUN_MODE_TEACH:
					{
						LED_On(LED_STANDBY);
						LED_Off(LED_ERROR);
						LED_Toggle(LED_READY);
					}
						break;
					case RUN_MODE_FOLLOW:
					{
						LED_Toggle(LED_STANDBY);
						LED_Off(LED_ERROR);
						LED_Toggle(LED_READY);
					}
						break;
				}
			}
			break;
		}
	}
}

void LED_Init(void) // Config 3 LED : PC10 - PC12
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(LED_READY_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_READY_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_READY_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(LED_IDLE_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_IDLE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_IDLE_PORT,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(LED_ERROR_PORT_CLK,ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED_ERROR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_ERROR_PORT,&GPIO_InitStructure);
}

void LED_On(LED_TYPE_ENUM e_LED)
{
	switch(e_LED)
	{
		case LED_ERROR:
			GPIO_SetBits(LED_ERROR_PORT, LED_ERROR_PIN);
			break;
		case LED_STANDBY:
			GPIO_SetBits(LED_IDLE_PORT, LED_IDLE_PIN);
			break;
		case LED_READY:
			GPIO_SetBits(LED_READY_PORT, LED_READY_PIN);
			break;
	}
}

void LED_Off(LED_TYPE_ENUM e_LED)
{
	switch(e_LED)
	{
		case LED_ERROR:
			GPIO_ResetBits(LED_ERROR_PORT, LED_ERROR_PIN);
			break;
		case LED_STANDBY:
			GPIO_ResetBits(LED_IDLE_PORT, LED_IDLE_PIN);
			break;
		case LED_READY:
			GPIO_ResetBits(LED_READY_PORT, LED_READY_PIN);
			break;
	}
}

void LED_Toggle(LED_TYPE_ENUM e_LED)
{
	switch(e_LED)
	{
		case LED_ERROR:
			GPIO_ToggleBits(LED_ERROR_PORT, LED_ERROR_PIN);
			break;
		case LED_STANDBY:
			GPIO_ToggleBits(LED_IDLE_PORT, LED_IDLE_PIN);
			break;
		case LED_READY:
			GPIO_ToggleBits(LED_READY_PORT, LED_READY_PIN);
			break;
	}
}

void LED_Off_All(void)
{
	GPIO_ResetBits(LED_ERROR_PORT, LED_ERROR_PIN);
	GPIO_ResetBits(LED_IDLE_PORT, LED_IDLE_PIN);
	GPIO_ResetBits(LED_READY_PORT, LED_READY_PIN);
}
