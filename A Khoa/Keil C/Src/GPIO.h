#ifndef __GPIO_H
#define __GPIO_H
#include <stdint.h>
#include <stdbool.h>
#include "control.h"

#define BUTTON_START_PORT 									GPIOA
#define BUTTON_START_PORT_CLK								RCC_AHB1Periph_GPIOA
#define BUTTON_START_PIN        						GPIO_Pin_3
#define BUTTON_START_PIN_SOURCE 						GPIO_PinSource3
#define BUTTON_START_EXTI_LINE							EXTI_Line3
#define BUTTON_START_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOA
#define BUTTON_START_EXTI_PIN_SOURCE				EXTI_PinSource3
#define BUTTON_START_EXTI_IRQn							EXTI3_IRQn

#define BUTTON_STOP_PORT 										GPIOA
#define BUTTON_STOP_PORT_CLK								RCC_AHB1Periph_GPIOA
#define BUTTON_STOP_PIN        							GPIO_Pin_6
#define BUTTON_STOP_PIN_SOURCE 							GPIO_PinSource6
#define BUTTON_STOP_EXTI_LINE								EXTI_Line6
#define BUTTON_STOP_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOA
#define BUTTON_STOP_EXTI_PIN_SOURCE					EXTI_PinSource6
#define BUTTON_STOP_EXTI_IRQn								EXTI9_5_IRQn

#define BUTTON_HOME_PORT 										GPIOA
#define BUTTON_HOME_PORT_CLK								RCC_AHB1Periph_GPIOA
#define BUTTON_HOME_PIN        							GPIO_Pin_4
#define BUTTON_HOME_PIN_SOURCE 							GPIO_PinSource4
#define BUTTON_HOME_EXTI_LINE								EXTI_Line4
#define BUTTON_HOME_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOA
#define BUTTON_HOME_EXTI_PIN_SOURCE					EXTI_PinSource4
#define BUTTON_HOME_EXTI_IRQn								EXTI4_IRQn

#define HOME_SERVO1_PORT 										GPIOB
#define HOME_SERVO1_PORT_CLK								RCC_AHB1Periph_GPIOB
#define HOME_SERVO1_PIN        							GPIO_Pin_12
#define HOME_SERVO1_PIN_SOURCE 							GPIO_PinSource12
#define HOME_SERVO1_EXTI_LINE								EXTI_Line12
#define HOME_SERVO1_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOB
#define HOME_SERVO1_EXTI_PIN_SOURCE					EXTI_PinSource12
#define HOME_SERVO1_EXTI_IRQn								EXTI15_10_IRQn

#define HOME_SERVO2_PORT 										GPIOB
#define HOME_SERVO2_PORT_CLK								RCC_AHB1Periph_GPIOB
#define HOME_SERVO2_PIN        							GPIO_Pin_13
#define HOME_SERVO2_PIN_SOURCE 							GPIO_PinSource13
#define HOME_SERVO2_EXTI_LINE								EXTI_Line13
#define HOME_SERVO2_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOB
#define HOME_SERVO2_EXTI_PIN_SOURCE					EXTI_PinSource13
#define HOME_SERVO2_EXTI_IRQn								EXTI15_10_IRQn

#define HOME_SERVO3_PORT 										GPIOB
#define HOME_SERVO3_PORT_CLK								RCC_AHB1Periph_GPIOB
#define HOME_SERVO3_PIN        							GPIO_Pin_14
#define HOME_SERVO3_PIN_SOURCE 							GPIO_PinSource14
#define HOME_SERVO3_EXTI_LINE								EXTI_Line14
#define HOME_SERVO3_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOB
#define HOME_SERVO3_EXTI_PIN_SOURCE					EXTI_PinSource14
#define HOME_SERVO3_EXTI_IRQn								EXTI15_10_IRQn

#define HOME_SERVO4_PORT 										GPIOB
#define HOME_SERVO4_PORT_CLK								RCC_AHB1Periph_GPIOB
#define HOME_SERVO4_PIN        							GPIO_Pin_15
#define HOME_SERVO4_PIN_SOURCE 							GPIO_PinSource15
#define HOME_SERVO4_EXTI_LINE								EXTI_Line15
#define HOME_SERVO4_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOB
#define HOME_SERVO4_EXTI_PIN_SOURCE					EXTI_PinSource15
#define HOME_SERVO4_EXTI_IRQn								EXTI15_10_IRQn

#define HOME_SERVO5_PORT 										GPIOA
#define HOME_SERVO5_PORT_CLK								RCC_AHB1Periph_GPIOA
#define HOME_SERVO5_PIN        							GPIO_Pin_2
#define HOME_SERVO5_PIN_SOURCE 							GPIO_PinSource2
#define HOME_SERVO5_EXTI_LINE								EXTI_Line2
#define HOME_SERVO5_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOA
#define HOME_SERVO5_EXTI_PIN_SOURCE					EXTI_PinSource2
#define HOME_SERVO5_EXTI_IRQn								EXTI2_IRQn

#define HOME_SERVO6_PORT 										GPIOA
#define HOME_SERVO6_PORT_CLK								RCC_AHB1Periph_GPIOA
#define HOME_SERVO6_PIN        							GPIO_Pin_1
#define HOME_SERVO6_PIN_SOURCE 							GPIO_PinSource1
#define HOME_SERVO6_EXTI_LINE								EXTI_Line1
#define HOME_SERVO6_EXTI_PORT_SOURCE				EXTI_PortSourceGPIOA
#define HOME_SERVO6_EXTI_PIN_SOURCE					EXTI_PinSource1
#define HOME_SERVO6_EXTI_IRQn								EXTI1_IRQn

#define RELAY_1_PORT 				GPIOB
#define RELAY_1_PORT_CLK		RCC_AHB1Periph_GPIOB
#define RELAY_1_PIN         GPIO_Pin_5

#define RELAY_2_PORT 				GPIOB
#define RELAY_2_PORT_CLK		RCC_AHB1Periph_GPIOB
#define RELAY_2_PIN         GPIO_Pin_4

#define RELAY_3_PORT 				GPIOB
#define RELAY_3_PORT_CLK		RCC_AHB1Periph_GPIOB
#define RELAY_3_PIN         GPIO_Pin_3

#define RELAY_4_PORT 				GPIOA
#define RELAY_4_PORT_CLK		RCC_AHB1Periph_GPIOC
#define RELAY_4_PIN         GPIO_Pin_7

#define RELAY_5_PORT 				GPIOB
#define RELAY_5_PORT_CLK		RCC_AHB1Periph_GPIOC
#define RELAY_5_PIN         GPIO_Pin_2

#define RELAY_6_PORT 				GPIOB
#define RELAY_6_PORT_CLK		RCC_AHB1Periph_GPIOC
#define RELAY_6_PIN         GPIO_Pin_1

typedef enum
{
	RELAY_1,
	RELAY_2,
	RELAY_3,
	RELAY_4,
	RELAY_5,
	RELAY_6,
}RELAY_ENUM;

void Button_Init(void); 

void Servo_Home_Switch_Init(void); 
void Relay_Init(void);
void Relay_On(RELAY_ENUM e_relay);
void Relay_Off(RELAY_ENUM e_relay);
void Reset_Button_Press_Status(void);
bool Get_Button_Press_Status(void);
uint8_t Get_Button_Start_Press_Status(void);
uint8_t Get_Button_Stop_Press_Status(void);
uint8_t Get_Button_Home_Press_Status(void);
uint8_t Get_Servo_Home_Status(SERVO_ENUM e_servo);
#endif
