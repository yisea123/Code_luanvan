#ifndef LED_H
#define LED_H

#define LED_READY_PORT 				GPIOC
#define LED_READY_PORT_CLK		RCC_AHB1Periph_GPIOC
#define LED_READY_PIN         GPIO_Pin_3

#define LED_IDLE_PORT 				GPIOA
#define LED_IDLE_PORT_CLK			RCC_AHB1Periph_GPIOA
#define LED_IDLE_PIN          GPIO_Pin_15

#define LED_ERROR_PORT 				GPIOC
#define LED_ERROR_PORT_CLK		RCC_AHB1Periph_GPIOC
#define LED_ERROR_PIN         GPIO_Pin_12

typedef enum 
{
	LED_ERROR,
	LED_STANDBY,
	LED_READY,
}LED_TYPE_ENUM;


void LED_Init(void); 
void LED_On(LED_TYPE_ENUM e_LED);
void LED_Off(LED_TYPE_ENUM e_LED);
void LED_Toggle(LED_TYPE_ENUM e_LED);
void LED_Display(void);
void LED_Off_All(void);
#endif