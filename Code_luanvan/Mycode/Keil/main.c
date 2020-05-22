#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Define.h"
#include "system_timetick.h"




//uint8_t 	txbuff[BUFF_SIZE];
//uint8_t 	rxbuff[BUFF_RX];
bool rx_flag = false;
uint32_t pulse_1 = 0;
uint8_t Dir=0;
uint32_t pulse = 0;
uint8_t pulse_count[pulse_size];
uint8_t pulse_count2[pulse_size];
void delay_01ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock /2 /(PSC+1) = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

 	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

int main()
{
	pulse_count[0] = 1;
	uint32_t ui_delay;
	init_UART_DMA();
	FSMC_Init();
	EXTILine2_Init();
	Servo_pulse(0,0);
	
	while(1)
	{
			if (rx_flag)
			{
				Servo_pulse(pulse,Dir);
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				ui_delay = 10;
				while(ui_delay--);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
				rx_flag = false;
			}
			txbuff[0] = 'O';
			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
			DMA1_Stream3->NDTR = 1;		
			DMA_Cmd(DMA1_Stream3, ENABLE);
			delay_01ms(500);
		

	}
}
//Ngat RX DMA
void DMA1_Stream1_IRQHandler(void)
{
  
	/* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
	
	// frame: pulse-dir (xx-x)
	//
	pulse = (rxbuff[0] - 0x30)*10 +(rxbuff[1] - 0x30);
	Dir = rxbuff[3] - 0x30;
	pulse_1 = (Dir << 7);
	pulse_1 |= pulse;
	rx_flag = true;
	DMA_Cmd(DMA1_Stream1, ENABLE);
	
}
// Ngat button de stop motor
//void EXTI15_10_IRQHandler(void)
//{
//	 /* Make sure that interrupt flag is set */
//    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
//        /* Do your stuff when PB2 is changed */
//				
//			
//				i_pulse_cycle = 0;
//        i_pulse_width = 0;
//				Servo_pulse_cycle(i_pulse_cycle,Dir);
//				Servo_pulse_width(i_pulse_width,Dir);
//			
//        /* Clear interrupt flag */
//        EXTI_ClearITPendingBit(EXTI_Line12);
//    }
//}


