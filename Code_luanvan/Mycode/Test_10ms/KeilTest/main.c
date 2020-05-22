#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Define.h"
#include "system_timetick.h"
#include "arm_math.h"

uint8_t rx_flag = 0;
int ui_delay;

float theta_0 = 0, theta_f = 0;
const float pulse_row_base = 10000;

float32_t xf, yf,phi;

// Khai bao mang luu du lieu
float32_t last_theta[MAX_SERVO], new_theta[MAX_SERVO];
float32_t joint_1[6] = {0.0}, joint_2[6] = {0.0}, joint_3[6] = {0.0},joint_4[6] = {0.0};
uint8_t pulse[MAX_SERVO],dir[MAX_SERVO];
int fix_pulse[MAX_SERVO]={0};
float add[MAX_SERVO];

/*================================================*/

// Mang tinh toan encoder

float enc_angle_cur[MAX_SERVO] = {0.0,0.0,0.0,0.0};
float enc_dp[MAX_SERVO] = {0.0,0.0,0.0,0.0};
int32_t enc_p0[MAX_SERVO] = {0,0,0,0}; 
int32_t enc_p1[MAX_SERVO] = {0,0,0,0};
/*============================================*/

float enc_pos[MAX_SERVO] = {0.0};
int counter[MAX_SERVO] ={0};

float pos_timer =0;
int quang = 0;
uint8_t temp;
float error_arr[50];
float array[20];
float timer_arr[20];
float k = 0;
int test;

int pos_temp =0;
int main()
{
	SysTick_Config(SystemCoreClock / 1000);
	
//	uint32_t ui_delay;
	init_UART_DMA();
	FSMC_Init();
	EXTILine2_Init();
	PWM_Init();
	
	delay_01ms(10000);
	TIM_SetCompare3(TIM2,0);
	rx_flag = 0;
	FSMC_ENC_Reset();
	for (int i =0;i<MAX_SERVO;i++)
	{
		pulse[i] = 0;
		dir[i] = 0;
		set_servo_pulse(pulse[i],dir[i],i);
		fix_pulse[i] = 0;
	}
	pulse_write();
	
	/* ========= HOME POSITIONING ========= */
		int ui_delay;

	/* ==================================== */
	while(1)
	{
		/*Nho cau hinh lai thach anh 8Mhz/20Mhz */
		/* Nho goi ham FSMC_ENC_Update*/
				

		/*======Doc encoder=======*/
//		last_theta[SERVO1] = 738;
//		inverse_kinematic(-10,30,med);
				//FSMC_ENC_Update();
//				c = FSMC_ENC_Get_Counter(SERVO2);
//				p = FSMC_ENC_Get_Pos(SERVO2);  

//      pulse[SERVO4] = 5;
//		  dir[SERVO4] = 1;
//		  set_servo_pulse(pulse[SERVO4],dir[SERVO4],3);
//		  pulse_write();
//		  delay_01ms(10);
		
			if (rx_flag)
			{
				switch(rxbuff[0])
				{
					case 'h': 
					{
						home_position();
						TIM_SetCompare3(TIM2,923);
						delay_01ms(10000);
						TIM_SetCompare3(TIM2,0);
//						pulse[SERVO2] = 5;
//						dir[SERVO2] = 1;
//						set_servo_pulse(pulse[SERVO2],dir[SERVO2],SERVO2);
//						pulse_write();
//						delay_01ms(15);
//						FSMC_ENC_Update();
						break;
					}
					    
					case 't': 
					{
						// Cap nhat cac gia tri goc ban dau cho cac dong co
//						 for (int i =0; i < MAX_SERVO;i++)
//							{
//								last_theta[i] = enc_angle_cur[i];
//							}
//							
						// Tinh toan vi tri cuoi tu chuoi rxbuff nhan duoc
						 for (int i = 0 ;i < 4;i++)
							{
								pos_temp = (rxbuff[5*i+3]-0x30)*100 + (rxbuff[5*i+4]-0x30)*10 + (rxbuff[5*i+5]-0x30);
								if (rxbuff[5*i+2] == '0')
								{
									new_theta[i] = last_theta[i] + pos_temp;
								}
								else
								{
									new_theta[i] = last_theta[i] - pos_temp;
								}
							}
							test_motor(1.0f);
							break;
					}
					
					case 'g':
					{
						// Cap nhat cac gia tri goc ban dau cho cac dong co
						
						// Motor 1 
						last_theta[SERVO1] = enc_angle_cur[SERVO1]*8.2;
						
						// Motor 2,3,4
						for (int i =1; i < MAX_SERVO;i++)
							{
								last_theta[i] = enc_angle_cur[i];
							}
							
						xf = (rxbuff[3]-0x30)*10+(rxbuff[4]-0x30) + (rxbuff[5]-0x30)*0.1 + (rxbuff[6]-0x30)*0.01;
						yf = (rxbuff[9]-0x30)*10+(rxbuff[10]-0x30) + (rxbuff[11]-0x30)*0.1 + (rxbuff[12]-0x30)*0.01;
						phi = (rxbuff[15]-0x30)*100+(rxbuff[16]-0x30)*10 + (rxbuff[17]-0x30) + (rxbuff[18]-0x30)*0.1;
							
						if (rxbuff[2] == '1')
						{
								xf = -xf;
						}
						else {}
						
						if (rxbuff[8] == '1')
						{
							  yf = -yf;
						}
						else{}
							
						if (rxbuff[13] == '1')
						{
							  phi = -phi;
						}
						else{}
							
						get_position(xf,yf,phi,med);
						break;
					}
					case 'o':
					{
					  TIM_SetCompare3(TIM2,629);
						break;
					}
							
				}
				
				rx_flag = 0;
				
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
	/* Clear the DMA1_Stream1 TCIF1 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
	
	// frame: new_theta (x-xxxx-xxxx)
	rx_flag = 1;
	DMA_Cmd(DMA1_Stream1, ENABLE);
	
}


