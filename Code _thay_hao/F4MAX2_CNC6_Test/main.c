#include "stm32f4xx.h"
#include "system_timetick.h"
#include "driver.h"
#include "math.h"
#include "stdio.h"
#define PI 3.14159265

uint16_t encval0, encval1, encval2;
float a1 = 197, a2 = 185, d3 = 110;
float angleonestep1 = 0.0045, angleonestep2 = 0.0028125, angleonestep3 = 0.72;
float Tf = 2000;
float Ts = 1;
float last_theta1 = -56.009293 , last_theta2 = 118.004646 , last_theta3 = 720;
float delta_x, delta_y, delta_z, PULSE1, PULSE2, PULSE3, Xk, Yk, Zk ;
uint16_t DIR1, DIR2, DIR3;
float X_batdau, Y_batdau, Z_batdau, X_muctieu, Y_muctieu, Z_muctieu;
float *inverse_kinematic(float x, float y, float z);
int calculate_pulse(float X_batdau,float Y_batdau, float Z_batdau, float X_muctieu, float Y_muctieu, float Z_muctieu, uint16_t K, uint16_t Kf);


int main(void)
{	
	uint16_t i;
	int16_t j=0;  
	
	uint16_t K = 0;
	uint16_t Kf = Tf/Ts;
	float muctieu_X[] = {197,197,197,210,217,217,210,197,205,210,217,217,210,197,197,227,227,227,227,247,247,227,247,247,197,257,257,260,274,277,277,197};
	float muctieu_Y[] = {0,0,40,40,36,24,20,20,20,20,16,4,0,0,0,40,40,0,0,40,40,20,0,0,0,40,5,0,0,5,40,0};
	float muctieu_Z[] = {10,0,0,0,0,0,0,0,20,0,0,0,0,0,10,10,0,0,10,10,0,0,0,10,10,0,0,0,0,0,0,10};
	uint16_t n = 32;
	float odd1=0, odd2=0,odd3=0;
	uint16_t PULSEOUT1, PULSEOUT2, PULSEOUT3, PULSE_RDN1, PULSE_RDN2, PULSE_RDN3;
	

	delay_01ms(30000); // when set period = 10000, waiting loop do not exit, why?
										// -> test set/clear bit in while(1) to see what happend
  
	/* Enable SysTick at 1ms */
	
  SysTick_Config(SystemCoreClock/1000); 
	
	delay_01ms(100);
  
	init_main();	
	
	/*
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/
	//uint32_t count =0;
  while(1){
		
		
		if (tick_flag){
			tick_flag = 0;	
			
			//if (tick_count >= 1)	
			//{
			//tick_count = 0;
				if(j < n-1)
				{
					X_batdau = muctieu_X[j];
					Y_batdau = muctieu_Y[j];
					Z_batdau = muctieu_Z[j];
					X_muctieu = muctieu_X[j+1];
					Y_muctieu = muctieu_Y[j+1];
					Z_muctieu = muctieu_Z[j+1];
					calculate_pulse(X_batdau, Y_batdau, Z_batdau, X_muctieu, Y_muctieu, Z_muctieu, K, Kf);
					if(K<Kf)
					{
						
						odd1 += PULSE1;// MOTOR 1
						PULSE_RDN1 = floor(odd1);
						PULSEOUT1 = PULSE_RDN1;
						odd1 -= PULSEOUT1;
					
						fsmc_write(CS_DDA1, PULSEOUT1 + DIR1);
							
						GPIO_SetBits(GPIOD, GPIO_Pin_12);
						for(i=0; i<20; i++);
						GPIO_ResetBits(GPIOD, GPIO_Pin_12);
							
						odd2 += PULSE2;// MOTOR 2
						PULSE_RDN2 = floor(odd2);
						PULSEOUT2 = PULSE_RDN2;
						odd2 -= PULSEOUT2;
						
						fsmc_write(CS_DDA2, PULSEOUT2 + DIR2);
							
						GPIO_SetBits(GPIOD, GPIO_Pin_12);
						for(i=0; i<20; i++);
						GPIO_ResetBits(GPIOD, GPIO_Pin_12);
						
						odd3 += PULSE3;// MOTOR 3
						PULSE_RDN3 = floor(odd3);
						PULSEOUT3 = PULSE_RDN3;
						odd3 -= PULSEOUT3;
						
						fsmc_write(CS_DDA3, PULSEOUT3 + DIR3);
							
						GPIO_SetBits(GPIOD, GPIO_Pin_12);
						for(i=0; i<20; i++);
						GPIO_ResetBits(GPIOD, GPIO_Pin_12);
						
						K++;
					}
					else if(K==Kf)
					{
						K=0;
						j++;
						odd1 = 0;
						odd2 = 0;
						odd3 = 0;
						fsmc_write(CS_DDA1,0);
						fsmc_write(CS_DDA2,0);
						fsmc_write(CS_DDA3,0);


					}
				}	
					
		 }

			//j = !j;
			
			/*count++;
			if(count<=6400){
				//fsmc_write(CS_DDA1, 3 + 0x8000 );		
				fsmc_write(CS_DDA2, 3 + 0x8000 );
				// fsmc_write(CS_DDA3,  +0x8000 );
				//fsmc_write(CS_DDA4, 20); 
				//fsmc_write(CS_DDA5, 50);
				//fsmc_write(CS_DDA6, 100); 
				
				while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) == Bit_SET){};
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				for(i=0; i<20; i++);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
					
					
			}
			
			else
				{ 
					
					//fsmc_write(CS_DDA1, 0);
					fsmc_write(CS_DDA2, 0);
					//fsmc_write(CS_DDA3, 0);
			//	fsmc_write(CS_DDA4, 20 + 0x8000); 
			//	fsmc_write(CS_DDA5, 50 + 0x8000);
			//	fsmc_write(CS_DDA6, 100+ 0x8000); 
				
				while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) == Bit_SET){};
				GPIO_SetBits(GPIOD, GPIO_Pin_12);
				for(i=0; i<20; i++);
				GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			}*/
				
				//for PWM
				//fsmc_write(STEPPER_BASE_ADDR, 100);
				//fsmc_write(STEPPER_BASE_ADDR + (1<<1), 250); 
				//fsmc_write(STEPPER_BASE_ADDR + (2<<1), 500);
				//fsmc_write(STEPPER_BASE_ADDR + (3<<1), 750); 
				//while(1);
					
			encval0 = fsmc_read(CS_ENC1);
			encval1 = fsmc_read(CS_ENC2);
			encval2 = fsmc_read(CS_ENC3);
		
		
		/*
		if (tick_count == 100)
		{
			tick_count = 0;
			GPIO_ToggleBits(GPIOA, GPIO_Pin_1);
		}
		*/

		//}
	}
}

float *inverse_kinematic(float x, float y, float z)
{
	double c1, s1, c2, s2;
	static float delta_theta[3];
	float new_theta1, new_theta2, new_theta3;
	c2 = (pow(x,2) + pow(y,2) - pow(a1,2) - pow(a2,2))/(2*a1*a2);
	s2 = sqrt(1-pow(c2,2));
	c1 = ((a1 + a2*c2)*x + (a2*s2*y))/(pow(x,2) + pow(y,2));
	s1 = ((a1 + a2*c2)*y - a2*s2*x)/(pow(x,2) + pow(y,2));
	new_theta1 = (atan2(s1,c1))*(180/PI); // khop 1
	new_theta2 = (atan2(s2,c2))*(180/PI); // khop 2
	new_theta3 = z*72; // khop 3
	delta_theta[0] = new_theta1 - last_theta1;
	delta_theta[1] = new_theta2 - last_theta2;
	delta_theta[2] = new_theta3 - last_theta3;
	last_theta1 = new_theta1;
	last_theta2 = new_theta2;
	last_theta3 = new_theta3;
	return delta_theta;
	
}

int calculate_pulse(float X_batdau, float Y_batdau, float Z_batdau, float X_muctieu, float Y_muctieu, float Z_muctieu, uint16_t K, uint16_t Kf)
{
	float *gocquay, Xi, Yi, Zi;
	if(K<Kf)
	{
		delta_x = (X_muctieu - X_batdau);
		delta_y = (Y_muctieu - Y_batdau);
		delta_z = (Z_muctieu - Z_batdau);
		Xk = X_batdau +(K*delta_x)/Kf;
		Yk = Y_batdau +(K*delta_y)/Kf;
		Zk = Z_batdau +(K*delta_z)/Kf;
		
		gocquay = inverse_kinematic(Xk, Yk, Zk);
		
		PULSE1 = *gocquay/(angleonestep1);
		if(PULSE1 < 0)
		{
			DIR1 = 0x8000;
			PULSE1 = -PULSE1;
		}
		else
		{
			DIR1 = 0;
		}
		PULSE2 = *(gocquay+1)/(angleonestep2);
		if(PULSE2 < 0)
		{
			DIR2 = 0;
			PULSE2 = -PULSE2;
		}
		else
		{
			DIR2 = 0x8000;
		}
		PULSE3 = *(gocquay+2)/(angleonestep3);
		if(PULSE3 < 0)
		{
			DIR3 = 0X8000;
			PULSE3 = -PULSE3;
		}
		else
		{
			DIR3 = 0;
		}
	}
}


