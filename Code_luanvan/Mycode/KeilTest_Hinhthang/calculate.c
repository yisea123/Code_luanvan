#include <math.h>
#include <stdio.h>
#include <string.h>
#include <Define.h>
#include <arm_math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

// Tinh toan so luong xung cho truc 
// k (step): lan phat xung thu k
const float32_t A_1s[36] = {1.0  ,0.0   ,0.0   ,0.0  ,0.0 ,0.0,
															 0.0  ,1.0   ,0.0   ,0.0  ,0.0 ,0.0,
															 0.0 ,0.0   ,0.5 ,0.0  ,0.0 ,0.0,
															 -10.0,-6.0  ,-1.5,10.0 ,-4.0,0.5,
															 15.0 ,8.0   ,1.5 ,-15.0,7.0 ,-1.0,
															 -6.0 ,-3.0  ,-0.5,6.0  ,-3.0 ,0.5};

const float32_t A_25s[36] = {1.0,    0.0,   0.0,  0.0,       0.0,   0.0,
														0.0,    1.0,   0.0,  0.0,       0.0,   0.0,
														0.0,    0.0,   0.5,  0.0,       0.0,   0.0,
													 -0.64,   -0.96,   -0.6,    0.64,   -0.64,    0.2,
														0.384,    0.512,    0.24,   -0.384,    0.4479,   -0.16,
													 -0.06144,   -0.0768,   -0.032,    0.06144,   -0.0768,    0.032};

const float32_t A_4s[36] = {1.0,         0.0,         0.0,         0.0,         0.0,         0.0,
																 0.0,   1.0 ,       0.0  ,       0.0  ,       0.0  ,      0.0,
																 0.0,         0.0,    0.5,         0.0,         0.0,         0.0,
													 -5.0/32.0,   -0.375,   -0.375,    5.0/32.0,   -0.25,    0.125,
														15.0/256.0,    0.125,    3.0/32.0,   -15.0/256.0,   7.0/64.0,   -0.0625,
													 -3.0/512.0,   -3.0/256.0,   -1.0/128.0,    3.0/512.0,   -3.0/256.0,    1.0/128.0};
										

// Tinh gia tri xung co ca 4 truc 
void pulse_total(uint16_t k,float tf)
{   
	  float t;
	  t = (float)(k*interval);
		pulse_calc(SERVO1,t,tf);
		pulse_calc(SERVO2,t,tf);
		pulse_calc(SERVO3,t,tf);
		pulse_calc(SERVO4,t,tf);
	
}

// Ham tinh gia tri xung va dir cho 1 truc
void pulse_calc(SERVO_ENUM servo,float t,float tf)
{
	float temp;
//	float error;
	if(vmax[servo] == 0)
	{
		pulse[servo] = 0;
		dir[servo] = 0;
	}
	else
	{
		  if (t == (tf-0.002))
			{
			    int a = 0 ;
			}
				// Tinh toan vi tri moi sau moi chu ki
			if (t<=tc[servo])
			{
				temp = (vmax[servo]/(2*tc[servo]))*(2*t-interval)*interval;
			}
		  else if ( (tc[servo] < t) && (t<= (tf-tc[servo])))
			{
				temp = vmax[servo]*interval;
			}
			else if (((tf-tc[servo]) < t) &&(t <= tf))
			{
				temp = (vmax[servo]/(2*tc[servo]))*0.001*(2*(tf-t)+0.001);
			}
			else
			{
				// do nothing
			}
			
			last_theta[servo] += temp;

			// Tinh toan luong xung phat ra 
			temp = temp*pulse_row_base/360.0;
				
				
			// Xac dinh DIR  
			if (temp <= 0)
			{
				temp = -temp; 
				dir[servo] = 0;
			}
			else 
			{
				 dir[servo] = 1;
			}
			
//			if (servo == SERVO3)
//			{
//				if (dir[SERVO3] == 0)
//				{
//					temp = temp*0.9171;
//				}
//				else{}
//			}
//			else{}
				
				
			// Cong don xung
			if ((temp - (int)temp) > 0.7)
			{
				 pulse[servo] = (int)temp +1;
				 add[servo]= add[servo] + (temp-(int)temp) - 1 ;
			}	
			else 
			{
				pulse[servo] = (int)temp;
				add[servo]+=(temp-(int)temp);
			}
			
			// Kiem tra, bu xung bi thieu
			if (add[servo] >=1)
			{
				add[servo]--;
				pulse[servo]++;
			}
			else if (add[servo] <=-1)
			{
				if( pulse[servo] > 0)
				add[servo]++;
				pulse[servo]--;
			}
			else
			{
				// do nothing
			}
			
			// Bu xung thieu tu encoder
//				if(fix_pulse[servo] > 0)	
//			{
//				pulse[servo]++;
//				fix_pulse[servo]--;
//			}
//			else if(fix_pulse[servo] < 0)
//			{
//				if (pulse[servo] > 1)
//				{
//				pulse[servo]--;
//				fix_pulse[servo]++;
//				}
//				else
//				{
//					// do nothing
//				}
//			}
//			else
//			{
//				// do nothing
//			}
			
			
					
			// Update old position by new position
			
			//last_theta[servo] = new_theta[servo];
	}
		
}


// Ham tinh cac he so a0..a5 cho quy dao bac 5 
void factor_calc(SERVO_ENUM servo,float32_t theta1_0,float32_t theta1_f, float time)
{
  if (servo == SERVO3)
	{		
		if (theta1_f>theta1_0)
		{
		}
		else{}
	}
	else{}
			if (theta1_0 != theta1_f)
			{
				vmax[servo] = (theta1_f-theta1_0)/(time)*(4.0/3.0);
				tc[servo] = time/4.0;
				
			}
			else
			{
				vmax[servo] = 0;
				tc[servo] = 0;
			}
		
	if (servo == SERVO3)
	{		
		if (theta1_f<theta1_0)
		{
				vmax[servo] = vmax[servo]*0.9915f;
		}
		else
		{
			  vmax[servo] = vmax[servo]*(360.0f/359.8f);
		}
	}
	else
	{
		// do nothing
	}
}


// Ham giai dong hoc nguoc de tinh toan gia tri goc theta o diem dau va diem cuoi
void inverse_kinematic(float xf, float yf,float phi, float time)
{
	/* new_theta[SERVO1] --> khop 1
		 new_theta[SERVO2] --> khop 2
     new_theta[SERVO3] --> khop gap
	   new_theta[SERVO4] --> truc z */
	float c2,s2;
	float temp2,temp1,temp0;
		
	// Tinh goc cua cac khop tai diem co toa do (xf,yf)
	c2 = (xf*xf + yf*yf - L1*L1 - L2*L2)/(2*L1*L2);
	s2 = sqrt(1-c2*c2);
	
	temp2 = atan2(s2,c2);
	temp1 = (atan2(yf,xf) - atan2(L2*sin(temp2),L1+L2*cos(temp2)))*RAD2DEG*8.2;
	
	temp2 = -temp2;
	temp0 = (atan2(yf,xf) - atan2(L2*sin(temp2),L1+L2*cos(temp2)))*RAD2DEG*8.2;
	
	if ((fabs(temp1-last_theta[SERVO1])-fabs(temp0-last_theta[SERVO1])) >= 0)
	{
		new_theta[SERVO2] = temp2*RAD2DEG;
		new_theta[SERVO1] = temp0;
	}
	else
	{
		new_theta[SERVO2] = -temp2*RAD2DEG;
		new_theta[SERVO1] = temp1;
	}
	
	if (new_theta[SERVO1]/8.2f <=90)
	{
		new_theta[SERVO3] = 180.0 - (new_theta[SERVO1]/8.2f + new_theta[SERVO2] + phi);
	}
	else
	{
		new_theta[SERVO3] = 180.0 - new_theta[SERVO1]/8.2f - new_theta[SERVO2] - phi;
	}
	
	if (new_theta[SERVO3] > 180.0)
	{
		new_theta[SERVO3] -= 360.0;
	}
	else
	{
		// do nothing
	}
	// Tinh cac he so cho khop 1
	factor_calc(SERVO1,last_theta[SERVO1],new_theta[SERVO1],time);
	// Tinh cac he so cho khop 2
	factor_calc(SERVO2,last_theta[SERVO2],new_theta[SERVO2],time);
	// Tinh cac he so cho khop gap
	factor_calc(SERVO3,last_theta[SERVO3],new_theta[SERVO3],time);
}


// Doc encoder va kiem tra, bu xung
void read_fix_pulse(SERVO_ENUM servo,int k,int cover_step)
{
	float error = 0;
	FSMC_ENC_Update();
	if (k%cover_step == 0)
			{
				for (int servo = 0;servo< MAX_SERVO; servo++)
				{
						if (servo == SERVO3)
						{
							enc_pos[servo] = FSMC_ENC_Get_Pos_m3(dir[servo]);
						}
						else
						{
							enc_pos[servo] = FSMC_ENC_Get_Pos(servo);
						}
						
						if (servo == SERVO1)
						{
							error = enc_pos[servo]*8.2f - new_theta[servo];
							
						}
						else if (servo == SERVO2)
						{
							error = enc_pos[servo] - new_theta[servo];
							error_arr[k] = error;
							k++;
						}
					
						// Calculate missing pulses
						fix_pulse[servo] = fix_pulse_calc(error,25000,dir[servo]);
				}
			}
			else
			{
				// do nothing
			}
}

void reset_fix_pulse(void)
{
		for (int i =0;i<4;i++)
	{
		fix_pulse[i] = 0;
	}
}
int fix_pulse_calc(float error,float pulse_row,int dir)
{
	int temp;
		if(fabs(error) > 0.5)
		{
			temp = -(error*pulse_row/360);
		}
		else
		{
			// do nothing
		}
		
		if (dir == 0)
		{
			temp = -temp;
		}
		else
		{
			// do nothing
		}
		return temp;
}



// Tinh cac tham so cho van toc hinh thang




// Delay 0.1ms
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
