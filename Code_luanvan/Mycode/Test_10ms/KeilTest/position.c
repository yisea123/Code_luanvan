#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include <Define.h>
#include <arm_math.h>



void home_position(void)
{
	// Reset pulse and dir of all motors
	reset_puldir();
	
	pulse[SERVO4] = 20;
	dir[SERVO4] = 0;
	// Dua truc Z ve home va nang len 1cm
	while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13) != 1)
	{
		set_servo_pulse(pulse[SERVO4],dir[SERVO4],SERVO4);
		pulse_write();
		delay_01ms(100);
	}
	
	FSMC_ENC_Reset();
	
	factor_calc(0,0,joint_1,high);
	factor_calc(0,0,joint_2,high);
	factor_calc(0,0,joint_3,high);
	factor_calc(0,360,joint_4,high);
  write_pulse_4motor(high,15,100);
	
	// Lay vi tri home cho truc 1 va 2 
	pulse[SERVO1] = 5;
	pulse[SERVO2] = 1;
	dir[SERVO1] = 0;
	dir[SERVO2] = 0;
	
	while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) != 1)
	{
		for (int i = 0;i < SERVO3;i++)
		{
			set_servo_pulse(pulse[i],dir[i],i);
		}
		
		pulse_write();
		delay_01ms(15);
	}
	
	// Lay vi tri home cho tay gap
	pulse[SERVO3] = 2;
	dir[SERVO3] = 0;
	while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) != 1)
	{
		set_servo_pulse(pulse[SERVO3],dir[SERVO3],SERVO3);
		pulse_write();
		delay_01ms(15);
	}
	
	
	
	FSMC_ENC_Reset_Counter();
	for(int index = 0; index < MAX_SERVO; index++)
	{
		enc_p0[index] = 0; 
		enc_p1[index] = 0;
	}
	
	for (int i =0;i<SERVO4;i++)
	{
		enc_angle_cur[i] = 0.0;
		last_theta[i] = 0;
		fix_pulse[i] = 0;
	}

	delay_01ms(10000);
	servo_move_home();
 
	// Cap nhat vi tri home
	
	// Motor 3
	last_theta[SERVO3] = 0;
	enc_angle_cur[SERVO3] = 0.0;
	// Motor 2
	last_theta[SERVO2] = 0;
	enc_angle_cur[SERVO2] = 0.0;
	
	
	// Motor 1
	last_theta[SERVO1] = enc_angle_cur[SERVO1]*8.2;
	
	
	
}

void servo_move_home(void)
{
	
	factor_calc(0,738,joint_1,home_speed);
	factor_calc(0,152.5,joint_2,home_speed);
	factor_calc(0,0,joint_3,home_speed);
  factor_calc(0,0,joint_4,home_speed);
	float error =0;
	int k = 0;
	
	write_pulse_4motor(home_speed,11,200);
	
}

void get_position(float xf, float yf,float phi, float speed)
{

	// Tinh toan goc khop tai vi tri moi
	inverse_kinematic(xf,yf,phi,speed);
	
	float pulse_row_m3;
 
	
	// Xuat xung cho dong co
	
	float error =0;
	int k = 0;
	
	write_pulse_4motor(speed,100,200);
  
}

void test_motor(float speed)
{
	int k = 0;
	float pulse_row_m3 = pulse_row_base;
	float error = 0;
	int cover_step = (int)(speed*50.0);
	
	// Tinh toan he so a0..a5 cho 4 dong co
	
	factor_calc(last_theta[SERVO1],new_theta[SERVO1],joint_1,speed);
	factor_calc(last_theta[SERVO2],new_theta[SERVO2],joint_2,speed);
	factor_calc(last_theta[SERVO3],new_theta[SERVO3],joint_3,speed);
	factor_calc(last_theta[SERVO4],new_theta[SERVO4],joint_4,speed);
	

	for (int step = 1;step <=(speed*100);step++)
	{
		pulse_total(step);
		
		for (int i = 0;i < 4;i++)
		{
			set_servo_pulse(pulse[i],dir[i],i);
			
		}
		
		// Kich phat xung
		pulse_write();
		delay_01ms(100);
		
		FSMC_ENC_Update();
		
		if (step%100 == 0)
		{
			for (int servo = 0;servo< MAX_SERVO; servo++)
			{
				
					enc_pos[servo] = FSMC_ENC_Get_Pos(servo);
										
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
				  
					else
					{
						error = enc_pos[servo] - new_theta[servo];
					}
					// Calculate missing pulses
				//	fix_pulse[servo] = fix_pulse_calc(error,pulse_row_base,dir[servo]);
			}
		}
		else
		{
			// do nothing
		}
	}
	reset_fix_pulse();
}



void write_pulse_4motor(float speed,uint8_t delay_time,int cover_step)
{
	float error;
	float pulse_row_m3 = pulse_row_base;
	int k =0;
	

	for (int step = 1;step <=(speed*cycle);step++)
		{
			pulse_total(step);
			
			for (int i = 0;i < 4;i++)
			{
				set_servo_pulse(pulse[i],dir[i],i);
				
			}
			
			// Kich phat xung
			pulse_write();
			delay_01ms(delay_time);
			
			FSMC_ENC_Update();
			if (step%cover_step == 0)
			{
				for (int servo = 0;servo< MAX_SERVO; servo++)
				{
					
						enc_pos[servo] = FSMC_ENC_Get_Pos(servo);
						
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
						//fix_pulse[servo] = fix_pulse_calc(error,pulse_row_base,dir[servo]);
				}
			}
			else
			{
				// do nothing
			}
		}
			reset_fix_pulse();
}

void reset_puldir()
{
	for (int i =0;i< MAX_SERVO;i++)
	{
		pulse[i] = 0;
		dir[i] = 0;
	}
}

void write_pulse_motor(float speed,uint8_t delay_time,int cover_step)
{
	
}

