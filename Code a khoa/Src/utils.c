#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"
#include "define.h"
#include "utils.h"


float Sub_Mod_Pi(float angle1,float angle2)//result:-PI->PI
{
	if (angle1-angle2 > 180.0)
		return angle1-angle2-360.0;
	else if (angle1-angle2 < -180.0)
		return angle1-angle2+360.0;
	else
		return angle1-angle2;
}


float Add_Mod_Pi(float angle1,float angle2)//result:-PI->PI
{
	if (angle1+angle2 > 180.0)
		return angle1+angle2-360.0;
	else if (angle1+angle2 < -180.0)
		return angle1+angle2+360.0;
	else
		return angle1+angle2;
}


/**	@brief Convert euler rate to body rate
 *	@param[in]	euler_angle		Array of roll, pitch, yaw angles
 *	@param[in]	euler_rate 		Array of roll, pitch, yaw angle rate
 *	@param[out]	body_rate 		Array of body rates
 *	@return			void.
 */
void Euler_To_Body_Rate(float *euler_angle, float *euler_rate, float *body_rate)
{
	body_rate[0] = euler_rate[0]-sin(euler_angle[1])*euler_rate[2];
	body_rate[1] = cos(euler_angle[0])*euler_rate[1]+sin(euler_angle[0])*cos(euler_angle[1])*euler_rate[2];
	body_rate[2] = -sin(euler_angle[0])*euler_rate[1]+cos(euler_angle[0])*cos(euler_angle[1])*euler_rate[2];	
}


/**	@brief Convert body rate to euler rate
 *	@param[in]	euler_angle		Array of roll, pitch, yaw angles
 *	@param[out]	euler_rate 		Array of roll, pitch, yaw angle rate
 *	@param[in]	body_rate 		Array of body rates
 *	@return			void.
 */
void Body_To_Euler_Rate(float *euler_angle, float *euler_rate, float *body_rate)
{
	euler_rate[0] = body_rate[0]+sin(euler_angle[0])*tan(euler_angle[1])*body_rate[1]+cos(euler_angle[0])*tan(euler_angle[1])*body_rate[2];
	euler_rate[1] = cos(euler_angle[0])*body_rate[1]-sin(euler_angle[0])*body_rate[2];
	euler_rate[2] = sin(euler_angle[0])/cos(euler_angle[1])*body_rate[1]+cos(euler_angle[0])/cos(euler_angle[1])*body_rate[2];
}


void my_delay_us(uint32_t micros)
{
	RCC_ClocksTypeDef RCC_Clocks;	
	/* Get system clocks */
	RCC_GetClocksFreq(&RCC_Clocks);	
	micros = micros * (RCC_Clocks.HCLK_Frequency / 4000000) - 10;
  /* 4 cycles for one loop */
	while (micros--);
}


void delay_us(uint16_t period)
{

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void delay_01ms(uint16_t period)
{

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void IntToStr2(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
	 else y[0] = ' ';
	y[1] = a + 0x30;
}
void IntToStr3(int32_t u, uint8_t *y)
{
	int32_t a;
 
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
	 else y[0] = ' ';
	y[2] = a % 10 + 0x30; 
	a = a / 10; 
	y[1] = a + 0x30;
}
void IntToStr4(int32_t u, uint8_t *y)
{
	int32_t a;
	a = u;
	if (a<0)
	{ 
		a = -a; 
		y[0] = '-';
	}
	else y[0] = ' ';
	y[3] = a % 10 + 0x30; 
	a = a / 10; 
	y[2] = a % 10 + 0x30; 
	a = a / 10; 
	y[1] = a + 0x30;
}
void IntToStr5(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';
     
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
void IntToStr6(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';

   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}


void IntToStr7(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';

   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}


void IntToStr8(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';

	 y[7] = a % 10 + 0x30; 
   a = a / 10;
   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
void IntToStr9(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';
	 y[8] = a % 10 + 0x30; 
   a = a / 10;
	 y[7] = a % 10 + 0x30; 
   a = a / 10;
   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
void IntToStr10(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';
	 y[9] = a % 10 + 0x30; 
   a = a / 10;
	 y[8] = a % 10 + 0x30; 
   a = a / 10;
	 y[7] = a % 10 + 0x30; 
   a = a / 10;
   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
void IntToStr11(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';
	 y[10] = a % 10 + 0x30; 
   a = a / 10;	
	 y[9] = a % 10 + 0x30; 
   a = a / 10;
	 y[8] = a % 10 + 0x30; 
   a = a / 10;
	 y[7] = a % 10 + 0x30; 
   a = a / 10;
   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;	
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
void IntToStr12(int32_t u, uint8_t *y)
{
	int32_t a;
     
   a = u;
   if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
   else y[0] = ' ';
	 y[11] = a % 10 + 0x30; 
   a = a / 10;
	 y[10] = a % 10 + 0x30; 
   a = a / 10;
	 y[9] = a % 10 + 0x30; 
   a = a / 10;
	 y[8] = a % 10 + 0x30; 
   a = a / 10;
   y[7] = a % 10 + 0x30; 
   a = a / 10;	
   y[6] = a % 10 + 0x30; 
   a = a / 10;
   y[5] = a % 10 + 0x30; 
   a = a / 10;     
   y[4] = a % 10 + 0x30; 
   a = a / 10;
   y[3] = a % 10 + 0x30; 
   a = a / 10;
   y[2] = a % 10 + 0x30; 
   a = a / 10;
   y[1] = a + 0x30;
}
int my_atoi(char *p)
{
	int res=0;
	int sign=1;
	while (((*p<'0') || (*p>'9')) && (*p != '-'))// check infinite loop
	{ 
		p++;
	}
	if (*p == '-')
	{
		sign = -1;
		p++;
	}	
	while ((*p>='0') && (*p<='9'))
	{
		res = res*10 + (*p-'0');
		p++;
	}
	res *= sign;
	return res;	
}
