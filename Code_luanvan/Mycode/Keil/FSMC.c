#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//#include "FSMC.h"
//#include "control.h"
#include "Define.h"

#define d_PULSE_BASE_ADDR 				0x60000000

//#define d_ENC_BASE_ADDR 					0x60000020
#define d_MAX_PULSE			 		50
#define d_MIN_PULSE			 		0
//#define d_PULSE_PRESCALE 					4.0f
#define d_MAX_PULSE_CYCLE					32678 //20KHz
#define d_MIN_PULSE_CYCLE					0

uint32_t data_width, data_cycle;

//static float enc_angle_cur[MAX_SERVO] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//static float enc_dp[MAX_SERVO] = {0, 0, 0, 0, 0, 0};
//static int32_t enc_p0[MAX_SERVO] = {0, 0, 0, 0, 0, 0}; 
//static int32_t enc_p1[MAX_SERVO] = {0, 0, 0, 0, 0, 0};


//uint16_t FSMC_Read(uint32_t ui_enc_channel)
//{
//    /*
//        + HADDR are internal AHB address lines that are translated to external memory 
//        + NOR/PSRAM address mapping:
//            - Bank select: HADDR[27:26]
//                . 00, 01, 10, 11: Bank 1 NOR/PSRAM 1,2,3,4
//            - Data width & external memory: HADDR[25:0]
//                . 16 bit: HADDR[25:1]>>1 --> FSMC uses HADDR[25:1] to generate the address for external memory FSMC_A[24:0]  
//            - BANK_NOR_ADDR
//        + 4 encoder channnels have 4 address: 0x00,0x01,0x02,0x03
//    */
//    uint32_t address;
//    uint16_t res = 0;

//    address = (d_ENC_BASE_ADDR + (ui_enc_channel<<1));
//    res = *(volatile uint16_t *)address;   // casting the address to pointer then get value in this address
//    return res;
//}

void FSMC_Write(uint32_t ui_address, uint32_t ui_data)
{
	*(volatile uint16_t *)ui_address = ui_data;  
}


// i_pulse_width[] la mang chua gia tri xung xuat cho servo
void Servo_pulse( uint32_t i_pulse_1, uint32_t Dir_1)
{
	
	uint32_t ui_delay;
	uint32_t ui_pulse_out = 0;
//	for(ui_index = 0; ui_index < 6; ui_index++)
//	{
		if(i_pulse_1 > d_MAX_PULSE)
		{
			i_pulse_1 = d_MAX_PULSE;
		}
			
		ui_pulse_out = (Dir_1 << 7); //Chon chieu quay
		ui_pulse_out += i_pulse_1;
		
		//while(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) == Bit_SET);
		FSMC_Write(d_PULSE_BASE_ADDR, ui_pulse_out);
		ui_pulse_out = 0;
		ui_delay = 50;
		while(ui_delay--);
	
}


	















//void FSMC_ENC_Update(void)
//{
//	float servo_pulse_offset[MAX_SERVO] = {1, 1, 2, 1, 1, 1};
//	for(int index = 0; index < MAX_SERVO; index++)
//	{
//		enc_p0[index] = (int32_t)FSMC_Read(index);
//		enc_dp[index] = (float)((enc_p0[index] - enc_p1[index])/servo_pulse_offset[index]);
//		if (enc_dp[index] > 32768)
//			enc_dp[index] -= 65536;
//		else if (enc_dp[index] < -32768)
//			enc_dp[index] += 65536;
//		enc_p1[index] = enc_p0[index];
//		enc_angle_cur[index] += (enc_dp[index] * DRIVER_PULSE_TO_DEGREE) / (d_PULSE_PRESCALE);
//	}		
//}
//void FSMC_ENC_Reset(void)
//{
//	for(int index = 0; index < MAX_SERVO; index++)
//	{
//		enc_angle_cur[index] = 0.0;
//	}
//}

//float FSMC_ENC_Get_Pos(SERVO_ENUM e_servo)
//{
//	if(e_servo >= MAX_SERVO)
//		return 0;
//	float temp;
//	switch((int)e_servo)
//	{
//		case SERVO1:
//			temp = enc_angle_cur[SERVO1];
//		break;
//		case SERVO2:
//			temp = enc_angle_cur[SERVO2];
//		break;
//		case SERVO3:
//			temp = enc_angle_cur[SERVO3];
//		break;
//		case SERVO4:
//			temp = enc_angle_cur[SERVO4];
//		break;
//		case SERVO5:
//			temp = enc_angle_cur[SERVO5];
//		break;
//		case SERVO6:
//			temp = enc_angle_cur[SERVO6];
//		break;
//	}
//	return temp;
//}
//float FSMC_ENC_Get_Vel(SERVO_ENUM e_servo)
//{
//	if(e_servo >= MAX_SERVO)
//		return 0;
//	float temp;
//	switch((int)e_servo)
//	{
//		case SERVO1:
//			temp = enc_dp[SERVO1]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//		case SERVO2:
//			temp = enc_dp[SERVO2]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//		case SERVO3:
//			temp = enc_dp[SERVO3]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//		case SERVO4:
//			temp = enc_dp[SERVO4]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//		case SERVO5:
//			temp = enc_dp[SERVO5]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//		case SERVO6:
//			temp = enc_dp[SERVO6]*F_CTRL*DRIVER_PULSE_TO_DEGREE;
//		break;
//	}
//	return temp;
//}
