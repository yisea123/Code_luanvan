#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Define.h"

#define d_PULSE_BASE_ADDR 				0x60000000
#define d_ENC_BASE_ADDR 					0x60000020
#define d_MAX_PULSE			 		50
#define d_MIN_PULSE			 		0
#define d_PULSE_PRESCALE 					4.0f
#define d_MAX_PULSE_CYCLE					32678 //20KHz
#define d_MIN_PULSE_CYCLE					0

const float enc_pulse_per_row[4] = {100000.0,350.0,350.0,1000.0};
float servo_pulse_offset[MAX_SERVO] = {4.1,1.0,1.0,1.0};

uint32_t data_width, data_cycle;



void FSMC_ENC_Update(void)
{
	
	for(int index = 0; index < MAX_SERVO; index++)
	{
		enc_p0[index] = (int32_t)FSMC_Read(index);
		enc_dp[index] = (float)(enc_p0[index] - enc_p1[index]);
		if (enc_dp[index] > 32768)
			enc_dp[index] -= 65536;
		else if (enc_dp[index] < -32768)
			enc_dp[index] += 65536;
		enc_p1[index] = enc_p0[index];
		if (index != SERVO3)
		{
			enc_angle_cur[index] += (enc_dp[index] * 360.0f) / (enc_pulse_per_row[index]*d_PULSE_PRESCALE*servo_pulse_offset[index]); 
		}
		else
		{
			if (enc_dp[index] < 0)
			{
				enc_angle_cur[index] += (enc_dp[index] * 360.0f) / (322.24f*d_PULSE_PRESCALE); 
			}
			else 
			{
				enc_angle_cur[index] += (enc_dp[index] * 360.0f) / (325.18f*d_PULSE_PRESCALE); 
			}
		}
	}		
}

void FSMC_ENC_Reset(void)
{
	
	for(int index = 0; index < MAX_SERVO; index++)
	{
		enc_angle_cur[index] = 0.0;
		enc_p0[index] = 0; 
		enc_p1[index] = 0;
	}
	GPIO_SetBits(ENC_RST_PORT, ENC_RST_PIN);
	delay_01ms(5);
	GPIO_ResetBits(ENC_RST_PORT, ENC_RST_PIN);
	
}

void FSMC_ENC_Reset_Counter(void)
{
	GPIO_SetBits(ENC_RST_PORT, ENC_RST_PIN);
	delay_01ms(5);
	GPIO_ResetBits(ENC_RST_PORT, ENC_RST_PIN);
	
}


int32_t FSMC_ENC_Get_Counter(SERVO_ENUM e_servo)
{
	//FSMC_ENC_Update();

	return enc_p0[e_servo];
	
}

float FSMC_ENC_Get_Pos(SERVO_ENUM e_servo)
{
  //FSMC_ENC_Update();
	 
	return enc_angle_cur[e_servo];
 }


float FSMC_ENC_Get_Pos_m3(int dir)
{
  //FSMC_ENC_Update();

	if (dir ==0)
	{
	  return enc_angle_cur[SERVO3]/0.9171;
	}
	else
	{
		return enc_angle_cur[SERVO3];
	}
}

uint16_t FSMC_Read(uint32_t ui_enc_channel)
{
    /*
        + HADDR are internal AHB address lines that are translated to external memory 
        + NOR/PSRAM address mapping:
            - Bank select: HADDR[27:26]
                . 00, 01, 10, 11: Bank 1 NOR/PSRAM 1,2,3,4
            - Data width & external memory: HADDR[25:0]
                . 16 bit: HADDR[25:1]>>1 --> FSMC uses HADDR[25:1] to generate the address for external memory FSMC_A[24:0]  
            - BANK_NOR_ADDR
        + 4 encoder channnels have 4 address: 0x00,0x01,0x02,0x03
    */
    uint32_t address;
    uint16_t res = 0;
		
    address = (d_ENC_BASE_ADDR + (ui_enc_channel<<1));
    res = *(volatile uint16_t *)address;   // casting the address to pointer then get value in this address
    return res;
}



void FSMC_Write(uint32_t ui_address, uint32_t ui_data)
{
	*(volatile uint16_t *)ui_address = ui_data;  
}


// i_pulse_width[] la mang chua gia tri xung xuat cho servo
void set_servo_pulse(uint8_t pulse, uint8_t dir,int i)
{
	i*=2;
	uint32_t ui_delay;
	uint32_t ui_pulse_out = 0;
	
	if(pulse > d_MAX_PULSE)
	{
		pulse = d_MAX_PULSE;
	}
		
	ui_pulse_out = (dir << 7); //Chon chieu quay
	ui_pulse_out += pulse;
	
	
		FSMC_Write(d_PULSE_BASE_ADDR + i, ui_pulse_out);
		ui_pulse_out = 0;
		ui_delay = 50;
		while(ui_delay--);
	  // Kich phat xung
//		GPIO_SetBits(GPIOD, GPIO_Pin_12);
//		ui_delay = 10;
//		while(ui_delay--);
//		GPIO_ResetBits(GPIOD, GPIO_Pin_12);

}

void pulse_write(void)
{
		// Kich phat xung 2 lan 
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			int	ui_delay = 10;
			while(ui_delay--);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			ui_delay = 10;
			while(ui_delay--);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			
}

