#include <stdint.h>
#include <stdbool.h>
//#include "control.h"

#define DRIVER_PULSE_PER_REV 50000
#define DRIVER_DEGREE_TO_PULSE (float)(DRIVER_PULSE_PER_REV / 360.0f)
#define DRIVER_PULSE_TO_DEGREE (float) (360.0f / DRIVER_PULSE_PER_REV)
	
//#define PI (float)3.141592653589793f
//#define RAD2DEG (float)180.0f/PI
//#define DEG2RAG (float)PI/180.0f
#define F_CTRL	1000
#define T_CTRL	(float)(1.0f/F_CTRL)
//#define Debug_CPLD

#define MAX_CLK 20000000 //20MHz
#define		BUFF_RX			4
#define		BUFF_SIZE			1 //73
#define   pulse_size 1000
extern uint8_t 	txbuff[BUFF_SIZE];
extern uint8_t 	rxbuff[BUFF_RX];
extern uint8_t  pulse_count[pulse_size];

typedef enum
{
	SERVO1,
	SERVO2,
	SERVO3,
	SERVO4,
	SERVO5,
	SERVO6,
	MAX_SERVO
}
SERVO_ENUM;

//FSMC_PWM
void FSMC_Write(uint32_t ui_address, uint32_t ui_data);
uint16_t FSMC_Read(uint32_t ui_enc_channel);

void FSMC_Init(void);

// Ham rai xung cho driver
void Servo_pulse( uint32_t i_pulse_1, uint32_t Dir_1);

//void Servo_Move_Test(int32_t i_pulse_num);


//FSMC_Encoder
void FSMC_ENC_Update(void);
float FSMC_ENC_Get_Pos(SERVO_ENUM e_servo);
float FSMC_ENC_Get_Vel(SERVO_ENUM e_servo);
void FSMC_ENC_Reset(void);

//Uart_DMA
void init_UART_DMA(void);

//External Interrupt
void EXTILine2_Init(void);
