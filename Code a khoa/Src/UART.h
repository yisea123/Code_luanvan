#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "control.h"

#define d_PC_RX_BUFF_SIZE 				512
#define d_PC_TX_BUFF_SIZE 				512

#define d_DMA_RX_BUFF_SIZE				512
#define d_DMA_TX_BUFF_SIZE				512


#define d_CMD_MSG_LEN 						100
#define d_DATA_NUM_BYTE						30

#define PC_USART             USART3
#define PC_USART_CLK         RCC_APB1Periph_USART3
#define PC_PORT              GPIOB
#define PC_PORT_CLK						RCC_AHB1Periph_GPIOB
#define PC_TX                GPIO_Pin_10
#define PC_TX_SOURCE         GPIO_PinSource10
#define PC_RX                GPIO_Pin_11
#define PC_RX_SOURCE         GPIO_PinSource11
#define PC_AF                GPIO_AF_USART3
#define PC_BAUDRATE          (uint32_t)921600
#define PC_DATA_REG          (uint32_t)PC_USART + 0x04
#define PC_TX_DMA_STREAM     DMA1_Stream3
#define PC_TX_DMA_CHANNEL    DMA_Channel_4
#define PC_TX_STREAM_IRQ     DMA1_Stream3_IRQn
#define PC_TX_DMA_FLAG       DMA_FLAG_TCIF3   
#define PC_RX_DMA_STREAM     DMA1_Stream1
#define PC_RX_DMA_CHANNEL    DMA_Channel_4
#define PC_RX_STREAM_IRQ     DMA1_Stream1_IRQn
#define PC_RX_DMA_FLAG       DMA_FLAG_TCIF1



void UART_PC_Init(void);

void UART_PC_Send(uint8_t *ui8_data, uint32_t ui32_data_cnt);
void UART_PC_Send_Feedback(char *c_msg, uint32_t ui32_msg_len);

void UART_Read_Cmd(void);
void UART_PC_Send_PL_Pos(void);
void UART_PC_Send_Servo_Data_Pos(void);
void UART_PC_Send_PL_Pos_Test(void);
void UART_PC_Send_Servo_Data_Pos_Test(void);
typedef bool (*CMD_HANDLER_FUNC)(AXIS_ENUM, uint8_t*, uint32_t);

typedef struct
{
	char* c_cmd_msg;
	uint32_t ui32_data_num_byte;
	CMD_HANDLER_FUNC b_handler;
}CMD_HANDLER_STRUCT;

typedef enum
{
	START,
	STOP,
	SERVO_POS,
	SERVO_VEL,
	POS,
	VEL,
	ALL_POS,
	HOME_SERVO,
	HOME_MODEL,
	MODEL_PARA,
	MODEL_CONTROL,
	MODEL_TEST,
	P_TEST,
	X_TEST,
	Y_TEST,
	Z_TEST,
	ROLL_TEST,
	PITCH_TEST,
	YAW_TEST,
	CIRCLE_TEST,
	SQUARE_TEST,
	RESET_SERVO,
	SERVO_RESET,
	SERVO_ON_OFF,
	MODEL_RESET,
	MODEL_UPDATE,
	SERVO_UPDATE,
	MAX_CMD,
}CMD_ENUM;
#endif