#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
//#include "system_state.h"
//#include "IMU_Quest.h"                  /* Model's header file */
//#include "rtwtypes.h"                  /* MathWorks types */
#include "string.h"
#include "UART.h"
#include "CALCULATOR.h"
#include "utils.h"
#include "control.h"
#include "FSMC.h"

//uint8_t ui8_pc_tx_buff[d_PC_TX_BUFF_SIZE] = {0};
uint8_t ui8_pc_rx_buff[d_PC_RX_BUFF_SIZE] = {0};

uint8_t ui8_dma_rx_buff[d_DMA_RX_BUFF_SIZE] = {0};
uint8_t ui8_dma_tx_buff[d_DMA_TX_BUFF_SIZE] = {0};

uint8_t frame[d_PC_TX_BUFF_SIZE] = {0};
uint32_t len = 0;
int32_t temp;

bool b_update_platform_pos_enable = false;
bool b_update_servo_pos_enable = false;

// Extern function
extern bool Start_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Stop_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Servo_Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Servo_Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Vel_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool All_Pos_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Home_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Home_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Update_Model_Parametter_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Pulse_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_X_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Y_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Z_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Roll_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Pitch_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Yaw_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Circle_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Test_Square_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Model_Test_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Set_Model_Control_Mode_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Reset_Servo_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Servo_On_Off_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Reset_Model_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Enable_Update_Data_Platform_Pos(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
bool Enable_Update_Data_Servo_Pos(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);

// Struct variable
CMD_HANDLER_STRUCT st_cmd_handler[MAX_CMD] = 
{
	{"s Start", 2, Start_Handler},
	{"s Stop", 2, Stop_Handler},
	{"Servo_Pos", 7, Servo_Pos_Handler},
	{"Servo_Vel", 4, Servo_Vel_Handler},
	{"Pos", 7, Pos_Handler},
	{"All_Pos", 241, All_Pos_Handler},
	{"Vel", 7, Vel_Handler},
	{"s Home", 2, Home_Handler},
	{"m Home", 2, Home_Model_Handler},
	{"m Para", 12, Update_Model_Parametter_Handler},
	{"m Control", 3, Set_Model_Control_Mode_Handler},
	{"m Test", 3, Set_Model_Test_Mode_Handler},
	{"p Test", 2, Set_Test_Pulse_Mode_Handler},
	{"x Test", 2, Set_Test_X_Mode_Handler},
	{"y Test", 2, Set_Test_Y_Mode_Handler},
	{"z Test", 2, Set_Test_Z_Mode_Handler},
	{"roll Test", 2, Set_Test_Roll_Mode_Handler},
	{"pitch Test", 2, Set_Test_Pitch_Mode_Handler},
	{"yaw Test", 2, Set_Test_Yaw_Mode_Handler},
	{"c Test", 2, Set_Test_Circle_Mode_Handler},
	{"s Test", 2, Set_Test_Square_Mode_Handler},
	{"r Servo", 2, Reset_Servo_Handler},
	{"ServoOnOff", 6, Servo_On_Off_Handler},
	{"r Model", 2, Reset_Model_Handler},
	{"p Update", 2, Enable_Update_Data_Platform_Pos},
	{"s Update", 2, Enable_Update_Data_Servo_Pos},
};

// Static variable
static uint8_t ui8_cmd_msg[d_CMD_MSG_LEN];
static uint32_t ui32_cmd_msg_cnt = 0;
static uint32_t ui32_data_byte_idx = 0;
static uint32_t ui32_remain_byte = 0;


// Enum variable
CMD_ENUM e_current_cmd;
AXIS_ENUM e_current_axis;

// Main function
void UART_PC_Init(void) // Config UART4: STM to PC: (PA0-TX, PA1-RX); 
{
	RCC_AHB1PeriphClockCmd(PC_PORT_CLK, ENABLE);
	
	//Init GPIO pin PA0, PA1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PC_TX|PC_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PC_PORT,&GPIO_InitStructure);
	
	//Set alternate function
	GPIO_PinAFConfig(PC_PORT, PC_TX_SOURCE, PC_AF);  
	GPIO_PinAFConfig(PC_PORT, PC_RX_SOURCE, PC_AF);
	
	//Configure UART4
	RCC_APB1PeriphClockCmd(PC_USART_CLK,ENABLE);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = PC_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(PC_USART,&USART_InitStructure);
	
	/* Enable global interrupts for UART */
//	NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_Init(&NVIC_InitStructure);
	
	//Enable UART
	USART_Cmd(PC_USART, ENABLE);
	USART_ClearFlag(PC_USART, USART_FLAG_TC);
	
/* Enable IDLE line detection for DMA processing */
//	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
	
	//Setup for DMA UART4 Interrupt
//	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = PC_TX_STREAM_IRQ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//Setup for UART4 Rx DMA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	DMA_DeInit(PC_RX_DMA_STREAM);
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = 							PC_RX_DMA_CHANNEL; 
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = 			(uint32_t)&ui8_dma_rx_buff[0];
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)PC_USART + 0x04;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = 							DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = 				DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Circular;
	DMA_InitStructure.DMA_BufferSize = 						d_DMA_RX_BUFF_SIZE;
	DMA_InitStructure.DMA_MemoryBurst = 					DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = 			DMA_PeripheralBurst_Single;
	DMA_Init(PC_RX_DMA_STREAM, &DMA_InitStructure);
	
	USART_DMACmd(PC_USART, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(PC_RX_DMA_STREAM, ENABLE);
	
	//Setup for UART4 Tx DMA
	DMA_DeInit(PC_TX_DMA_STREAM);
	
	DMA_InitStructure.DMA_Channel = 							PC_TX_DMA_CHANNEL; 
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr = 			(uint32_t)&ui8_dma_tx_buff[0];
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)PC_USART + 0x04;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = 							DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = 				DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_BufferSize = 						0;
	DMA_InitStructure.DMA_MemoryBurst = 					DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = 			DMA_PeripheralBurst_Single;
	
	DMA_Init(PC_TX_DMA_STREAM, &DMA_InitStructure);
	
	// Enable DMA Stream Transfer Complete interrupt
	DMA_ITConfig(PC_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
	
	USART_DMACmd(PC_USART, USART_DMAReq_Tx, ENABLE);
	DMA_Cmd(PC_TX_DMA_STREAM, ENABLE);
	
}


void UART_PC_Send(uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	DMA_Cmd(PC_TX_DMA_STREAM, DISABLE);
	memcpy(ui8_dma_tx_buff, ui8_data, ui32_data_cnt);
	DMA_ClearFlag(PC_TX_DMA_STREAM, PC_TX_DMA_FLAG);
	PC_TX_DMA_STREAM->NDTR = ui32_data_cnt;
	DMA_Cmd(PC_TX_DMA_STREAM, ENABLE);
}
void UART_PC_Send_Model_Data(void)
{
	uint8_t frame[52];
	uint32_t len = 0;
	int32_t temp;
	frame[len++] = 'm';
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(PLATFORM_JOINT_ANGLE)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(BASE_JOINT_ANGLE)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(PLATFORM_RADIUS)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(BASE_RADIUS)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(SERVO_ARM_LENGTH)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(CONNECTING_ARM_LENGTH)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(DEFAULT_Z_HEIGHT)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_1)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_2)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_3)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_4)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_5)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	temp =(int32_t)(Get_Model_Parametter(ANGLE_OF_SERVO_ARM_6)*10000);
	IntToStr8(temp,&frame[len]);
	len += 8;
	frame[len++] = '.';
	frame[len++] = '\r';
	frame[len++] = '\n';
}
void UART_PC_Send_PL_Pos(void)
{
	len = 0;
	temp = 0;
	frame[len++] = 'p';
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(X_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(Y_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(Z_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(ROLL_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(PITCH_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(YAW_AXIS)*10);
	IntToStr4(temp,&frame[len]);
	len += 4;
	frame[len++] = '.';
//	frame[len++] = '\r';
	frame[len++] = '\n';
	UART_PC_Send((uint8_t*)frame,len);
}
void UART_PC_Send_PL_Pos_Test(void)
{
	len = 0;
	temp = 0;
//	frame[len++] = 'p';
//	frame[len++] = '.';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(X_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(Y_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(Z_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(ROLL_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(PITCH_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Pl_Coords_Data_Pos(YAW_AXIS)*1000);
	IntToStr6(temp,&frame[len]);
	len += 6;
//	frame[len++] = ' ';
	frame[len++] = '\r';
	frame[len++] = '\n';
	UART_PC_Send((uint8_t*)frame,len);
}
void UART_PC_Send_IMU_Data(void)
{
	
}
void UART_PC_Send_Servo_Data_Pos(void)
{
	len = 0;
	temp = 0;
	frame[len++] = 's';
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO1)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO1)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO2)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO2)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO3)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO3)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO4)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO4)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO5)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO5)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO6)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO6)*10);
	IntToStr5(temp, &frame[len]);
	len += 5;
	frame[len++] = '.';
//	frame[len++] = '\r';
	frame[len++] = '\n';
	UART_PC_Send((uint8_t*)frame,len);
}
void UART_PC_Send_Servo_Data_Pos_Test(void)
{
	len = 0;
	temp = 0;
//	frame[len++] = 's';
//	frame[len++] = '.';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO1)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO1)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO2)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO2)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO3)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO3)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO4)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO4)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO5)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO5)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(Get_Servo_Data_Pos(SERVO6)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
	frame[len++] = ' ';
	temp = (int32_t)(FSMC_ENC_Get_Pos(SERVO6)*100);
	IntToStr6(temp, &frame[len]);
	len += 6;
//	frame[len++] = '.';
	frame[len++] = '\r';
	frame[len++] = '\n';
	UART_PC_Send((uint8_t*)frame,len);
}
void UART_PC_Send_Feedback(char *c_msg, uint32_t ui32_msg_len)
{
	uint32_t ui32_k;
	char c_feedback[100];
	sprintf(c_feedback,"set data %d byte ",ui32_msg_len);
	ui32_k = strlen(c_feedback);
	if (ui32_k + ui32_msg_len >= 100)
		return;
	memcpy(c_feedback + ui32_k, c_msg, ui32_msg_len);
	c_feedback[ui32_k + ui32_msg_len] = '\n';
	UART_PC_Send((uint8_t*)c_feedback, ui32_msg_len + ui32_k + 1);
}

static void UART_Reset_Cmd_Msg(void)
{
	memset(ui8_cmd_msg,0,ui32_cmd_msg_cnt);
	ui32_cmd_msg_cnt = 0;
}
void UART_Read_Cmd(void)
{
	static uint32_t cmd_time_tick=0;
	static uint32_t cmd_read_ind=0, cmd_wr_ind=0;
	static uint32_t cmd_wr_ind_1 = 0;
	static uint8_t state = 0;
	uint32_t i, j, len;
	
	/* get buffer index of new data */
	cmd_wr_ind = d_DMA_RX_BUFF_SIZE - PC_RX_DMA_STREAM->NDTR;
	if(cmd_wr_ind_1 != cmd_wr_ind)
	{
		cmd_wr_ind_1 = cmd_wr_ind;
		cmd_time_tick = SysTick_GetTick();
	}
	else if(SysTick_IsTimeout(cmd_time_tick, 500))
	{
		cmd_wr_ind_1 = cmd_wr_ind;
		state = 0;
		UART_Reset_Cmd_Msg();
	}
	if(cmd_read_ind <= cmd_wr_ind)
		len = cmd_wr_ind - cmd_read_ind;
	else
		len = d_DMA_RX_BUFF_SIZE - (cmd_read_ind - cmd_wr_ind);
	for(i = 0; i < len; i++)
	{
		j = (cmd_read_ind + i) % d_DMA_RX_BUFF_SIZE;
		ui8_cmd_msg[ui32_cmd_msg_cnt++] = ui8_dma_rx_buff[j];
		if(ui32_cmd_msg_cnt >= d_CMD_MSG_LEN)
		{
			UART_Reset_Cmd_Msg();
			continue;
		}
		switch(state)
		{
			case 0: /* Wait for cmd header */
			{	
				uint32_t idx = 0;
				for(idx = 0; idx < MAX_CMD; idx++)
				{
					char *c_pstr;
					if(idx < MAX_CMD)
					{
						if(strncmp((char*)(ui8_cmd_msg + 2), 
									st_cmd_handler[idx].c_cmd_msg, 
											strlen(st_cmd_handler[idx].c_cmd_msg)) == 0)
						{
							c_pstr = (char*)ui8_cmd_msg;
							e_current_axis = INVALID_AXIS;
							switch(*c_pstr)
							{
								case 'a':
									e_current_axis = X_AXIS;
									break;
								case 'b':
									e_current_axis = Y_AXIS;
									break;
								case 'c':
									e_current_axis = Z_AXIS;
									break;
								case 'r':
									e_current_axis = ROLL_AXIS;
									break;
								case 'p':
									e_current_axis = PITCH_AXIS;
									break;
								case 'y':
									e_current_axis = YAW_AXIS;
									break;
								case 'j':
									e_current_axis = ALL_AXIS;
									break;
								case '1':
									e_current_axis = SERVO_1_AXIS;
									break;
								case '2':
									e_current_axis = SERVO_2_AXIS;
									break;
								case '3':
									e_current_axis = SERVO_3_AXIS;
									break;
								case '4':
									e_current_axis = SERVO_4_AXIS;
									break;
								case '5':
									e_current_axis = SERVO_5_AXIS;
									break;
								case '6':
									e_current_axis = SERVO_6_AXIS;
									break;
								case 'm':
									e_current_axis = ALL_SERVO_AXIS;
									break;
							}
							if(e_current_axis == INVALID_AXIS)
							{
								UART_Reset_Cmd_Msg();
								break;
							}
							c_pstr += 2;
							state = 1;
						}
						else if(strncmp((char*)ui8_cmd_msg, 
											st_cmd_handler[idx].c_cmd_msg, 
												strlen(st_cmd_handler[idx].c_cmd_msg)) == 0)
						{
							c_pstr = (char*) ui8_cmd_msg;			
							state = 1;
						}
						if(state)
						{
							e_current_cmd = idx;
							ui32_data_byte_idx = ui32_cmd_msg_cnt;
							ui32_remain_byte = st_cmd_handler[idx].ui32_data_num_byte;
							if(!ui32_remain_byte)
							{
								if(ui8_cmd_msg[ui32_cmd_msg_cnt - 1] == '\n')
								{
									st_cmd_handler[e_current_cmd].b_handler(e_current_axis, 
									&ui8_cmd_msg[ui32_data_byte_idx], 
									st_cmd_handler[e_current_cmd].ui32_data_num_byte);
								}
								state = 0;
								UART_Reset_Cmd_Msg();
							}
							break;
						}
					}
				}
				break;
			}
			case 1: /* Wait for data byte */
			{
				ui32_remain_byte--;
				if(!ui32_remain_byte)
				{
					if(ui8_cmd_msg[ui32_cmd_msg_cnt - 1] == '\n')
					{
						st_cmd_handler[e_current_cmd].b_handler(e_current_axis, 
						&ui8_cmd_msg[ui32_data_byte_idx], 
						st_cmd_handler[e_current_cmd].ui32_data_num_byte);
					}
					state = 0;
					UART_Reset_Cmd_Msg();
				}
				break;
			}
		}
	}
	cmd_read_ind= cmd_wr_ind;
}
bool Enable_Update_Data_Platform_Pos(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	b_update_platform_pos_enable = true;
	return true;
}
bool Enable_Update_Data_Servo_Pos(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt)
{
	b_update_servo_pos_enable = true;
	return true;
}