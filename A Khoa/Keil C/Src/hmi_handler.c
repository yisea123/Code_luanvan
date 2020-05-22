#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_timetick.h"
#include "define.h"
#include "hmi_handler.h"
#include "params.h"
#include "control.h"
#include "utils.h"
#include "CALCULATOR.h"
#include "lightmodbus/core.h"
#include "lightmodbus/master.h"
#include "lightmodbus/slave.h"
#include <lightmodbus/slave/sregs.h>
#include <lightmodbus/slave/scoils.h>
#include "GPIO.h"

//extern parametter
extern float theta_platform;
extern float theta_base;
extern float platform_radius;
extern float base_radius;
extern float length_servo;
extern float length_connecting_arm;
extern float z_home;
extern float theta_servo_1;
extern float theta_servo_2;
extern float theta_servo_3;
extern float theta_servo_4;
extern float theta_servo_5;
extern float theta_servo_6;
extern SERVO_RUN_DATA_STRUCT st_servo_run_data;
extern bool b_is_new_data_model;
extern bool b_is_teach_run_lap;
extern uint32_t pl_coords_buffer_index;

extern uint32_t home_mode_speed;
//extern function

uint8_t hmi_txbuff[HMI_TXBUFF_SIZE]={0};

/* Modbus related variables */
ModbusSlave sstatus;

//Registers and coils
uint8_t inputDiscretes[10] = {0};
uint8_t coils[11] = { 0 };
uint16_t regs[HMI_REG_PARAM_MAX*2] = { 0 };
uint16_t inputRegs[HMI_INPUT_REG_PARAMS_MAX*2] = { 0 };

//For storing exit codes
uint8_t sec;

void HMI_HAL_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
   
  RCC_AHB1PeriphClockCmd(HMI_PORT_CLK, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(HMI_USART_CLK, ENABLE);
	
  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin   = HMI_TX | HMI_RX;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(HMI_PORT, &GPIO_InitStructure);	 
  GPIO_PinAFConfig(HMI_PORT, HMI_TX_SOURCE, HMI_AF);	  
	GPIO_PinAFConfig(HMI_PORT, HMI_RX_SOURCE, HMI_AF);
  
  /* USART configuration */
	USART_InitStructure.USART_BaudRate = HMI_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl =  USART_HardwareFlowControl_None;
  USART_Init(HMI_USART, &USART_InitStructure);
	USART_Cmd(HMI_USART, ENABLE);
    
  USART_ClearFlag(HMI_USART, USART_FLAG_TC);
  //USART_ClearFlag(HMI_USART, USART_FLAG_RXNE);

	/* DMA TX configuration */
	DMA_DeInit(HMI_TX_DMA_STREAM);		
	DMA_InitStructure.DMA_Channel 				= HMI_TX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)&hmi_txbuff[0]; 
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= HMI_DATA_REG;   
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal;
	DMA_InitStructure.DMA_BufferSize 			= 0;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(HMI_TX_DMA_STREAM, &DMA_InitStructure);
	// Enable DMA Stream Transfer Complete interrupt
	//DMA_ITConfig(HMI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
	// Enable USART DMA TX request
  USART_DMACmd(HMI_USART, USART_DMAReq_Tx, ENABLE);  
  // Enable DMA TX Channel
  //DMA_Cmd(HMI_TX_DMA_STREAM, ENABLE);
	
    /* DMA RX configuration */
	DMA_DeInit(HMI_RX_DMA_STREAM);	
	DMA_InitStructure.DMA_Channel 				= HMI_RX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)sstatus.request.frame;
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= HMI_DATA_REG;   
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_BufferSize 			= HMI_RXBUFF_SIZE;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
  DMA_Init(HMI_RX_DMA_STREAM, &DMA_InitStructure);
	// Enable DMA Stream Receive complete interrupt
	//DMA_ITConfig(HMI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	// Enable  request
  USART_DMACmd(HMI_USART, USART_DMAReq_Rx, ENABLE);
	// Enable DMA RX Channel
	DMA_Cmd(HMI_RX_DMA_STREAM, ENABLE); 
}

void HMI_Modbus_Init(void)
{
	//Init slave (input registers and discrete inputs work just the same)
	sstatus.address = 1;
	sstatus.registers = regs;
	sstatus.registerCount = HMI_REG_PARAM_MAX*2;
	sstatus.inputRegisters = inputRegs;
	sstatus.inputRegisterCount = HMI_INPUT_REG_PARAMS_MAX*2;	
	sstatus.coils = coils;
	sstatus.coilCount = 88;
	sstatus.discreteInputs = inputDiscretes;
	sstatus.discreteInputCount = 80;
	modbusSlaveInit( &sstatus );	
	modbusParseRequest16Callback_Register(myModbusParseRequest16Callback);
	modbusParseRequest05Callback_Register(myModbusParseRequest05Callback);
}

void HMI_Init(void)
{
	HMI_HAL_Init();
	HMI_Modbus_Init();
	HMI_Update_Model_Data();
	HMI_Set_DisInp_On(HMI_STT_SERVO_1);
	HMI_Set_DisInp_On(HMI_STT_SERVO_2);
	HMI_Set_DisInp_On(HMI_STT_SERVO_3);
	HMI_Set_DisInp_On(HMI_STT_SERVO_4);
	HMI_Set_DisInp_On(HMI_STT_SERVO_5);
	HMI_Set_DisInp_On(HMI_STT_SERVO_6);
}

//Function Interrupt For UART4: 485 to STM
void DMA1_Stream2_IRQHandler(void)
{
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
}
void HMI_Update_Model_Data(void)
{
	HMI_Update_RegParams_Param(HMI_PL_JOINT_ANGLE,(int32_t)(theta_platform*10000));
	HMI_Update_RegParams_Param(HMI_BASE_JOINT_ANGLE,(int32_t)(theta_base*10000));
	HMI_Update_RegParams_Param(HMI_PL_RADIUS,(int32_t)(platform_radius*10000));
	HMI_Update_RegParams_Param(HMI_BASE_RADIUS,(int32_t)(base_radius*10000));
	HMI_Update_RegParams_Param(HMI_SV_ARM_LENGTH,(int32_t)(length_servo*10000));
	HMI_Update_RegParams_Param(HMI_CON_ARM_LENGTH,(int32_t)(length_connecting_arm*10000));
	HMI_Update_RegParams_Param(HMI_DEFAUT_Z_HEIGHT,(int32_t)(z_home*10000));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_1,(int32_t)(theta_servo_1));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_2,(int32_t)(theta_servo_2));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_3,(int32_t)(theta_servo_3));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_4,(int32_t)(theta_servo_4));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_5,(int32_t)(theta_servo_5));
	HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_6,(int32_t)(theta_servo_6));
}
void HMI_Send(uint8_t *data, uint32_t data_size)
{
	/* sstatus.response.frame */
}

void HMI_Process(void)
{
	static uint32_t tick_start;
	uint16_t k1;
	static uint16_t k2 = 0;

	k1 = HMI_RX_DMA_STREAM->NDTR;		 
	k1 = HMI_RXBUFF_SIZE - k1;

	if (k1 != k2)
	{
		k2 = k1;
		tick_start = SysTick_GetTick();
	}
	else if ((SysTick_IsTimeout(tick_start, 20)) && (k2 != 0))		// timeout -> detect new frame
  {
		k2 = 0;
		DMA_Cmd(HMI_RX_DMA_STREAM, DISABLE);
  
		/* check if rx data are unavailable then skip */
		//Pretend frame is being sent to slave
		sstatus.request.length = k1;

		//Let slave parse frame
		sec = modbusParseRequest( &sstatus );
		if (sec == MODBUS_ERROR_OK)
		{
			if (sstatus.response.length)
			{
				memcpy(hmi_txbuff,sstatus.response.frame,sstatus.response.length);
				DMA_ClearFlag(HMI_TX_DMA_STREAM, HMI_TX_DMA_FLAG);
				HMI_TX_DMA_STREAM->NDTR = sstatus.response.length;
				DMA_Cmd(HMI_TX_DMA_STREAM, ENABLE);
				//test
			}
		}
		HMI_RX_DMA_STREAM->NDTR = HMI_RXBUFF_SIZE;
		DMA_Cmd(HMI_RX_DMA_STREAM, ENABLE);
	}
}

void HMI_Update_InpParams_Param(HMI_INPUT_REG_PARAMS_ENUM param, int32_t value)
{
	if (param > HMI_INPUT_REG_PARAMS_MAX)
	{
		return;
	}
	else
	{
		sstatus.inputRegisters[(int)param<<1] = (value & 0xffff);
		sstatus.inputRegisters[(int)(param<<1)+1] = (value >> 16) & 0xffff;
	}
}
void HMI_Update_RegParams_Param(HMI_REG_PARAMS_ENUM param, int32_t value)
{
	if (param > HMI_REG_PARAM_MAX)
	{
		return;
	}
	else
	{
		sstatus.registers[(int)param<<1] = (value & 0xffff);
		sstatus.registers[(int)(param<<1)+1] = (value >> 16) & 0xffff;
	}
}
void HMI_Set_DisInp_On(HMI_INP_DIS_ENUM param)
{
	inputDiscretes[param >> 3] |= 1<<(param & 0x07);
}
void HMI_Set_DisInp_Off(HMI_INP_DIS_ENUM param)
{
	inputDiscretes[param >> 3] &= ~(1<<(param & 0x07));
}
void HMI_Update_DisInpParams_Param(void)
{
	switch((int)Get_System_State())
	{
		case SYSTEM_STATE_ERROR:
		{
			HMI_Set_DisInp_On(HMI_SYS_STT_ERROR);
			HMI_Set_DisInp_Off(HMI_SYS_STT_IDLE);
			HMI_Set_DisInp_Off(HMI_SYS_STT_RUN);
		}
		break;
		case SYSTEM_STATE_IDLE:
		{
			HMI_Set_DisInp_Off(HMI_SYS_STT_ERROR);
			HMI_Set_DisInp_On(HMI_SYS_STT_IDLE);
			HMI_Set_DisInp_Off(HMI_SYS_STT_RUN);
		}
		break;
		case SYSTEM_STATE_RUN:
		{
			HMI_Set_DisInp_Off(HMI_SYS_STT_ERROR);
			HMI_Set_DisInp_Off(HMI_SYS_STT_IDLE);
			HMI_Set_DisInp_On(HMI_SYS_STT_RUN);
		}
		break;
	}
	switch((int)Get_Run_Mode())
	{
		case RUN_MODE_NONE:
		{
			HMI_Set_DisInp_On(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_Off(HMI_RUN_MODE_FOLLOW);
		}
			break;
		case RUN_MODE_HOME:
		{
			HMI_Set_DisInp_Off(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_On(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_Off(HMI_RUN_MODE_FOLLOW);
		}
			break;
		case RUN_MODE_TEST:
		{
			HMI_Set_DisInp_Off(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_On(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_Off(HMI_RUN_MODE_FOLLOW);
		}
			break;
		case RUN_MODE_JOG:
		{
			HMI_Set_DisInp_Off(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_On(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_Off(HMI_RUN_MODE_FOLLOW);
		}
			break;
		case RUN_MODE_TEACH:
		{
			HMI_Set_DisInp_Off(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_On(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_Off(HMI_RUN_MODE_FOLLOW);
		}
			break;
		case RUN_MODE_FOLLOW:
		{
			HMI_Set_DisInp_Off(HMI_RUN_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_HOME);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEST);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_JOG);
			HMI_Set_DisInp_Off(HMI_RUN_MOD_TEACH);
			HMI_Set_DisInp_On(HMI_RUN_MODE_FOLLOW);
		}
			break;
	}
	
	switch((int)Get_Home_Mode())
	{
		case HOME_MODE_NONE:
			HMI_Set_DisInp_On(HMI_HOME_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_HOME_MOD_SERVO);
			HMI_Set_DisInp_Off(HMI_HOME_MOD_MODEL);
			break;
		case HOME_MODE_SERVO:
			HMI_Set_DisInp_Off(HMI_HOME_MOD_NONE);
			HMI_Set_DisInp_On(HMI_HOME_MOD_SERVO);
			HMI_Set_DisInp_Off(HMI_HOME_MOD_MODEL);
			break;
		case HOME_MODE_MODEL:
			HMI_Set_DisInp_Off(HMI_HOME_MOD_NONE);
			HMI_Set_DisInp_On(HMI_HOME_MOD_SERVO);
			HMI_Set_DisInp_Off(HMI_HOME_MOD_MODEL);
			break;
	}
	switch((int)Get_Control_Mode())
	{
		case CONTROL_MODE_FEEDFORWARD_POS:
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
		case CONTROL_MODE_FEEDFORWARD_POS_VEL:
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
		case CONTROL_MODE_FEEDBACK_IMU:
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
		case CONTROL_MODE_FEEDBACK_ENC:
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
		case CONTROL_MODE_FEEDBACK_ENC_IMU:
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
		case CONTROL_MODE_FEEDBACK_ENC_IMU_LIDAR:
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FF_POS_VEL);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_ENC);
			HMI_Set_DisInp_Off(HMI_CTRL_MOD_FB_IMU_ENC);
			HMI_Set_DisInp_On(HMI_CTRL_MOD_FB_IMU_ENC_LIDAR);
			break;
	}
	switch((int)Get_Test_Mode())
	{
		case TEST_MODE_NONE:
			HMI_Set_DisInp_On(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_PID:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_On(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_X:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_On(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_X_Y_LEFT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_On(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
		break;
		case TEST_MODE_X_Y_RIGHT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
		break;
		case TEST_MODE_Y:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_Y_Z_LEFT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_On(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_Y_Z_RIGHT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_Z:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_X_Z_LEFT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_On(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_X_Z_RIGHT:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_ROLL:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_On(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_PITCH:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_On(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_YAW:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_On(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_CIRCLE:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_On(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_SQUARE:
			HMI_Set_DisInp_Off(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_On(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_RANDOM_1:
			HMI_Set_DisInp_On(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_On(HMI_TEST_MOD_RANDOM_1);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_RANDOM_2);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_RANDOM_2:
			HMI_Set_DisInp_On(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_On(HMI_TEST_MOD_SQUARE);
		HMI_Set_DisInp_Off(HMI_TEST_MOD_ECLIPSE);
			break;
		case TEST_MODE_ECLIPSE:
			HMI_Set_DisInp_On(HMI_TEST_MOD_NONE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PID);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PULSE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Y_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Y_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_Z);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_LEFT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_X_Z_RIGHT);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_ROLL);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_PITCH);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_YAW);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_CIRCLE);
			HMI_Set_DisInp_Off(HMI_TEST_MOD_SQUARE);
			HMI_Set_DisInp_On(HMI_TEST_MOD_ECLIPSE);
			break;
	}
}
void myModbusParseRequest05Callback(uint8_t coil_address)
{
	switch (coil_address)
	{
		case BT_START:
			Start_Handler(0,0,0);
			break;
		case BT_STOP:
			Stop_Handler(0,0,0);
			break;
		case BT_CONTROL:
			Set_Model_Control_Mode_Handler(0,0,0);
			break;
		case BT_TEST:
			Set_Model_Test_Mode_Handler(0,0,0);
			break;
		case BT_TEACH:
			Set_Model_Teach_Mode_Handler(0,0,0);
			break;
		case BT_JOG:
			Set_Model_Jog_Mode_Handler(0,0,0);
			break;
		case BT_HOME:
			Home_Handler(0,0,0);
			break;
		case BT_HOME_SERVO:
			Home_Servo_Handler(0,0,0);
			break;
		case BT_HOME_MODEL:
			Home_Model_Handler(0,0,0);
			break;
		case BT_SERVO_1:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT1))
					{
						Set_Servo_Data_Pos_Up(SERVO1);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO1);
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
			break;
		case BT_SERVO_2:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT2))
					{
						Set_Servo_Data_Pos_Up(SERVO2);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO2);
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
			break;
		case BT_SERVO_3:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT3))
					{
						Set_Servo_Data_Pos_Up(SERVO3);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO3);
					}
					Set_Servo_State(STATE_CALCULATING);
			  }
			}
			break;
		case BT_SERVO_4:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT4))
					{
						Set_Servo_Data_Pos_Up(SERVO4);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO4);
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
			break;
		case BT_SERVO_5:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT5))
					{
						Set_Servo_Data_Pos_Up(SERVO5);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO5);
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
			break;
		case BT_SERVO_6:
			if(Get_Home_Mode() == HOME_MODE_NONE)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					st_servo_run_data.ui32_period_run_total = home_mode_speed;
					if((coils[1] & BIT6))
					{
						Set_Servo_Data_Pos_Up(SERVO6);
					}
					else
					{
						Set_Servo_Data_Pos_Down(SERVO6);
					}
					Set_Servo_State(STATE_CALCULATING);
				}
			}
			break;
		case BT_SERVO_RESET:
			if(Get_Run_Mode() == RUN_MODE_HOME)
			{
				if(Get_Home_Mode() == HOME_MODE_NONE)
				{
					Reset_Servo_Handler(0,0,0);
				}
			}
			break;
		case BT_SERVO_ON_OFF_1:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT0)
				{
					Relay_Off(RELAY_1);
					inputDiscretes[HMI_STT_SERVO_1 >> 3] |= 1<<(HMI_STT_SERVO_1 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO1,true);
				}
				else
				{
					Relay_On(RELAY_1);
					inputDiscretes[HMI_STT_SERVO_1 >> 3] &= ~(1<<(HMI_STT_SERVO_1 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO1,false);
				}
				
			}
			break;
		case BT_SERVO_ON_OFF_2:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT1)
				{
					Relay_Off(RELAY_2);
					inputDiscretes[HMI_STT_SERVO_2 >> 3] |= 1<<(HMI_STT_SERVO_2 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO2,true);					
				}
				else
				{
					Relay_On(RELAY_2);
					inputDiscretes[HMI_STT_SERVO_2 >> 3] &= ~(1<<(HMI_STT_SERVO_2 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO2,false);
				}
			}
			break;
		case BT_SERVO_ON_OFF_3:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT2)
				{
					Relay_Off(RELAY_3);
					inputDiscretes[HMI_STT_SERVO_3 >> 3] |= 1<<(HMI_STT_SERVO_3 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO3,true);
				}
				else
				{
					Relay_On(RELAY_3);
					inputDiscretes[HMI_STT_SERVO_3 >> 3] &= ~(1<<(HMI_STT_SERVO_3 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO3,false);					
				}
			}
			break;
		case BT_SERVO_ON_OFF_4:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT3)
				{
					Relay_Off(RELAY_4);
					inputDiscretes[HMI_STT_SERVO_4 >> 3] |= 1<<(HMI_STT_SERVO_4 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO4,true);
				}
				else
				{
					Relay_On(RELAY_4);
					inputDiscretes[HMI_STT_SERVO_4 >> 3] &= ~(1<<(HMI_STT_SERVO_4 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO4,false);
				}
			}
			break;
		case BT_SERVO_ON_OFF_5:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT4)
				{
					Relay_Off(RELAY_5);
					inputDiscretes[HMI_STT_SERVO_5 >> 3] |= 1<<(HMI_STT_SERVO_5 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO5,true);
				}
				else
				{
					Relay_On(RELAY_5);
					inputDiscretes[HMI_STT_SERVO_5 >> 3] &= ~(1<<(HMI_STT_SERVO_5 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO5,false);
				}
			}
			break;
		case BT_SERVO_ON_OFF_6:
			if(Get_System_State() == SYSTEM_STATE_IDLE)
			{
				if(coils[2] & BIT5)
				{
					Relay_Off(RELAY_6);
					inputDiscretes[HMI_STT_SERVO_6 >> 3] |= 1<<(HMI_STT_SERVO_6 & 0x07);
					Set_Servo_Data_STT_On_Off(SERVO6,true);
				}
				else
				{
					Relay_On(RELAY_6);
					inputDiscretes[HMI_STT_SERVO_6 >> 3] &= ~(1<<(HMI_STT_SERVO_6 & 0x07));
					Set_Servo_Data_STT_On_Off(SERVO6,false);
				}
			}
			break;
		case BT_JOG_X:
			if((coils[2] & BIT6))
			{
				Set_Model_Jog_Mode_X_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_X_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_Y:
			if((coils[2] & BIT7))
			{
				Set_Model_Jog_Mode_Y_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Y_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_Z:
			if((coils[3] & BIT0))
			{
				Set_Model_Jog_Mode_Z_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Z_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_ROLL:
			if((coils[3] & BIT1))
			{
				Set_Model_Jog_Mode_Roll_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Roll_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_PITCH:
			if((coils[3] & BIT2))
			{
				Set_Model_Jog_Mode_Pitch_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Pitch_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_YAW:
			if((coils[3] & BIT3))
			{
				Set_Model_Jog_Mode_Yaw_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Yaw_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_X_Y_LEFT:
			if((coils[3] & BIT4))
			{
				Set_Model_Jog_Mode_X_Y_Left_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_X_Y_Left_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_X_Y_RIGHT:
			if((coils[3] & BIT5))
			{
				Set_Model_Jog_Mode_X_Y_Right_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_X_Y_Right_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_X_Z_LEFT:
			if((coils[3] & BIT6))
			{
				Set_Model_Jog_Mode_X_Z_Left_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_X_Z_Left_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_X_Z_RIGHT:
			if((coils[3] & BIT7))
			{
				Set_Model_Jog_Mode_X_Z_Right_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_X_Z_Right_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_Y_Z_LEFT:
			if((coils[4] & BIT0))
			{
				Set_Model_Jog_Mode_Y_Z_Left_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Y_Z_Left_Down_Handler(0,0,0);
			}
			break;
		case BT_JOG_Y_Z_RIGHT:
			if((coils[4] & BIT1))
			{
				Set_Model_Jog_Mode_Y_Z_Right_Up_Handler(0,0,0);
			}
			else
			{
				Set_Model_Jog_Mode_Y_Z_Right_Down_Handler(0,0,0);
			}
			break;
		case BT_RUN_MODEL:
		{
			if(Get_System_State() == SYSTEM_STATE_RUN)
			{
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
						if(Get_Teach_Mode() == TEACH_MODE_NONE)
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								Set_Teach_Mode(TEACH_MODE_RUN_INIT); 
							}
						}
				}
			}
			break;
		}
		case BT_HOME_TEACH_MODEL:
		{
			if(Get_System_State() == SYSTEM_STATE_RUN)
			{
				if(Get_Servo_State() == STATE_STOP)
				{
					if(Get_Run_Mode() == RUN_MODE_TEACH)
					{
						Reset_Axis_Model();
						Set_Servo_State(STATE_HOMING_POS);
						st_servo_run_data.ui32_period_run_total = 10*F_CTRL;
					}
					else if(Get_Run_Mode() == RUN_MODE_JOG)
					{
						Reset_Axis_Model();
						Set_Servo_State(STATE_HOMING_POS);
						st_servo_run_data.ui32_period_run_total = 10*F_CTRL;
					}
				}
			}
			else if(Get_System_State() == SYSTEM_STATE_ERROR)
			{
				Set_System_State(SYSTEM_STATE_RUN);
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
					Reset_Axis_Model();
					Set_Servo_State(STATE_HOMING_POS);
					st_servo_run_data.ui32_period_run_total = 10*F_CTRL;
				}
				else if(Get_Run_Mode() == RUN_MODE_JOG)
				{
					Reset_Axis_Model();
					Set_Servo_State(STATE_HOMING_POS);
					st_servo_run_data.ui32_period_run_total = 10*F_CTRL;
				}
			}
			break;
		}
		case BT_BACK_MODEL:
		{
			if(Get_System_State() == SYSTEM_STATE_RUN)
			{
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
						if(Get_Teach_Mode() == TEACH_MODE_NONE)
						{
							if(Get_Servo_State() == STATE_STOP)
							{
								Set_Teach_Mode(TEACH_MODE_BACK); 
							}
						}
				}
			}
			break;
		}
		case BT_TEACH_MODEL:
		{
			if(Get_System_State() == SYSTEM_STATE_RUN)
			{
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
					if(Get_Teach_Mode() == TEACH_MODE_NONE)
					{
						if(Get_Servo_State() == STATE_STOP)
						{
							Set_Teach_Mode(TEACH_MODE_GO);
						}
					}
				}
				else if(Get_Run_Mode() == RUN_MODE_JOG)
				{
					Set_Run_Mode(RUN_MODE_TEACH);
					Set_Teach_Mode(TEACH_MODE_GO);
				}
			}
			else if(Get_System_State() == SYSTEM_STATE_ERROR)
			{
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
					if(Get_Teach_Mode() == TEACH_MODE_NONE)
					{
						if(Get_Servo_State() == STATE_STOP)
						{
							Set_Teach_Mode(TEACH_MODE_GO);
						}
					}
				}
				else if(Get_Run_Mode() == RUN_MODE_JOG)
				{
					if(Get_Servo_State() == STATE_STOP)
					{
						Set_Run_Mode(RUN_MODE_TEACH);
						Set_Teach_Mode(TEACH_MODE_GO);
					}
				}
			}
			break;
		}
		case BT_NEXT_MODEL:
		{
			if(Get_System_State() == SYSTEM_STATE_RUN)
			{
				if(Get_Run_Mode() == RUN_MODE_TEACH)
				{
					if(Get_Teach_Mode() == TEACH_MODE_NONE)
					{
						if(Get_Servo_State() == STATE_STOP)
						{
							Set_Teach_Mode(TEACH_MODE_NEXT); 
						}
					}
				}
			}
			break;
		}
		case BT_LAP_MODEL:
			if(b_is_teach_run_lap)
			{
				b_is_teach_run_lap = false;
				HMI_Set_DisInp_Off(HMI_STT_TEACH_LAP);
			}
			else
			{
				b_is_teach_run_lap = true;
				HMI_Set_DisInp_On(HMI_STT_TEACH_LAP);
			}
			break;
		case BT_STOP_MODEL:
		{
			Set_Servo_State(STATE_STOP);
			Set_Teach_Mode(TEACH_MODE_NONE);
			break;
		}
		case BT_TEST_MODE_X:
			Set_Test_X_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_Y:
			Set_Test_Y_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_Z:
			Set_Test_Z_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_X_Y:
			if((coils[5] & BIT4))
			{
				Set_Test_X_Y_Left_Mode_Handler(0,0,0);
			}
			else
			{
				Set_Test_X_Y_Right_Mode_Handler(0,0,0);
			}
			break;
		case BT_TEST_MODE_X_Z:
			if((coils[5] & BIT5))
			{
				Set_Test_X_Z_Left_Mode_Handler(0,0,0);
			}
			else
			{
				Set_Test_X_Z_Right_Mode_Handler(0,0,0);
			}
			break;
		case BT_TEST_MODE_Y_Z:
			if((coils[5] & BIT6))
			{
				Set_Test_Y_Z_Left_Mode_Handler(0,0,0);
			}
			else
			{
				Set_Test_Y_Z_Right_Mode_Handler(0,0,0);
			}
			break;
		case BT_TEST_MODE_ROLL:
			Set_Test_Roll_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_PITCH:
			Set_Test_Pitch_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_YAW:
			Set_Test_Yaw_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_PULSE:
			Set_Test_Pulse_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_CIRCLE:
			Set_Test_Circle_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_SQUARE:
			Set_Test_Square_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_RANDOM_1:
			Set_Test_Random_1_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_RANDOM_2:
			Set_Test_Random_2_Mode_Handler(0,0,0);
			break;
		case BT_FOLLOW:
			Set_Follow_Mode_Handler(0,0,0);
			break;
		case BT_TEST_MODE_ECLPISE:
			Set_Test_Eclipse_Mode_Handler(0,0,0);
			break;
		case BT_RESET_TEACH:
			Reset_Teach_Index();
			break;
		default:
			break;
	}
}

void myModbusParseRequest16Callback(uint8_t reg_address)
{
	float temp;
	switch (reg_address>>1)
	{
		case HMI_PL_JOINT_ANGLE:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(PLATFORM_JOINT_ANGLE,temp);
			PARAMS_Save_Float(EEP_PL_JOINT_ANGLE,temp);
			HMI_Update_RegParams_Param(HMI_PL_JOINT_ANGLE,(int32_t)(temp*10000));
		break;
		case HMI_BASE_JOINT_ANGLE:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(BASE_JOINT_ANGLE,temp);
			PARAMS_Save_Float(EEP_BASE_JOINT_ANGLE,temp);
			HMI_Update_RegParams_Param(HMI_BASE_JOINT_ANGLE,(int32_t)(temp*10000));
			break;
		case HMI_PL_RADIUS:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(PLATFORM_RADIUS,temp);
			PARAMS_Save_Float(EEP_PL_RADIUS,temp);
			HMI_Update_RegParams_Param(HMI_PL_RADIUS,(int32_t)(temp*10000));
			break;
		case HMI_BASE_RADIUS:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(BASE_RADIUS,temp);
			PARAMS_Save_Float(EEP_BASE_RADIUS,temp);
			HMI_Update_RegParams_Param(HMI_BASE_RADIUS,(int32_t)(temp*10000));
			break;
		case HMI_SV_ARM_LENGTH:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(SERVO_ARM_LENGTH,temp);
			PARAMS_Save_Float(EEP_SV_ARM_LENGTH,temp);
			HMI_Update_RegParams_Param(HMI_SV_ARM_LENGTH,(int32_t)(temp*10000));
			break;
		case HMI_CON_ARM_LENGTH:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(CONNECTING_ARM_LENGTH,temp);
			PARAMS_Save_Float(EEP_CON_ARM_LENGTH,temp);
			HMI_Update_RegParams_Param(HMI_CON_ARM_LENGTH,(int32_t)(temp*10000));
			break;
		case HMI_DEFAUT_Z_HEIGHT:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.0001f;
			Set_Model_Parametter(DEFAULT_Z_HEIGHT,temp);
			PARAMS_Save_Float(EEP_DEFAUT_Z_HEIGHT,temp);
			HMI_Update_RegParams_Param(HMI_DEFAUT_Z_HEIGHT,(int32_t)(temp*10000));
			break;
		case HMI_ANGLE_SV_ARM_1:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_1,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_1,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_1,(int32_t)(temp));
			break;
		case HMI_ANGLE_SV_ARM_2:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_2,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_2,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_2,(int32_t)(temp));
			break;
		case HMI_ANGLE_SV_ARM_3:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_3,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_3,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_3,(int32_t)(temp));
			break;
		case HMI_ANGLE_SV_ARM_4:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_4,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_4,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_4,(int32_t)(temp));
			break;
		case HMI_ANGLE_SV_ARM_5:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_5,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_5,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_5,(int32_t)(temp));
			break;
		case HMI_ANGLE_SV_ARM_6:
			b_is_new_data_model = true;
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16));
			Set_Model_Parametter(ANGLE_OF_SERVO_ARM_6,temp);
			PARAMS_Save_Float(EEP_ANGLE_SV_ARM_6,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_SV_ARM_6,(int32_t)(temp));
			break;
		case HMI_SV_ANG_1:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO1, temp); //Set to servo_data[SERVO].f_setpoint_angle
			HMI_Update_RegParams_Param(HMI_SV_ANG_1,(int32_t)(temp*10.0f));
		break;
		case HMI_SV_VEL_1: //Home speed
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Speed(RUN_MODE_HOME,temp);
			HMI_Update_RegParams_Param(HMI_SV_VEL_1,(int32_t)(temp*10.0f));
			break;
		case HMI_SV_ACC_1:
			break;
		case HMI_SV_ANG_2:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO2, temp);
			HMI_Update_RegParams_Param(HMI_SV_ANG_2,(int32_t)(temp*10));
		break;
		case HMI_SV_VEL_2: // Test speed
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Speed(RUN_MODE_TEST,temp);
			HMI_Update_RegParams_Param(HMI_SV_VEL_2,(int32_t)(temp*10.0f));
		break;
		case HMI_SV_ACC_2:
			break;
		case HMI_SV_ANG_3:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO3, temp);
			HMI_Update_RegParams_Param(HMI_SV_ANG_3,(int32_t)(temp*10));
			break;
		case HMI_SV_VEL_3: // Jog speed
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Speed(RUN_MODE_JOG,temp);
			HMI_Update_RegParams_Param(HMI_SV_VEL_3,(int32_t)(temp*10.0f));
		break;
		case HMI_SV_ACC_3:
			break;
		case HMI_SV_ANG_4:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO4, temp);
			HMI_Update_RegParams_Param(HMI_SV_ANG_4,(int32_t)(temp*10));
			break;
		case HMI_SV_VEL_4: // Teach speed
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Speed(RUN_MODE_TEACH,temp);
			HMI_Update_RegParams_Param(HMI_SV_VEL_4,(int32_t)(temp*10.0f));
		break;
		case HMI_SV_ACC_4:
			break;
		case HMI_SV_ANG_5:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO5, temp);
			HMI_Update_RegParams_Param(HMI_SV_ANG_5,(int32_t)(temp*10));
			break;
		case HMI_SV_VEL_5: // Follow speed
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Speed(RUN_MODE_FOLLOW,temp);
			HMI_Update_RegParams_Param(HMI_SV_VEL_5,(int32_t)(temp*10.0f));	
		break;
		case HMI_SV_ACC_5:
			break;
		case HMI_SV_ANG_6:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Servo_Data_Pos(SERVO6, temp);
			HMI_Update_RegParams_Param(HMI_SV_ANG_6,(int32_t)(temp*10));
			break;
		case HMI_SV_VEL_6:
			break;
		case HMI_SV_ACC_6:
			break;
		case HMI_POS_X:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(X_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_POS_X,(int32_t)(temp*1000));
			break;
		case HMI_POS_Y:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(Y_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_POS_Y,(int32_t)(temp*1000));
			break;
		case HMI_POS_Z:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(Z_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_POS_Z,(int32_t)(temp*1000));
			break;
		case HMI_ANGLE_ROLL:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(ROLL_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_ROLL,(int32_t)(temp*1000));
			break;
		case HMI_ANGLE_PITCH:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(PITCH_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_PITCH,(int32_t)(temp*1000));
			break;
		case HMI_ANGLE_YAW:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.001f;
			Set_Pl_Coords_Data_Pos(YAW_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_ANGLE_YAW,(int32_t)(temp*1000));
			break;
		case HMI_VEL_X:
			break;
		case HMI_VEL_Y:
			break;
		case HMI_VEL_Z:
			break;
		case HMI_VEL_ROLL:
			break;
		case HMI_VEL_PITCH:
			break;
		case HMI_VEL_YAW:
			break;
		case HMI_ACC_X:
			break;
		case HMI_ACC_Y:
			break;
		case HMI_ACC_Z:
			break;
		case HMI_ACC_ROLL:
			break;
		case HMI_ACC_PITCH:
			break;
		case HMI_ACC_YAW:
			break;
		case HMI_TEST_POS_FROM_X:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(X_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_X,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_X:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(X_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_X,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_X:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(X_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_X,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_FROM_Y:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(Y_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_Y,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_Y:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(Y_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_Y,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_Y:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(Y_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_Y,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_FROM_Z:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(Z_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_Z,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_Z:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(Z_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_Z,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_Z:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(Z_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_Z,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_FROM_ROLL:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(ROLL_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_ROLL,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_ROLL:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(ROLL_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_ROLL,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_ROLL:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(ROLL_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_ROLL,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_FROM_PITCH:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(PITCH_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_PITCH,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_PITCH:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(PITCH_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_PITCH,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_PITCH:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(PITCH_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_PITCH,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_FROM_YAW:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Min_Axis(YAW_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_FROM_YAW,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_TO_YAW:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Max_Axis(YAW_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_TO_YAW,(int32_t)(temp*100));
			break;
		case HMI_TEST_POS_VEL_YAW:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Test_Axis(YAW_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_TEST_POS_VEL_YAW,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_X:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(X_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_X,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_Y:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(Y_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_Y,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_Z:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(Z_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_Z,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_ROLL:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(ROLL_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_ROLL,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_PITCH:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(PITCH_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_PITCH,(int32_t)(temp*100));
			break;
		case HMI_JOG_FEED_YAW:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Pl_Coords_Data_Step_Jog_Axis(YAW_AXIS,temp);
			HMI_Update_RegParams_Param(HMI_JOG_FEED_YAW,(int32_t)(temp*100));
			break;
		case HMI_TEST_CIRCLE_RADIUS:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f;
			Set_Circle_Test_Data(CIRCLE_RADIUS,temp,0);
			HMI_Update_RegParams_Param(HMI_TEST_CIRCLE_RADIUS,(int32_t)(temp*100));
			break;
		case HMI_TEST_CIRCLE_SPEED:
			temp = (float)((int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f;
			Set_Circle_Test_Data(CIRCLE_SPEED,temp,0);
			HMI_Update_RegParams_Param(HMI_TEST_CIRCLE_SPEED,(int32_t)(temp*10));
			break;
		case HMI_TEST_CIRCLE_MODE:
			temp = (int)(sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16);
			switch((int)temp)
			{
				case 1: 
					Set_Circle_Test_Data(CIRCLE_MODE,0,CIRCLE_MODE_X_Y);
					break;
				case 2:
					Set_Circle_Test_Data(CIRCLE_MODE,0,CIRCLE_MODE_Y_Z);
					break;
				case 3:
					Set_Circle_Test_Data(CIRCLE_MODE,0,CIRCLE_MODE_X_Z);
					break;
				default:
					Set_Circle_Test_Data(CIRCLE_MODE,0,CIRCLE_MODE_X_Y);
				break;
			}
			HMI_Update_RegParams_Param(HMI_TEST_CIRCLE_MODE,(int32_t)(temp));
			break;
		case HMI_TEST_RECTANGLE_X_FROM:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Square_Test_Data(SQUARE_X_FROM, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_X_FROM,(int32_t)(temp*100));
			break;
		case HMI_TEST_RECTANGLE_Y_FROM:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Square_Test_Data(SQUARE_Y_FROM, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_Y_FROM,(int32_t)(temp*100));
			break;
		case HMI_TEST_RECTANGLE_X_TO:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Square_Test_Data(SQUARE_X_TO, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_X_TO,(int32_t)(temp*100));
			break;
		case HMI_TEST_RECTANGLE_Y_TO:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Square_Test_Data(SQUARE_Y_TO, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_Y_TO,(int32_t)(temp*100));
			break;
		case HMI_TEST_RECTANGLE_SPEED:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f);
			Set_Square_Test_Data(SQUARE_SPEED, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_SPEED,(int32_t)(temp*10));
			break;
		case HMI_TEST_RECTANGLE_MODE:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16)));
			switch((int)(temp))
			{
				case 1:
					Set_Square_Test_Data(SQUARE_MODE,0 , SQUARE_MODE_X_Y);
					break;
				case 2:
					Set_Square_Test_Data(SQUARE_MODE,0 , SQUARE_MODE_Y_Z);
					break;
				case 3:
					Set_Square_Test_Data(SQUARE_MODE,0 , SQUARE_MODE_X_Z);
					break;
				default:
					Set_Square_Test_Data(SQUARE_MODE,0 , SQUARE_MODE_X_Y);
					break;
			}
			HMI_Update_RegParams_Param(HMI_TEST_RECTANGLE_MODE,(int32_t)(temp));
			break;
		case HMI_TEST_ELIPSE_A:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Eclipse_Test_Data(ECLIPSE_A_RADIUS, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_ELIPSE_A,(int32_t)(temp*100));
			break;
		case HMI_TEST_ELIPSE_B:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.01f);
			Set_Eclipse_Test_Data(ECLIPSE_B_RADIUS, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_ELIPSE_B,(int32_t)(temp*100));
			break;
		case HMI_TEST_ELIPSE_SPEED:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16))*0.1f);
			Set_Eclipse_Test_Data(ECLIPSE_SPEED, temp, 0);
			HMI_Update_RegParams_Param(HMI_TEST_ELIPSE_SPEED,(int32_t)(temp*10));
			break;
		case HMI_TEST_ELIPSE_MODE:
			temp = (float)((int)((sstatus.registers[reg_address])|(sstatus.registers[reg_address+1]<<16)));
			switch((int)(temp))
			{
				case 1:
					Set_Eclipse_Test_Data(ECLIPSE_MODE, temp, ECLIPSE_MODE_X_Y);
					break;
				case 2:
					Set_Eclipse_Test_Data(ECLIPSE_MODE, temp, ECLIPSE_MODE_Y_Z);
					break;
				case 3:
					Set_Eclipse_Test_Data(ECLIPSE_MODE, temp, ECLIPSE_MODE_X_Z);
					break;
			}
			HMI_Update_RegParams_Param(HMI_TEST_ELIPSE_MODE,(int32_t)(temp));
			break;
		default:
			break;
	}
}

