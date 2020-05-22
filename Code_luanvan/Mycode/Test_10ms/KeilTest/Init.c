#include "stm32f4xx.h"
#include "system_timetick.h"
#include "Define.h"

uint8_t 	txbuff[BUFF_SIZE];
uint8_t 	rxbuff[BUFF_RX];

void init_UART_DMA(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
	DMA_InitTypeDef  	DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;	
   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* Connect USART3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); 

  /* GPIO Configuration for UART3 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
	
	/* Enable USART */
  USART_Cmd(USART3, ENABLE);
	
	/* Enable USART3 DMA Tx */
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); 
	
	/* Enable USART3 DMA Rx*/
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	
	/* DMA1 Stream1 Channel4 for USART3 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_RX;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream1, ENABLE);
		
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

//  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
	
	
	
	/* DMA1 Stream3 Channel4 for UART3 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream3, ENABLE);

}

void FSMC_Init(void)
{
	FSMC_NORSRAMInitTypeDef FSMC_InitStructure, *FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMInitStructure = &FSMC_InitStructure;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 1. Enable the clock for the FSMC and associated GPIOs */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	/* 2. FSMC pins configuration */
	//enable alternate function FSMC at pins
	//PD0: AD2, PD1: AD3, PD4: NOE, PD5: NWE
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	//PD7: CS, PD8: AD13, PD9: AD14, PD10: AD15 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	//PD14: AD0, PD15: AD1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
	
	//PE7: AD4, PE8: AD5, PE9: AD6, PE10: AD7, PE11: AD8, PE12: AD9, PE13: AD10, PE14: AD11, PE15: AD12
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);
	
	//PB7: NADV
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_FSMC);
	
	//config IO
	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | \
																	GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | \
																	GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10| \
																	GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 3. Declare a FSMCInitStruct structure */
	FSMC_NORSRAMInitStructure->FSMC_Bank = 																							FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure->FSMC_DataAddressMux = 																		FSMC_DataAddressMux_Enable;
	FSMC_NORSRAMInitStructure->FSMC_MemoryType = 																				FSMC_MemoryType_NOR;
	FSMC_NORSRAMInitStructure->FSMC_MemoryDataWidth = 																	FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure->FSMC_BurstAccessMode = 																	FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure->FSMC_AsynchronousWait = 																	FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure->FSMC_WaitSignalPolarity = 																FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure->FSMC_WrapMode = 																					FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure->FSMC_WaitSignalActive = 																	FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure->FSMC_WriteOperation = 																		FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure->FSMC_WaitSignal = 																				FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure->FSMC_ExtendedMode = 																			FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure->FSMC_WriteBurst = 																				FSMC_WriteBurst_Disable;
	
	//when extend mode disable, we need some more configure code
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime = 			0xF;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime = 			0xF;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime = 				0xF;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration = 0x0;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision = 					0x0;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_DataLatency = 					0x0;
	FSMC_NORSRAMInitStructure->FSMC_ReadWriteTimingStruct->FSMC_AccessMode = 						FSMC_AccessMode_C; 
	
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_AddressSetupTime = 					0xF;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_AddressHoldTime = 					0xF;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_DataSetupTime = 						0x20;//0xF;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_BusTurnAroundDuration = 		0x0;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_CLKDivision = 							0x0;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_DataLatency = 							0x0;
//	FSMC_NORSRAMInitStructure->FSMC_WriteTimingStruct->FSMC_AccessMode = 								FSMC_AccessMode_C;
	
	/* 4. Initialize the FSMC Controller by calling the function */
	FSMC_NORSRAMInit(&FSMC_InitStructure);
	FSMC_Bank1->BTCR[1] = 0x20010303;
	//FSMC_Bank1->BTCR[0] = 0x0000105B;				//BCR1
	//BTR1
	//FSMC_Bank1->BTCR[1] = 0x20020707;			//10MHz
	//FSMC_Bank1->BTCR[1] = 0x20010303;			//21MHz
	//FSMC_Bank1->BTCR[1] = 0x20010301;				//42MHz
	/* 5. Then enable the NOR/SRAM Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
	
	/* 6. Config some pin for control read and write data */
	//PD11: Encoder reset, PD12: Pulse write enable
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_11);
	//PD13: Limit_Z, PD3: LS1 & LS2 signal, PD2: Home signal
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
}

void EXTILine2_Init(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

  /* Enable GPIOB clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PB12 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect EXTI Line2 to PB12 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

  /* Configure EXTI Line12 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line12 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void PWM_Init()
{
	// Enable Port A2
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Clock for GPIOA */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  
	/* Alternating functions for pins */  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	
	/* Set pins on port A */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	
	// Enable clock for Timer2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
		/* TIM 2 setup */
		TIM_BaseStruct.TIM_Prescaler = 199;
		TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseStruct.TIM_Period = 8399;
		TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_BaseStruct.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
		TIM_Cmd(TIM2, ENABLE);
		
	
	// Enable PWM2
	
	  TIM_OCInitTypeDef TIM_OCStruct;
	

	/* TIM2 */
		TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCStruct.TIM_Pulse = 629; /* 923: Open, 629: Close */
    TIM_OC3Init(TIM2, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
}
