#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
//#include "system_state.h"
//#include "IMU_Quest.h"                  /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "string.h"
#include "IMU_handler.h"
#include <stdbool.h>
#include "define.h"
#include "UART.h"
#include "utils.h"

uint8_t ui8_imu_dma_rx_buff[d_IMU_RX_BUFF_SIZE] = {0};
uint8_t ui8_imu_msg[MAX_MSG_SIZE] = {0};


IMU_STRUCT imu_data = {0,  
											 0.0,0.0,0.0,
											 0.0,0.0,0.0,
											 0.0,0.0,0.0};

void UART_IMU_Init(void) // Config USART2: STM to IMU: (PA2-RX, PA3-TX);
{
	//Setup for GPIO
	RCC_AHB1PeriphClockCmd(IMU_PORT_CLK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IMU_TX|IMU_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(IMU_PORT,&GPIO_InitStructure);
	
	//Setup for UART
	RCC_APB1PeriphClockCmd(IMU_USART_CLK,ENABLE);
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = IMU_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(IMU_USART,&USART_InitStructure);
	
	//Setup for DMA USART2 Interrupt
//	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//Setup DMA
	RCC_AHB1PeriphClockCmd(IMU_USART_DMA_CLK, ENABLE);
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(IMU_RX_DMA_STREAM);
	DMA_InitStructure.DMA_Channel = 							IMU_RX_DMA_CHANNEL; 
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = 			(uint32_t)&ui8_imu_dma_rx_buff[0];
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)IMU_USART + 0x04;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = 							DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = 				DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Circular;
	DMA_InitStructure.DMA_BufferSize = 						d_IMU_RX_BUFF_SIZE;
	DMA_InitStructure.DMA_MemoryBurst = 					DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = 			DMA_PeripheralBurst_Single;
	DMA_Init(IMU_RX_DMA_STREAM, &DMA_InitStructure);
	USART_DMACmd(IMU_USART, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(IMU_RX_DMA_STREAM, ENABLE);
}

void read_imu_board(void)
{
	static uint32_t imu_time_tick;
	static uint32_t RFReInd=0, RFWrInd=0;
	static uint32_t RFWrInd_1 = 0;
	static uint32_t start_ind = 0, end_ind = 0;
	uint32_t i, j, k, len, len1;
	//bool done = false;

	//get buffer index of new data
	RFWrInd = d_IMU_RX_BUFF_SIZE- IMU_RX_DMA_STREAM->NDTR;	
	if (RFWrInd_1 != RFWrInd)
	{
		RFWrInd_1 = RFWrInd;
		imu_time_tick = SysTick_GetTick();		
	}
	else if (SysTick_IsTimeout(imu_time_tick, 1000))
  {
		// reset
		imu_data.isavailable = false;
		RFWrInd_1 = RFWrInd;	
  }
	if (RFReInd <= RFWrInd) len = RFWrInd - RFReInd;
	else len = d_IMU_RX_BUFF_SIZE - (RFReInd - RFWrInd);
	
	//scan through new data
	for (i = 0; i < len; i++)
	{
			j = (RFReInd + i) % d_IMU_RX_BUFF_SIZE;
#if IMU_USE_ADIS
			if (ui8_imu_dma_rx_buff[j] == '\r')
#else		
			if (ui8_imu_dma_rx_buff[j] == '\n')// Copy entire msg to msg buffer
#endif
			{
					end_ind = j;
					if (start_ind < end_ind) len1 = end_ind - start_ind + 1;
					else len1 = d_IMU_RX_BUFF_SIZE - (start_ind - end_ind) + 1;
					if (len1 <= MAX_MSG_SIZE)
					{
						for (k = 0; k < len1; k++)
						{
								ui8_imu_msg[k] = ui8_imu_dma_rx_buff[(start_ind + k) % d_IMU_RX_BUFF_SIZE];
						}
						//process msg buffer
						imu_parse(ui8_imu_msg,len1);
					}
					start_ind = (end_ind + 1) % d_IMU_RX_BUFF_SIZE;
			}
			
	}
	RFReInd = RFWrInd;		
}

void imu_parse(uint8_t *imu_str, uint32_t len)
{
	static uint8_t field_ind[MAX_PARAMS_NUM+1];
	char *p; // cast to this pointer to avoid warning from compiler for my_atoi function
	uint32_t ind0, ind1, arg_cnt=0;
	int32_t idx;
	int32_t space;
	
#ifdef IMU_USE_XSENS_PRDID
	if (strncmp (imu_str, "$PRDID", 6 ))//unmatched
	{
		return;
	}
	idx = 7;//ignore ','
#elif defined IMU_USE_XSENS_PSON
	if (strncmp (imu_str, "$PSONCMS", 8 ))//unmatched
	{
		return;
	}
	idx = 9;//ignore ','

//	If end delimiter is '\r'	
//		if (strncmp (imu_str+1, "$PSONCMS", 8 ))//unmatched
//	{
//		uint8_t i;
//		i=0;
//		return;
//	}
//	idx = 10;//ignore ','	
#else	//adis
	if (imu_str[0] != 0x0A)
		return;
	idx = 1;
#endif    
	if (imu_getfieldind(imu_str, idx, field_ind,&arg_cnt) != ERR_NONE)
		return;
	//check arg length
	for (idx = 0; idx<arg_cnt; idx++)
	{
		ind0 = field_ind[idx];
		ind1 = field_ind[idx+1];
		space = ind1 - ind0;
		if ((space > 1) || ((space == 1) & (imu_str[ind1] == '\r')))
		{
			//valid arg
		}
		else//invalid arg
		{
			return;
		}
	}
#ifdef IMU_USE_XSENS_PRDID
	p = (char*)&imu_str[field_ind[1]];
	imu_data.roll_est = atof (p);
	p = (char*)&imu_str[field_ind[0]];
	imu_data.pitch_est = atof (p);
	p = (char*)&imu_str[field_ind[2]];
	imu_data.yaw_est = atof (p);	
#elif defined IMU_USE_XSENS_PSON
	{
		float q1,q2,q3,q4;
		p = (char*)&imu_str[field_ind[0]];
		q1 = atof (p);
		p = (char*)&imu_str[field_ind[1]];
		q2 = atof (p);
		p = (char*)&imu_str[field_ind[2]];
		q3 = atof (p);
		p = (char*)&imu_str[field_ind[3]];
		q4 = atof (p);
		
		//Estimate roll, pitch, yaw
		imu_data.yaw_est = -atan2(2*q2*q3+2*q1*q4,2*q1*q1+2*q2*q2-1)*RAD2DEG;		
		imu_data.pitch_est = -asin(2*q2*q4-2*q1*q3)*RAD2DEG;		
		imu_data.roll_est = -atan2(2*q3*q4+2*q1*q2,2*q1*q1+2*q4*q4-1)*RAD2DEG;				
		
		//Get accel parametter
		p = (char*)&imu_str[field_ind[4]];
		imu_data.acc_x = atof (p);
		p = (char*)&imu_str[field_ind[5]];
		imu_data.acc_y = atof (p);
		p = (char*)&imu_str[field_ind[6]];
		imu_data.acc_z = atof (p);		
		
		//Get gyro parametter
		p = (char*)&imu_str[field_ind[7]];
		imu_data.gyro_x = atof (p);
		p = (char*)&imu_str[field_ind[8]];
		imu_data.gyro_y = atof (p);
		p = (char*)&imu_str[field_ind[9]];
		imu_data.gyro_z = atof (p);
		
		//Get magnet parametter
		p = (char*)&imu_str[field_ind[10]];
		imu_data.mag_x = atof (p);
		p = (char*)&imu_str[field_ind[11]];
		imu_data.mag_y = atof (p);
		p = (char*)&imu_str[field_ind[12]];
		imu_data.mag_z = atof (p);
		
	#if SEND_MAG
		sprintf(txbuff_temp,"$PSONCMS,%7.4f,%7.4f,%7.4f\r\n",imu_data.mag_x,imu_data.mag_y,imu_data.mag_z);
		tx_len_temp = strlen(txbuff_temp);
	#endif		
	}		
#else	
	//Estimated angle
	p = (char*)&imu_str[field_ind[ROLL_EST_IDX]];
	//imu_data.roll_est = my_atoi (p)*0.01f;
	imu_data.roll_est = my_atoi(p) * 0.001f;
	//imu_data.roll_est = 0;
	
	p = (char*)&imu_str[field_ind[PITCH_EST_IDX]];
	//imu_data.pitch_est = my_atoi (p)*0.01f;
	imu_data.pitch_est = my_atoi(p) * 0.001f;

	p = (char*)&imu_str[field_ind[YAW_EST_IDX]];
	//imu_data.yaw_est = my_atoi (p)*0.01f;  
	imu_data.yaw_est = my_atoi(p) * 0.001f;  
	//imu_data.yaw_est = 0;
	
	//MY_T_PREDICT_PERIOD	= calculate_predict_period(imu_data.roll_est,imu_data.pitch_est);//

#if IMU_USE_PREDICT	
	imu_data.roll_est = roll_est_predict(imu_data.roll_est);
	imu_data.pitch_est = pitch_est_predict(imu_data.pitch_est);
	imu_data.yaw_est = yaw_est_predict(imu_data.yaw_est);	
#endif
	//Adis mag
//	p = (char*)&imu_str[field_ind[6]];
//	imu_data.mag_x = my_atoi (p);
//	p = (char*)&imu_str[field_ind[7]];
//	imu_data.mag_y = my_atoi (p);
//	p = (char*)&imu_str[field_ind[8]];
//	imu_data.mag_z = my_atoi (p); 		
	
	//gyro
	{
		float tmp1,tmp2,tmp3;
		p = (char*)&imu_str[field_ind[GYRO_X_IDX]];
		tmp1 = my_atoi (p);//*RAD2DEGREE*0.1f;
		p = (char*)&imu_str[field_ind[GYRO_Y_IDX]];
		tmp2 = my_atoi (p);//*RAD2DEGREE*0.1f;
		p = (char*)&imu_str[field_ind[GYRO_Z_IDX]];
		tmp3 = my_atoi (p);//*RAD2DEGREE*0.1f;

//	imu_data.gyro_x = tmp1 * 1.0374 + tmp2 * 0.0092 + tmp3 * 0.0517;
//	imu_data.gyro_y = tmp1 * (-0.0071) + tmp2 * 1.0015 + tmp3 * (-0.0379);
//	imu_data.gyro_z = tmp1 * (-0.0515) + tmp2 * 0.0147 + tmp3 * 1.0311;
		
		imu_data.gyro_x = tmp1;
		imu_data.gyro_y = tmp2; 
		imu_data.gyro_z = tmp3; 
	}

	//acc
	p = (char*)&imu_str[field_ind[6]];
	imu_data.acc_x = my_atoi (p);//*RAD2DEGREE*0.1f;
	p = (char*)&imu_str[field_ind[7]];
	imu_data.acc_y = my_atoi (p);//*RAD2DEGREE*0.1f;
	p = (char*)&imu_str[field_ind[8]];
	imu_data.acc_z = my_atoi (p);//*RAD2DEGREE*0.1f;
	
	//fog
	//p = (char*)&imu_str[field_ind[12]];
	//imu_data.fog_z = my_atoi(p)*0.01;//unit: mdeg/s
#if IMU_USE_PREDICT		
	imu_data.fog_z = fog_base_predict(imu_data.fog_z);
#endif	
	//imu_data.fog_z_integral += imu_data.fog_z * 0.001 * T_CTRL;
#endif	
	imu_data.isavailable = TRUE;
}

E_ERROR_IMU_MSG_T imu_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd,	uint32_t *arg_cnt)
{
	E_ERROR_IMU_MSG_T err = ERR_NONE;
	uint8_t i = startInd;
	uint16_t mark_pos=0;
	*arg_cnt = 0;
	fieldInd[mark_pos++] = i;
	while ((buf[i] != '\n') && (buf[i] != '\r'))
	{
		if (((buf[i]>='0') && (buf[i]<='9')) || (buf[i] == '-'))
		{
			//find next non digit character
			do{
				i++;
			}
			while (((buf[i]>='0') && (buf[i]<='9')) || (buf[i] == '.'));
			fieldInd[mark_pos++] = i+1;
			(*arg_cnt)++;
			if (*arg_cnt > MAX_PARAMS_NUM)
			{
				err = ERR_TOO_MANY_ARG;
				return err;
			}
		}
		i++;
	}
	fieldInd[mark_pos] = i;
	return err;
}
void read_adis(void)
{
//	uint16_t addr[] = {0x0E00,							  // temperature
//							 0x1200, 0x1600, 0x1A00, 	  // gyro
//							 0x1E00, 0x2200, 0x2600,	  // accelerometer
//							 0x2800, 0x2A00, 0x2C00,	  // magnetometer
//							 0x6A00,	0x6C00, 0x6E00,	  // euler angle		
//							 0x2E00};						  // barometer
//  int16_t i, tbuff;

//  		
//	SPI_Cmd(SPI2, DISABLE);			
//  
//  for(i=0; i<14; i++)
//	{
//		my_delay_us(3);
//	  SPI_Cmd(SPI2, ENABLE);
//    	
//		SPI_I2S_SendData(SPI2, (uint16_t)addr[i]);
//		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
//   	tbuff = SPI_I2S_ReceiveData(SPI2);
//	
//		SPI_Cmd(SPI2, DISABLE);

//		if (i==1)
//	  		marg[i-1] = (real_T)tbuff * 0.565 + 2500.0;		// TEM_OUT (unit 0.01oC)
//		else if ((i==2)||(i==3)||(i==4))
//	  		marg[i-1] = (real_T)tbuff * 2.0*3.14159/18.0;	// GYRO_OUT (unit mrad/s)
//		else if ((i==5)||(i==6)||(i==7))
//	  		marg[i-1] = (real_T)tbuff * 0.8;						// ACC_OUT (unit mg)
//		else if ((i==8)||(i==9)||(i==10))		  
//	  		marg[i-1] = (real_T)tbuff * 0.1;						// MAG_OUT (unit mgauss)
//		else if ((i==11)||(i==12)||(i==13))
//	  		marg[i-1] = (real_T)tbuff * 180.0/3276.8;		   // EULER_OUT (unit 0.1o)
//  }

 
// Axis transformation -x,y,-z
/*
		marg[1] = -marg[1];
		marg[3] = -marg[3];
		marg[4] = -marg[4];
		marg[6] = -marg[6];
		marg[7] = -marg[7];
		marg[9] = -marg[9];	
*/
// Magnetic offset calibration
	
		//marg[7] = marg[7] + 45;
		//marg[8] = marg[8] + 5;
		//marg[9] = marg[9] +45;
}
void reset_adis(void)
{
  		
	cmdw_adis(0x8002);				// select Page 2

	cmdw_adis(0xA8AA);				// HARD_IRON_X 420(1A4)
	cmdw_adis(0xA900);
	cmdw_adis(0xAA00);				// HARD_IRON_Y 0 	(32)
	cmdw_adis(0xAB00);
	cmdw_adis(0xACA4);				// HARD_IRON_Z 420(1A4)
	cmdw_adis(0xAD01);
//	cmdw_adis(0x8200);				// reset EKF
//	cmdw_adis(0x8380);				

	cmdw_adis(0x8003);				// select Page 3

   cmdw_adis(0x8C04);				// Sample rate 2460/(4+1)  	
	cmdw_adis(0x8D00);

	cmdw_adis(0xD008);				// Body frame anable, mag enable
	cmdw_adis(0xD102);				// Adaptive EKF enable 01, fade enable 02
	cmdw_adis(0x860C);				// Data ready on DIO1
	cmdw_adis(0x8701);				// Alarm indicator on DIO2

	cmdw_adis(0xE0AC);			   // Measurement covariance, gyro
	cmdw_adis(0xE1C5);
	cmdw_adis(0xE227);
	cmdw_adis(0xE337);

	cmdw_adis(0xE4FF);			   // Measurement covariance, gyro bias
	cmdw_adis(0xE5E6);
	cmdw_adis(0xE65B);
	cmdw_adis(0xE72E);

	cmdw_adis(0xEC5F);    	   	// Measurement covariance, acc
	cmdw_adis(0xED70);
	cmdw_adis(0xEE89);
	cmdw_adis(0xEF31);

	cmdw_adis(0xF077);			   // Measurement covariance, magnetic
	cmdw_adis(0xF1CC);
	cmdw_adis(0xF2AB);
	cmdw_adis(0xF332);
	
	cmdw_adis(0x8208);				//	Flash update
	cmdw_adis(0x8300);

 	delay_01ms(20000);
//	cmdw_adis(0x8000);				// select Page 0

	while(1);
}
void cmdr_adis(uint16_t *buff, uint16_t addr, uint16_t N)
{
	uint16_t i, addr1, temp;
	SPI_Cmd(SPI2, ENABLE);

	addr1 = addr; 
	temp  = (addr1<<8);

	SPI_I2S_SendData(SPI2, (uint16_t)temp);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	temp = SPI_I2S_ReceiveData(SPI2);

	SPI_Cmd(SPI2, DISABLE);			

	for(i=0; i<N; i++)
	{
		my_delay_us(10);
		SPI_Cmd(SPI2, ENABLE);
	
		addr1 = addr1 + 2; 
		temp = (addr1<<8);

		SPI_I2S_SendData(SPI2, (uint16_t)temp);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		buff[i] = SPI_I2S_ReceiveData(SPI2);

		SPI_Cmd(SPI2, DISABLE);
	}
}


void cmdw_adis(uint16_t data)
{
  	uint16_t temp;

  	SPI_Cmd(SPI2, ENABLE);
	my_delay_us(5);
  	SPI_I2S_SendData(SPI2, (uint16_t)data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
   temp = SPI_I2S_ReceiveData(SPI2);
	my_delay_us(5);
	SPI_Cmd(SPI2, DISABLE);
	my_delay_us(20);	
}

