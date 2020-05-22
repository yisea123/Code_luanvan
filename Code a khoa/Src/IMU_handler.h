#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define d_IMU_RX_BUFF_SIZE 				512
#define MAX_MSG_SIZE 							200

#define ROLL_EST_IDX 							0
#define PITCH_EST_IDX 						1
#define YAW_EST_IDX 							2

//#define ROLL_ADIS_IDX 				    3
//#define PITCH_ADIS_IDX						4
//#define YAW_ADIS_IDX 							5

#define GYRO_X_IDX 								3
#define GYRO_Y_IDX 								4
#define GYRO_Z_IDX 								5

#define MIN_PARAMS_NUM GYRO_Z_ADIS	//3 estimated angle + 3 adis angle + 3 gyro
#define MAX_PARAMS_NUM 						20

#define IMU_USART             UART4
#define IMU_USART_CLK         RCC_APB1Periph_UART4
#define IMU_USART_DMA_CLK			RCC_AHB1Periph_DMA1
#define IMU_PORT              GPIOC
#define IMU_PORT_CLK					RCC_AHB1Periph_GPIOC
#define IMU_TX                GPIO_Pin_10
#define IMU_TX_SOURCE         GPIO_PinSource10
#define IMU_RX                GPIO_Pin_11
#define IMU_RX_SOURCE         GPIO_PinSource11
#define IMU_AF                GPIO_AF_UART4
#define IMU_BAUDRATE          (uint32_t)921600 //115200
#define IMU_DATA_REG          (uint32_t)IMU_USART + 0x04
#define IMU_TX_DMA_STREAM     DMA1_Stream4
#define IMU_TX_DMA_CHANNEL    DMA_Channel_4
#define IMU_TX_STREAM_IRQ     DMA1_Stream4_IRQn
#define IMU_TX_DMA_FLAG       DMA_FLAG_TCIF4   
#define IMU_RX_DMA_STREAM     DMA1_Stream2
#define IMU_RX_DMA_CHANNEL    DMA_Channel_4
#define IMU_RX_STREAM_IRQ     DMA1_Stream2_IRQn
#define IMU_RX_DMA_FLAG       DMA_FLAG_TCIF2

typedef struct
{
  bool isavailable;
	
	float roll_est;/* unit 0.01 degree */
  float pitch_est;
  float yaw_est;
	
	float roll_adis;
  float pitch_adis;
  float yaw_adis;
	
	double gyro_x;
	double gyro_y;	
	double gyro_z;
	
	double fog_z;
	double fog_z_integral;
	
	double fog_y;
	double fog_y_integral;
	
	float mag_x;
	float mag_y;
	float mag_z;
	
	float acc_x;
	float acc_y;
	float acc_z;	
}IMU_STRUCT;

typedef enum{
	ERR_MISSING_ARG=0,
	ERR_TOO_MANY_ARG,
	ERR_NONE
}E_ERROR_IMU_MSG_T;

void UART_IMU_Init(void);
void imu_parse(uint8_t *imu_str,uint32_t len);
void read_imu_board(void);
E_ERROR_IMU_MSG_T imu_getfieldind(uint8_t *buf, uint8_t startInd, uint8_t *fieldInd,	uint32_t *arg_cnt);
void read_adis(void);
void cmdr_adis(uint16_t *buff, uint16_t addr, uint16_t N);
void cmdw_adis(uint16_t data);
#endif