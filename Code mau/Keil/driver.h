
#include "stm32f4xx.h"
#include "system_timetick.h"

#define CS_DDA1 		0x60000000
#define CS_DDA2 		0x60000002
#define CS_DDA3 		0x60000004
#define CS_DDA4 		0x60000006
#define CS_DDA5 		0x60000008
#define CS_DDA6 		0x6000000A

#define CS_ENC1 		0x60000020
#define CS_ENC2 		0x60000022
#define CS_ENC3 		0x60000024
#define CS_ENC4 		0x60000026
#define CS_ENC5 		0x60000028
#define CS_ENC6 		0x6000002A


#define		BUFF_SIZE			50 
extern uint8_t txbuff[BUFF_SIZE];

void limitswitch_init(void);
void init_uart(void);
void init_main(void);
void fsmc_write(uint32_t address, uint16_t data);
uint16_t fsmc_read(uint32_t enc_channel);

void IntToStr6(int16_t u, uint8_t *y);
void send_data(void);

void delay_us(uint16_t period);
void delay_01ms(uint16_t period);
