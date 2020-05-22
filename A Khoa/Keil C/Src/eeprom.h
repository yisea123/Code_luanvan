#include <stdint.h>
#include <stdbool.h>
/* 
 * 5-EEPROM: I2C1
 */
#define EEP_I2C               I2C1
#define EEP_I2C_CLK           RCC_APB1Periph_I2C1
#define EEP_PORT              GPIOB
#define EEP_SCL               GPIO_Pin_8
#define EEP_SCL_SOURCE        GPIO_PinSource8
#define EEP_SDA               GPIO_Pin_9
#define EEP_SDA_SOURCE        GPIO_PinSource9
#define EEP_AF                GPIO_AF_I2C1
#define EEP_BAUDRATE          400000  // 400kHz
#define EEP_DATA_REG          (uint32_t)EEP_I2C + 0x10
#define I2C_TIMEOUT           100000
//Note: EEPROM AT24C04 address is 8 bits:|1010|A2|A1|Page|R/W|
#define EEP_ADD               0xA0
#define EEP_PAGE_0            0x00
#define EEP_PAGE_1            0x02

bool EEP_WriteBytes(uint8_t* c, uint16_t r, uint16_t l);
bool EEP_ReadBytes(uint8_t* c, uint16_t r, uint16_t l);
void EEP_Init(void);
