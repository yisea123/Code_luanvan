#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "eeprom.h"
#include "utils.h"

/*******************************************************************************
 * @fn     I2C_ReadBytes     
 * @brief  Read/Receive l bytes from sensors
 * @param  c: pointer to array that will store bytes from sensors
 * @param  s: sensor's address
 * @param  a: start address of the register of the sensor
 * @param  l: length of the array
 * @retval None
 ******************************************************************************/
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l)
{
	uint32_t time;

    if (l == 0)   return false;

	/* While the bus is busy */
	time = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	}
	
	/* Send Sensor address for write */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Transmitter);
    	
	/* Test on EV6 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time--) == 0) return false;
	} 
    	
	/* Send the Register address to read from: Only one byte address */
	I2C_SendData(I2Cx, a);  
	
	/* Test on EV8 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition a second time to RESTART bus */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	} 
	
	/* Send Sensor address for read */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Receiver);  

    /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
    __disable_irq();
    	
    /* Test on EV6 and clear it */
    time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if ((time--) == 0) return false;
    }

    while(l)
    {
        if(l == 1)
        {
            /* This configuration should be in the last second transfer byte */
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
            
            /* Send STOP Condition */
            I2C_GenerateSTOP(I2Cx, ENABLE);
        }

        /* Test on EV7 and clear it */
        time = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            if ((time--) == 0) return false;
        }

        /* Read the byte received from the Sensor */
        /* Point to the next location where the byte read will be saved */
        *c = I2C_ReceiveData(I2Cx);
        c++;
        
        /* Decrease the read bytes counter */
        l--;

    }

    /* Wait to make sure that STOP control bit has been cleared */
    time = I2C_TIMEOUT;
    while(I2Cx->CR1 & I2C_CR1_STOP)
    {
        if((time--) == 0) return false;
    }  
    
    /* Re-Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);  
    
    /* ENABLE ALL INTERRUPT */
    __enable_irq();     
    return true; 
}

/*******************************************************************************
 * @fn     I2C_WriteBytes     
 * @brief  Write/Send l bytes to sensors
 * @param  c: pointer to array that will store bytes to send to sensors
 * @param  s: sensor's address
 * @param  a: start address of the register of the sensor
 * @param  l: length of the array
 * @retval None
 ******************************************************************************/
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, uint8_t *c, uint8_t s, uint8_t a, uint8_t l)
{
	uint32_t time;
    I2C_AcknowledgeConfig(I2Cx, ENABLE); 
	/* While the bus is busy */
	time = I2C_TIMEOUT;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
	{
		if((time--) == 0) return false;
	}
	
	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((time--) == 0) return false;
	}
	
	/* Send Sensor address for write */
	I2C_Send7bitAddress(I2Cx, s, I2C_Direction_Transmitter);
    	
	/* Test on EV6 and clear it */
	time = I2C_TIMEOUT;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time--) == 0) return false;
	} 
    
	/* Send the Register address to read from: Only one byte address */
	I2C_SendData(I2Cx, a);  

    /* Test on EV8 and clear it */
    time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if (time-- == 0) return false;
    }
    
    /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
    __disable_irq();
    while(l)
    {
        /* Send the data & increase the pointer of write buffer */
        I2C_SendData(I2Cx, *c); 
        c++;
        l--;  
        /* Test on EV8_2 to ensure data is transmitted, can used EV_8 for faster transmission*/
        time = I2C_TIMEOUT;
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        {
            if ((time--) == 0) return false;
        }  
    }

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2Cx, ENABLE);
    /* ENABLE ALL INTERRUPT */
    __enable_irq();
    return true;
}

/*******************************************************************************
 * @fn     EEP_Init     
 * @brief  Interrupt to handle the reception data
 * @param  None
 * @retval None
 ******************************************************************************/
void EEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	/* GPIO configuration */
	GPIO_InitStructure.GPIO_Pin   = EEP_SCL | EEP_SDA; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;		//note
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, EEP_SDA_SOURCE, EEP_AF);    
	GPIO_PinAFConfig(GPIOB, EEP_SCL_SOURCE, EEP_AF);  

	/* I2C configuration */
	RCC_APB1PeriphClockCmd(EEP_I2C_CLK, ENABLE);
	I2C_DeInit(EEP_I2C);
	I2C_InitStructure.I2C_ClockSpeed 			= EEP_BAUDRATE;	
	I2C_InitStructure.I2C_Mode 					= I2C_Mode_I2C;					
	I2C_InitStructure.I2C_DutyCycle 			= I2C_DutyCycle_2;							
	I2C_InitStructure.I2C_OwnAddress1 			= 0;
	I2C_InitStructure.I2C_Ack 					= I2C_Ack_Enable;						
	I2C_InitStructure.I2C_AcknowledgedAddress 	= I2C_AcknowledgedAddress_7bit;
	I2C_Init(EEP_I2C, &I2C_InitStructure);
	I2C_Cmd(EEP_I2C, ENABLE);
}

/******************************************************************************
 * @fn     EEP_ReadBytes     
 * @brief  Read bytes from EEPROM memory
 * @param  c: pointer to read array data
 * @param  r: start regiter address from 0 to 511 (EEPROM 24C04: 512 bytes)
 *            And it is divided into 2 pages (256 bytes/page)
 * @param  l: length of data to read
 * @retval true or false
 ******************************************************************************/
bool EEP_ReadBytes(uint8_t* c, uint16_t r, uint16_t l)
{
    uint16_t reg_add;
    uint8_t eep_add;
    uint16_t i;
    
    for (i = 0; i < l; i++)
    {
        eep_add = EEP_ADD;
        reg_add = r + i;
        if (reg_add > 511)
        {
            /* Out of range */
            return false;
        }
        else if (reg_add > 255)
        {
            /* Page 1: insert bit page 1 in eeprom address */
            reg_add -= 256;
            eep_add |= EEP_PAGE_1;
        }
        /*
        else (reg_add < 255) Page 0: do nothing
        */
        
        // Random read 1 byte
        if (I2C_ReadBytes(EEP_I2C, c, eep_add, reg_add, 1) == false) 
        {
            return false;
        }
        c++;
    }
    return true;
}

/******************************************************************************
 * @fn     EEP_WriteBytes     
 * @brief  Write bytes in EEPROM memory
 * @param  c: pointer to write array data
 * @param  r: start regiter address from 0 to 511 (EEPROM 24C04: 512 bytes)
 *            And it is divided into 2 pages (256 bytes/page)
 * @param  l: length of data to write
 * @retval true or false
 ******************************************************************************/
bool EEP_WriteBytes(uint8_t* c, uint16_t r, uint16_t l)
{
    uint16_t reg_add;
    uint8_t eep_add;
    uint16_t i;
    
    for (i = 0; i < l; i++)
    {
        eep_add = EEP_ADD;
        reg_add = r + i;
        
        if (reg_add > 511) 
        {
            /* Out of range */
            return false;
        }
        else if (reg_add > 255)
        {
            /* Page 1: insert bit page 1 in eeprom address */
            reg_add -= 256;
            eep_add |= EEP_PAGE_1;
        }
        /*
        else (reg_add < 255) page 0: do nothing
        */
        
        // Random write 1 byte into eeprom
        if (I2C_WriteBytes(EEP_I2C, c, eep_add, reg_add, 1) == false)
        {
            return false;
        }
        c++;
        my_delay_us(5000); //Important: Maximum write cycle time = 5ms
    }
    return true;
}
