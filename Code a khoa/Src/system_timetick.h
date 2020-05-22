
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*  These function configure scheduler timer
    Sampling time is set by changing value PERIOD
    in the file "system_timetick.c"
*/

extern uint32_t tick_count;
extern uint16_t tick_flag;


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
uint32_t SysTick_GetTick(void);
bool SysTick_IsTimeout(uint32_t start_time, uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */


