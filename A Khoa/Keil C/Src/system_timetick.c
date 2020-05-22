
/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include "system_timetick.h"
#include "GPIO.h"
uint32_t	tick_count;
uint16_t	tick_flag;


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}
	  
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  tick_flag = 1;
  tick_count++;
}
uint32_t SysTick_GetTick(void)
{
  return tick_count;
}
bool SysTick_IsTimeout(uint32_t start_time, uint32_t timeout_ms)
{
	return (tick_count - start_time > timeout_ms*F_CTRL/1000);
}