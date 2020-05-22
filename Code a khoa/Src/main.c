#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
#include "utils.h"
#include "system_state.h"
#include "IMU_Quest.h"                  /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "GPIO.h"
#include "LED.h"
#include "FSMC.h"
#include "UART.h"
#include "control.h"
#include "define.h"
#include "CALCULATOR.h"
#include "hmi_handler.h"
#include "eeprom.h"
#include "params.h"

extern bool Start_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Stop_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Home_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);
extern bool Home_Button_Handler(AXIS_ENUM e_axis, uint8_t *ui8_data, uint32_t ui32_data_cnt);

extern bool b_update_platform_pos_enable;
extern bool b_update_servo_pos_enable;

bool b_is_new_data_model;
uint32_t cnt, send_index;

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */    

  /* Enable SysTick at T_CTRL interrupt */
  SysTick_Config(SystemCoreClock/F_CTRL);		
  delay_us(500);
	Model_Init();
	Control_Init();
	LED_Init();
	delay_us(500);
  Button_Init();
	delay_us(500);
	Servo_Home_Switch_Init();
	delay_us(500);
	FSMC_Init();
	delay_us(500);
	UART_PC_Init();
	delay_us(500);
	System_Init();
	delay_us(500);
	Relay_Init();
	delay_us(500);
	FSMC_ENC_Update();
	FSMC_ENC_Reset();
	EEP_Init();
	delay_us(500);
	PARAMS_Load_All();
	HMI_Init();
	delay_us(500);
	//PARAMS_Load_All();
//	delay_us(500);
//	while(1)
//	{
//		if(tick_flag)
//		{
//			tick_flag = 0;
//			int32_t i32_pulse_num = 1;
//			Servo_Move_Test(i32_pulse_num);
//		} 		
//	}  
//	while(1)
//	{
//		if(tick_flag)
//		{
//			tick_flag = 0;
//			int32_t i32_pulse_num[6] = {50,1,1,1,1,1};
//			Servo_Move(i32_pulse_num);
//		} 		
//	} 
	while(1)
	{
		if (tick_flag)
		{
			if(Get_Button_Start_Press_Status() == Bit_RESET)
			{
				Start_Handler(0,0,0);
			}
			if(Get_Button_Stop_Press_Status() == Bit_RESET)
			{
				Stop_Handler(0,0,0);
			}
			if(Get_Button_Home_Press_Status() == Bit_RESET)
			{
				Home_Button_Handler(0,0,0);
			}
			if(b_is_new_data_model)
			{
				b_is_new_data_model = false;
				Cal_Permanant_Parameter();
			}
			tick_flag = 0;
			FSMC_ENC_Update();
			System_Process();//control hexapod
			LED_Display();
			UART_Read_Cmd();//handle cmd
			HMI_Process();
			HMI_Update_DisInpParams_Param();
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_1, (int32_t)(FSMC_ENC_Get_Pos(SERVO1)*10));
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_2, (int32_t)(FSMC_ENC_Get_Pos(SERVO2)*10));
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_3, (int32_t)(FSMC_ENC_Get_Pos(SERVO3)*10));
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_4, (int32_t)(FSMC_ENC_Get_Pos(SERVO4)*10));
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_5, (int32_t)(FSMC_ENC_Get_Pos(SERVO5)*10));
			HMI_Update_InpParams_Param(HMI_SER_ENC_ANGLE_6, (int32_t)(FSMC_ENC_Get_Pos(SERVO6)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_1, (int32_t)(Get_Servo_Data_Pos(SERVO1)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_2, (int32_t)(Get_Servo_Data_Pos(SERVO2)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_3, (int32_t)(Get_Servo_Data_Pos(SERVO3)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_4, (int32_t)(Get_Servo_Data_Pos(SERVO4)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_5, (int32_t)(Get_Servo_Data_Pos(SERVO5)*10));
			HMI_Update_InpParams_Param(HMI_SER_SET_ANGLE_6, (int32_t)(Get_Servo_Data_Pos(SERVO6)*10));
			cnt++;
			if(cnt == 20)
			{
				cnt = 0;
				UART_PC_Send_Servo_Data_Pos_Test();
			}
			if(b_update_servo_pos_enable)
			{
				b_update_servo_pos_enable = false;
				UART_PC_Send_Servo_Data_Pos();
			}
			if(b_update_platform_pos_enable)
			{
				b_update_platform_pos_enable = false;
				UART_PC_Send_PL_Pos();
			}
		}
  }    
}


