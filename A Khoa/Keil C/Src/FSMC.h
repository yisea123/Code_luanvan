#ifndef FSMC_H
#define FSMC_H

#include <stdint.h>
#include <stdbool.h>
#include "control.h"

void FSMC_Write(uint32_t ui_address, uint32_t ui_data);
uint16_t FSMC_Read(uint32_t ui_enc_channel);
void FSMC_Init(void);
void Servo_Move(int32_t i_pulse_num[]);
void Servo_Move_Test(int32_t i_pulse_num);
void FSMC_ENC_Update(void);
float FSMC_ENC_Get_Pos(SERVO_ENUM e_servo);
float FSMC_ENC_Get_Vel(SERVO_ENUM e_servo);
void FSMC_ENC_Reset(void);
#endif