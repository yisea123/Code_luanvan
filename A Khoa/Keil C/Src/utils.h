#ifndef _UTILS_H
#define _UTILS_H

#include <stdbool.h>
#include <stdint.h>

float Sub_Mod_Pi(float angle1,float angle2);
float Add_Mod_Pi(float angle1,float angle2);

void Euler_To_Body_Rate(float *euler_angle, float *euler_rate, float *body_rate);
void Body_To_Euler_Rate(float *euler_angle, float *euler_rate, float *body_rate);

void my_delay_us(uint32_t micros);
void delay_us(uint16_t period);
void delay_01ms(uint16_t period);

void IntToStr2(int32_t u, uint8_t *y);
void IntToStr3(int32_t u, uint8_t *y);
void IntToStr4(int32_t u, uint8_t *y);
void IntToStr5(int32_t u, uint8_t *y);
void IntToStr6(int32_t u, uint8_t *y);
void IntToStr7(int32_t u, uint8_t *y);
void IntToStr8(int32_t u, uint8_t *y);
void IntToStr9(int32_t u, uint8_t *y);
void IntToStr10(int32_t u, uint8_t *y);
void IntToStr11(int32_t u, uint8_t *y);
void IntToStr12(int32_t u, uint8_t *y);
int my_atoi(char *p);
#endif