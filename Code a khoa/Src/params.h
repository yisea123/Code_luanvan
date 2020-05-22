#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum{
	EEP_CODE_VERSION=0,
	EEP_PL_JOINT_ANGLE,
	EEP_BASE_JOINT_ANGLE,
	EEP_PL_RADIUS,
	EEP_BASE_RADIUS,
	EEP_SV_ARM_LENGTH,
	EEP_CON_ARM_LENGTH,
	EEP_DEFAUT_Z_HEIGHT,
	EEP_ANGLE_SV_ARM_1,
	EEP_ANGLE_SV_ARM_2,
	EEP_ANGLE_SV_ARM_3,
	EEP_ANGLE_SV_ARM_4,
	EEP_ANGLE_SV_ARM_5,
	EEP_ANGLE_SV_ARM_6,
	EEP_PARAM_MAX
}PARAMS_ENUM;

typedef union{
	float f;
	uint8_t byte[4];
	uint32_t u32_data;
}PARAM_DATA_UNION;

bool PARAMS_Save_Float(PARAMS_ENUM param_type, float f);
bool PARAMS_Save_Array(PARAMS_ENUM param_type, uint8_t *data);
bool PARAMS_Load_Float(PARAMS_ENUM param_type, float *f);
bool PARAMS_Load_Array(PARAMS_ENUM param_type, uint8_t *data);
bool PARAMS_Save_Uint32(PARAMS_ENUM param_type, uint32_t data);
bool PARAMS_Load_Uint32(PARAMS_ENUM param_type, uint32_t *data);
void PARAMS_Load_All(void);
void PARAMS_Save_Counter(void);
void PARAMS_Save_Default(void);
#endif
