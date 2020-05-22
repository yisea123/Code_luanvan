#include "params.h"
#include "eeprom.h"
#include "control.h"
#include "define.h"

extern float theta_platform;
extern float theta_base;
extern float platform_radius;
extern float base_radius;
extern float length_servo;
extern float length_connecting_arm;
extern float z_home;
extern float theta_servo_1;
extern float theta_servo_2;
extern float theta_servo_3;
extern float theta_servo_4;
extern float theta_servo_5;
extern float theta_servo_6;

uint8_t CODE_VESION_ARRAY[] = {0,0,0,1};

void PARAMS_Save_Default(void)
{
	PARAMS_Save_Array(EEP_CODE_VERSION, CODE_VESION_ARRAY);
	PARAMS_Save_Float(EEP_PL_JOINT_ANGLE,theta_platform);
	PARAMS_Save_Float(EEP_BASE_JOINT_ANGLE,theta_base);
	PARAMS_Save_Float(EEP_PL_RADIUS,platform_radius);
	PARAMS_Save_Float(EEP_BASE_RADIUS,base_radius);
	PARAMS_Save_Float(EEP_SV_ARM_LENGTH,length_servo);
	PARAMS_Save_Float(EEP_CON_ARM_LENGTH,length_connecting_arm);
	PARAMS_Save_Float(EEP_DEFAUT_Z_HEIGHT,z_home);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_1,theta_servo_1);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_2,theta_servo_2);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_3,theta_servo_3);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_4,theta_servo_4);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_5,theta_servo_5);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_6,theta_servo_6);
}
void PARAMS_Load_All(void)
{
	PARAM_DATA_UNION data;
	PARAMS_Load_Array(EEP_CODE_VERSION, data.byte);
	if ((data.byte[0] != CODE_VESION_ARRAY[0]) || (data.byte[1] != CODE_VESION_ARRAY[1]) 
		|| (data.byte[2] != CODE_VESION_ARRAY[2]) || (data.byte[3] != CODE_VESION_ARRAY[3]))
	{
		PARAMS_Save_Default();
	}
	PARAMS_Load_Float(EEP_PL_JOINT_ANGLE, &theta_platform);
	PARAMS_Load_Float(EEP_BASE_JOINT_ANGLE,&theta_base);
	PARAMS_Load_Float(EEP_PL_RADIUS,&platform_radius);
	PARAMS_Load_Float(EEP_BASE_RADIUS,&base_radius);
	PARAMS_Load_Float(EEP_SV_ARM_LENGTH,&length_servo);
	PARAMS_Load_Float(EEP_CON_ARM_LENGTH,&length_connecting_arm);
	PARAMS_Load_Float(EEP_DEFAUT_Z_HEIGHT,&z_home);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_1,&theta_servo_1);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_2,&theta_servo_2);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_3,&theta_servo_3);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_4,&theta_servo_4);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_5,&theta_servo_5);
	PARAMS_Load_Float(EEP_ANGLE_SV_ARM_6,&theta_servo_6);
}

void PARAMS_Save_All_Parametter(void)
{
	PARAMS_Save_Float(EEP_PL_JOINT_ANGLE,theta_platform);
	PARAMS_Save_Float(EEP_BASE_JOINT_ANGLE,theta_base);
	PARAMS_Save_Float(EEP_PL_RADIUS,platform_radius);
	PARAMS_Save_Float(EEP_BASE_RADIUS,base_radius);
	PARAMS_Save_Float(EEP_SV_ARM_LENGTH,length_servo);
	PARAMS_Save_Float(EEP_CON_ARM_LENGTH,length_connecting_arm);
	PARAMS_Save_Float(EEP_DEFAUT_Z_HEIGHT,z_home);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_1,theta_servo_1);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_2,theta_servo_2);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_3,theta_servo_3);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_4,theta_servo_4);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_5,theta_servo_5);
	PARAMS_Save_Float(EEP_ANGLE_SV_ARM_6,theta_servo_6);
}
bool PARAMS_Save_Array(PARAMS_ENUM param_type, uint8_t *data)
{
	return EEP_WriteBytes(data, (uint32_t)param_type<<2, 4);
}
bool PARAMS_Load_Array(PARAMS_ENUM param_type, uint8_t *data)
{
	bool res;
	res = EEP_ReadBytes(data, (uint32_t)param_type<<2, 4);
	return res;
}
bool PARAMS_Save_Uint32(PARAMS_ENUM param_type, uint32_t data)
{
	return EEP_WriteBytes((uint8_t*)&data, (uint32_t)param_type<<2, 4);
}
bool PARAMS_Load_Uint32(PARAMS_ENUM param_type, uint32_t *data)
{
	bool res;
	res = EEP_ReadBytes((uint8_t*)data, (uint32_t)param_type<<2, 4);
	return res;
}
bool PARAMS_Save_Float(PARAMS_ENUM param_type, float f)
{
	PARAM_DATA_UNION data;
	data.f = f;
	return PARAMS_Save_Array(param_type, data.byte);
}
bool PARAMS_Load_Float(PARAMS_ENUM param_type, float *f)
{
	PARAM_DATA_UNION data;
	bool res;
	res = EEP_ReadBytes(data.byte, (uint32_t)param_type<<2, 4);
	*f = data.f;
	return res;
}
