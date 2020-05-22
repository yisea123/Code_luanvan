#ifndef CALCULATOR_H
#define CALCULATOR_H
#include "control.h"

typedef enum
{
	PLATFORM_JOINT_ANGLE,
	BASE_JOINT_ANGLE,
	PLATFORM_RADIUS,
	BASE_RADIUS,
	SERVO_ARM_LENGTH,
	CONNECTING_ARM_LENGTH,
	DEFAULT_Z_HEIGHT,
	ANGLE_OF_SERVO_ARM_1,
	ANGLE_OF_SERVO_ARM_2,
	ANGLE_OF_SERVO_ARM_3,
	ANGLE_OF_SERVO_ARM_4,
	ANGLE_OF_SERVO_ARM_5,
	ANGLE_OF_SERVO_ARM_6,
	MAX_PARRAMETTER
}PARAMETTER_ENUM;

void Model_Init(void);
void Cal_Permanant_Parameter(void);
void Cal_Servo_Degree(float x_translate, float y_translate, float z_translate, float roll_translate, float pitch_translate, float yaw_translate, SERVO_DATA_STRUCT *servo_data);
void Set_Model_Parametter(PARAMETTER_ENUM e_para, float value);
float Get_Model_Parametter(PARAMETTER_ENUM e_para);
#endif
