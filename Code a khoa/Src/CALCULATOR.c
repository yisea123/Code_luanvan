#include <Math.h>
//#include "IMU_Quest.h"                  /* Model's header file */
//#include "rtwtypes.h"                  /* MathWorks types */
#include "CALCULATOR.h"
#include <stdint.h>
#include <stdbool.h>
#include "control.h"
#include "define.h"
#include "hmi_handler.h"

#define SERVO_REVERSE -1

//Model parametter
float theta_platform;
float theta_servo_1;
float theta_servo_2;
float theta_servo_3;
float theta_servo_4;
float theta_servo_5;
float theta_servo_6;
float theta_base;
float platform_radius;
float base_radius;
float length_servo;
float length_connecting_arm;
float z_home;

float theta_base_radian;
float theta_platform_radian; 
float theta_servo_1_radian;
float theta_servo_2_radian; 
float theta_servo_3_radian; 
float theta_servo_4_radian;
float theta_servo_5_radian;
float theta_servo_6_radian;

float base_rotation_x_1, base_rotation_x_2, base_rotation_x_3, base_rotation_x_4, base_rotation_x_5, base_rotation_x_6;
float base_rotation_y_1, base_rotation_y_2, base_rotation_y_3, base_rotation_y_4, base_rotation_y_5, base_rotation_y_6;
float base_rotation_z_1, base_rotation_z_2, base_rotation_z_3, base_rotation_z_4, base_rotation_z_5, base_rotation_z_6;

float p_x_1, p_x_2, p_x_3, p_x_4, p_x_5, p_x_6;
float p_y_1, p_y_2, p_y_3, p_y_4, p_y_5, p_y_6;

void Model_Init(void)
{
	theta_platform =				7.3380f;
	theta_servo_1 =					120.0000f;
	theta_servo_2 = 				-60.0000f;
	theta_servo_3 =					0.0000f;
	theta_servo_4 =					180.0000f;
	theta_servo_5 =	 				-120.0000f;
	theta_servo_6 =					60.0000f;
	theta_base =						20.0000f;
	platform_radius	=				33.2150f;
	base_radius	=						33.4890f;
	length_servo =					23.0000f;
	length_connecting_arm =	57.5650f;
	z_home =								48.9811f;
	Cal_Permanant_Parameter();
}
void Cal_Permanant_Parameter(void)
{
	// Convert to radian
	theta_base_radian = theta_base*PI/180.0000f;
	theta_platform_radian = theta_platform * PI / 180.0000f;
	theta_servo_1_radian = theta_servo_1 * PI / 180.0000f;
	theta_servo_2_radian = theta_servo_2 * PI / 180.0000f;
	theta_servo_3_radian = theta_servo_3 * PI / 180.0000f;
	theta_servo_4_radian = theta_servo_4 * PI / 180.0000f;
	theta_servo_5_radian = theta_servo_5 * PI / 180.0000f;
	theta_servo_6_radian = theta_servo_6 * PI / 180.0000f;
	
	// Calculate degree of base and x,y,z
	base_rotation_x_1 = base_radius*cosf(PI/6.0000f + theta_base_radian);
	base_rotation_x_2 = base_radius*cosf(PI/6.0000f - theta_base_radian);
	base_rotation_x_3 = base_radius*cosf(-PI/2.0000f + theta_base_radian);
	base_rotation_x_4 = -base_radius*cosf(-PI/2.0000f + theta_base_radian);
	base_rotation_x_5 = -base_radius*cosf(PI/6.0000f - theta_base_radian);
	base_rotation_x_6 = -base_radius*cosf(PI/6.0000f + theta_base_radian);
	
	base_rotation_y_1 = base_radius*sinf(PI/6.0000f + theta_base_radian);
	base_rotation_y_2 = base_radius*sinf(PI/6.0000f - theta_base_radian);
	base_rotation_y_3 = base_radius*sinf(-PI/2.0000f + theta_base_radian);
	base_rotation_y_4 = base_radius*sinf(-PI/2.0000f + theta_base_radian);
	base_rotation_y_5 = base_radius*sinf(PI/6.0000f - theta_base_radian);
	base_rotation_y_6 = base_radius*sinf(PI/6.0000f + theta_base_radian);
	
	base_rotation_z_1 = 0.0000f;
	base_rotation_z_2 = 0.0000f;
	base_rotation_z_3 = 0.0000f;
	base_rotation_z_4 = 0.0000f;
	base_rotation_z_5 = 0.0000f;
	base_rotation_z_6 = 0.0000f;
	
	// Calculate platform_coords
	p_x_1 = platform_radius*cosf(PI/6.0000f + theta_platform_radian);
	p_x_2 = platform_radius*cosf(PI/6.0000f - theta_platform_radian);
	p_x_3 = platform_radius*cosf(-PI/2.0000f + theta_platform_radian);
	p_x_4 = -platform_radius*cosf(-PI/2.0000f + theta_platform_radian);
	p_x_5 = -platform_radius*cosf(PI/6.0000f - theta_platform_radian);
	p_x_6 = -platform_radius*cosf(PI/6.0000f + theta_platform_radian);
	
	p_y_1 = platform_radius*sinf(PI/6.0000f + theta_platform_radian);
	p_y_2 = platform_radius*sinf(PI/6.0000f - theta_platform_radian);
	p_y_3 = platform_radius*sinf(-PI/2.0000f + theta_platform_radian);
	p_y_4 = platform_radius*sinf(-PI/2.0000f + theta_platform_radian);
	p_y_5 = platform_radius*sinf(PI/6.0000f - theta_platform_radian);
	p_y_6 = platform_radius*sinf(PI/6.0000f + theta_platform_radian);
}

void Cal_Servo_Degree(float x_translate, float y_translate, float z_translate, float roll_translate, float pitch_translate, float yaw_translate, SERVO_DATA_STRUCT *servo_data)
{
	//Init parametter
	float platform_pivot_x_1, platform_pivot_x_2, platform_pivot_x_3;
	float platform_pivot_x_4, platform_pivot_x_5, platform_pivot_x_6;
	float platform_pivot_y_1, platform_pivot_y_2, platform_pivot_y_3;
	float platform_pivot_y_4, platform_pivot_y_5, platform_pivot_y_6;
	float platform_pivot_z_1, platform_pivot_z_2, platform_pivot_z_3;
	float platform_pivot_z_4, platform_pivot_z_5, platform_pivot_z_6;

	float delta_L_x_1, delta_L_x_2, delta_L_x_3, delta_L_x_4, delta_L_x_5, delta_L_x_6;
	float delta_L_y_1, delta_L_y_2, delta_L_y_3, delta_L_y_4, delta_L_y_5, delta_L_y_6;
	float delta_L_z_1, delta_L_z_2, delta_L_z_3, delta_L_z_4, delta_L_z_5, delta_L_z_6;

	float virtual_leg_length_1, virtual_leg_length_2, virtual_leg_length_3, virtual_leg_length_4, virtual_leg_length_5, virtual_leg_length_6;

	float L_1, L_2, L_3, L_4, L_5, L_6;
	float M_1, M_2, M_3, M_4, M_5, M_6;
	float N_1, N_2, N_3, N_4, N_5, N_6;
	
	// Calculate platform pivot
	 platform_pivot_x_1 = p_x_1 * cosf(roll_translate)*cosf(yaw_translate) + p_y_1*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	 platform_pivot_x_2 = p_x_2 * cosf(roll_translate)*cosf(yaw_translate) + p_y_2*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	 platform_pivot_x_3 = p_x_3 * cosf(roll_translate)*cosf(yaw_translate) + p_y_3*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	 platform_pivot_x_4 = p_x_4 * cosf(roll_translate)*cosf(yaw_translate) + p_y_4*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	 platform_pivot_x_5 = p_x_5 * cosf(roll_translate)*cosf(yaw_translate) + p_y_5*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	 platform_pivot_x_6 = p_x_6 * cosf(roll_translate)*cosf(yaw_translate) + p_y_6*(sinf(pitch_translate)*sinf(roll_translate)*cosf(roll_translate)-cosf(pitch_translate)*sinf(yaw_translate)) + x_translate;
	
	 platform_pivot_y_1 = p_x_1 * cosf(roll_translate)*sinf(yaw_translate) + p_y_1*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	 platform_pivot_y_2 = p_x_2 * cosf(roll_translate)*sinf(yaw_translate) + p_y_2*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	 platform_pivot_y_3 = p_x_3 * cosf(roll_translate)*sinf(yaw_translate) + p_y_3*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	 platform_pivot_y_4 = p_x_4 * cosf(roll_translate)*sinf(yaw_translate) + p_y_4*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	 platform_pivot_y_5 = p_x_5 * cosf(roll_translate)*sinf(yaw_translate) + p_y_5*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	 platform_pivot_y_6 = p_x_6 * cosf(roll_translate)*sinf(yaw_translate) + p_y_6*(cosf(pitch_translate)*cosf(roll_translate)+sinf(pitch_translate)*sinf(roll_translate)*sinf(yaw_translate)) + y_translate;
	
	 platform_pivot_z_1 = -p_x_1 * sinf(roll_translate) + p_y_1 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;
	 platform_pivot_z_2 = -p_x_2 * sinf(roll_translate) + p_y_2 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;
	 platform_pivot_z_3 = -p_x_3 * sinf(roll_translate) + p_y_3 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;
	 platform_pivot_z_4 = -p_x_4 * sinf(roll_translate) + p_y_4 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;
	 platform_pivot_z_5 = -p_x_5 * sinf(roll_translate) + p_y_5 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;
	 platform_pivot_z_6 = -p_x_6 * sinf(roll_translate) + p_y_6 * (sinf(pitch_translate)*cosf(roll_translate)) + z_translate + z_home;

	// Calculate virtual led length
	 delta_L_x_1 =	base_rotation_x_1 - platform_pivot_x_1;
	 delta_L_x_2 =	base_rotation_x_2 - platform_pivot_x_2;
	 delta_L_x_3 =	base_rotation_x_3 - platform_pivot_x_3;
	 delta_L_x_4 =	base_rotation_x_4 - platform_pivot_x_4;
	 delta_L_x_5 =	base_rotation_x_5 - platform_pivot_x_5;
	 delta_L_x_6 =	base_rotation_x_6 - platform_pivot_x_6;
	
	 delta_L_y_1 =	base_rotation_y_1 - platform_pivot_y_1;
	 delta_L_y_2 =	base_rotation_y_2 - platform_pivot_y_2;
	 delta_L_y_3 =	base_rotation_y_3 - platform_pivot_y_3;
	 delta_L_y_4 =	base_rotation_y_4 - platform_pivot_y_4;
	 delta_L_y_5 =	base_rotation_y_5 - platform_pivot_y_5;
	 delta_L_y_6 =	base_rotation_y_6 - platform_pivot_y_6;
	
	 delta_L_z_1 =	base_rotation_z_1 - platform_pivot_z_1;
	 delta_L_z_2 =	base_rotation_z_2 - platform_pivot_z_2;
	 delta_L_z_3 =	base_rotation_z_3 - platform_pivot_z_3;
	 delta_L_z_4 =	base_rotation_z_4 - platform_pivot_z_4;
	 delta_L_z_5 =	base_rotation_z_5 - platform_pivot_z_5;
	 delta_L_z_6 =	base_rotation_z_6 - platform_pivot_z_6;
	
	 virtual_leg_length_1 =	sqrt(delta_L_x_1*delta_L_x_1 + delta_L_y_1*delta_L_y_1 + delta_L_z_1*delta_L_z_1);
	 virtual_leg_length_2 =	sqrt(delta_L_x_2*delta_L_x_2 + delta_L_y_2*delta_L_y_2 + delta_L_z_2*delta_L_z_2);
	 virtual_leg_length_3 =	sqrt(delta_L_x_3*delta_L_x_3 + delta_L_y_3*delta_L_y_3 + delta_L_z_3*delta_L_z_3);
	 virtual_leg_length_4 =	sqrt(delta_L_x_4*delta_L_x_4 + delta_L_y_4*delta_L_y_4 + delta_L_z_4*delta_L_z_4);
	 virtual_leg_length_5 =	sqrt(delta_L_x_5*delta_L_x_5 + delta_L_y_5*delta_L_y_5 + delta_L_z_5*delta_L_z_5);
	 virtual_leg_length_6 =	sqrt(delta_L_x_6*delta_L_x_6 + delta_L_y_6*delta_L_y_6 + delta_L_z_6*delta_L_z_6);
	
	// Calculate L, M, N
	float sub_length_arm_servo = length_connecting_arm*length_connecting_arm - length_servo*length_servo;
	 L_1	= virtual_leg_length_1*virtual_leg_length_1 - sub_length_arm_servo;
	 L_2	= virtual_leg_length_2*virtual_leg_length_2 - sub_length_arm_servo;
	 L_3	= virtual_leg_length_3*virtual_leg_length_3 - sub_length_arm_servo;
	 L_4	= virtual_leg_length_4*virtual_leg_length_4 - sub_length_arm_servo;
	 L_5	= virtual_leg_length_5*virtual_leg_length_5 - sub_length_arm_servo;
	 L_6	= virtual_leg_length_6*virtual_leg_length_6 - sub_length_arm_servo;

	 M_1	= 2.0000f * length_servo * (platform_pivot_z_1 - base_rotation_z_1);
	 M_2	= 2.0000f * length_servo * (platform_pivot_z_2 - base_rotation_z_2);
	 M_3	= 2.0000f * length_servo * (platform_pivot_z_3 - base_rotation_z_3);
	 M_4	= 2.0000f * length_servo * (platform_pivot_z_4 - base_rotation_z_4);
	 M_5	= 2.0000f * length_servo * (platform_pivot_z_5 - base_rotation_z_5);
	 M_6	= 2.0000f * length_servo * (platform_pivot_z_6 - base_rotation_z_6);

	 N_1	= 2.0000f * length_servo * ((cosf(theta_servo_1_radian)) * (platform_pivot_x_1 - base_rotation_x_1) + sinf(theta_servo_1_radian) * (platform_pivot_y_1 - base_rotation_y_1));
	 N_2	= 2.0000f * length_servo * ((cosf(theta_servo_2_radian)) * (platform_pivot_x_2 - base_rotation_x_2) + sinf(theta_servo_2_radian) * (platform_pivot_y_2 - base_rotation_y_2));
	 N_3	= 2.0000f * length_servo * ((cosf(theta_servo_3_radian)) * (platform_pivot_x_3 - base_rotation_x_3) + sinf(theta_servo_3_radian) * (platform_pivot_y_3 - base_rotation_y_3));
	 N_4	= 2.0000f * length_servo * ((cosf(theta_servo_4_radian)) * (platform_pivot_x_4 - base_rotation_x_4) + sinf(theta_servo_4_radian) * (platform_pivot_y_4 - base_rotation_y_4));
	 N_5	= 2.0000f * length_servo * ((cosf(theta_servo_5_radian)) * (platform_pivot_x_5 - base_rotation_x_5) + sinf(theta_servo_5_radian) * (platform_pivot_y_5 - base_rotation_y_5));
	 N_6	= 2.0000f * length_servo * ((cosf(theta_servo_6_radian)) * (platform_pivot_x_6 - base_rotation_x_6) + sinf(theta_servo_6_radian) * (platform_pivot_y_6 - base_rotation_y_6));
	
	float asinf1, asinf2, asinf3, asinf4, asinf5, asinf6;
	
	asinf1 = L_1/sqrtf(M_1*M_1+N_1*N_1);
	asinf2 = L_2/sqrtf(M_2*M_2+N_2*N_2);
	asinf3 = L_3/sqrtf(M_3*M_3+N_3*N_3);
	asinf4 = L_4/sqrtf(M_4*M_4+N_4*N_4);
	asinf5 = L_5/sqrtf(M_5*M_5+N_5*N_5);
	asinf6 = L_6/sqrtf(M_6*M_6+N_6*N_6);
	
//	if( (L_1/sqrt(M_1*M_1+N_1*N_1) > 1)  ||
//			(L_1/sqrt(M_1*M_1+N_1*N_1) < -1) ||
//			(L_2/sqrt(M_2*M_2+N_2*N_2) > 1)  ||
//			(L_2/sqrt(M_2*M_2+N_2*N_2) < -1) ||
//			(L_3/sqrt(M_3*M_3+N_3*N_3) > 1)  ||
//			(L_3/sqrt(M_3*M_3+N_3*N_3) < -1) ||
//			(L_4/sqrt(M_4*M_4+N_4*N_4) > 1)  ||
//			(L_4/sqrt(M_4*M_4+N_4*N_4) < -1) ||
//			(L_5/sqrt(M_5*M_5+N_5*N_5) > 1)  ||
//			(L_5/sqrt(M_5*M_5+N_5*N_5) < -1) ||
//			(L_6/sqrt(M_6*M_6+N_6*N_6) > 1)  ||
//			(L_6/sqrt(M_6*M_6+N_6*N_6) < -1))
	if(	asinf1 < -1 || asinf1 > 1 ||
			asinf2 < -1 || asinf2 > 1 ||
			asinf3 < -1 || asinf3 > 1 ||
			asinf4 < -1 || asinf4 > 1 ||
			asinf5 < -1 || asinf5 > 1 ||
			asinf6 < -1 || asinf6 > 1)
	{
		HMI_Set_DisInp_On(HMI_SYS_STT_ERROR);
		Set_Servo_State(STATE_ERROR);
		return;
	}
	else
	{
		servo_data[SERVO1].f_setpoint_angle = (asinf(L_1/sqrt(M_1*M_1+N_1*N_1)) - atanf(N_1/M_1))*RAD2DEG;
		servo_data[SERVO2].f_setpoint_angle = (asinf(L_2/sqrt(M_2*M_2+N_1*N_2)) - atanf(N_2/M_2))*SERVO_REVERSE*RAD2DEG;
		servo_data[SERVO3].f_setpoint_angle = (asinf(L_3/sqrt(M_3*M_3+N_1*N_3)) - atanf(N_3/M_3))*RAD2DEG;
		servo_data[SERVO4].f_setpoint_angle = (asinf(L_4/sqrt(M_4*M_4+N_1*N_4)) - atanf(N_4/M_4))*SERVO_REVERSE*RAD2DEG;
		servo_data[SERVO5].f_setpoint_angle = (asinf(L_5/sqrt(M_5*M_5+N_1*N_5)) - atanf(N_5/M_5))*RAD2DEG;
		servo_data[SERVO6].f_setpoint_angle = (asinf(L_6/sqrt(M_6*M_6+N_1*N_6)) - atanf(N_6/M_6))*SERVO_REVERSE*RAD2DEG;
		Set_Servo_State(STATE_CALCULATING);
		HMI_Set_DisInp_Off(HMI_SYS_STT_ERROR);
	}
	
	// Calculate servo angle:
//	Servo_Angle_Radian[0] = asinf(L_1/sqrt(M_1*M_1+N_1*N_1)) + atan(N_1/M_1);
//	Servo_Angle_Radian[1] = asinf(L_2/sqrt(M_2*M_2+N_1*N_2)) + atan(N_2/M_2);
//	Servo_Angle_Radian[2] = asinf(L_3/sqrt(M_3*M_3+N_1*N_3)) + atan(N_3/M_3);
//	Servo_Angle_Radian[3] = asinf(L_4/sqrt(M_4*M_4+N_1*N_4)) + atan(N_4/M_4);
//	Servo_Angle_Radian[4] = asinf(L_5/sqrt(M_5*M_5+N_1*N_5)) + atan(N_5/M_5);
//	Servo_Angle_Radian[5] = asinf(L_6/sqrt(M_6*M_6+N_1*N_6)) + atan(N_6/M_6);
}

void Set_Model_Parametter(PARAMETTER_ENUM e_para, float value)
{
	switch((int)e_para)
	{
		case PLATFORM_JOINT_ANGLE:
			theta_platform = value;
			break;
		case BASE_JOINT_ANGLE:
			theta_base = value;
			break;
		case PLATFORM_RADIUS:
			platform_radius = value;
			break;
		case BASE_RADIUS:
			base_radius = value;
			break;
		case SERVO_ARM_LENGTH:
			length_servo = value;
			break;
		case CONNECTING_ARM_LENGTH:
			length_connecting_arm = value;
			break;
		case DEFAULT_Z_HEIGHT:
			z_home = value;
			break;
		case ANGLE_OF_SERVO_ARM_1:
			theta_servo_1 = value;
			break;
		case ANGLE_OF_SERVO_ARM_2:
			theta_servo_2 = value;
			break;
		case ANGLE_OF_SERVO_ARM_3:
			theta_servo_3 = value;
			break;
		case ANGLE_OF_SERVO_ARM_4:
			theta_servo_4 = value;
			break;
		case ANGLE_OF_SERVO_ARM_5:
			theta_servo_5 = value;
			break;
		case ANGLE_OF_SERVO_ARM_6:
			theta_servo_6 = value;
			break;
	}
	Cal_Permanant_Parameter();
}
float Get_Model_Parametter(PARAMETTER_ENUM e_para)
{
	switch((int)e_para)
	{
		case PLATFORM_JOINT_ANGLE:
			return theta_platform;
			break;
		case BASE_JOINT_ANGLE:
			return theta_base;
			break;
		case PLATFORM_RADIUS:
			return platform_radius;
			break;
		case BASE_RADIUS:
			return base_radius;
			break;
		case SERVO_ARM_LENGTH:
			return length_servo;
			break;
		case CONNECTING_ARM_LENGTH:
			return length_connecting_arm;
			break;
		case DEFAULT_Z_HEIGHT:
			return z_home;
			break;
		case ANGLE_OF_SERVO_ARM_1:
			return theta_servo_1;
			break;
		case ANGLE_OF_SERVO_ARM_2:
			return theta_servo_2;
			break;
		case ANGLE_OF_SERVO_ARM_3:
			return theta_servo_3;
			break;
		case ANGLE_OF_SERVO_ARM_4:
			return theta_servo_4;
			break;
		case ANGLE_OF_SERVO_ARM_5:
			return theta_servo_5;
			break;
		case ANGLE_OF_SERVO_ARM_6:
			return theta_servo_6;
			break;
	}
	return 0;
}