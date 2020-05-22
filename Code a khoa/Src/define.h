#ifndef DEFINE_H
#define DEFINE_H

/* Control algorithm */
//#define USE_VEL_LOOP
//#define TEST_PID_VEL
//#define USE_STEPPING_MOTOR
//#define TEST_SINE_WAVE

/* PID calculate */
//#define PID_METHOD1
//#define PID_METHOD2

/* Feedback */
//#define FEEDBACK_ENCODER
//#define FEEDBACK_IMU

/* xsens imu */
//#define IMU_USE_XSENS_PRDID
//#define IMU_USE_XSENS_PSON
/* adis imu */
//#define IMU_USE_ADIS//ADIS or estimate

/* send data */
//#define SEND_CMD_FEEDBACK
//#define SEND_MAG
//#define SEND_ANGLE

//#define ENC1_USE_VEL
//#define ENC1_USE_POS
//#define ENC1_SWITCH

#define PI (float)3.141592653589793f
#define RAD2DEG (float)180.0f/PI
#define DEG2RAG (float)PI/180.0f
#define F_CTRL	1000
#define T_CTRL	(float)(1.0f/F_CTRL)
#define Debug_CPLD
/*
 * MAY IN DEFINE	
*/
//#warning nho chinh lai define, khong dao xung cpld
//#define NUM_Z_PULSE 15
//#define TRU_ENC_PULSE_PER_REV 2048
//#define TRONG_ENC_PULSE_PER_REV 81920
//#define TRONG_STEP_PER_REV 81920
//#define FWD_MM	(float)250
//#define FWD_TOTAL_PULSE (FWD_MM*(81920/590))//81920*250/590~34711
//#define WAIT_TIME_MS 500
//#define D_BAN_IN_PLUS_GAP_MM (float)200
//#define D_BAN_IN_PLUS_GAP_PULSE (D_BAN_IN_PLUS_GAP_MM*(81920/590))//81920*200/590~ 27769
//#define REV_NUM_PERIOD 100
//#define REV_STEP REV_TOTAL_PULSE
//#define K ((float)490/(590*15))
//#define OFFSET_TRONG_STEP 1

/*
 *	FOR TESTING
 */
//#define NUM_Z_PULSE 3
//#define TRU_ENC_PULSE_PER_REV 2048
//#define TRONG_ENC_PULSE_PER_REV 81920
//#define TRONG_STEP_PER_REV 81920
//#define FWD_MM	(float)250
//#define FWD_TOTAL_PULSE (FWD_MM*(81920/590))//81920*250/590~34711
//#define WAIT_TIME_MS 500
//#define D_BAN_IN_PLUS_GAP_MM (float)50
//#define D_BAN_IN_PLUS_GAP_PULSE (D_BAN_IN_PLUS_GAP_MM*(81920/590))//81920*200/590~ 27769
//#define REV_NUM_PERIOD 100
//#define REV_STEP REV_TOTAL_PULSE
//#define K ((float)490/(590*15))
//#define OFFSET_TRONG_STEP 1

#endif
