#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#define   MAX_CLK 20000000 //20MHz
#define		BUFF_RX			21
#define		BUFF_SIZE			1 //73


// So xung tren vong khi xuat xung
extern const float pulse_row_base;

// So xung tren vong cua encoder tung dong co
#define ENC_PULSE_PER_REV_m3 350.0f
#define ENC_PULSE_PER_REV_m2 350.0f
#define ENC_PULSE_PER_REV_m1 108879.0f

// Cac gia tri lien quan den chu ki va thoi gian xuat xung
#define cycle 100 // So chu ki xuat xung
#define interval 0.01 // Chu ki dieu khien = 1ms
#define pulse_interval 10 // Chu ki nap xung moi = 10*0.1ms = 1ms
#define home_speed 2.5f

// tf level
#define high 1.0f
#define med 2.5f
#define low 4.0f


#define DRIVER_DEGREE_TO_PULSE (float)(ENC_PULSE_PER_REV_m1 / 360.0f)
#define DRIVER_PULSE_TO_DEGREE (float) (360.0f / ENC_PULSE_PER_REV_m1)
	
//#define PI (float)3.141592653589793f
#define RAD2DEG (float)180.0f/PI
//#define DEG2RAG (float)PI/180.0f
#define F_CTRL	1000
#define T_CTRL	(float)(1.0f/F_CTRL)
//#define Debug_CPLD





// Chieu dai 2 khop cua canh tay
#define   L1 21.0f
#define   L2 20.5f




// Define cac gia tri lien quan den encoder
#define ENC_RST_PORT 									GPIOD
#define ENC_RST_PORT_CLK							RCC_AHB1Periph_GPIOD
#define ENC_RST_PIN        						GPIO_Pin_11


typedef enum
{
	SERVO1,
	SERVO2,
	SERVO3,
	SERVO4,
	MAX_SERVO
}
SERVO_ENUM;


extern int ui_delay;

// Buffer DMA
extern uint8_t 	txbuff[BUFF_SIZE];
extern uint8_t 	rxbuff[BUFF_RX];

// Mang luu vi tri hien tai va truoc do cua canh tay
extern float32_t last_theta[MAX_SERVO], new_theta[MAX_SERVO];


// Mang luu cac he so a0..a5 cua 4 khop
extern float32_t joint_1[6], joint_2[6], joint_3[6],joint_4[6];

// Mang luu gia tri xung/vong cua tung khop 
extern uint8_t pulse[MAX_SERVO],dir[MAX_SERVO];
extern float add[MAX_SERVO];
extern int fix_pulse[MAX_SERVO];

// Doc encoder
extern float enc_pos[MAX_SERVO];

extern int counter[MAX_SERVO];
extern int quang;
extern float error_arr[50];
extern float array[20];

extern float timer_arr[20];
extern float pos_timer;

extern float enc_angle_cur[MAX_SERVO];
extern float enc_dp[MAX_SERVO];
extern int32_t enc_p0[MAX_SERVO];
extern int32_t enc_p1[MAX_SERVO];
extern float servo_pulse_offset[MAX_SERVO];



/*===================FSMC_PWM=================*/


void FSMC_Write(uint32_t ui_address, uint32_t ui_data);
uint16_t FSMC_Read(uint32_t ui_enc_channel);

void FSMC_Init(void);


// Dua xung vao buffer
void set_servo_pulse(uint8_t pulse, uint8_t dir,int i);

// Kich phat xung 
void pulse_write(void);

//FSMC_Encoder
void FSMC_ENC_Update(void);
float FSMC_ENC_Get_Pos(SERVO_ENUM e_servo);
float FSMC_ENC_Get_Pos_m3(int dir);
float FSMC_ENC_Get_Vel(SERVO_ENUM e_servo);
void FSMC_ENC_Reset(void);
void FSMC_ENC_Reset_Counter(void);
int32_t FSMC_ENC_Get_Counter(SERVO_ENUM e_servo);


//Uart_DMA
void init_UART_DMA(void);

//External Interrupt
void EXTILine2_Init(void);

// Delay 0.1ms
void delay_01ms(uint16_t period);

// PWM
void PWM_Init(void);
/* ===================== Calculating Function ============================= */

// Calculate a0..a5 factor
void factor_calc(float32_t theta1_0,float32_t theta1_f,float32_t *C_f32, float time);

// Inverse Kinematic
void inverse_kinematic(float xf, float yf,float phi, float time);	

// Pulse Calculate

void pulse_calc(SERVO_ENUM servo,float32_t *A,float t); // --> tinh gia tri xung cho 1 truc

void pulse_total(uint16_t k); // --> tinh gia tri xung cho tat ca cac truc

int fix_pulse_calc(float error, float pulse_row,int dir);

void read_fix_pulse(SERVO_ENUM servo,int k,int cover_step);

/* ====================================================================== */


/* ===================== Positioning Function =========================== */

// Home position

void home_position(void);
void servo_move_home(void);
void reset_fix_pulse(void);

// Test position 
void test_motor(float tf);
//void test_motor(SERVO_ENUM servo,float tf, float theta_0,float theta_f);

// Get position to new point
void get_position(float xf, float yf,float phi, float tf);

// Write pulse periodically
void write_pulse_4motor(float tf,uint8_t delay_time,int cover_step);

// Reset number of pulse and dir of 4 motor
void reset_puldir(void);

