#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "control.h"





typedef bool (*CMD_HANDLER_FUNC)(AXIS_ENUM, uint8_t*,uint32_t);

typedef struct{
	char* cmd;
	uint32_t data_num_byte;
	CMD_HANDLER_FUNC handler;
}CMD_HANDLER_STRUCT;

typedef enum{
	GSYNC=0,
	STOP,
	VEL,
	POS,
	GET,
	ESTOP,
	STAB,
	GSTAB,//MAX_AXIS_CMD,
	SERVO_RESET,
	MAX_CMD,
}CMD_ENUM;

void read_cmd(void);

#endif