#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "define.h"
#include "cmd_handler.h"
#include "driver.h"
#include "system_timetick.h"
#include "control.h"

#define CMD_BUFF_LEN 512
#define CMD_MSG_LEN 100
#define DATA_NUM_BYTE 20

uint8_t cmd_buff[CMD_BUFF_LEN];
static uint8_t cmd_msg[CMD_MSG_LEN];
static uint32_t cmd_msg_cnt = 0;
static uint8_t data[DATA_NUM_BYTE];
static uint32_t data_cnt=0;
static uint32_t remain_byte=0;
CMD_ENUM current_cmd;
AXIS_ENUM current_axis;

extern uint8_t cmd_buff[CMD_BUFF_LEN];

//static bool gsync_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool stop_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool vel_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
static bool pos_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool get_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool estop_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool stab_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool gstab_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);
//static bool servo_reset_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt);

CMD_HANDLER_STRUCT cmd_handler[MAX_CMD] =
{
//	{"gsyn",0,gsync_handler},
//	{"stop",0,stop_handler},
//	{"vel",5,vel_handler},
	{"pos",9,pos_handler},
//	{"get",0,get_handler},
//	{"estop",0,estop_handler},
//	{"stab",5,stab_handler},
//	{"gstab",0,gstab_handler},
//	{"Servoreset",0,servo_reset_handler}	
};

static void reset_cmd_msg(void)
{
	memset(cmd_msg,0,cmd_msg_cnt);
	cmd_msg_cnt = 0;
}

//void read_cmd(void)
//{
//	static uint32_t cmd_time_tick=0;
//	static uint32_t cmd_read_ind=0, cmd_wr_ind=0;
//	static uint32_t cmd_wr_ind_1 = 0;
//	//static uint32_t start_ind = 0, end_ind = 0;
//	static uint8_t state = 0;
//	static uint32_t pre_msg_cnt=0;
//	uint32_t i, j, len;

//	/* get buffer index of new data */
//	cmd_wr_ind = CMD_BUFF_LEN- DIS_RX_DMA_STREAM->NDTR;	
//	if (cmd_wr_ind_1 != cmd_wr_ind)
//	{
//		cmd_wr_ind_1 = cmd_wr_ind;
//		cmd_time_tick = SysTick_GetTick();		
//	}
//	else if (SysTick_IsTimeout(cmd_time_tick, 500))
//  {
//		/* reset */
//		cmd_wr_ind_1 = cmd_wr_ind;	
//		state = 0;
//		reset_cmd_msg();
//  }
//	if (cmd_read_ind <= cmd_wr_ind) len = cmd_wr_ind - cmd_read_ind;
//	else len = CMD_BUFF_LEN - (cmd_read_ind- cmd_wr_ind);
//	
//	/* scan through new data */
//	pre_msg_cnt = cmd_msg_cnt;
//	for (i = 0; i < len; i++)
//	{
//		j = (cmd_read_ind + i) % CMD_BUFF_LEN;
//		cmd_msg[cmd_msg_cnt++] = cmd_buff[j];
//		if (cmd_msg_cnt >= CMD_MSG_LEN)
//		{
//			reset_cmd_msg();
//			continue;
//		}
//		switch (state)
//		{
//			case 0:/* wait for cmd header */
//			{
//				uint32_t idx = 0;
//				for (idx=0;idx<MAX_CMD;idx++)
//				{
//					char *pstr;
//					if (idx < SERVO_RESET)
//					{
//						if (strncmp((char*)cmd_msg+2,cmd_handler[idx].cmd,strlen(cmd_handler[idx].cmd)) == 0)
//						{
//							pstr = (char*)cmd_msg;
//							current_axis = INVALID_AXIS;
//							switch (*pstr)
//							{
//								case 'a':
//									current_axis = A_AXIS;
//									break;
//								case 'e':
//									current_axis = E_AXIS;
//									break;
//								case 'p':
//									current_axis = P_AXIS;
//									break;
//							}
//							if (current_axis == INVALID_AXIS)
//							{
//								reset_cmd_msg();
//								break;
//							}
//							pstr += 2;
//							state = 1;
//						}
//					}
//					else if (strncmp((char*)cmd_msg,cmd_handler[idx].cmd,strlen(cmd_handler[idx].cmd)) == 0)
//					{
//						pstr = (char*)cmd_msg;	
//						state = 1;
//					}
//					if (state)
//					{
//						current_cmd = idx;
//						remain_byte = cmd_handler[idx].data_num_byte;
//						data_cnt = 0;					
//						if (!remain_byte)
//						{
//							cmd_handler[current_cmd].handler(current_axis,data,data_cnt);
//							state = 0;
//							reset_cmd_msg();
//						}
//						break;
//					}					
//				}			
//				break;
//			}
//			case 1:/* wait for data bytes*/			
//			{
//				uint32_t tmp= pre_msg_cnt;
//				while ((tmp < cmd_msg_cnt) && remain_byte)
//				{
//					data[data_cnt++] = cmd_msg[tmp++];				
//					remain_byte--;
//				} 
//				if (!remain_byte)
//				{
//					cmd_handler[current_cmd].handler(current_axis,data,data_cnt);
//					state = 0;
//					reset_cmd_msg();
//				}
//				break;
//			}
//		}
//	}
//	cmd_read_ind= cmd_wr_ind;		
//}

//static bool gsync_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	axis_set_state(axis,STATE_HOME);
//	return true;
//}

//static bool stop_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	axis_set_state(axis,STATE_STOP);
//	return true;
//}

//static bool vel_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	int32_t velocity;
//	velocity = (int32_t)data[1]|(data[2]<<8)|(data[3]<<16)|(data[4]<<24);
//	axis_set_state(axis,STATE_VEL_CTRL);
//	axis_set_vel(axis, velocity);
//	return true;
//}

static bool pos_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
{
//	int32_t temp;
//	axis_set_state(axis,STATE_POS_VEL_CTRL);
//	temp = (int32_t)data[1]|(data[2]<<8)|(data[3]<<16)|(data[4]<<24);
//	axis_set_pos(axis, temp);
//	temp = (int32_t)data[5]|(data[6]<<8)|(data[7]<<16)|(data[8]<<24);
//	axis_set_vel(axis, temp);	
//	return true;
}

//static bool get_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	return true;
//}

//static bool estop_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	axis_set_state(axis,STATE_STOP);
//	return true;
//}

//static bool stab_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	//axis_set_state(axis,STATE_STAB);
//	return true;
//}

//static bool gstab_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	return true;
//}

//static bool servo_reset_handler(AXIS_ENUM axis, uint8_t* data,uint32_t data_cnt)
//{
//	return true;
//}
