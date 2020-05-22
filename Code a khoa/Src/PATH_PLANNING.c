#include <Math.h>
#include "PATH_PLANNING.h"
#include <stdint.h>
#include <stdbool.h>
#include "control.h"
#include "define.h"

bool b_first_cycle = true, b_first_cycle_one_axis_trajectory = true, b_first_cycle_two_axis_trajectory = true;
float f_dimension_first, f_dimension_second, f_half_perimeter; // dimension of rectangle
int i_edge = 0; // the edge is running.
uint32_t ui32_t_offset; // time to scale run mode
float f_time_run_first, f_time_run_second; // time to run every axis
float f_travel_one_axis;
float f_travel_two_axis_first, f_travel_two_axis_second;

extern PLATFORM_COORDINATOR_DATA_STRUCT st_pl_coords_data[MAX_AXIS];

void circle_trajectory(float radius, float dir, float f_hz, uint32_t f_t_s,CIRCLE_MODE_ENUM c_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data)
{
	switch((int)c_mode)
	{
		case CIRCLE_MODE_X_Y:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis = (float)(radius*cosf((float)(2*PI*f_t_s*dir)/f_hz)); // 	formula of circle
			st_pl_coords_data[Y_AXIS].f_next_axis = (float)(radius*sinf((float)(2*PI*f_t_s*dir)/f_hz));
			st_pl_coords_data[Z_AXIS].f_next_axis = 0;
			break;
		}
		case CIRCLE_MODE_Y_Z:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis =  0;// 	formula of circle
			st_pl_coords_data[Y_AXIS].f_next_axis = (float)(radius*cosf((float)(2*PI*f_t_s*dir)/f_hz));
			st_pl_coords_data[Z_AXIS].f_next_axis = (float)(radius*sinf((float)(2*PI*f_t_s*dir)/f_hz));
			break;
		}
		case CIRCLE_MODE_X_Z:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis = (float)(radius*cosf((float)(2*PI*f_t_s*dir)/f_hz)); // 	formula of circle
			st_pl_coords_data[Y_AXIS].f_next_axis = 0;
			st_pl_coords_data[Z_AXIS].f_next_axis = (float)(radius*sinf((float)(2*PI*f_t_s*dir)/f_hz));
			break;
		}
		default:
		break;
	}
}

void rectangle_trajectory(float first_begin, float second_begin, float first_end, float second_end, bool dir, float f_hz, uint32_t ui32_t_s, SQUARE_MODE_ENUM s_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data)
{
	switch((int)(s_mode))
	{
		case SQUARE_MODE_X_Y:
		{
			if (b_first_cycle)
			{
				f_dimension_first = first_end - first_begin; // calculate dimension of rectangle
				f_dimension_second = second_end - second_begin;
				f_half_perimeter = f_dimension_first + f_dimension_second;
				f_time_run_first = f_hz/2*f_dimension_first/f_half_perimeter;
				f_time_run_second = f_hz/2*f_dimension_second/f_half_perimeter;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				b_first_cycle = false;
				return;
			}
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			if (dir) //run along the X asix -> Y asix -> X asix -> Y asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_begin ;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 0;
					}
				}
			}
			else // run along the Y asix -> X asix -> Y asix -> X asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_end - f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Y_AXIS].f_next_axis = second_begin;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = 0;
						i_edge = 0;
					}
				}
			}
			st_pl_coords_data[Z_AXIS].f_next_axis = 0;
			break;
		}
		case SQUARE_MODE_Y_Z:
		{
			if (b_first_cycle)
			{
				f_dimension_first = first_end - first_begin; // calculate dimension of rectangle
				f_dimension_second = second_end - second_begin;
				f_half_perimeter = f_dimension_first + f_dimension_second;
				f_time_run_first = f_hz/2*f_dimension_first/f_half_perimeter;
				f_time_run_second = f_hz/2*f_dimension_second/f_half_perimeter;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				b_first_cycle = false;
				return;
			}
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			if (dir) //run along the X asix -> Y asix -> X asix -> Y asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin ;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 0;
					}
				}
			}
			else // run along the Y asix -> X asix -> Y asix -> X asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end - f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[Y_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin;
					if (st_pl_coords_data[Y_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 0;
					}
				}
			}
			st_pl_coords_data[X_AXIS].f_next_axis = 0;
			break;
		}
		case SQUARE_MODE_X_Z:
		{
			if (b_first_cycle)
			{
				f_dimension_first = first_end - first_begin; // calculate dimension of rectangle
				f_dimension_second = second_end - second_begin;
				f_half_perimeter = f_dimension_first + f_dimension_second;
				f_time_run_first = f_hz/2*f_dimension_first/f_half_perimeter;
				f_time_run_second = f_hz/2*f_dimension_second/f_half_perimeter;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				b_first_cycle = false;
				return;
			}
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			if (dir) //run along the X asix -> Y asix -> X asix -> Y asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin ;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 0;
					}
				}
			}
			else // run along the Y asix -> X asix -> Y asix -> X asix
			{
				if (i_edge==0)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin + f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 1;
					}
				}
				else if (i_edge==1)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_begin + f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end;
					if (st_pl_coords_data[X_AXIS].f_next_axis == first_end)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 2;
					}
				}
				else if (i_edge==2)
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_end - f_dimension_second*(ui32_t_s-ui32_t_offset)/f_time_run_second;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == second_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 3;
					}
				}
				else
				{
					st_pl_coords_data[X_AXIS].f_next_axis = first_end - f_dimension_first*(ui32_t_s-ui32_t_offset)/f_time_run_first;
					st_pl_coords_data[Z_AXIS].f_next_axis = second_begin;
					if (st_pl_coords_data[Z_AXIS].f_next_axis == first_begin)
					{
						ui32_t_offset = ui32_t_s;
						i_edge = 0;
					}
				}
			}
			st_pl_coords_data[Y_AXIS].f_next_axis = 0;
			break;
		}
		default:
			break;
	}
	
}
void ellipse_trajectory(float first_radius, float second_radius, float dir, float f_hz, uint32_t ui32_t_s,ECLIPSE_MODE_ENUM e_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data)
{
	switch((int)e_mode)
	{
		case ECLIPSE_MODE_X_Y:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis = (float)(first_radius*cosf((float)(2*PI*ui32_t_s*dir/f_hz))); // 	formula of ellipse
			st_pl_coords_data[Y_AXIS].f_next_axis = (float)(second_radius*sinf((float)(2*PI*ui32_t_s*dir/f_hz)));
			st_pl_coords_data[Z_AXIS].f_next_axis = 0;
			break;
		}
		case ECLIPSE_MODE_Y_Z:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis = 0;
			st_pl_coords_data[Y_AXIS].f_next_axis = (float)(first_radius*cosf((float)(2*PI*ui32_t_s*dir/f_hz))); // 	formula of ellipse
			st_pl_coords_data[Z_AXIS].f_next_axis = (float)(second_radius*sinf((float)(2*PI*ui32_t_s*dir/f_hz)));
			break;
		}
		case ECLIPSE_MODE_X_Z:
		{
			st_pl_coords_data[X_AXIS].f_current_axis = st_pl_coords_data[X_AXIS].f_next_axis;
			st_pl_coords_data[Y_AXIS].f_current_axis = st_pl_coords_data[Y_AXIS].f_next_axis;
			st_pl_coords_data[Z_AXIS].f_current_axis = st_pl_coords_data[Z_AXIS].f_next_axis;
			st_pl_coords_data[X_AXIS].f_next_axis = (float)(first_radius*cosf((float)(2*PI*ui32_t_s*dir/f_hz))); // 	formula of ellipse
			st_pl_coords_data[Y_AXIS].f_next_axis = 0;
			st_pl_coords_data[Z_AXIS].f_next_axis = (float)(second_radius*sinf((float)(2*PI*ui32_t_s*dir/f_hz)));
			break;
		}
		default:
			break;
	}
	
}
void one_axis_trajectory(float f_axis_test_begin, float f_axis_test_end, bool dir, float f_hz, uint32_t ui32_t_s,AXIS_ENUM a_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data )
{
	switch ((int) a_mode)
	{
		case X_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		case Y_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		case Z_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		case ROLL_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		case PITCH_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		case YAW_AXIS:
		{
			if(b_first_cycle_one_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				f_travel_one_axis = f_axis_test_end - f_axis_test_begin;
				b_first_cycle_one_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_test_begin + f_travel_one_axis*ui32_t_offset*f_hz;
			}
			else
			{
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_test_begin - f_travel_one_axis*ui32_t_offset*f_hz;
			}
			break;
		}
		default:
			break;
	}
}
void two_axis_trajectory(float f_axis_first_test_begin, float f_axis_first_test_end, float f_axis_second_test_begin, float f_axis_second_test_end, bool dir, float f_hz, uint32_t ui32_t_s,TEST_MODE_AXIS_ENUM ta_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data )
{
	switch ((int) ta_mode)
	{
		case X_Y_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		case X_Z_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[X_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		case Y_Z_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[Y_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[Z_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		case ROLL_PITCH_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_current_axis = 0;
				st_pl_coords_data[YAW_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		case ROLL_YAW_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_current_axis = 0;
				st_pl_coords_data[PITCH_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[ROLL_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		case PITCH_YAW_AXIS:
		{
			if (b_first_cycle_two_axis_trajectory)
			{
				st_pl_coords_data[X_AXIS].f_current_axis = 0;
				st_pl_coords_data[X_AXIS].f_next_axis = 0;
				st_pl_coords_data[Y_AXIS].f_current_axis = 0;
				st_pl_coords_data[Y_AXIS].f_next_axis = 0;
				st_pl_coords_data[Z_AXIS].f_current_axis = 0;
				st_pl_coords_data[Z_AXIS].f_next_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_current_axis = 0;
				st_pl_coords_data[ROLL_AXIS].f_next_axis = 0;
				f_travel_two_axis_first = f_axis_first_test_end - f_axis_first_test_begin;
				f_travel_two_axis_second = f_axis_second_test_end - f_axis_second_test_begin;
				b_first_cycle_two_axis_trajectory = false;
			}
			if(dir == true)
			{
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_first_test_begin + f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_second_test_begin + f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			else
			{
				st_pl_coords_data[PITCH_AXIS].f_next_axis = f_axis_first_test_begin - f_travel_two_axis_first*ui32_t_offset*f_hz;
				st_pl_coords_data[YAW_AXIS].f_next_axis = f_axis_second_test_begin - f_travel_two_axis_second/f_travel_two_axis_first*ui32_t_s*f_hz;
			}
			break;
		}
		default:
			break;
	}
}