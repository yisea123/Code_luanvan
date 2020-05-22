#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include "control.h"

void circle_trajectory(float radius, float dir, float f_hz, uint32_t f_t_s,CIRCLE_MODE_ENUM c_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data);
void rectangle_trajectory(float x_begin, float y_begin, float x_end, float y_end, bool dir, float f_hz, uint32_t ui32_t_s, SQUARE_MODE_ENUM s_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data);
void ellipse_trajectory(float first_radius, float second_radius, float dir, float f_hz, uint32_t ui32_t_s,ECLIPSE_MODE_ENUM e_clipse, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data);
void one_axis_trajectory(float f_axis_test_begin, float f_axis_test_end, bool dir, float f_hz, uint32_t ui32_t_s,AXIS_ENUM a_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data );
void two_axis_trajectory(float f_axis_first_test_begin, float f_axis_first_test_end, float f_axis_second_test_begin, float f_axis_second_test_end, bool dir, float f_hz, uint32_t ui32_t_s,TEST_MODE_AXIS_ENUM ta_mode, PLATFORM_COORDINATOR_DATA_STRUCT *st_pl_coords_data );
#endif