/*
 * File: rt_atan2_snf.c
 *
 * Real-Time Workshop code generated for Simulink model IMU_Quest.
 *
 * Model version                        : 1.50
 * Real-Time Workshop file version      : 7.4  (R2009b)  29-Jun-2009
 * Real-Time Workshop file generated on : Wed Nov 21 17:03:12 2012
 * TLC version                          : 7.4 (Jul 14 2009)
 * C/C++ source code generated on       : Wed Nov 21 17:03:12 2012
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "rt_atan2_snf.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <math.h>

/* Calls ATAN2, with guards against domain error and non-finites */
real_T rt_atan2_snf(real_T a, real_T b)
{
  real_T retValue;
  if (rtIsNaN(a) || rtIsNaN(b)) {
    retValue = (rtNaN);
  } else {
    if (rtIsInf(a) && rtIsInf(b)) {
      if (b > 0.0) {
        b = 1.0;
      } else {
        b = -1.0;
      }

      if (a > 0.0) {
        a = 1.0;
      } else {
        a = -1.0;
      }

      retValue = atan2(a,b);
    } else if (b == 0.0) {
      if (a > 0.0) {
        retValue = (RT_PI)/2.0;
      } else if (a < 0.0) {
        retValue = -(RT_PI)/2.0;
      } else {
        retValue = 0.0;
      }
    } else {
      retValue = atan2(a,b);
    }
  }

  return retValue;
}                                      /* end rt_atan2_snf */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
