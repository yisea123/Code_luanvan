/*
 * File: IMU_Quest_data.c
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

#include "IMU_Quest.h"
#include "IMU_Quest_private.h"

/* Block parameters (auto storage) */
Parameters_IMU_Quest IMU_Quest_P = {
  /*  Expression: [1;0;0;0]
   * Referenced by: '<S1>/Unit Delay1'
   */
  { 1.0, 0.0, 0.0, 0.0 },
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<S1>/Unit Delay'
                                        */
  0.0095,                              /* Expression: 0.0095
                                        * Referenced by: '<Root>/delta'
                                        */
  0.001,                               /* Expression: 0.001
                                        * Referenced by: '<Root>/Gain'
                                        */
  2.0,                                 /* Expression: 2
                                        * Referenced by: '<S9>/Gain'
                                        */
  2.0,                                 /* Expression: 2
                                        * Referenced by: '<S7>/Gain'
                                        */
  2.0,                                 /* Expression: 2
                                        * Referenced by: '<S6>/Gain'
                                        */
  5.7295779513082323E+002,             /* Expression: 1800/pi
                                        * Referenced by: '<Root>/Gain2'
                                        */
  255.0,                               /* Expression: 255
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  200.0,                               /* Computed Parameter: PulseGenerator_Period
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  100.0,                               /* Computed Parameter: PulseGenerator_Duty
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  0.0                                  /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
};

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
