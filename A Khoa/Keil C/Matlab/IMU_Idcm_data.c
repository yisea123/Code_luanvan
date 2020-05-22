/*
 * File: IMU_Idcm_data.c
 *
 * Real-Time Workshop code generated for Simulink model IMU_Idcm.
 *
 * Model version                        : 1.58
 * Real-Time Workshop file version      : 7.4  (R2009b)  29-Jun-2009
 * Real-Time Workshop file generated on : Tue Dec 04 05:44:18 2012
 * TLC version                          : 7.4 (Jul 14 2009)
 * C/C++ source code generated on       : Tue Dec 04 05:44:19 2012
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM 7
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "IMU_Idcm.h"
#include "IMU_Idcm_private.h"

/* Block parameters (auto storage) */
Parameters_IMU_Idcm IMU_Idcm_P = {
  /*  Expression: [0;0;0]
   * Referenced by: '<S3>/Unit Delay'
   */
  { 0.0, 0.0, 0.0 },

  /*  Expression: 1e-2*eye(3)
   * Referenced by: '<S3>/Unit Delay1'
   */
  { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 },

  /*  Expression: Q2
   * Referenced by: '<S1>/Q2'
   */
  { 8.0E-012, 0.0, 0.0, 0.0, 8.0E-012, 0.0, 0.0, 0.0, 8.0E-012 },

  /*  Expression: Rmag
   * Referenced by: '<S1>/Rmag'
   */
  { 2.0E-006, 0.0, 0.0, 0.0, 2.0E-006, 0.0, 0.0, 0.0, 2.0E-006 },
  0.2,                                 /* Expression: mag_Rbar
                                        * Referenced by: '<S1>/mag_Rbar'
                                        */
  1000.0,                              /* Expression: mag_sigma
                                        * Referenced by: '<S1>/mag_sigma'
                                        */

  /*  Expression: [0;0;0]
   * Referenced by: '<S2>/Unit Delay'
   */
  { 0.0, 0.0, 0.0 },

  /*  Expression: 1e-2*eye(3)
   * Referenced by: '<S2>/Unit Delay1'
   */
  { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 },

  /*  Expression: Q1
   * Referenced by: '<S1>/Q1'
   */
  { 8.0E-011, 0.0, 0.0, 0.0, 8.0E-011, 0.0, 0.0, 0.0, 8.0E-011 },

  /*  Expression: Racc
   * Referenced by: '<S1>/Racc'
   */
  { 0.00004, 0.0, 0.0, 0.0, 0.00004, 0.0, 0.0, 0.0, 0.00004 },
  0.1,                                 /* Expression: acc_Rbar
                                        * Referenced by: '<S1>/acc_Rbar'
                                        */
  5.0,                                 /* Expression: acc_sigma
                                        * Referenced by: '<S1>/acc_sigma'
                                        */
  0.001,                               /* Expression: 0.001
                                        * Referenced by: '<Root>/Gain'
                                        */
  0.00981,                             /* Expression: 0.00981
                                        * Referenced by: '<Root>/Gain1'
                                        */
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<S2>/Unit Delay3'
                                        */
  1.0,                                 /* Expression: 1
                                        * Referenced by: '<S3>/Unit Delay2'
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
