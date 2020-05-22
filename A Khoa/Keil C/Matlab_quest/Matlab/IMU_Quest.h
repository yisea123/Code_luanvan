/*
 * File: IMU_Quest.h
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

#ifndef RTW_HEADER_IMU_Quest_h_
#define RTW_HEADER_IMU_Quest_h_
#ifndef IMU_Quest_COMMON_INCLUDES_
# define IMU_Quest_COMMON_INCLUDES_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_SATURATE.h"
#include "rt_atan2_snf.h"
#include "rt_pow_snf.h"
#endif                                 /* IMU_Quest_COMMON_INCLUDES_ */

#include "IMU_Quest_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE[4];         /* '<S1>/Unit Delay1' */
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
  int32_T clockTickCounter;            /* '<Root>/Pulse Generator' */
} D_Work_IMU_Quest;

/* Parameters (auto storage) */
struct Parameters_IMU_Quest_ {
  real_T UnitDelay1_X0[4];             /* Expression: [1;0;0;0]
                                        * Referenced by: '<S1>/Unit Delay1'
                                        */
  real_T UnitDelay_X0;                 /* Expression: 1
                                        * Referenced by: '<S1>/Unit Delay'
                                        */
  real_T delta_Value;                  /* Expression: 0.0095
                                        * Referenced by: '<Root>/delta'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain_Gain_g;                  /* Expression: 2
                                        * Referenced by: '<S9>/Gain'
                                        */
  real_T Gain_Gain_o;                  /* Expression: 2
                                        * Referenced by: '<S7>/Gain'
                                        */
  real_T Gain_Gain_m;                  /* Expression: 2
                                        * Referenced by: '<S6>/Gain'
                                        */
  real_T Gain2_Gain;                   /* Expression: 1800/pi
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T PulseGenerator_Amp;           /* Expression: 255
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_Period;        /* Computed Parameter: PulseGenerator_Period
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_Duty;          /* Computed Parameter: PulseGenerator_Duty
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_PhaseDelay;    /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_IMU_Quest {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_IMU_Quest IMU_Quest_P;

/* Block states (auto storage) */
extern D_Work_IMU_Quest IMU_Quest_DWork;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  RTW declares the memory for these signals
 * and exports their symbols.
 *
 */
extern real_T fgyro[3];                /* '<Root>/Gyro  Sensor' */
extern real_T facc[3];                 /* '<Root>/Acc  Sensor' */
extern real_T fmag[3];                 /* '<Root>/Mag  Sensor' */
extern real_T Out1[3];                 /* '<Root>/Gain2' */
extern real_T Out2;                    /* '<Root>/Pulse Generator' */

/* Model entry point functions */
extern void IMU_Quest_initialize(void);
extern void IMU_Quest_step(void);
extern void IMU_Quest_terminate(void);

/* Real-time Model object */
extern RT_MODEL_IMU_Quest *IMU_Quest_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : IMU_Quest
 * '<S1>'   : IMU_Quest/QUEST Filter
 * '<S2>'   : IMU_Quest/Quaternions to Euler Angles
 * '<S3>'   : IMU_Quest/QUEST Filter/QUEST filter
 * '<S4>'   : IMU_Quest/Quaternions to Euler Angles/Quaternion Normalize
 * '<S5>'   : IMU_Quest/Quaternions to Euler Angles/Subsystem
 * '<S6>'   : IMU_Quest/Quaternions to Euler Angles/Subsystem1
 * '<S7>'   : IMU_Quest/Quaternions to Euler Angles/Subsystem2
 * '<S8>'   : IMU_Quest/Quaternions to Euler Angles/Subsystem3
 * '<S9>'   : IMU_Quest/Quaternions to Euler Angles/Subsystem4
 * '<S10>'  : IMU_Quest/Quaternions to Euler Angles/Quaternion Normalize/Quaternion Modulus
 * '<S11>'  : IMU_Quest/Quaternions to Euler Angles/Quaternion Normalize/Quaternion Modulus/Quaternion Norm
 */
#endif                                 /* RTW_HEADER_IMU_Quest_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
