/*
 * File: IMU_Idcm.h
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

#ifndef RTW_HEADER_IMU_Idcm_h_
#define RTW_HEADER_IMU_Idcm_h_
#ifndef IMU_Idcm_COMMON_INCLUDES_
# define IMU_Idcm_COMMON_INCLUDES_
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_atan2_snf.h"
#include "rt_pow_snf.h"
#endif                                 /* IMU_Idcm_COMMON_INCLUDES_ */

#include "IMU_Idcm_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE[3];          /* '<S3>/Unit Delay' */
  real_T UnitDelay1_DSTATE[9];         /* '<S3>/Unit Delay1' */
  real_T UnitDelay_DSTATE_h[3];        /* '<S2>/Unit Delay' */
  real_T UnitDelay1_DSTATE_p[9];       /* '<S2>/Unit Delay1' */
  real_T UnitDelay3_DSTATE;            /* '<S2>/Unit Delay3' */
  real_T UnitDelay2_DSTATE;            /* '<S3>/Unit Delay2' */
  int32_T clockTickCounter;            /* '<Root>/Pulse Generator' */
} D_Work_IMU_Idcm;

/* Parameters (auto storage) */
struct Parameters_IMU_Idcm_ {
  real_T UnitDelay_X0[3];              /* Expression: [0;0;0]
                                        * Referenced by: '<S3>/Unit Delay'
                                        */
  real_T UnitDelay1_X0[9];             /* Expression: 1e-2*eye(3)
                                        * Referenced by: '<S3>/Unit Delay1'
                                        */
  real_T Q2_Value[9];                  /* Expression: Q2
                                        * Referenced by: '<S1>/Q2'
                                        */
  real_T Rmag_Value[9];                /* Expression: Rmag
                                        * Referenced by: '<S1>/Rmag'
                                        */
  real_T mag_Rbar_Value;               /* Expression: mag_Rbar
                                        * Referenced by: '<S1>/mag_Rbar'
                                        */
  real_T mag_sigma_Value;              /* Expression: mag_sigma
                                        * Referenced by: '<S1>/mag_sigma'
                                        */
  real_T UnitDelay_X0_i[3];            /* Expression: [0;0;0]
                                        * Referenced by: '<S2>/Unit Delay'
                                        */
  real_T UnitDelay1_X0_n[9];           /* Expression: 1e-2*eye(3)
                                        * Referenced by: '<S2>/Unit Delay1'
                                        */
  real_T Q1_Value[9];                  /* Expression: Q1
                                        * Referenced by: '<S1>/Q1'
                                        */
  real_T Racc_Value[9];                /* Expression: Racc
                                        * Referenced by: '<S1>/Racc'
                                        */
  real_T acc_Rbar_Value;               /* Expression: acc_Rbar
                                        * Referenced by: '<S1>/acc_Rbar'
                                        */
  real_T acc_sigma_Value;              /* Expression: acc_sigma
                                        * Referenced by: '<S1>/acc_sigma'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.001
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0.00981
                                        * Referenced by: '<Root>/Gain1'
                                        */
  real_T UnitDelay3_X0;                /* Expression: 1
                                        * Referenced by: '<S2>/Unit Delay3'
                                        */
  real_T UnitDelay2_X0;                /* Expression: 1
                                        * Referenced by: '<S3>/Unit Delay2'
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
struct RT_MODEL_IMU_Idcm {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_IMU_Idcm IMU_Idcm_P;

/* Block states (auto storage) */
extern D_Work_IMU_Idcm IMU_Idcm_DWork;

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
extern void IMU_Idcm_initialize(void);
extern void IMU_Idcm_step(void);
extern void IMU_Idcm_terminate(void);

/* Real-time Model object */
extern RT_MODEL_IMU_Idcm *IMU_Idcm_M;

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
 * '<Root>' : IMU_Idcm
 * '<S1>'   : IMU_Idcm/DCM Kalman Filter
 * '<S2>'   : IMU_Idcm/DCM Kalman Filter/Kalman Filter 1
 * '<S3>'   : IMU_Idcm/DCM Kalman Filter/Kalman Filter 2
 * '<S4>'   : IMU_Idcm/DCM Kalman Filter/Kalman Filter 1/Kalman Filter 1
 * '<S5>'   : IMU_Idcm/DCM Kalman Filter/Kalman Filter 2/Kalman Filter 2
 */
#endif                                 /* RTW_HEADER_IMU_Idcm_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
