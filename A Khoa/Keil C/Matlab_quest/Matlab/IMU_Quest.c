/*
 * File: IMU_Quest.c
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

/* user code (top of source file) */
/* System '<Root>' */
#include "stm32f4xx.h"
#include "IMU_handler.h"
/* Exported block signals */
real_T fgyro[3];                       /* '<Root>/Gyro  Sensor' */
real_T facc[3];                        /* '<Root>/Acc  Sensor' */
real_T fmag[3];                        /* '<Root>/Mag  Sensor' */
real_T Out1[3];                        /* '<Root>/Gain2' */
real_T Out2;                           /* '<Root>/Pulse Generator' */
real_T  marg[15];
real_T  euler[3];

/* Block states (auto storage) */
D_Work_IMU_Quest IMU_Quest_DWork;

/* Real-time model */
RT_MODEL_IMU_Quest IMU_Quest_M_;
RT_MODEL_IMU_Quest *IMU_Quest_M = &IMU_Quest_M_;

/* Model step function */
void IMU_Quest_step(void)
{
  real_T eml_qk[4];
  real_T eml_theta;
  real_T eml_qs[4];
  real_T eml_B;
  real_T rtb_qk[4];
  real_T tmp[16];
  real_T tmp_0[16];
  int32_T tmp_1;
  real_T eml_mag_idx;
  real_T eml_mag_idx_0;
  real_T rtb_exacc_idx;
  real_T rtb_TmpSignalConversionAtSFun_0;
  real_T rtb_TmpSignalConversionAtSFun_1;
  real_T rtb_TmpSignalConversionAtSFun_2;

  {
    /* user code (Output function Header) */
    /* System '<Root>' */
    uint16_t i;

    /* user code (Output function Body) */
    /* System '<Root>' */
    read_adis();
    for (i=0; i<3; i++) {
      fgyro[i] = marg[i+1];
      facc[i] = marg[i+4];
      fmag[i] = marg[i+7];
    }

    /* SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport4' incorporates:
     *  Gain: '<Root>/Gain'
     *  Inport: '<Root>/Gyro  Sensor'
     */
    rtb_TmpSignalConversionAtSFun_0 = IMU_Quest_P.Gain_Gain * fgyro[0];
    rtb_TmpSignalConversionAtSFun_1 = IMU_Quest_P.Gain_Gain * fgyro[1];
    rtb_TmpSignalConversionAtSFun_2 = IMU_Quest_P.Gain_Gain * fgyro[2];

    /* Embedded MATLAB: '<S1>/QUEST filter' incorporates:
     *  Constant: '<Root>/delta'
     *  Inport: '<Root>/Acc  Sensor'
     *  Inport: '<Root>/Mag  Sensor'
     *  UnitDelay: '<S1>/Unit Delay'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    eml_qk[0] = IMU_Quest_DWork.UnitDelay1_DSTATE[0];
    eml_qk[1] = IMU_Quest_DWork.UnitDelay1_DSTATE[1];
    eml_qk[2] = IMU_Quest_DWork.UnitDelay1_DSTATE[2];
    eml_qk[3] = IMU_Quest_DWork.UnitDelay1_DSTATE[3];

    /* Embedded MATLAB Function 'QUEST Filter/QUEST filter': '<S3>:1' */
    /* '<S3>:1:4' */
    /* '<S3>:1:6' */
    /*  gyro measurement is considered as input  */
    /* '<S3>:1:7' */
    /*  measurement value from accelerometer sensor */
    /* '<S3>:1:8' */
    /*  measurement value from magnetic sensor */
    /* '<S3>:1:9' */
    eml_B = sqrt((rt_pow_snf(fmag[0], 2.0) + rt_pow_snf(fmag[1], 2.0)) +
                 rt_pow_snf(fmag[2], 2.0));
    eml_mag_idx = fmag[0] / eml_B;
    eml_mag_idx_0 = fmag[1] / eml_B;
    rtb_exacc_idx = fmag[2] / eml_B;

    /*  from accelerometer */
    /* '<S3>:1:15' */
    eml_B = rt_atan2_snf(facc[1], facc[2]);

    /* '<S3>:1:16' */
    eml_theta = asin((-facc[0]) / sqrt((rt_pow_snf(facc[0], 2.0) + rt_pow_snf
      (facc[1], 2.0)) + rt_pow_snf(facc[2], 2.0)));

    /*  from magnetic sensor */
    /* '<S3>:1:19' */
    /* '<S3>:1:20' */
    /* '<S3>:1:21' */
    /* '<S3>:1:25' */
    rtb_exacc_idx = rt_atan2_snf(-(eml_mag_idx_0 * cos(eml_B) - rtb_exacc_idx *
      sin(eml_B)), (eml_mag_idx_0 * sin(eml_theta) * sin(eml_B) + eml_mag_idx *
                    cos(eml_theta)) + rtb_exacc_idx * sin(eml_theta) * cos(eml_B));

    /* =========================================================================== */
    /* -- Convert euler angle into quaternion */
    /*  initial quaternion */
    /* '<S3>:1:70' */
    /* '<S3>:1:72' */
    /* '<S3>:1:74' */
    /* '<S3>:1:76' */
    /* '<S3>:1:78' */
    eml_qs[0] = cos(eml_B / 2.0) * cos(eml_theta / 2.0) * cos(rtb_exacc_idx /
      2.0) + sin(eml_B / 2.0) * sin(eml_theta / 2.0) * sin(rtb_exacc_idx / 2.0);
    eml_qs[1] = sin(eml_B / 2.0) * cos(eml_theta / 2.0) * cos(rtb_exacc_idx /
      2.0) - cos(eml_B / 2.0) * sin(eml_theta / 2.0) * sin(rtb_exacc_idx / 2.0);
    eml_qs[2] = cos(eml_B / 2.0) * sin(eml_theta / 2.0) * cos(rtb_exacc_idx /
      2.0) + sin(eml_B / 2.0) * cos(eml_theta / 2.0) * sin(rtb_exacc_idx / 2.0);
    eml_qs[3] = cos(eml_B / 2.0) * cos(eml_theta / 2.0) * sin(rtb_exacc_idx /
      2.0) - sin(eml_B / 2.0) * sin(eml_theta / 2.0) * cos(rtb_exacc_idx / 2.0);

    /* '<S3>:1:79' */
    eml_B = sqrt(((rt_pow_snf(eml_qs[0], 2.0) + rt_pow_snf(eml_qs[1], 2.0)) +
                  rt_pow_snf(eml_qs[2], 2.0)) + rt_pow_snf(eml_qs[3], 2.0));
    eml_qs[0] /= eml_B;
    eml_qs[1] /= eml_B;
    eml_qs[2] /= eml_B;
    eml_qs[3] /= eml_B;
    if (eml_qs[0] < 0.0) {
      /* '<S3>:1:26' */
      /* '<S3>:1:27' */
      eml_qs[0] = -eml_qs[0];
      eml_qs[1] = -eml_qs[1];
      eml_qs[2] = -eml_qs[2];
      eml_qs[3] = -eml_qs[3];
    }

    /*  initial value of qk  */
    if (IMU_Quest_DWork.UnitDelay_DSTATE == 1.0) {
      /* '<S3>:1:31' */
      /* '<S3>:1:32' */
      eml_qk[0] = eml_qs[0];
      eml_qk[1] = eml_qs[1];
      eml_qk[2] = eml_qs[2];
      eml_qk[3] = eml_qs[3];
    }

    /*  compute error value between gyros and (accs + mags) */
    /* '<S3>:1:36' */
    /*  compute estimated value of qk using first order approximation */
    /* '<S3>:1:39' */
    /* '<S3>:1:44' */
    tmp[0] = 0.0;
    tmp[4] = -rtb_TmpSignalConversionAtSFun_0;
    tmp[8] = -rtb_TmpSignalConversionAtSFun_1;
    tmp[12] = -rtb_TmpSignalConversionAtSFun_2;
    tmp[1] = rtb_TmpSignalConversionAtSFun_0;
    tmp[5] = 0.0;
    tmp[9] = rtb_TmpSignalConversionAtSFun_2;
    tmp[13] = -rtb_TmpSignalConversionAtSFun_1;
    tmp[2] = rtb_TmpSignalConversionAtSFun_1;
    tmp[6] = -rtb_TmpSignalConversionAtSFun_2;
    tmp[10] = 0.0;
    tmp[14] = rtb_TmpSignalConversionAtSFun_0;
    tmp[3] = rtb_TmpSignalConversionAtSFun_2;
    tmp[7] = rtb_TmpSignalConversionAtSFun_1;
    tmp[11] = -rtb_TmpSignalConversionAtSFun_0;
    tmp[15] = 0.0;
    for (tmp_1 = 0; tmp_1 < 4; tmp_1++) {
      tmp_0[tmp_1 << 2] = tmp[tmp_1 << 2] * 0.0005;
      tmp_0[1 + (tmp_1 << 2)] = tmp[(tmp_1 << 2) + 1] * 0.0005;
      tmp_0[2 + (tmp_1 << 2)] = tmp[(tmp_1 << 2) + 2] * 0.0005;
      tmp_0[3 + (tmp_1 << 2)] = tmp[(tmp_1 << 2) + 3] * 0.0005;
    }

    for (tmp_1 = 0; tmp_1 < 4; tmp_1++) {
      rtb_qk[tmp_1] = ((((tmp_0[tmp_1 + 4] * eml_qk[1] + tmp_0[tmp_1] * eml_qk[0])
                         + tmp_0[tmp_1 + 8] * eml_qk[2]) + tmp_0[tmp_1 + 12] *
                        eml_qk[3]) + eml_qk[tmp_1]) + (eml_qs[tmp_1] -
        eml_qk[tmp_1]) * IMU_Quest_P.delta_Value;
    }

    /*  normalization */
    /* '<S3>:1:47' */
    eml_B = sqrt(((rt_pow_snf(rtb_qk[0], 2.0) + rt_pow_snf(rtb_qk[1], 2.0)) +
                  rt_pow_snf(rtb_qk[2], 2.0)) + rt_pow_snf(rtb_qk[3], 2.0));
    rtb_qk[0] /= eml_B;
    rtb_qk[1] /= eml_B;
    rtb_qk[2] /= eml_B;
    rtb_qk[3] /= eml_B;
    if (rtb_qk[0] < 0.0) {
      /* '<S3>:1:49' */
      /* '<S3>:1:50' */
      rtb_qk[0] = -rtb_qk[0];
      rtb_qk[1] = -rtb_qk[1];
      rtb_qk[2] = -rtb_qk[2];
      rtb_qk[3] = -rtb_qk[3];
    }

    /*  compute external acceleration */
    /* exacc = R'*acc - [0; 0; 1000]; */
    /* '<S3>:1:62' */
    /* '<S3>:1:63' */

    /* Sum: '<S11>/Sum' incorporates:
     *  Product: '<S11>/Product'
     *  Product: '<S11>/Product1'
     *  Product: '<S11>/Product2'
     *  Product: '<S11>/Product3'
     */
    eml_B = ((rtb_qk[0] * rtb_qk[0] + rtb_qk[1] * rtb_qk[1]) + rtb_qk[2] *
             rtb_qk[2]) + rtb_qk[3] * rtb_qk[3];

    /* Math: '<S10>/Math Function' */
    /* Operator : signedsqrt */
    eml_B = eml_B < 0.0 ? -sqrt(fabs(eml_B)) : sqrt(eml_B);

    /* Product: '<S4>/Product' */
    eml_theta = rtb_qk[0] / eml_B;

    /* Product: '<S4>/Product1' */
    rtb_exacc_idx = rtb_qk[1] / eml_B;

    /* Product: '<S4>/Product2' */
    eml_mag_idx = rtb_qk[2] / eml_B;

    /* Product: '<S4>/Product3' */
    eml_B = rtb_qk[3] / eml_B;

    /* Gain: '<S7>/Gain' incorporates:
     *  Product: '<S7>/Product1'
     *  Product: '<S7>/Product2'
     *  Sum: '<S7>/Sum'
     */
    eml_mag_idx_0 = (eml_theta * eml_mag_idx - rtb_exacc_idx * eml_B) *
      IMU_Quest_P.Gain_Gain_o;

    /* Gain: '<Root>/Gain2' incorporates:
     *  Gain: '<S6>/Gain'
     *  Gain: '<S9>/Gain'
     *  Product: '<S5>/Product'
     *  Product: '<S5>/Product1'
     *  Product: '<S5>/Product2'
     *  Product: '<S5>/Product3'
     *  Product: '<S6>/Product2'
     *  Product: '<S6>/Product3'
     *  Product: '<S8>/Product'
     *  Product: '<S8>/Product1'
     *  Product: '<S8>/Product2'
     *  Product: '<S8>/Product3'
     *  Product: '<S9>/Product1'
     *  Product: '<S9>/Product2'
     *  Sum: '<S5>/Sum'
     *  Sum: '<S6>/Sum'
     *  Sum: '<S8>/Sum'
     *  Sum: '<S9>/Sum'
     *  Trigonometry: '<S2>/Trigonometric Function1'
     *  Trigonometry: '<S2>/Trigonometric Function2'
     *  Trigonometry: '<S2>/Trigonometric Function3'
     */
    Out1[0] = rt_atan2_snf((eml_theta * rtb_exacc_idx + eml_mag_idx * eml_B) *
      IMU_Quest_P.Gain_Gain_g, ((eml_theta * eml_theta - rtb_exacc_idx *
      rtb_exacc_idx) - eml_mag_idx * eml_mag_idx) + eml_B * eml_B) *
      IMU_Quest_P.Gain2_Gain;
    Out1[1] = IMU_Quest_P.Gain2_Gain * asin(rt_SATURATE(eml_mag_idx_0, -1.0, 1.0));
    Out1[2] = rt_atan2_snf((eml_B * eml_theta + rtb_exacc_idx * eml_mag_idx) *
      IMU_Quest_P.Gain_Gain_m, ((eml_theta * eml_theta + rtb_exacc_idx *
      rtb_exacc_idx) - eml_mag_idx * eml_mag_idx) - eml_B * eml_B) *
      IMU_Quest_P.Gain2_Gain;

    /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
    Out2 = ((real_T)IMU_Quest_DWork.clockTickCounter <
            IMU_Quest_P.PulseGenerator_Duty) &&
      (IMU_Quest_DWork.clockTickCounter >= 0) ? IMU_Quest_P.PulseGenerator_Amp :
      0.0;
    if ((real_T)IMU_Quest_DWork.clockTickCounter >=
        IMU_Quest_P.PulseGenerator_Period - 1.0) {
      IMU_Quest_DWork.clockTickCounter = 0;
    } else {
      IMU_Quest_DWork.clockTickCounter = IMU_Quest_DWork.clockTickCounter + 1;
    }

    /* user code (Output function Trailer) */
    /* System '<Root>' */
    for (i=0; i<3; i++)
      euler[i] = Out1[i];
    if (Out2 != 0.0)
      GPIO_SetBits(GPIOB, GPIO_Pin_1);
    else
      GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  }

  /* Update for UnitDelay: '<S1>/Unit Delay1' */
  IMU_Quest_DWork.UnitDelay1_DSTATE[0] = rtb_qk[0];
  IMU_Quest_DWork.UnitDelay1_DSTATE[1] = rtb_qk[1];
  IMU_Quest_DWork.UnitDelay1_DSTATE[2] = rtb_qk[2];
  IMU_Quest_DWork.UnitDelay1_DSTATE[3] = rtb_qk[3];

  /* Update for UnitDelay: '<S1>/Unit Delay' */
  IMU_Quest_DWork.UnitDelay_DSTATE = 0.0;
}

/* Model initialize function */
void IMU_Quest_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(IMU_Quest_M, (NULL));

  /* block I/O */

  /* exported global signals */
  Out1[0] = 0.0;
  Out1[1] = 0.0;
  Out1[2] = 0.0;
  Out2 = 0.0;

  /* states (dwork) */
  (void) memset((void *)&IMU_Quest_DWork, 0,
                sizeof(D_Work_IMU_Quest));

  /* external inputs */
  (void) memset(fgyro,0,
                3*sizeof(real_T));
  (void) memset(facc,0,
                3*sizeof(real_T));
  (void) memset(fmag,0,
                3*sizeof(real_T));

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator' */
  IMU_Quest_DWork.clockTickCounter = 0;

  /* InitializeConditions for UnitDelay: '<S1>/Unit Delay1' */
  IMU_Quest_DWork.UnitDelay1_DSTATE[0] = IMU_Quest_P.UnitDelay1_X0[0];
  IMU_Quest_DWork.UnitDelay1_DSTATE[1] = IMU_Quest_P.UnitDelay1_X0[1];
  IMU_Quest_DWork.UnitDelay1_DSTATE[2] = IMU_Quest_P.UnitDelay1_X0[2];
  IMU_Quest_DWork.UnitDelay1_DSTATE[3] = IMU_Quest_P.UnitDelay1_X0[3];

  /* InitializeConditions for UnitDelay: '<S1>/Unit Delay' */
  IMU_Quest_DWork.UnitDelay_DSTATE = IMU_Quest_P.UnitDelay_X0;
}

/* Model terminate function */
void IMU_Quest_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
