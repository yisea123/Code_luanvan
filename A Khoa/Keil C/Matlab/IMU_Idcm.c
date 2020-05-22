/*
 * File: IMU_Idcm.c
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

/* user code (top of source file) */
/* System '<Root>' */
#include "stm32f4xx.h"
#include "driver.h"

/* Exported block signals */
real_T fgyro[3];                       /* '<Root>/Gyro  Sensor' */
real_T facc[3];                        /* '<Root>/Acc  Sensor' */
real_T fmag[3];                        /* '<Root>/Mag  Sensor' */
real_T Out1[3];                        /* '<Root>/Gain2' */
real_T Out2;                           /* '<Root>/Pulse Generator' */

/* Block states (auto storage) */
D_Work_IMU_Idcm IMU_Idcm_DWork;

/* Real-time model */
RT_MODEL_IMU_Idcm IMU_Idcm_M_;
RT_MODEL_IMU_Idcm *IMU_Idcm_M = &IMU_Idcm_M_;

/* Model step function */
void IMU_Idcm_step(void)
{
  real_T eml_mobs[3];
  real_T eml_Qd[9];
  real_T eml_R[9];
  int8_T eml_I[9];
  real_T eml_x[9];
  real_T eml_b_x[9];
  int32_T eml_p;
  int32_T eml_p_0;
  int32_T eml_p_1;
  real_T eml_absx;
  real_T eml_absx_0;
  real_T eml_absx_1;
  int32_T eml_itmp;
  real_T eml_mobs_0[6];
  real_T eml_Qd_0[9];
  real_T rtb_TmpSignalConversionAtSFun_f[9];
  real_T rtb_xk[3];
  real_T rtb_outP_[9];
  real_T rtb_outP__h[9];
  real_T eml_mobs_1[3];
  real_T tmp[9];
  real_T eml_b_x_0[9];
  real_T rtb_Gain_idx;
  real_T rtb_Gain_idx_0;
  real_T rtb_Gain_idx_1;
  real_T rtb_Gain1_idx;
  real_T rtb_Gain1_idx_0;
  real_T rtb_Gain1_idx_1;
  static int8_T tmp_0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

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

    /* SignalConversion: '<S4>/TmpSignal ConversionAt SFunction Inport7' incorporates:
     *  Inport: '<Root>/Mag  Sensor'
     */

    /* Gain: '<Root>/Gain' incorporates:
     *  Inport: '<Root>/Gyro  Sensor'
     */
    rtb_Gain_idx = IMU_Idcm_P.Gain_Gain * fgyro[0];

    /* Gain: '<Root>/Gain1' incorporates:
     *  Inport: '<Root>/Acc  Sensor'
     */
    rtb_Gain1_idx = IMU_Idcm_P.Gain1_Gain * facc[0];
    rtb_TmpSignalConversionAtSFun_f[0] = rtb_Gain_idx;
    rtb_Gain_idx_1 = rtb_Gain_idx;

    /* Gain: '<Root>/Gain' incorporates:
     *  Inport: '<Root>/Gyro  Sensor'
     */
    rtb_Gain_idx = IMU_Idcm_P.Gain_Gain * fgyro[1];

    /* Gain: '<Root>/Gain1' incorporates:
     *  Inport: '<Root>/Acc  Sensor'
     */
    rtb_Gain1_idx_0 = IMU_Idcm_P.Gain1_Gain * facc[1];
    rtb_TmpSignalConversionAtSFun_f[1] = rtb_Gain_idx;
    rtb_Gain_idx_0 = rtb_Gain_idx;

    /* Gain: '<Root>/Gain' incorporates:
     *  Inport: '<Root>/Gyro  Sensor'
     */
    rtb_Gain_idx = IMU_Idcm_P.Gain_Gain * fgyro[2];

    /* Gain: '<Root>/Gain1' incorporates:
     *  Inport: '<Root>/Acc  Sensor'
     */
    rtb_Gain1_idx_1 = IMU_Idcm_P.Gain1_Gain * facc[2];
    rtb_TmpSignalConversionAtSFun_f[2] = rtb_Gain_idx;
    rtb_TmpSignalConversionAtSFun_f[3] = rtb_Gain1_idx;
    rtb_TmpSignalConversionAtSFun_f[4] = rtb_Gain1_idx_0;
    rtb_TmpSignalConversionAtSFun_f[5] = rtb_Gain1_idx_1;
    rtb_TmpSignalConversionAtSFun_f[6] = fmag[0];
    rtb_TmpSignalConversionAtSFun_f[7] = fmag[1];
    rtb_TmpSignalConversionAtSFun_f[8] = fmag[2];

    /* Embedded MATLAB: '<S2>/Kalman Filter 1' incorporates:
     *  Constant: '<S1>/Q1'
     *  Constant: '<S1>/Racc'
     *  Constant: '<S1>/acc_Rbar'
     *  Constant: '<S1>/acc_sigma'
     *  UnitDelay: '<S2>/Unit Delay'
     *  UnitDelay: '<S2>/Unit Delay1'
     */
    /* Embedded MATLAB Function 'DCM Kalman Filter/Kalman Filter 1/Kalman Filter 1': '<S4>:1' */
    /*  EKF  Extended Kalman Filter */
    /*  */
    /*    [xk, Pk, outxh_, outP_] = ekf(inxh_, inP_, Rv, Rn, obs) */
    /*  */
    /*    This filter assumes the following standard state-space model: */
    /*  */
    /*      x(k) = ffun[x(k-1),v(k-1),U1(k-1)] */
    /*      y(k) = hfun[x(k),n(k),U2(k)] */
    /*  */
    /*    where x is the system state, v the process noise, n the observation noise */
    /*    and y the noisy observation of the system. */
    /*  */
    /*    INPUT */
    /*          inxh_                predicted state at time k       */
    /*          inP_                 predicted state covariance at time */
    /*          Sv                   process noise Sv = chol(Q)' */
    /*          Sn                   measurement noise Sn = sqrtm(R)  */
    /*          obs                  noisy observations at time k */
    /*  */
    /*    OUTPUT */
    /*          xk                   estimates of state starting at time k  */
    /*          Pk                   state covariance */
    /*          outxh_               predicted state for time k+1       */
    /*          outP_                predicted state covariance for time k+1 */
    /*  */
    /* ========================================================================== */
    /* '<S4>:1:28' */
    /*  measurement value from accelerometer */
    /* '<S4>:1:29' */
    eml_mobs[0] = rtb_TmpSignalConversionAtSFun_f[3] * 0.981;
    eml_mobs[1] = rtb_TmpSignalConversionAtSFun_f[4] * 0.981;
    eml_mobs[2] = rtb_TmpSignalConversionAtSFun_f[5] * 0.981;

    /*  gyro measurement is considered as input value of the model */
    /* '<S4>:1:31' */
    /* Qd  = Rv* eye(3); */
    /* '<S4>:1:35' */
    /* '<S4>:1:36' */
    for (eml_p = 0; eml_p < 9; eml_p++) {
      eml_Qd[eml_p] = IMU_Idcm_P.Q1_Value[eml_p];
      eml_R[eml_p] = IMU_Idcm_P.Racc_Value[eml_p];
    }

    /* '<S4>:1:39' */
    if (fabs(sqrt((rt_pow_snf(eml_mobs[0], 2.0) + rt_pow_snf(eml_mobs[1], 2.0))
                  + rt_pow_snf(eml_mobs[2], 2.0)) - 9.81) >
        IMU_Idcm_P.acc_sigma_Value) {
      /* '<S4>:1:41' */
      /* '<S4>:1:42' */
      /* '<S4>:1:43' */
      for (eml_p = 0; eml_p < 9; eml_p++) {
        eml_Qd[eml_p] = IMU_Idcm_P.Q1_Value[eml_p] * IMU_Idcm_P.acc_Rbar_Value;
        eml_R[eml_p] = IMU_Idcm_P.Racc_Value[eml_p] / IMU_Idcm_P.acc_Rbar_Value;
      }
    }

    /*  initialise variables and parameters     */
    /*  compute Kalman gain */
    /* '<S4>:1:56' */
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 * eml_p_0] * (real_T)tmp_0[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 *
          eml_p_0 + 1] * (real_T)tmp_0[eml_p + 3] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 *
          eml_p_0 + 2] * (real_T)tmp_0[eml_p + 6] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_x[eml_p + 3 * eml_p_0] = (((real_T)tmp_0[3 * eml_p_0 + 1] *
          eml_b_x_0[eml_p + 3] + (real_T)tmp_0[3 * eml_p_0] * eml_b_x_0[eml_p])
          + (real_T)tmp_0[3 * eml_p_0 + 2] * eml_b_x_0[eml_p + 6]) + eml_R[3 *
          eml_p_0 + eml_p];
      }
    }

    memcpy((void *)&eml_b_x[0], (void *)&eml_x[0], 9U * sizeof(real_T));
    eml_p = 0;
    eml_p_0 = 3;
    eml_p_1 = 6;
    eml_absx = fabs(eml_x[0]);
    eml_absx_0 = fabs(eml_x[1]);
    eml_absx_1 = fabs(eml_x[2]);
    if ((eml_absx_0 > eml_absx) && (eml_absx_0 > eml_absx_1)) {
      eml_p = 3;
      eml_p_0 = 0;
      eml_b_x[0] = eml_x[1];
      eml_b_x[1] = eml_x[0];
      eml_absx = eml_b_x[3];
      eml_b_x[3] = eml_b_x[4];
      eml_b_x[4] = eml_absx;
      eml_absx = eml_b_x[6];
      eml_b_x[6] = eml_b_x[7];
      eml_b_x[7] = eml_absx;
    } else {
      if (eml_absx_1 > eml_absx) {
        eml_p = 6;
        eml_p_1 = 0;
        eml_b_x[0] = eml_x[2];
        eml_b_x[2] = eml_x[0];
        eml_absx = eml_b_x[3];
        eml_b_x[3] = eml_b_x[5];
        eml_b_x[5] = eml_absx;
        eml_absx = eml_b_x[6];
        eml_b_x[6] = eml_b_x[8];
        eml_b_x[8] = eml_absx;
      }
    }

    eml_b_x[1] /= eml_b_x[0];
    eml_b_x[2] /= eml_b_x[0];
    eml_b_x[4] -= eml_b_x[1] * eml_b_x[3];
    eml_b_x[5] -= eml_b_x[2] * eml_b_x[3];
    eml_b_x[7] -= eml_b_x[1] * eml_b_x[6];
    eml_b_x[8] -= eml_b_x[2] * eml_b_x[6];
    if (fabs(eml_b_x[5]) > fabs(eml_b_x[4])) {
      eml_itmp = eml_p_0;
      eml_p_0 = eml_p_1;
      eml_p_1 = eml_itmp;
      eml_absx = eml_b_x[1];
      eml_b_x[1] = eml_b_x[2];
      eml_b_x[2] = eml_absx;
      eml_absx = eml_b_x[4];
      eml_b_x[4] = eml_b_x[5];
      eml_b_x[5] = eml_absx;
      eml_absx = eml_b_x[7];
      eml_b_x[7] = eml_b_x[8];
      eml_b_x[8] = eml_absx;
    }

    eml_b_x[5] /= eml_b_x[4];
    eml_b_x[8] -= eml_b_x[5] * eml_b_x[7];
    eml_absx = (eml_b_x[5] * eml_b_x[1] - eml_b_x[2]) / eml_b_x[8];
    eml_absx_0 = (-(eml_b_x[7] * eml_absx + eml_b_x[1])) / eml_b_x[4];
    eml_x[eml_p] = ((1.0 - eml_b_x[3] * eml_absx_0) - eml_b_x[6] * eml_absx) /
      eml_b_x[0];
    eml_x[eml_p + 1] = eml_absx_0;
    eml_x[eml_p + 2] = eml_absx;
    eml_absx = (-eml_b_x[5]) / eml_b_x[8];
    eml_absx_0 = (1.0 - eml_b_x[7] * eml_absx) / eml_b_x[4];
    eml_x[eml_p_0] = (-(eml_b_x[3] * eml_absx_0 + eml_b_x[6] * eml_absx)) /
      eml_b_x[0];
    eml_x[eml_p_0 + 1] = eml_absx_0;
    eml_x[eml_p_0 + 2] = eml_absx;
    eml_absx = 1.0 / eml_b_x[8];
    eml_absx_0 = (-eml_b_x[7]) * eml_absx / eml_b_x[4];
    eml_x[eml_p_1] = (-(eml_b_x[3] * eml_absx_0 + eml_b_x[6] * eml_absx)) /
      eml_b_x[0];
    eml_x[eml_p_1 + 1] = eml_absx_0;
    eml_x[eml_p_1 + 2] = eml_absx;
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          (real_T)tmp_0[3 * eml_p_0] * IMU_Idcm_DWork.UnitDelay1_DSTATE_p[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = (real_T)tmp_0[3 * eml_p_0 + 1] *
          IMU_Idcm_DWork.UnitDelay1_DSTATE_p[eml_p + 3] + eml_b_x_0[3 * eml_p_0
          + eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = (real_T)tmp_0[3 * eml_p_0 + 2] *
          IMU_Idcm_DWork.UnitDelay1_DSTATE_p[eml_p + 6] + eml_b_x_0[3 * eml_p_0
          + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x[eml_p + 3 * eml_p_0] = eml_b_x[3 * eml_p_0 + eml_p] + eml_x[3 *
          eml_p_0] * eml_b_x_0[eml_p];
        eml_b_x[eml_p + 3 * eml_p_0] = eml_x[3 * eml_p_0 + 1] * eml_b_x_0[eml_p
          + 3] + eml_b_x[3 * eml_p_0 + eml_p];
        eml_b_x[eml_p + 3 * eml_p_0] = eml_x[3 * eml_p_0 + 2] * eml_b_x_0[eml_p
          + 6] + eml_b_x[3 * eml_p_0 + eml_p];
      }
    }

    /*  update estimate with measurement Zk */
    /* '<S4>:1:59' */
    for (eml_p = 0; eml_p < 3; eml_p++) {
      eml_mobs_1[eml_p] = eml_mobs[eml_p] - (((real_T)tmp_0[eml_p + 3] *
        IMU_Idcm_DWork.UnitDelay_DSTATE_h[1] + (real_T)tmp_0[eml_p] *
        IMU_Idcm_DWork.UnitDelay_DSTATE_h[0]) + (real_T)tmp_0[eml_p + 6] *
        IMU_Idcm_DWork.UnitDelay_DSTATE_h[2]);
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      rtb_xk[eml_p] = ((eml_b_x[eml_p + 3] * eml_mobs_1[1] + eml_b_x[eml_p] *
                        eml_mobs_1[0]) + eml_b_x[eml_p + 6] * eml_mobs_1[2]) +
        IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p];
    }

    /* '<S4>:1:60' */
    eml_absx = sqrt((rt_pow_snf(rtb_xk[0], 2.0) + rt_pow_snf(rtb_xk[1], 2.0)) +
                    rt_pow_snf(rtb_xk[2], 2.0));

    /* '<S4>:1:61' */
    rtb_xk[0] /= eml_absx;
    rtb_xk[1] /= eml_absx;
    rtb_xk[2] /= eml_absx;

    /*  compute error covariance for updated estimate */
    /* '<S4>:1:64' */
    /*  project ahead */
    /* '<S4>:1:68' */
    /* '<S4>:1:71' */
    for (eml_p = 0; eml_p < 9; eml_p++) {
      eml_I[eml_p] = 0;
    }

    eml_I[0] = 1;
    eml_I[4] = 1;
    eml_I[8] = 1;
    eml_b_x_0[0] = 0.0;
    eml_b_x_0[3] = rtb_TmpSignalConversionAtSFun_f[2];
    eml_b_x_0[6] = -rtb_TmpSignalConversionAtSFun_f[1];
    eml_b_x_0[1] = -rtb_TmpSignalConversionAtSFun_f[2];
    eml_b_x_0[4] = 0.0;
    eml_b_x_0[7] = rtb_TmpSignalConversionAtSFun_f[0];
    eml_b_x_0[2] = rtb_TmpSignalConversionAtSFun_f[1];
    eml_b_x_0[5] = -rtb_TmpSignalConversionAtSFun_f[0];
    eml_b_x_0[8] = 0.0;
    for (eml_p = 0; eml_p < 3; eml_p++) {
      eml_x[3 * eml_p] = eml_b_x_0[3 * eml_p] * 0.01 + (real_T)eml_I[3 * eml_p];
      eml_x[1 + 3 * eml_p] = eml_b_x_0[3 * eml_p + 1] * 0.01 + (real_T)eml_I[3 *
        eml_p + 1];
      eml_x[2 + 3 * eml_p] = eml_b_x_0[3 * eml_p + 2] * 0.01 + (real_T)eml_I[3 *
        eml_p + 2];
    }

    /* '<S4>:1:72' */
    /* '<S4>:1:73' */
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 * eml_p_0] * (real_T)tmp_0[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 *
          eml_p_0 + 1] * (real_T)tmp_0[eml_p + 3] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 *
          eml_p_0 + 2] * (real_T)tmp_0[eml_p + 6] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        tmp[eml_p + 3 * eml_p_0] = (((real_T)tmp_0[3 * eml_p_0 + 1] *
          eml_b_x_0[eml_p + 3] + (real_T)tmp_0[3 * eml_p_0] * eml_b_x_0[eml_p])
          + (real_T)tmp_0[3 * eml_p_0 + 2] * eml_b_x_0[eml_p + 6]) + eml_R[3 *
          eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] + tmp[3 *
          eml_p_0] * eml_b_x[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 1] * eml_b_x[eml_p +
          3] + eml_b_x_0[3 * eml_p_0 + eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 2] * eml_b_x[eml_p +
          6] + eml_b_x_0[3 * eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        tmp[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE_p[3 *
          eml_p_0 + eml_p] - ((eml_b_x_0[eml_p + 3] * eml_b_x[eml_p_0 + 3] +
          eml_b_x_0[eml_p] * eml_b_x[eml_p_0]) + eml_b_x_0[eml_p + 6] *
                              eml_b_x[eml_p_0 + 6]);
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] + tmp[3 *
          eml_p_0] * eml_x[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 1] * eml_x[eml_p + 3]
          + eml_b_x_0[3 * eml_p_0 + eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 2] * eml_x[eml_p + 6]
          + eml_b_x_0[3 * eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        rtb_outP__h[eml_p + 3 * eml_p_0] = ((eml_b_x_0[eml_p + 3] *
          eml_x[eml_p_0 + 3] + eml_b_x_0[eml_p] * eml_x[eml_p_0]) +
          eml_b_x_0[eml_p + 6] * eml_x[eml_p_0 + 6]) + eml_Qd[3 * eml_p_0 +
          eml_p];
      }
    }

    /* SignalConversion: '<S5>/TmpSignal ConversionAt SFunction Inport8' incorporates:
     *  Inport: '<Root>/Mag  Sensor'
     */
    rtb_TmpSignalConversionAtSFun_f[0] = rtb_Gain_idx_1;
    rtb_TmpSignalConversionAtSFun_f[1] = rtb_Gain_idx_0;
    rtb_TmpSignalConversionAtSFun_f[2] = rtb_Gain_idx;
    rtb_TmpSignalConversionAtSFun_f[3] = rtb_Gain1_idx;
    rtb_TmpSignalConversionAtSFun_f[4] = rtb_Gain1_idx_0;
    rtb_TmpSignalConversionAtSFun_f[5] = rtb_Gain1_idx_1;
    rtb_TmpSignalConversionAtSFun_f[6] = fmag[0];
    rtb_TmpSignalConversionAtSFun_f[7] = fmag[1];
    rtb_TmpSignalConversionAtSFun_f[8] = fmag[2];

    /* Embedded MATLAB: '<S3>/Kalman Filter 2' incorporates:
     *  Constant: '<S1>/Q2'
     *  Constant: '<S1>/Rmag'
     *  Constant: '<S1>/mag_Rbar'
     *  Constant: '<S1>/mag_sigma'
     *  UnitDelay: '<S3>/Unit Delay'
     *  UnitDelay: '<S3>/Unit Delay1'
     */
    /* Embedded MATLAB Function 'DCM Kalman Filter/Kalman Filter 2/Kalman Filter 2': '<S5>:1' */
    /*  EKF  Extended Kalman Filter */
    /*  */
    /*    [xk, Pk, outxh_, outP_] = ekf(inxh_, inP_, Rv, Rn, obs) */
    /*  */
    /*    This filter assumes the following standard state-space model: */
    /*  */
    /*      x(k) = ffun[x(k-1),v(k-1),U1(k-1)] */
    /*      y(k) = hfun[x(k),n(k),U2(k)] */
    /*  */
    /*    where x is the system state, v the process noise, n the observation noise */
    /*    and y the noisy observation of the system. */
    /*  */
    /*    INPUT */
    /*          inxh_                predicted state at time k       */
    /*          inP_                 predicted state covariance at time */
    /*          Sv                   process noise Sv = chol(Q)' */
    /*          Sn                   measurement noise Sn = sqrtm(R)  */
    /*          obs                  noisy observations at time k */
    /*  */
    /*    OUTPUT */
    /*          xk                   estimates of state starting at time k  */
    /*          Pk                   state covariance */
    /*          outxh_               predicted state for time k+1       */
    /*          outP_                predicted state covariance for time k+1 */
    /*  */
    /* ========================================================================== */
    /* '<S5>:1:28' */
    for (eml_p = 0; eml_p < 6; eml_p++) {
      eml_mobs_0[eml_p] = rtb_TmpSignalConversionAtSFun_f[eml_p + 3];
    }

    /*  measurement value from accelerometer and magnetic sensor */
    /* '<S5>:1:29' */
    /*  gyro measurement is considered as input value of the model */
    /* '<S5>:1:31' */
    /* '<S5>:1:32' */
    /* Qd  = Rv* eye(3); */
    /* '<S5>:1:36' */
    /* '<S5>:1:37' */
    for (eml_p = 0; eml_p < 9; eml_p++) {
      eml_Qd_0[eml_p] = IMU_Idcm_P.Q2_Value[eml_p];
      eml_R[eml_p] = IMU_Idcm_P.Rmag_Value[eml_p];
    }

    /* '<S5>:1:40' */
    if (fabs(sqrt((rt_pow_snf(eml_mobs_0[3], 2.0) + rt_pow_snf(eml_mobs_0[4],
            2.0)) + rt_pow_snf(eml_mobs_0[5], 2.0)) - 700.0) >
        IMU_Idcm_P.mag_sigma_Value) {
      /* '<S5>:1:42' */
      /* '<S5>:1:43' */
      /* '<S5>:1:44' */
      for (eml_p = 0; eml_p < 9; eml_p++) {
        eml_Qd_0[eml_p] = IMU_Idcm_P.Q2_Value[eml_p] * IMU_Idcm_P.mag_Rbar_Value;
        eml_R[eml_p] = IMU_Idcm_P.Rmag_Value[eml_p] / IMU_Idcm_P.mag_Rbar_Value;
      }
    }

    /* measurement */
    /* '<S5>:1:49' */
    eml_absx = rt_atan2_snf(rtb_xk[1], rtb_xk[2]);

    /* '<S5>:1:50' */
    eml_absx_0 = sin(eml_absx);

    /* '<S5>:1:51' */
    eml_absx_1 = cos(eml_absx);

    /* '<S5>:1:53' */
    rtb_Gain_idx = -rtb_xk[0];

    /*  sin(theta) */
    /* '<S5>:1:54' */
    rtb_Gain1_idx = sqrt(1.0 - rt_pow_snf(rtb_Gain_idx, 2.0));

    /*  cos(theta) */
    /* '<S5>:1:56' */
    eml_absx = (eml_mobs_0[4] * rtb_Gain_idx * eml_absx_0 + eml_mobs_0[3] *
                rtb_Gain1_idx) + eml_mobs_0[5] * rtb_Gain_idx * eml_absx_1;

    /* '<S5>:1:57' */
    rtb_Gain_idx_1 = eml_mobs_0[4] * eml_absx_1 - eml_mobs_0[5] * eml_absx_0;

    /* '<S5>:1:58' */
    rtb_Gain1_idx_0 = (-rtb_Gain_idx_1) / sqrt(rt_pow_snf(eml_absx, 2.0) +
      rt_pow_snf(rtb_Gain_idx_1, 2.0));

    /* '<S5>:1:59' */
    rtb_Gain_idx_0 = eml_absx / sqrt(rt_pow_snf(eml_absx, 2.0) + rt_pow_snf
      (rtb_Gain_idx_1, 2.0));

    /* '<S5>:1:61' */
    /* '<S5>:1:62' */
    /* '<S5>:1:63' */
    /* '<S5>:1:64' */
    /*  initialise variables and parameters     */
    /*  compute Kalman gain */
    /* '<S5>:1:75' */
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          IMU_Idcm_DWork.UnitDelay1_DSTATE[3 * eml_p_0] * (real_T)tmp_0[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE[3 *
          eml_p_0 + 1] * (real_T)tmp_0[eml_p + 3] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE[3 *
          eml_p_0 + 2] * (real_T)tmp_0[eml_p + 6] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_Qd[eml_p + 3 * eml_p_0] = (((real_T)tmp_0[3 * eml_p_0 + 1] *
          eml_b_x_0[eml_p + 3] + (real_T)tmp_0[3 * eml_p_0] * eml_b_x_0[eml_p])
          + (real_T)tmp_0[3 * eml_p_0 + 2] * eml_b_x_0[eml_p + 6]) + eml_R[3 *
          eml_p_0 + eml_p];
      }
    }

    memcpy((void *)&eml_b_x[0], (void *)&eml_Qd[0], 9U * sizeof(real_T));
    eml_p = 0;
    eml_p_0 = 3;
    eml_p_1 = 6;
    eml_absx = fabs(eml_Qd[0]);
    rtb_Gain_idx_1 = fabs(eml_Qd[1]);
    rtb_Gain1_idx_1 = fabs(eml_Qd[2]);
    if ((rtb_Gain_idx_1 > eml_absx) && (rtb_Gain_idx_1 > rtb_Gain1_idx_1)) {
      eml_p = 3;
      eml_p_0 = 0;
      eml_b_x[0] = eml_Qd[1];
      eml_b_x[1] = eml_Qd[0];
      eml_absx = eml_b_x[3];
      eml_b_x[3] = eml_b_x[4];
      eml_b_x[4] = eml_absx;
      eml_absx = eml_b_x[6];
      eml_b_x[6] = eml_b_x[7];
      eml_b_x[7] = eml_absx;
    } else {
      if (rtb_Gain1_idx_1 > eml_absx) {
        eml_p = 6;
        eml_p_1 = 0;
        eml_b_x[0] = eml_Qd[2];
        eml_b_x[2] = eml_Qd[0];
        eml_absx = eml_b_x[3];
        eml_b_x[3] = eml_b_x[5];
        eml_b_x[5] = eml_absx;
        eml_absx = eml_b_x[6];
        eml_b_x[6] = eml_b_x[8];
        eml_b_x[8] = eml_absx;
      }
    }

    eml_b_x[1] /= eml_b_x[0];
    eml_b_x[2] /= eml_b_x[0];
    eml_b_x[4] -= eml_b_x[1] * eml_b_x[3];
    eml_b_x[5] -= eml_b_x[2] * eml_b_x[3];
    eml_b_x[7] -= eml_b_x[1] * eml_b_x[6];
    eml_b_x[8] -= eml_b_x[2] * eml_b_x[6];
    if (fabs(eml_b_x[5]) > fabs(eml_b_x[4])) {
      eml_itmp = eml_p_0;
      eml_p_0 = eml_p_1;
      eml_p_1 = eml_itmp;
      eml_absx = eml_b_x[1];
      eml_b_x[1] = eml_b_x[2];
      eml_b_x[2] = eml_absx;
      eml_absx = eml_b_x[4];
      eml_b_x[4] = eml_b_x[5];
      eml_b_x[5] = eml_absx;
      eml_absx = eml_b_x[7];
      eml_b_x[7] = eml_b_x[8];
      eml_b_x[8] = eml_absx;
    }

    eml_b_x[5] /= eml_b_x[4];
    eml_b_x[8] -= eml_b_x[5] * eml_b_x[7];
    eml_absx = (eml_b_x[5] * eml_b_x[1] - eml_b_x[2]) / eml_b_x[8];
    rtb_Gain_idx_1 = (-(eml_b_x[7] * eml_absx + eml_b_x[1])) / eml_b_x[4];
    eml_Qd[eml_p] = ((1.0 - eml_b_x[3] * rtb_Gain_idx_1) - eml_b_x[6] * eml_absx)
      / eml_b_x[0];
    eml_Qd[eml_p + 1] = rtb_Gain_idx_1;
    eml_Qd[eml_p + 2] = eml_absx;
    eml_absx = (-eml_b_x[5]) / eml_b_x[8];
    rtb_Gain_idx_1 = (1.0 - eml_b_x[7] * eml_absx) / eml_b_x[4];
    eml_Qd[eml_p_0] = (-(eml_b_x[3] * rtb_Gain_idx_1 + eml_b_x[6] * eml_absx)) /
      eml_b_x[0];
    eml_Qd[eml_p_0 + 1] = rtb_Gain_idx_1;
    eml_Qd[eml_p_0 + 2] = eml_absx;
    eml_absx = 1.0 / eml_b_x[8];
    rtb_Gain_idx_1 = (-eml_b_x[7]) * eml_absx / eml_b_x[4];
    eml_Qd[eml_p_1] = (-(eml_b_x[3] * rtb_Gain_idx_1 + eml_b_x[6] * eml_absx)) /
      eml_b_x[0];
    eml_Qd[eml_p_1 + 1] = rtb_Gain_idx_1;
    eml_Qd[eml_p_1 + 2] = eml_absx;
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          (real_T)tmp_0[3 * eml_p_0] * IMU_Idcm_DWork.UnitDelay1_DSTATE[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = (real_T)tmp_0[3 * eml_p_0 + 1] *
          IMU_Idcm_DWork.UnitDelay1_DSTATE[eml_p + 3] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = (real_T)tmp_0[3 * eml_p_0 + 2] *
          IMU_Idcm_DWork.UnitDelay1_DSTATE[eml_p + 6] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x[eml_p + 3 * eml_p_0] = eml_b_x[3 * eml_p_0 + eml_p] + eml_Qd[3 *
          eml_p_0] * eml_b_x_0[eml_p];
        eml_b_x[eml_p + 3 * eml_p_0] = eml_Qd[3 * eml_p_0 + 1] * eml_b_x_0[eml_p
          + 3] + eml_b_x[3 * eml_p_0 + eml_p];
        eml_b_x[eml_p + 3 * eml_p_0] = eml_Qd[3 * eml_p_0 + 2] * eml_b_x_0[eml_p
          + 6] + eml_b_x[3 * eml_p_0 + eml_p];
      }
    }

    /*  update estimate with measurement Zk */
    /* '<S5>:1:77' */
    eml_mobs[0] = rtb_Gain1_idx * rtb_Gain1_idx_0;
    eml_mobs[1] = eml_absx_0 * rtb_Gain_idx * rtb_Gain1_idx_0 + eml_absx_1 *
      rtb_Gain_idx_0;
    eml_mobs[2] = eml_absx_1 * rtb_Gain_idx * rtb_Gain1_idx_0 - eml_absx_0 *
      rtb_Gain_idx_0;
    for (eml_p = 0; eml_p < 3; eml_p++) {
      eml_mobs_1[eml_p] = eml_mobs[eml_p] - (((real_T)tmp_0[eml_p + 3] *
        IMU_Idcm_DWork.UnitDelay_DSTATE[1] + (real_T)tmp_0[eml_p] *
        IMU_Idcm_DWork.UnitDelay_DSTATE[0]) + (real_T)tmp_0[eml_p + 6] *
        IMU_Idcm_DWork.UnitDelay_DSTATE[2]);
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      eml_mobs[eml_p] = ((eml_b_x[eml_p + 3] * eml_mobs_1[1] + eml_b_x[eml_p] *
                          eml_mobs_1[0]) + eml_b_x[eml_p + 6] * eml_mobs_1[2]) +
        IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p];
    }

    /* '<S5>:1:78' */
    eml_absx = sqrt((rt_pow_snf(eml_mobs[0], 2.0) + rt_pow_snf(eml_mobs[1], 2.0))
                    + rt_pow_snf(eml_mobs[2], 2.0));

    /* '<S5>:1:79' */
    eml_mobs[0] /= eml_absx;
    eml_mobs[1] /= eml_absx;
    eml_mobs[2] /= eml_absx;

    /*  compute error covariance for updated estimate */
    /* '<S5>:1:82' */
    /*  project ahead */
    /* '<S5>:1:85' */
    /* '<S5>:1:88' */
    for (eml_p = 0; eml_p < 9; eml_p++) {
      eml_I[eml_p] = 0;
    }

    eml_I[0] = 1;
    eml_I[4] = 1;
    eml_I[8] = 1;
    eml_b_x_0[0] = 0.0;
    eml_b_x_0[3] = rtb_TmpSignalConversionAtSFun_f[2];
    eml_b_x_0[6] = -rtb_TmpSignalConversionAtSFun_f[1];
    eml_b_x_0[1] = -rtb_TmpSignalConversionAtSFun_f[2];
    eml_b_x_0[4] = 0.0;
    eml_b_x_0[7] = rtb_TmpSignalConversionAtSFun_f[0];
    eml_b_x_0[2] = rtb_TmpSignalConversionAtSFun_f[1];
    eml_b_x_0[5] = -rtb_TmpSignalConversionAtSFun_f[0];
    eml_b_x_0[8] = 0.0;
    for (eml_p = 0; eml_p < 3; eml_p++) {
      eml_Qd[3 * eml_p] = eml_b_x_0[3 * eml_p] * 0.01 + (real_T)eml_I[3 * eml_p];
      eml_Qd[1 + 3 * eml_p] = eml_b_x_0[3 * eml_p + 1] * 0.01 + (real_T)eml_I[3 *
        eml_p + 1];
      eml_Qd[2 + 3 * eml_p] = eml_b_x_0[3 * eml_p + 2] * 0.01 + (real_T)eml_I[3 *
        eml_p + 2];
    }

    /* '<S5>:1:89' */
    /* '<S5>:1:90' */
    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] +
          IMU_Idcm_DWork.UnitDelay1_DSTATE[3 * eml_p_0] * (real_T)tmp_0[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE[3 *
          eml_p_0 + 1] * (real_T)tmp_0[eml_p + 3] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE[3 *
          eml_p_0 + 2] * (real_T)tmp_0[eml_p + 6] + eml_b_x_0[3 * eml_p_0 +
          eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        tmp[eml_p + 3 * eml_p_0] = (((real_T)tmp_0[3 * eml_p_0 + 1] *
          eml_b_x_0[eml_p + 3] + (real_T)tmp_0[3 * eml_p_0] * eml_b_x_0[eml_p])
          + (real_T)tmp_0[3 * eml_p_0 + 2] * eml_b_x_0[eml_p + 6]) + eml_R[3 *
          eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] + tmp[3 *
          eml_p_0] * eml_b_x[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 1] * eml_b_x[eml_p +
          3] + eml_b_x_0[3 * eml_p_0 + eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 2] * eml_b_x[eml_p +
          6] + eml_b_x_0[3 * eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        tmp[eml_p + 3 * eml_p_0] = IMU_Idcm_DWork.UnitDelay1_DSTATE[3 * eml_p_0
          + eml_p] - ((eml_b_x_0[eml_p + 3] * eml_b_x[eml_p_0 + 3] +
                       eml_b_x_0[eml_p] * eml_b_x[eml_p_0]) + eml_b_x_0[eml_p +
                      6] * eml_b_x[eml_p_0 + 6]);
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        eml_b_x_0[eml_p + 3 * eml_p_0] = 0.0;
        eml_b_x_0[eml_p + 3 * eml_p_0] = eml_b_x_0[3 * eml_p_0 + eml_p] + tmp[3 *
          eml_p_0] * eml_Qd[eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 1] * eml_Qd[eml_p + 3]
          + eml_b_x_0[3 * eml_p_0 + eml_p];
        eml_b_x_0[eml_p + 3 * eml_p_0] = tmp[3 * eml_p_0 + 2] * eml_Qd[eml_p + 6]
          + eml_b_x_0[3 * eml_p_0 + eml_p];
      }
    }

    for (eml_p = 0; eml_p < 3; eml_p++) {
      for (eml_p_0 = 0; eml_p_0 < 3; eml_p_0++) {
        rtb_outP_[eml_p + 3 * eml_p_0] = ((eml_b_x_0[eml_p + 3] * eml_Qd[eml_p_0
          + 3] + eml_b_x_0[eml_p] * eml_Qd[eml_p_0]) + eml_b_x_0[eml_p + 6] *
          eml_Qd[eml_p_0 + 6]) + eml_Qd_0[3 * eml_p_0 + eml_p];
      }
    }

    /* '<S5>:1:92' */
    eml_absx = eml_mobs[1] * rtb_xk[2] - eml_mobs[2] * rtb_xk[1];

    /* '<S5>:1:93' */
    /* '<S5>:1:94' */
    /* '<S5>:1:96' */
    eml_absx /= sqrt((rt_pow_snf(eml_mobs[2] * rtb_xk[0] - eml_mobs[0] * rtb_xk
      [2], 2.0) + rt_pow_snf(eml_absx, 2.0)) + rt_pow_snf(eml_mobs[0] * rtb_xk[1]
      - eml_mobs[1] * rtb_xk[0], 2.0));

    /* '<S5>:1:98' */
    /*  phi */
    /* '<S5>:1:99' */
    /*  theta */
    /* '<S5>:1:100' */

    /* Gain: '<Root>/Gain2' */
    Out1[0] = IMU_Idcm_P.Gain2_Gain * rt_atan2_snf(rtb_xk[1], rtb_xk[2]);
    Out1[1] = IMU_Idcm_P.Gain2_Gain * asin(-rtb_xk[0]);
    Out1[2] = IMU_Idcm_P.Gain2_Gain * rt_atan2_snf(eml_mobs[0], eml_absx);

    /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
    Out2 = ((real_T)IMU_Idcm_DWork.clockTickCounter <
            IMU_Idcm_P.PulseGenerator_Duty) && (IMU_Idcm_DWork.clockTickCounter >=
      0) ? IMU_Idcm_P.PulseGenerator_Amp : 0.0;
    if ((real_T)IMU_Idcm_DWork.clockTickCounter >=
        IMU_Idcm_P.PulseGenerator_Period - 1.0) {
      IMU_Idcm_DWork.clockTickCounter = 0;
    } else {
      IMU_Idcm_DWork.clockTickCounter = IMU_Idcm_DWork.clockTickCounter + 1;
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

  /* Update for UnitDelay: '<S3>/Unit Delay' */
  for (eml_p = 0; eml_p < 3; eml_p++) {
    IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p] = 0.0;
    IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p] = eml_Qd[eml_p] * eml_mobs[0] +
      IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p];
    IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p] = eml_Qd[eml_p + 3] * eml_mobs[1] +
      IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p];
    IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p] = eml_Qd[eml_p + 6] * eml_mobs[2] +
      IMU_Idcm_DWork.UnitDelay_DSTATE[eml_p];
  }

  /* Update for UnitDelay: '<S3>/Unit Delay1' */
  memcpy((void *)(&IMU_Idcm_DWork.UnitDelay1_DSTATE[0]), (void *)&rtb_outP_[0],
         9U * sizeof(real_T));

  /* Update for UnitDelay: '<S2>/Unit Delay' */
  for (eml_p = 0; eml_p < 3; eml_p++) {
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p] = 0.0;
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p] = eml_x[eml_p] * rtb_xk[0] +
      IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p];
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p] = eml_x[eml_p + 3] * rtb_xk[1] +
      IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p];
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p] = eml_x[eml_p + 6] * rtb_xk[2] +
      IMU_Idcm_DWork.UnitDelay_DSTATE_h[eml_p];
  }

  /* Update for UnitDelay: '<S2>/Unit Delay1' */
  memcpy((void *)(&IMU_Idcm_DWork.UnitDelay1_DSTATE_p[0]), (void *)&rtb_outP__h
         [0], 9U * sizeof(real_T));

  /* Update for UnitDelay: '<S2>/Unit Delay3' */
  IMU_Idcm_DWork.UnitDelay3_DSTATE = 0.0;

  /* Update for UnitDelay: '<S3>/Unit Delay2' */
  IMU_Idcm_DWork.UnitDelay2_DSTATE = 0.0;
}

/* Model initialize function */
void IMU_Idcm_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(IMU_Idcm_M, (NULL));

  /* block I/O */

  /* exported global signals */
  Out1[0] = 0.0;
  Out1[1] = 0.0;
  Out1[2] = 0.0;
  Out2 = 0.0;

  /* states (dwork) */
  (void) memset((void *)&IMU_Idcm_DWork, 0,
                sizeof(D_Work_IMU_Idcm));

  /* external inputs */
  (void) memset(fgyro,0,
                3*sizeof(real_T));
  (void) memset(facc,0,
                3*sizeof(real_T));
  (void) memset(fmag,0,
                3*sizeof(real_T));

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator' */
  IMU_Idcm_DWork.clockTickCounter = 0;

  {
    int32_T i;

    /* InitializeConditions for UnitDelay: '<S3>/Unit Delay' */
    IMU_Idcm_DWork.UnitDelay_DSTATE[0] = IMU_Idcm_P.UnitDelay_X0[0];
    IMU_Idcm_DWork.UnitDelay_DSTATE[1] = IMU_Idcm_P.UnitDelay_X0[1];
    IMU_Idcm_DWork.UnitDelay_DSTATE[2] = IMU_Idcm_P.UnitDelay_X0[2];

    /* InitializeConditions for UnitDelay: '<S2>/Unit Delay' */
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[0] = IMU_Idcm_P.UnitDelay_X0_i[0];
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[1] = IMU_Idcm_P.UnitDelay_X0_i[1];
    IMU_Idcm_DWork.UnitDelay_DSTATE_h[2] = IMU_Idcm_P.UnitDelay_X0_i[2];
    for (i = 0; i < 9; i++) {
      /* InitializeConditions for UnitDelay: '<S3>/Unit Delay1' */
      IMU_Idcm_DWork.UnitDelay1_DSTATE[i] = IMU_Idcm_P.UnitDelay1_X0[i];

      /* InitializeConditions for UnitDelay: '<S2>/Unit Delay1' */
      IMU_Idcm_DWork.UnitDelay1_DSTATE_p[i] = IMU_Idcm_P.UnitDelay1_X0_n[i];
    }

    /* InitializeConditions for UnitDelay: '<S2>/Unit Delay3' */
    IMU_Idcm_DWork.UnitDelay3_DSTATE = IMU_Idcm_P.UnitDelay3_X0;

    /* InitializeConditions for UnitDelay: '<S3>/Unit Delay2' */
    IMU_Idcm_DWork.UnitDelay2_DSTATE = IMU_Idcm_P.UnitDelay2_X0;
  }
}

/* Model terminate function */
void IMU_Idcm_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
