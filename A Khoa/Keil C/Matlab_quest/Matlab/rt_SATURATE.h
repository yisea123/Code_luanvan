/*
 * File: rt_SATURATE.h
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

#ifndef RTW_HEADER_rt_SATURATE_h_
#define RTW_HEADER_rt_SATURATE_h_
#define rt_SATURATE(sig,ll,ul)         (((sig) >= (ul)) ? (ul) : (((sig) <= (ll)) ? (ll) : (sig)) )
#endif                                 /* RTW_HEADER_rt_SATURATE_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
