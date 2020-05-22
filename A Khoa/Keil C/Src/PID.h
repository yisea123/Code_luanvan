/*
 * PID.h
 *
 *  Created on: May 9, 2015
 *      Author: AntoniTran
 */

#ifndef PID_H_
#define PID_H_
#include <stdint.h>
typedef struct{
	float *a;
	int p;
	float *b;
	int z;
	float *y;
	float *u;
} IIRFilter;
typedef struct
{
		float Kp;
		float Ki;
		float Kd;
		float Kff;//feedforward
		float pPart;
		float iPart;
		float dPart;
		float Ts;
		float PreSetPoint2;
		float PreSetPoint;
		float SetPoint;
		float PIDResult;
		float PIDResultTemp;
		float PIDError;
		float PIDErrorTemp1;
		float PIDErrorTemp2;
		uint8_t Enable;
		IIRFilter *filter;
} PIDType;

void PIDReset(PIDType *pidName);
void PIDSet(PIDType *pidName, float setPoint);
void PIDCalc(PIDType *pidName, float feedback, float maxResponse); //T (ms)
void PIDSetParams(PIDType *pidName,float Kp, float Ki, float Kd,float Tms);
void PIDSetFeedforward(PIDType *pidName, float Kff);
void PIDSetFilter(PIDType *pidName, IIRFilter *filter);
float FilterProcess(IIRFilter *filter, float u);
void EnablePID(PIDType *pidName);
void DisablePID(PIDType *pidName);
void PIDInit(PIDType *pidName);
#endif /* PID_H_ */
