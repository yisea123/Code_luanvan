/*
 * PID.c
 *
 *  Created on: Sep 1, 2016
 *      Author: Huan Nguyen
 */
#include "PID.h"
#include "define.h"
#include "utils.h"

void PIDSet(PIDType *pidName, float setPoint)
{
//	(*pidName).Enable = 0;
	(*pidName).PreSetPoint2 = 	(*pidName).PreSetPoint;
	(*pidName).PreSetPoint = 	(*pidName).SetPoint;	
	(*pidName).SetPoint = setPoint;
//	(*pidName).Enable = 1;
}

void PIDCalc(PIDType *pidName, float feedBack, float maxResponse)
{
	float feedPart=0.0f, diffSetPoint=0.0f;
	if (pidName->Enable)
	{
#ifdef PID_METHOD1
		float k1,k2,k3;

		(*pidName).PIDError = Sub_Mod_Pi((*pidName).SetPoint,feedBack);

		k1=(*pidName).Kp+(*pidName).Ki/2+(*pidName).Kd;
		k2=-(*pidName).Kp+(*pidName).Ki/2-2*(*pidName).Kd;
		k3=(*pidName).Kd;

		(*pidName).PIDResult=(*pidName).PIDResultTemp +
				k1*(*pidName).PIDError +
				k2*(*pidName).PIDErrorTemp1 +
				k3*(*pidName).PIDErrorTemp2 +
				(*pidName).Kff*(Sub_Mod_Pi((*pidName).SetPoint,(*pidName).PreSetPoint)+Sub_Mod_Pi((*pidName).PreSetPoint2,(*pidName).PreSetPoint));
		(*pidName).PIDErrorTemp2=(*pidName).PIDErrorTemp1;
		(*pidName).PIDErrorTemp1=(*pidName).PIDError;
		(*pidName).PreSetPoint2 = 	(*pidName).PreSetPoint;
		(*pidName).PreSetPoint = 	(*pidName).SetPoint;
		
		if ((*pidName).PIDResult > maxResponse)
			(*pidName).PIDResult = maxResponse;
		else if ((*pidName).PIDResult < -maxResponse)
			(*pidName).PIDResult = -maxResponse;

		(*pidName).PIDResultTemp = (*pidName).PIDResult;
#endif	

#ifdef PID_METHOD2
		(*pidName).PIDError = Sub_Mod_Pi((*pidName).SetPoint,feedBack);
		(*pidName).pPart = (*pidName).Kp * (*pidName).PIDError;
		(*pidName).iPart += (*pidName).Ki * ((*pidName).PIDError + (*pidName).PIDErrorTemp1)/2;
		(*pidName).dPart = (*pidName).Kd * ((*pidName).PIDError - (*pidName).PIDErrorTemp1);
		(*pidName).PIDErrorTemp1=(*pidName).PIDError;		
		(*pidName).PIDResult = (*pidName).pPart + (*pidName).iPart + (*pidName).dPart;
		
		diffSetPoint = Sub_Mod_Pi((*pidName).SetPoint,(*pidName).PreSetPoint);
//		if (__fabs(diffSetPoint) > 0.12)
//			 feedPart = 0.0015*diffSetPoint;
//		else
//			 feedPart = 0.0015*diffSetPoint;
		feedPart = 	(*pidName).Kff*diffSetPoint;
		(*pidName).PIDResult += feedPart;
		(*pidName).PreSetPoint = 	(*pidName).SetPoint;		
		
		
		if (pidName->filter)
		{
			pidName->PIDResult = FilterProcess(pidName->filter, pidName->PIDResult);
		}
		
		if ((*pidName).iPart > maxResponse)
			(*pidName).iPart = maxResponse;
		else if ((*pidName).iPart < -maxResponse)
			(*pidName).iPart = -maxResponse;		
		
		if ((*pidName).PIDResult > maxResponse)
			(*pidName).PIDResult = maxResponse;
		else if ((*pidName).PIDResult < -maxResponse)
			(*pidName).PIDResult = -maxResponse;		
#endif		
	}
}

void PIDReset(PIDType *pidName)
{
	(*pidName).PIDErrorTemp1=0.0;
	(*pidName).PIDErrorTemp2=0.0;
	(*pidName).PIDResultTemp=0.0;
	(*pidName).iPart = 0.0;
	(*pidName).PreSetPoint = 0.0;
	(*pidName).PreSetPoint2 = 0.0;
}

void PIDSetParams(PIDType *pidName,float Kp, float Ki, float Kd, float Ts)
{
	(*pidName).Ts = Ts;
	(*pidName).Kp=Kp;
	(*pidName).Ki=Ki*Ts;
	(*pidName).Kd=Kd/Ts;
	PIDReset(pidName);
}
void PIDSetFeedforward(PIDType *pidName, float Kff)
{
	(*pidName).Kff = Kff/(*pidName).Ts;
}
void PIDSetFilter(PIDType *pidName, IIRFilter *filter)
{
	(*pidName).filter = filter;
}
float FilterProcess(IIRFilter *filter, float u)
{
	float y=0.0f;
	int i;
	for (i=(filter->z)-1;i>0;i--)
	{
		//shift input
		filter->u[i] = filter->u[i-1];
		//calculate output
		y += filter->b[i]*filter->u[i];	
	}
	filter->u[0] = u;
	y += u*filter->b[0];

	for (i=(filter->p)-1;i>0;i--)
	{
		//shift output	
		filter->y[i] = filter->y[i-1];
		//calculate output
		y -= filter->a[i]*filter->y[i];
	}
	//calculate output
	y /= filter->a[0];
	filter->y[0] = y;

	return y;
}
void EnablePID(PIDType *pidName)
{
	(*pidName).Enable = 1;
	PIDReset(pidName);
}

void DisablePID(PIDType *pidName)
{
	(*pidName).Enable = 0;
}
void PIDInit(PIDType *pidName)
{
		pidName->Kp=0.0f;
		pidName->Ki=0.0f;
		pidName->Kd=0.0f;
		pidName->Kff=0.0f;//feedforward
		pidName->pPart=0.0f;
		pidName->iPart=0.0f;
		pidName->dPart=0.0f;
		pidName->Ts=1.0f;
		pidName->PreSetPoint2=0.0f;
		pidName->PreSetPoint=0.0f;
		pidName->SetPoint=0.0f;
		pidName->PIDResult=0.0f;
		pidName->PIDResultTemp=0.0f;
		pidName->PIDError=0.0f;
		pidName->PIDErrorTemp1=0.0f;
		pidName->PIDErrorTemp2=0.0f;
		pidName->Enable=0;
		pidName->filter=0;
}