/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: 底盘功率限制算法实现
  * Author			:
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "math.h"

float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;
fw_PID_Regulator_t PowerLimitationPID = POWER_LIMITATION_PID_DEFAULT;

#ifndef JUDGE_RM_2018 //JUDGE_RM_2019
//底盘功率限制
void PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if(PowerHeat.chassis_power_buffer-((PowerHeat.chassis_power-80)>0?(PowerHeat.chassis_power-80):0)*0.5f < 10.0f)
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if (Cap_Get_Cap_State() != CAP_STATE_RELEASE && rlease_flag == 1 )
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 2730; //10000
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于常态的基于自检测功率的功率限制
void CurBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80):0)*1.0f < 20.0f))
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于放电模式下的基于自检测功率的功率限制
void CapBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	else if(Cap_Get_Cap_Voltage() < 8 && (PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 50.0f))
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		float realPower = PowerHeat.chassis_power;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

#else //JUDGE_RM_2018
//底盘功率限制
void PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-80)>0?(PowerHeatData.chassisPower-80):0)*1.0f < 20.0f))
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if (Cap_Get_Cap_State() != CAP_STATE_RELEASE && rlease_flag == 1 )
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 2730; //10000
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于常态的基于自检测功率的功率限制
void CurBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeatData.chassisPowerBuffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80):0)*1.0f < 20.0f))
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于放电模式下的基于自检测功率的功率限制
void CapBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	else if(Cap_Get_Cap_Voltage() < 8 && (PowerHeatData.chassisPowerBuffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 50.0f))
	{
		//CM_current_max = 2730;
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		PowerLimitationPID.feedback = realPower;
		PowerLimitationPID.target = 75;
		PowerLimitationPID.Calc(&PowerLimitationPID);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

#endif
