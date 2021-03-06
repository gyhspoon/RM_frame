/**
  ******************************************************************************
  * File Name          : CANMotot.c
  * Description        : CAN电机统一驱动任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

//#define USE_GIMBAL_ENCODER

void ControlNM(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);
extern int16_t testIntensity;

uint8_t GMYReseted = 0;
uint8_t GMPReseted = 0;

//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				Chassis_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
//************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)
//************************************************************************
//使用云台电机时，请务必确定校准过零点
MotorINFO GMP  = Gimbal_MOTORINFO_Init(1.0,&ControlGMP,
									   fw_PID_INIT(0.5,0,0, 	100.0, 0, 0, 10.0),
									   fw_PID_INIT(10000.0,150.0,0, 	50000.0, 3000.0, 0, 30000.0));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY,
									   fw_PID_INIT(0.5,0,0, 					100.0, 0, 0, 10.0),
									   fw_PID_INIT(13000.0,200.0,0, 	50000.0, 3000.0, 0, 30000.0));
										 
MotorINFO SGMP  = Gimbal_MOTORINFO_Init(1.0,&ControlGMP,
									   fw_PID_INIT(0.5,0,0.3, 	10.0, 10.0, 10.0, 10.0),
									   fw_PID_INIT(12000,25,0, 	50000.0, 50000.0, 50000.0, 29000.0));
MotorINFO SGMY  = Normal_MOTORINFO_Init(1.0,&ControlGMY,
									   fw_PID_INIT(0.36,0,0, 	10.0, 10.0, 10.0, 10.0),
									   fw_PID_INIT(8500,22,0, 	50000.0, 50000.0, 50000.0, 30000.0));

//*************************************************************************
//			Normal_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
MotorINFO SSTIR = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(10.0, 0.0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(30, 0.0, 0.0, 		10000.0, 10000.0, 10000.0, 10000.0));
MotorINFO STIR = Normal_MOTORINFO_Init(3591.0f/187.0f,&ControlNM,
								fw_PID_INIT(400.0, 0.0, 0.0, 	20000.0, 0, 0, 20000.0),
								fw_PID_INIT(2.0, 0.15, 0.0, 		15000.0, 3000.0, 15000.0, 15000.0));

MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMY,&GMP,0,&STIR};
MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,&SGMY,&SGMP,&SSTIR,0};

void ControlNM(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6;		//单位：度每秒
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}
void ControlCM(MotorINFO* id)
{
	//TargetAngle 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->TargetAngle);
	id->offical_speedPID.fdb = id->RxMsgC6x0.RotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}
#ifdef USE_IMU
void ControlGMY(MotorINFO* id)
{
	if(id==0) return;

	#ifdef USE_CHASSIS_FOLLOW
	#ifndef USE_GIMBAL_ENCODER
	float 	ThisAngle = imu.yaw;
	static	float		ThisAngleZero = 0;
	static	uint8_t	ChassisLockRCD = 0;
	if(chassis_lock != ChassisLockRCD)
	{
		ThisAngleZero = ThisAngle;
		ChassisLockRCD = chassis_lock;
	}
	ThisAngle = (chassis_lock) ? ( - (float)(chassis_follow_center - id->RxMsgC6x0.angle) * 360.0f / 8192.0f + ThisAngleZero) : ThisAngle;
	#else
	float 	ThisAngle = - (float)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	#else
	double 	ThisAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	float 	ThisSpeed = -imu.wz;
	if(fabs(ThisSpeed) < 0.05) ThisSpeed = 0;	
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;

	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
		id->RealAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	if(fabs(id->RealAngle-id->TargetAngle)<1) GMYReseted = 1;
	id->lastRead = ThisAngle;
	#ifdef HERO_MAIN
	MINMAX(id->TargetAngle, id->RealAngle + (GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f / id->ReductionRate - 225.0f, id->RealAngle + (GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate + 90.0f);
	#endif
	if(GMYReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
	id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);	
}

void ControlGMP(MotorINFO* id)
{
	if(id==0) return;

	#ifdef USE_CHASSIS_FOLLOW
	#ifndef USE_GIMBAL_ENCODER
	float 	ThisAngle = imu.pit;
	ThisAngle = (chassis_lock) ? ( - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f) : ThisAngle;
	#else
	float 	ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	#else
	double 	ThisAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	float 	ThisSpeed = -imu.wy;
	if(fabs(ThisSpeed) < 0.05) ThisSpeed = 0;
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;
	
	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
		id->RealAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	if(fabs(id->RealAngle-id->TargetAngle)<1) GMPReseted = 1;
	id->lastRead = ThisAngle ;
	#ifdef HERO_MAIN
	MINMAX(id->TargetAngle, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 40.0f, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate + 20.0f);
	#endif
	if(GMPReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
//	if(GMPReseted==0) id->positionPID.outputMax = 50.0;
//	else id->positionPID.outputMax = 2000.0;
	id->Intensity = GM_PITCH_GRAVITY_COMPENSATION + PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
	MINMAX(id->Intensity,-id->speedPID.outputMax,id->speedPID.outputMax);	
}
#endif
//CAN
void setCAN11()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef CAN12
			can1_type = 2;
		#else
		#ifdef NO_RC_MODE
			can1_type = 3;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}
void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef NO_RC_MODE
			can1_type = 3;
		#else
		#ifdef CAN11
			can1_type = 1;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN22
			can2_type = 2;
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN21
			can2_type = 1;
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->RealAngle=0;
	id->TargetAngle=0;
	id->offical_speedPID.Reset(&(id->offical_speedPID));
	(id->Handle)(id);
	id->Intensity=0;
}

void Motor_ID_Setting()
{
	for(int i=0;i<4;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x200;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x200;
		}
	}
	for(int i=4;i<8;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x1ff;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x1ff;
		}
	}
}
