/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: �������
  *Author					: ���׺�
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "includes.h"

#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM

GMINFO_t aim;
Coordinate_t enemy_gun,enemy_scope,scope_gun;
uint8_t Rx_enemy_INFO[8],Tx_current_INFO[4],tx_buffer;
uint8_t find_enemy=0,aim_cnt=0,aim_mode=0,Tx_state=0;//������function�ּ���aim_mode����������ѡ���Ƿ����
int16_t current_yaw=0,current_pitch=0;
int16_t receive_cnt=0,receive_rcd=0;//���֡��
double bullet_speed=10.0,bullet_speed_adjust=0,yaw_adjust=0,pitch_adjust=0;;//������function�м���bullet_speed_adjust�����������ֶ�У׼

void InitAutoAim()
{
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Rx_enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
	scope_gun.x=0;scope_gun.y=0;scope_gun.z=0;
	enemy_scope.x=0;enemy_scope.y=0;enemy_scope.z=200;
	enemy_gun.x=0;enemy_gun.y=0;enemy_gun.z=200;
	scope_gun.x=0;scope_gun.y=0;scope_gun.z=0;
}

void AutoAimRxEnemyINFO()
{
	if(!find_enemy&&RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
		enemy_scope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.x=(enemy_scope.x>100)?(enemy_scope.x-200):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>100)?(enemy_scope.y-200):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Rx_enemy_INFO,8);
}

extern RC_Ctl_t RC_CtrlData;
void CANTxCurrentINFO()
{
	extern int16_t channelrcol;
	uint8_t stir_state= (channelrcol>0) ? (channelrcol * 7 / 661) : 0 ;
	
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x300;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	switch(WorkState)
	{
		case STOP_STATE: hcan1.pTxMsg->Data[0] = 0xff; break;
		case PREPARE_STATE: hcan1.pTxMsg->Data[0] = 0x00; break;
		case NORMAL_STATE: hcan1.pTxMsg->Data[0] = 0x01; break;
		case ADDITIONAL_STATE_ONE: hcan1.pTxMsg->Data[0] = 0x02; break;
		case ADDITIONAL_STATE_TWO: hcan1.pTxMsg->Data[0] = 0x03; break;
	}
	switch(inputmode)
	{
		case REMOTE_INPUT: hcan1.pTxMsg->Data[1] = 0x01; break;
		case KEY_MOUSE_INPUT: hcan1.pTxMsg->Data[1] = 0x02; break;
		case STOP: hcan1.pTxMsg->Data[1] = 0x03; break;
	}
	hcan1.pTxMsg->Data[2] = (uint8_t)(stir_state);
	hcan1.pTxMsg->Data[3] = (uint8_t)RC_CtrlData.mouse.press_l;
	hcan1.pTxMsg->Data[4] = (uint8_t)RC_CtrlData.mouse.press_r;
	hcan1.pTxMsg->Data[5] = (uint8_t)(RC_CtrlData.key.v & 0xff);
	hcan1.pTxMsg->Data[6] = (uint8_t)( (RC_CtrlData.key.v>>8) & 0xff);
	hcan1.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 3)
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
		#ifdef CAN11
			can1_type = 1;
		#else
		#ifdef CAN12
			can1_type = 2;
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

void enemyINFOProcess()
{
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	aim.y=-atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-yaw_adjust;
	if(aim_mode==0)
	{
		aim.p=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+pitch_adjust;
	}
//	else if(aim_mode==1)
//	{
//		aim.p=-atan(k_aim -
//		sqrt( k_aim*k_aim - 2*k_aim*((enemy_gun.y*cos(GMP_ANGLE)+enemy_gun.z*sin(GMP_ANGLE))/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE))) - 1))
//		/const_pi*180.0- (GM_PITCH_ZERO - GMP.RxMsg6623.angle) * 360.0 / 8192.0;
//	}
	//aim.y=-cos(4.0/200.0)/const_pi*180.0/100.0;
//	aim.y=-cos(4/200)/const_pi*180.0/100;
//	aim.p=0;
}

void autoAimGMCTRL()
{
	if(find_enemy)
	{
		//enemyINFOProcess();
		if(aim_cnt<2)
		{
			GMY.TargetAngle+=aim.y/2;
			GMP.TargetAngle+=aim.p/2;
			aim_cnt++;
		}
		else
		{
			find_enemy=0;
			aim_cnt=0;
		}
		//���֡��
		if(receive_cnt==0)
			auto_counter=1000;
		if(auto_counter==0)
		{
			receive_rcd=receive_cnt;
			receive_cnt=0;
			auto_counter=1000;
		}
	}
}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
