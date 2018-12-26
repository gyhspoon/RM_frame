/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: 自瞄程序
  *Author					: 管易恒
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
uint8_t Rx_enemy_INFO[8],Tx_current_yaw[4],tx_buffer;
uint8_t find_enemy=0,aim_cnt=0,aim_mode=0;//后续在function种加入aim_mode，供操作手选择是否吊射
int16_t current_yaw=0,receive_cnt=0,receive_rcd=0;
double bullet_speed=bullet_speed_big,bullet_speed_adjust=0;//后续在function中加入bullet_speed_adjust，供操作手手动校准

void InitAutoAim()
{
	InitAutoAimUart();
	InitAutoAimSpi();
	scope_gun.x=0;
	scope_gun.y=7.0;
	scope_gun.z=0;
}

void InitAutoAimUart()
{
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Rx_enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
}

void InitAutoAimSpi()
{
	Tx_current_yaw[0]=0x7f;
	Tx_current_yaw[1]=0x00;
	Tx_current_yaw[2]=0x00;
	Tx_current_yaw[3]=0x00;
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,(uint8_t *)&Tx_current_yaw,4,0xff);
}

void AutoAimRxEnemyINFO()
{
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(double)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(double)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
		enemy_scope.z=(double)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.x=(enemy_scope.x>100)?(enemy_scope.x-200):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>100)?(enemy_scope.y-200):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	RX_ENEMY_SIGNAL();
}

void TxCurrentYaw()
{
	current_yaw=GMY.RxMsg6623.angle-GM_YAW_ZERO;
	Tx_current_yaw[2]=(uint8_t)(current_yaw>>8&0xff);
	Tx_current_yaw[3]=(uint8_t)(current_yaw>>0&0xff);
	HAL_SPI_Transmit(&hspi4,(uint8_t *)&Tx_current_yaw,4,0xff);
}

void enemyINFOProcess()
{
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	aim.y=atan(enemy_gun.x/enemy_gun.z)/const_pi*180.0;
	aim.p=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0-(GM_PITCH_ZERO - GMP.RxMsg6623.angle) * 360.0 / 8192.0 ;
	//aim.p=atan(k_aim+((aim_mode)?(1.0):(-1.0))*sqrt(k_aim*k_aim-2.0*k_aim*enemy.y/sqrt(enemy.x*enemy.x+enemy.z*enemy.z)-1.0))/const_pi*180.0-GMP.RealAngle;
}

void autoAimGMCTRL()
{
	if(find_enemy)
	{
		enemyINFOProcess();
		if(aim_cnt<2)
		{
			GMY.TargetAngle-=aim.y/2;
			GMP.TargetAngle+=aim.p/2;
			aim_cnt++;
		}
		else
		{
			find_enemy=0;
			aim_cnt=0;
		}
		//检测帧数
		if(receive_cnt==0)
			auto_counter=1000;
		if(auto_counter==0)
		{
			receive_rcd=receive_cnt;
			receive_cnt=0;
			auto_counter=1000;
		}
	}
	TxCurrentYaw();
}

#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
