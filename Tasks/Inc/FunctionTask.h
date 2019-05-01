/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __FUNCTIONTASK_H
#define __FUNCTIONTASK_H

#include "includes.h"

//遥控常量区
#define RC_CHASSIS_SPEED_REF    	0.85f
#define RC_ROTATE_SPEED_REF 			0.07f
#define RC_GIMBAL_SPEED_REF				0.006f

#define IGNORE_RANGE 					200

//键鼠常量区
#define KEY_W				0x1
#define KEY_S				0x2
#define KEY_A				0x4
#define KEY_D				0x8
#define KEY_SHIFT		0x10
#define KEY_CTRL		0x20
#define KEY_Q				0x40
#define KEY_E				0x80
#define KEY_R				0x100
#define KEY_F				0x200
#define KEY_G				0x400
#define KEY_Z				0x800
#define KEY_X				0x1000
#define KEY_C				0x2000
#define KEY_V				0x4000
#define KEY_B				0x8000

#define NORMAL_FORWARD_BACK_SPEED 	600
#define NORMAL_LEFT_RIGHT_SPEED  		600/1.5f
#define HIGH_FORWARD_BACK_SPEED 		800
#define HIGH_LEFT_RIGHT_SPEED   		800/1.5f
#define LOW_FORWARD_BACK_SPEED 			300
#define LOW_LEFT_RIGHT_SPEED   			300/1.5f

#define CHASSIS_TWIST_ANGLE_LIMIT			45
#define CHASSIS_TWIST_ANGLE_LIMIT_45	12

#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60

#define MOUSE_TO_YAW_ANGLE_INC_FACT		((aim_mode != 0 && find_enemy) ? 0.03f : 0.06f)
#define MOUSE_TO_PITCH_ANGLE_INC_FACT	((aim_mode != 0	&& find_enemy) ? 0.03f : 0.06f)

#define MK_ROTATE_SPEED_REF 			0.90f

#define SHOOTMODE_GM_ADJUST_ANGLE			0.1f
#define SHOOTMODE_PITCH_INIT_BRIDGE		(-30.0f)
#define SHOOTMODE_PITCH_INIT_SUPPLY		(-35.0f)

#ifdef HERO_MAIN

//#define MAXHP1 300
//#define MAXHP2 500
//#define MAXHP3 700

//**heat limitation test**//
#define MAXHP1 2000
#define MAXHP2 3000
#define MAXHP3 4000
//************************//

#define COOLDOWN01 40
#define COOLDOWN02 60
#define COOLDOWN03 80

#define MAXHEAT01 240
#define MAXHEAT02 360
#define MAXHEAT03 480

#define COOLDOWN11 20
#define COOLDOWN12 40
#define COOLDOWN13 60

//#define MAXHEAT11 80
#define MAXHEAT11 150
#define MAXHEAT12 250
#define MAXHEAT13 400

#endif

#define OnePush(button,execution)\
{\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){\
		cache = (button);\
		cnt = 0;\
	}\
	else if(cnt == 5){\
		if(cache) execution;\
		cnt=11;\
	}\
	else if(cnt < 5) cnt++;\
}

#define OnePushZ(button,execution)\
{\
	static uint16_t cacheZ;\
	static uint8_t cntZ=0;\
	if(cacheZ != (button)){\
		cacheZ = (button);\
		cntZ = 0;\
	}\
	else if(cntZ == 5){\
		if(cacheZ) execution;\
		cntZ=11;\
	}\
	else if(cntZ < 5) cntZ++;\
}

#define OnePushF(button,execution)\
{\
	static uint16_t cacheF;\
	static uint8_t cntF=0;\
	if(cacheF != (button)){\
		cacheF = (button);\
		cntF = 0;\
	}\
	else if(cntF == 5){\
		if(cacheF) execution;\
		cntF=11;\
	}\
	else if(cntF < 5) cntF++;\
}

#define OnePushX(button,execution)\
{\
	static uint16_t cacheX;\
	static uint8_t cntX=0;\
	if(cacheX != (button)){\
		cacheX = (button);\
		cntX = 0;\
	}\
	else if(cntX == 5){\
		if(cacheX) execution;\
		cntX=11;\
	}\
	else if(cntX < 5) cntX++;\
}

#define OnePushQ(button,execution)\
{\
	static uint16_t cacheQ;\
	static uint8_t cntQ=0;\
	if(cacheQ != (button)){\
		cacheQ = (button);\
		cntQ = 0;\
	}\
	else if(cntQ == 5){\
		if(cacheQ) execution;\
		cntQ=11;\
	}\
	else if(cntQ < 5) cntQ++;\
}

#define OnePushE(button,execution)\
{\
	static uint16_t cacheE;\
	static uint8_t cntE=0;\
	if(cacheE != (button)){\
		cacheE = (button);\
		cntE = 0;\
	}\
	else if(cntE == 5){\
		if(cacheE) execution;\
		cntE=11;\
	}\
	else if(cntE < 5) cntE++;\
}

#define Delay(TIM,execution)\
{\
	static uint16_t time=TIM;\
	if(!time--)\
	{\
		time = TIM;\
		execution;\
	}\
}

typedef enum
{
	SHIFT,
	CTRL,
	SHIFT_CTRL,
	NO_CHANGE,
}KeyboardMode_e;

typedef enum
{
	SHORT_CLICK,
	LONG_CLICK,
	NO_CLICK,
}MouseMode_e;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

extern ChassisSpeed_Ref_t ChassisSpeedRef; 
extern int ChassisTwistGapAngle;
extern uint8_t ChassisTwistState;
extern int32_t auto_counter;
extern int32_t auto_counter_stir;
extern int32_t auto_counter_heat1;
extern uint8_t chassis_lock;
extern uint8_t chassis_change_forward_back;
extern int16_t chassis_follow_center;

void FunctionTaskInit(void);
void Limit_Position(void);
void OptionalFunction(void);
void FreshSuperCState(void);
void ChassisTwist(void);
void ChassisDeTwist(void);
void ShootOneBullet(void);
void Chassis_forward_back_Handler(void);

#endif /*__FUNCTIONTASK_H*/
