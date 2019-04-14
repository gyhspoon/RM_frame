/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

KeyboardMode_e KeyboardMode = NO_CHANGE;
KeyboardMode_e LastKeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef;
uint16_t LastKey=0;

int ChassisTwistGapAngle = 0;

int32_t auto_counter=0;		//用于准确延时的完成某事件
int32_t auto_counter_stir=0;
int32_t auto_counter_shoot=0;

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;
uint8_t SuperCTestMode = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;
uint8_t cdflag0 = 0;
uint8_t burst = 0;
uint16_t allowBullet0 = 0;
uint16_t FricSpeedLeft = 4000;
uint16_t FricSpeedRight = 4000;
uint8_t chassis_lock=0;

//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
}

void OptionalFunction()
{
	//if(Cap_Get_Cap_State()!=CAP_STATE_RELEASE)
	#ifdef USE_POWERLIMITATION
	PowerLimitation();
	#endif
}

void Limit_and_Synchronization()
{
	
}
//******************
//遥控器模式功能编写
//******************
int stirState=1,stirDirection=1,gateStep=1;

void RemoteControlProcess(Remote *rc)
{
	static WorkState_e LastState = NORMAL_STATE;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	
	if(WorkState == NORMAL_STATE)
	{
		//for debug SuperC
		if(LastState!= WorkState)
		{
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		ShootState = 0;
		FRICL.TargetAngle = 0;
		FRICR.TargetAngle = 0;
		STIR.TargetAngle=0;
		STIR.RealAngle=0;
		GATE.TargetAngle=0;
		GATE.RealAngle=0;
		gateStep=1;	
		aim_mode=0;
		
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		
		ShootState = 1;
		FRICL.TargetAngle = FricSpeedLeft;
		FRICR.TargetAngle = -FricSpeedRight;
		STIR.TargetAngle+=stirDirection;
		aim_mode=0;
		
		if (LastState != WorkState)
		{
			//Cap_State_Switch(CAP_STATE_STOP);
			if(ShootState)
			{
				stirDirection=stirState;
			}
		}
		
		//if(SuperCTestMode==1) ChassisTwistState = 1;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		if(LastState!= WorkState)
		{
			Cap_State_Switch(CAP_STATE_RELEASE);
		}
		
		if(Cap_Get_Cap_Voltage() > 10 && release_cnt >= 50)
		{
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF*1.5f;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF;
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		}
		/*
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		*/
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		
		ShootState = 1;
		STIR.TargetAngle+=stirDirection;
		FRICL.TargetAngle = FricSpeedLeft;
		FRICR.TargetAngle = -FricSpeedRight;
		aim_mode=0;
		if(LastState!= WorkState && ShootState)
		{
			ShootOneBullet();
			stirDirection=stirState;
		}

		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);	
	}
	OnePush(STIR.RxMsgC6x0.moment>5000 || STIR.RxMsgC6x0.moment<-5000,
	{
		STIR.TargetAngle-=stirDirection*15;
		stirDirection=0;
		stirState=-stirState;
	});
	//FreshSuperCState();
	LED_Show_SuperCap_Voltage(1);
	if(ChassisTwistState)
	{
		LJHTwist();
	}
	else ChassisDeTwist();
	AutoAimGMCTRL();
	LastState = WorkState;
	Limit_and_Synchronization();
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);


//------------
//原键鼠模式重分配
//@尹云鹏   controlTask changed
//左拨杆下才为真正的键鼠模式，上、中两档位可编辑
//------------
extern uint8_t sendfinish;  extern int32_t cps[4][4000];//用于串口发送功率数据@唐欣阳

void MouseKeyControlProcess(Mouse *mouse, Key *key,Remote *rc)
{	
	if(WorkState <= 0) return;
	if(WorkState == NORMAL_STATE)
	{
		//max=660
		channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		aim_mode=1;
		AutoAimGMCTRL();
		//chassis_lock=1;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		//mpu_offset_call();
	}
	//键鼠模式
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		MINMAX(mouse->x, -150, 150); 
		MINMAX(mouse->y, -150, 150); 
		
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
		GMP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
		#endif
		
		MouseModeFSM(mouse);
		
		switch(MouseRMode)
		{
			case SHORT_CLICK:
			{
				if(ShootState)
				{

				}
			}break;
			case LONG_CLICK:
			{
				if(ShootState)
				{
					
				}
			}break;
			default: break;
		}
		
		switch (MouseLMode)
		{
			case SHORT_CLICK:
			{
				
			}break;
			case LONG_CLICK:
			{
				
			}
			default: break;
		}

		KeyboardModeFSM(key);
		
		switch (KeyboardMode)																																								/******************************/
		{																																																		/*														*/
			case SHIFT_CTRL:																																									/*	shift_ctrl：无视热量限制	*/
			{																																																	/*														*/
				burst = 1;																																											/*														*/
				break;																																													/*														*/
			}																																																	/*														*/
			case CTRL:																																												/*	ctrl: 低速								*/
			{																																																	/*														*/
				OnePushF(key->v & KEY_F ,{aim_mode = (aim_mode!=2) ? 2 : 0;});																	/*	ctrl_f: 打符							*/
				burst = 0;																																											/*														*/
				break;																																													/*														*/
			}																																																	/*														*/
			case SHIFT:																																												/*	shift: 高速（超级电容）		*/
			{																																																	/*														*/
				if(key->v & KEY_B)																																							/*	shift_b：关摩擦轮，关激光	*/
				{																																																/*	-													*/
					FRICL.TargetAngle = 0;																																				/*	-													*/
					FRICR.TargetAngle = 0;																																				/*	-													*/
					HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);																/*	-													*/
					ShootState=0;																																									/*	-													*/
				}																																																/*														*/
				burst = 0;																																											/*														*/
				break;																																													/*														*/
			}																																																	/*														*/
			case NO_CHANGE:																																										/*	normal										*/
			{																																																	/*														*/
				if(key->v & KEY_B)																																							/*	b: 开摩擦轮，开激光				*/
				{																																																/*	-													*/
					FRICL.TargetAngle = FricSpeedLeft;																														/*	-													*/
					FRICR.TargetAngle = -FricSpeedRight;																													/*	-													*/
					HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);																	/*	-													*/
					ShootState=1;																																									/*	-													*/
				}																																																/*														*/
				burst = 0;																																											/*														*/
				if(key->v & KEY_R && !(LastKey & KEY_R)) chassis_lock = (chassis_lock != 1) ? 1 : 0;						/*	r: 底盘锁定/解锁					*/
				if(key->v & KEY_Q && !(LastKey & KEY_Q)) ChassisTwistState = (ChassisTwistState!=1) ? 1 : 0;		/*	q: 正常扭腰								*/
				if(key->v & KEY_E && !(LastKey & KEY_E)) ChassisTwistState = (ChassisTwistState!=2) ? 2 : 0;		/*	e: 45度扭腰								*/
				if(key->v & KEY_F) aim_mode = 1;																																/*	f: 按住f自瞄							*/
				else aim_mode = 0;																																							/*														*/
			}																																																	/*														*/
		}																																																		/******************************/
		
		//*********************************CM Movement Process******************************************
		if(key->v & KEY_W)  		//key: w
			ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
		else if(key->v & KEY_S) 	//key: s
			ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & KEY_D)  		//key: d
			ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
		else if(key->v & KEY_A) 	//key: a
			ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
		
		LastKey=key->v;
		//**********************************************************************************************
		
		//***********************************防卡弹*********************************
		if(ShootState) STIR.TargetAngle+=stirDirection;
		OnePush(STIR.RxMsgC6x0.moment>5000 || STIR.RxMsgC6x0.moment<-5000,{
			STIR.TargetAngle-=stirDirection*15;
			stirDirection=0;
			stirState=-stirState;
		});
		//**************************************************************************
		if(ChassisTwistState)
		{
			LJHTwist();
		}
		else ChassisDeTwist();
		AutoAimGMCTRL();
		LED_Show_SuperCap_Voltage(1);
		//FreshSuperCState();
		//Limit_and_Synchronization();
	}
}

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=SHIFT_CTRL;
		if(LastKeyboardMode != KeyboardMode)
		{
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		//速度控制
		if(Cap_Get_Cap_Voltage() > 10)
		{
			KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		}
		KeyboardMode=SHIFT;
		//电容状态控制
		if(LastKeyboardMode != KeyboardMode && Cap_Get_Cap_Voltage() > 8)
		{
			Cap_State_Switch(CAP_STATE_RELEASE);
		}
		else if(Cap_Get_Cap_Voltage() <= 10)
		{
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=CTRL;
		if(LastKeyboardMode != KeyboardMode)
		{
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}
	}
	else
	{
		KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		KeyboardMode=NO_CHANGE;
		if(LastKeyboardMode != KeyboardMode)
		{
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}
	}	
	LastKeyboardMode=KeyboardMode;
}

void MouseModeFSM(Mouse *mouse)
{
	static uint8_t counterl = 0;
	static uint8_t counterr = 0;
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			counterl++;
			if(mouse->press_l == 0)
			{
				MouseLMode = NO_CLICK;
				counterl = 0;
			}
			else if(counterl>=50)
			{
				MouseLMode = LONG_CLICK;
				counterl = 0;
			}
			else
			{
				MouseLMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_l==0)
			{
				MouseLMode = NO_CLICK;
			}
			else
			{
				MouseLMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_l)
			{
				if(ShootState)
				{
					ShootOneBullet();
					stirDirection=stirState;
				}
				MouseLMode = SHORT_CLICK;
			}
		}break;
	}
	
	switch (MouseRMode)
	{
		case SHORT_CLICK:
		{
			counterr++;
			if(mouse->press_r == 0)
			{
				MouseRMode = NO_CLICK;
				counterr = 0;
			}
			else if(counterr>=50)
			{
				MouseRMode = LONG_CLICK;
				counterr = 0;
			}
			else
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_r==0)
			{
				MouseRMode = NO_CLICK;
			}
			else
			{
				MouseRMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_r)
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
	}
}

//用于遥控器模式下超级电容测试模式的控制
void FreshSuperCState(void)
{
//	static uint8_t counter = 0;
//	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
//	{
//		counter++;
//		if(counter==40)
//		{
//			SuperCTestMode = (SuperCTestMode==1)?0:1;
//		}
//	}
//	else{
//		counter = 0;
//	}
	if(SuperCTestMode==1)
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin 
                          |LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin, GPIO_PIN_RESET);
	if(Control_SuperCap.C_voltage<1100)
	{
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);
	}
	else if(Control_SuperCap.C_voltage<1300)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1500)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1700)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1800)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1900)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2000)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2100)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin, GPIO_PIN_SET);
}

void ChassisTwist(void)
{
	switch (ChassisTwistState)
	{
		case 1:
			switch (ChassisTwistGapAngle)
			{
				case 0:
				{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
				case CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<15)
					{ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;}break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<15)
					{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
				}
				case CHASSIS_TWIST_ANGLE_LIMIT_45:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT_45:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;break;
				}
			}break;
		case 2:
			switch (ChassisTwistGapAngle)
			{
				case 0:
				{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT_45;}break;
				case CHASSIS_TWIST_ANGLE_LIMIT_45:
				{
					if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO + 1024) * 360 / 8192.0f - ChassisTwistGapAngle)<5)
					{ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT_45;}break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT_45:
				{
					if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO + 1024) * 360 / 8192.0f - ChassisTwistGapAngle)<5)
					{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT_45;}break;
				}
				case CHASSIS_TWIST_ANGLE_LIMIT:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT_45;break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT_45;break;
				}
			}break;
	}
}

void ChassisDeTwist(void)
{
	ChassisTwistGapAngle = 0;
}

void LJHTwist(void)
{
	ChassisTwist();
}

void ShootOneBullet(void)
{
	#ifndef USE_HEAT_LIMIT_HERO_MAIN
	GATE.TargetAngle -= 170;
	#else
	cdflag0 = (JUDGE_State == ONLINE && (maxHeat1 - fakeHeat1) < 100 && burst==0) ? 1 : 0;
	if(!cdflag0)
	{
		GATE.TargetAngle -= 170;
		fakeHeat1 += 100;
		auto_counter_shoot=500;
	}
	#endif
	if(find_enemy && aim_mode == 1)
	{
		//GMY.TargetAngle+=(((aim.yaw+aim_rcd.yaw) > 0) ? fabs(imu.wz) : -fabs(imu.wz))*10.0;
	}
}
