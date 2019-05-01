/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;
uint16_t counter = 0;
double rotate_speed = 0;
MusicNote SuperMario[] = {
	{H3, 100}, {0, 50}, 
	{H3, 250}, {0, 50}, 
	{H3, 100}, {0, 50}, 
	{0, 150},
	{H1, 100}, {0, 50},  
	{H3, 250}, {0, 50},
	{H5, 250}, {0, 50},
	{0, 300},
	{M5, 250}, {0, 50},
	{0, 300},
	{H1, 250}, {0, 50}
};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:				//准备模式
		{
			//if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 1000) prepare_time++;	
			if(prepare_time >= 1000 && imu.InitFinish == 1 && isCan11FirstRx == 1 && isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1)//imu初始化完成且所有can电机上电完成后进入正常模式
			{
				//playMusicSuperMario();
				CMRotatePID.Reset(&CMRotatePID);
				WorkState = NORMAL_STATE;
				prepare_time = 0;
			}
			for(int i=0;i<8;i++) 
			{
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			FunctionTaskInit();
		}break;
		case NORMAL_STATE:				//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}*/
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
			}*/
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			/*
			if (inputmode == REMOTE_INPUT)
			{
				if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
				if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			}*/
		}break;
		case STOP_STATE:				//紧急停止
		{
			for(int i=0;i<8;i++) 
			{
				if((can1[i]==&FRICL || can1[i]==&FRICR)&&(can2[i]==&CMFL || can2[i]==&CMFR || can2[i]==&CMBL || can2[i]==&CMBR))
				{
					can1[i]->FirstEnter=1;
					can1[i]->lastRead=0;
					can1[i]->RealAngle=0;
					can1[i]->TargetAngle=0;
					can1[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can1[i]->Handle)(can1[i]);
					can2[i]->FirstEnter=1;
					can2[i]->lastRead=0;
					can2[i]->RealAngle=0;
					can2[i]->TargetAngle=0;
					can2[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can2[i]->Handle)(can2[i]);
				}
				else if(can1[i]==&FRICL || can1[i]==&FRICR)
				{
					can1[i]->FirstEnter=1;
					can1[i]->lastRead=0;
					can1[i]->RealAngle=0;
					can1[i]->TargetAngle=0;
					can1[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can1[i]->Handle)(can1[i]);
					InitMotor(can2[i]);
				}
				else if(can2[i]==&CMFL || can2[i]==&CMFR || can2[i]==&CMBL || can2[i]==&CMBR)
				{
					can2[i]->FirstEnter=1;
					can2[i]->lastRead=0;
					can2[i]->RealAngle=0;
					can2[i]->TargetAngle=0;
					can2[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));
					(can2[i]->Handle)(can2[i]);
					InitMotor(can1[i]);
				}
				else
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				GMYReseted=0;
				GMPReseted=0;
				FunctionTaskInit();
			}
		}break;
		default: break;
	}
}
void ControlRotate(void)
{	
	#ifdef USE_CHASSIS_FOLLOW
	switch (ChassisTwistState)
	{
		case 1: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle; break;
		case 2: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO + 1024) * 360 / 8192.0f - ChassisTwistGapAngle; break;
		default: ChassisSpeedRef.rotate_ref=(GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle; break;
	}
	NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
	#endif
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = ChassisSpeedRef.rotate_ref;
	CMRotatePID.Calc(&CMRotatePID);
	if(ChassisTwistState) MINMAX(CMRotatePID.output,-50,50);
	//rotate_speed = CMRotatePID.output * 16 + ChassisSpeedRef.forward_back_ref * 0.01 + ChassisSpeedRef.left_right_ref * 0.01;
	rotate_speed = CMRotatePID.output * 30;
}

void Chassis_Data_Decoding()
{
	if(!chassis_lock)
	{
		ControlRotate();

		CMFL.TargetAngle = (  (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ rotate_speed)*12;
		CMFR.TargetAngle = (- (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ rotate_speed)*12;
		CMBL.TargetAngle = (  (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							- ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ rotate_speed)*12;
		CMBR.TargetAngle = (- (ChassisSpeedRef.forward_back_ref + fabs(rotate_speed*0.06)) *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)-sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							- ChassisSpeedRef.left_right_ref *(cos((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f)+sin((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 6.28 / 8192.0f))
							+ rotate_speed)*12;
	}
	else
	{
		CMFL.TargetAngle = 0;
		CMFR.TargetAngle = 0;
		CMBL.TargetAngle = 0;
		CMBR.TargetAngle = 0;
	}
	
//	rotate_speed-=0.1;
//	CMFL.TargetAngle = (  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
//	CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
//	CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
//	CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
}

//主控制循环
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	#ifdef NO_RC_MODE
	CANTxINFO();
	#endif
	
	if(WorkState > 0)
	{
		Chassis_Data_Decoding();
		
		#ifndef USE_CHASSIS_ADJUST
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		#else
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		if(CAP_STATE_RELEASE){chassisMixingPID(12,0,2, 4,0.01,0);}
		else{chassisMixingPID(12,0,2, 3,0.4,0);}
//		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		#endif
		
		OptionalFunction();
		
		#ifdef CAN11
		setCAN11();
		#endif
		#ifdef CAN12
		setCAN12();
		#endif
		#ifdef CAN21
		setCAN21();
		#endif
		#ifdef CAN22
		setCAN22();
		#endif
	}
}

void heatCalc()//1ms
{
	if(fakeHeat0 >= cooldown0/1000) fakeHeat0 -= cooldown0/1000;
	else fakeHeat0 = 0;
	if(fakeHeat1 >= cooldown1/1000) fakeHeat1 -= cooldown1/1000;
	else fakeHeat1 = 0;
}


//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//1ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		//imu解算
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		//主循环在时间中断中启动
		controlLoop();
		static uint8_t cap_time_cnt = 0;
		cap_time_cnt += 1;
		if (cap_time_cnt >= 2){
		   Cap_Run();
		   cap_time_cnt = 0;
		}

		//自瞄数据解算
//		static int aim_cnt = 0;
//		aim_cnt++;
//		if(aim_cnt == 3)
//		{
//			EnemyINFOProcess();
//			aim_cnt=0;
//		}
		EnemyINFOProcess();
		AutoAimGMCTRL();
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if(auto_counter_stir > 0) auto_counter_stir--;
		if(auto_counter_fps > 0) auto_counter_fps--;
		if(auto_counter_heat1 > 0) auto_counter_heat1--;
		
		if (rx_free == 1 && tx_free == 1)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				rx_free = 0;
				while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
				if (counter == 10) 
				{
					tx_free = 0;
//					Send_User_Data(); 
					Referee_Transmit_UserData();
					counter = 0;
				}
				else counter++;				
					rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame == 0) 
				{
				   WorkState = PREPARE_STATE;
				   HAL_UART_AbortReceive(&RC_UART);
				   while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
  				 rc_cnt = 0;
				   rc_first_frame = 1;
				}
			}
			rc_update = 0;
		}
	}
	else if (htim->Instance == htim10.Instance)  //10ms，处理上位机数据，优先级不高
	{
		#ifdef DEBUG_MODE
		//zykProcessData();
		#endif
	}
}
