/**
  ******************************************************************************
  * File Name          : includes.h
  * Description        : 统一包含文件
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __INCLUDES_H
#define __INCLUDES_H

#define HERO_MAIN

#define NO_RC_MODE
#define USE_AUTOAIM
#define USE_IMU
//#define USE_CHASSIS_FOLLOW
#define USE_HEAT_LIMIT_HERO_MAIN
//#define USE_POWERLIMITATION
#define USE_CHASSIS_ADJUST
#define JUDGE_RM_2018

//#define CAN11
//#define CAN12
#define CAN21
//#define CAN22

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "adc.h"
#include "math.h"
#include "dac.h"

#include "AuxDevice.h"
#include "RemoteTask.h"
#include "FunctionTask.h"
#include "pid_regulator.h"
#include "CANTask.h"
#include "MotorTask.h"
#include "ControlTask.h"
#include "drivers_ramp.h"
#include "AutoAimTask.h"
#include "JudgeTask.h"
#include "UpperTask.h"
//#include "CapControlTask.h"
#include "Cap2ControlTask.h"
#include "PowerLimitationTask.h"
#include "GyroReadTask.h"
#include "bsp_imu.h"

extern int16_t global_catch;		//用于检测一个其他文档里，不值得设置全局变量，但是临时需要读取的数据
//#include "visualscope.h"


#endif /* __INCLUDES_H */
