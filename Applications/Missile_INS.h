/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    Missile_INS.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for inertial navigation system.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */ 

#ifndef __MISSILE_INS_H
#define __MISSILE_INS_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "System_DataPool.h"

/* Private define ------------------------------------------------------------*/
			
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
	 
/* Exported variables --------------------------------------------------------*/
extern TaskHandle_t Location_Handle;
	 
/* Exported function declarations --------------------------------------------*/
void Missile_INS_Task_Init(void);

#endif
#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
