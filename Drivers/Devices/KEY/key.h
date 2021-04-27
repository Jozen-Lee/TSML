/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    key.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for key.
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

#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif	

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/

/* 按键模式 */
typedef enum
{
	SINGLE = 0,
	CONTINUES
} KEY_MODE;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class KEY
{
public:
	KEY(GPIO_TypeDef* GPIO_Port, uint32_t GPIO_PIN, GPIO_PinState _PinState)
	: KEY_GPIO_Port(GPIO_Port), KEY_GPIO_PIN(GPIO_PIN), DefaultState(_PinState){}
	uint8_t Scan(KEY_MODE mode);
private:
	/* 按键对应的IO口以及非按下的状态 */
	GPIO_TypeDef* KEY_GPIO_Port; 
	uint32_t KEY_GPIO_PIN;
	GPIO_PinState DefaultState;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
#endif	/* __cplusplus */

#endif	/* define */
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
