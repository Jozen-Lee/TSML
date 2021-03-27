/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    Controler.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for Missile controler.
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

#ifndef __CONTROLER_H
#define __CONTROLER_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/
#include "imu.h"
#include "Surface.h"
#include "Vision.h"

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Controler
{
public:
	uint8_t Control(void);
	
private:		
	uint8_t Climb(void);
	uint8_t Dive(void);
	uint8_t Turn(void);
	float action_flag = 3.0f;				// ����״̬�л��ı�־λ
	float time_stamp = 0;						// ��ǰ��ʱ���ǩ
	float record_reg[2];						// ������¼�����������ԭʼ�Ƕ�
	const float INIT_DEG = 20.0f;		// ����ĳ�ʼ�Ƕ�
	const float CENTER_X = COL / 2;	// �����X������
	const float CENTER_Y = ROW / 2;	// �����Y������
};
/* Exported variables --------------------------------------------------------*/
extern Controler controler;

/* Exported function declarations --------------------------------------------*/

#endif	

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
