/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Controler.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for Missile controler.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-03-11  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
		 
    @warning	
      -# 
	  
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

/* Includes ------------------------------------------------------------------*/
#include "Controler.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Controler controler;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  控制器的主控制函数
  * @return 0 success
  *         1 fail
  */
uint8_t Controler::Control()
{
	/* 俯冲和爬升的状态控制 */
	if(time_stamp < action_flag) Climb();
	else Dive();
	
	/* 转向控制 */
	Turn();
	
	/* 时间标签累加 */
	time_stamp += 0.1f;
	
	return 0;
}

/**
  * @brief  爬升的控制
	* @param void
  * @return 0 success
  *         1 fail
  */
uint8_t Controler::Climb(void)
{
	surface.Set_Degree(INIT_DEG, INIT_DEG);
	return 0;
}

/**
  * @brief  俯冲的控制
	* @param void
  * @return 0 success
  *         1 fail
  */
uint8_t Controler::Dive(void)
{
	float deg;
	deg = (CENTER_Y - vision.center.y) * 40.0f / CENTER_Y;
	surface.Set_Degree(deg, deg);
	return 0;
}

/**
  * @brief  转向的控制
	* @param void
  * @return 0 success
  *         1 fail
  */
uint8_t Controler::Turn(void)
{
	float deg;
	surface.Get_Deg(record_reg[0], record_reg[1]);
	deg = (CENTER_X - vision.center.x) * 20.0f / CENTER_Y;
	surface.Set_Degree(record_reg[0]-deg, record_reg[1]+deg);
	HAL_Delay(100);
	surface.Set_Degree(record_reg[0]+deg, record_reg[1]-deg);
	HAL_Delay(100);
	surface.Set_Degree(record_reg[0], record_reg[1]);
	return 0;
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
