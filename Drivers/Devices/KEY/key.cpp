/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    key.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for key.
  * @date    2021-03-15
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-03-15  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 创建实例
		 
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
#include "key.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**@brief 按键扫描
*@param[in]	mode	 SINGLE  不支持连按
									 CONTINUES 支持连按
	*@return 0 无按键按下
					 1 按键按下
*/
uint8_t KEY::Scan(KEY_MODE mode)
{
	/* 按键松开标志 */
	static uint8_t key_up=1;     
	
	/* 按键状态 */
	GPIO_PinState sta; 
	
	/* 支持连按 */
	if(mode == CONTINUES) key_up=1; 
	
	/* 按键检测 */
	sta = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_GPIO_PIN);
	if(key_up&&(sta != DefaultState))
	{
			HAL_Delay(150);
			key_up=0;
			return 1;  // 按键按下     
	}
	else 
	{
		key_up=1;
		return 0;	// 无按键按下
	}
	   	
}
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
