/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    array.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPL calculate functions.
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

#ifndef __ARRAY_H
#define __ARRAY_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include <algorithm>
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
/**
 * @brief Array 数组类
	* @note 模仿std::array的类结构
 */
template <typename T, int Size>
class Array
{
	public:
		~Array(){}
		
		float& operator[](int idx) { return data[idx]; }
		
		T* begin()
		{
			return data;
		}
		
		
		
		T* end()
		{
			return data + Size;
		}
		
		T data[Size];
};


/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/


#endif	

#endif	

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
