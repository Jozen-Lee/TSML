/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    my_filters.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for Filters.
  * @date    2021-02-02
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-02  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
													How to use this library 
  ==============================================================================
    @note
			- 低通滤波 
				-# LowPassFilter LF(trust);  
				-# 返回LF.f(num) 或者 LF << (in)  LF >> out 输出的时候才运算

			- 中值滤波 
				-# MedianFilter<Length> MDF;  
				-# 返回MDF.f(num) 或者 MDF << (in) MDF >> out 输出的时候才运算 

			- 均值滤波 
				-# MeanFilter<Length> MF;  
				-# 返回MF.f(num) 或者MF << (in)  MF >> out 输出的时候才运算 

  	@warning 
			- 低通滤波部分trust (0,1) ！！！注意超过不报错   中值滤波 均值滤波(长度[1,100])
			- 注意在其他文件中引入该库的头文件时,不能放在extern 'C'中
			- Standard C++11 required! 
  
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

 /* Includes ------------------------------------------------------------------*/
#include "my_filters.h"

/* Function prototypes -------------------------------------------------------*/

/* 低通滤波 */
void LowPassFilter::in(float num)							
{
	last_num = now_num;
	now_num = num;
}

float LowPassFilter::out()							
{
	return (now_num*Trust + last_num * (1 - Trust));
}


	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
