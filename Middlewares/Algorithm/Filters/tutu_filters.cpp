/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    tutu_filters.cpp
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
			- ��ͨ�˲� 
				-# LowPassFilter LF(trust);  
				-# ����LF.f(num) ���� LF << (in)  LF >> out �����ʱ�������

			- ��ֵ�˲� 
				-# MedianFilter<Length> MDF;  
				-# ����MDF.f(num) ���� MDF << (in) MDF >> out �����ʱ������� 

			- ��ֵ�˲� 
				-# MeanFilter<Length> MF;  
				-# ����MF.f(num) ����MF << (in)  MF >> out �����ʱ������� 

  	@warning 
			- ��ͨ�˲�����trust (0,1) ������ע�ⳬ��������   ��ֵ�˲� ��ֵ�˲�(����[1,100])
			- ע���������ļ�������ÿ��ͷ�ļ�ʱ,���ܷ���extern 'C'��
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

/* ��ͨ�˲� */
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
