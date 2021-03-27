/**
  ******************************************************************************
  * @file     Servo.h
  * @author   LJY 2250017028@qq.com
  * @brief    舵机控制模块
  * @version  1.2
  * @date     2021-03-02
  * @editby   Anthracene

  ==============================================================================
                     ##### How to use this conf #####
  ==============================================================================

  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */

#ifndef _SERVO_H_
#define _SERVO_H_

#include "main.h"

template<int CCR_MIN, int CCR_MAX, int MAX_DEG>
class Servo_Classdef
{
public:
	Servo_Classdef(TIM_HandleTypeDef *htimer,uint32_t _channel)
	:servoPwmTimer(htimer), channel(_channel), 
	DEG2CCR((float)(CCR_MAX - CCR_MIN) / (float)MAX_DEG), CCR_0_DEG((CCR_MAX + CCR_MIN) / 2)
	{}
	
	/* 设置初始角度 */  
	void Init(float int_deg)
	{	
		setDegree(int_deg);
	}
	
	/* 设置角度 */
	void setDegree(float degree)
	{
		static uint16_t CCR = 0;
		
		/* 更新角度 */
		degree_now = degree;
		
		CCR = degree * DEG2CCR + CCR_0_DEG;
		/* 输入限幅 */	
		if(CCR > CCR_MAX)
			CCR = CCR_MAX;
		else if(CCR < CCR_MIN)
			CCR = CCR_MIN;
		__HAL_TIM_SetCompare(this->servoPwmTimer, this->channel, CCR);
	}
	
	/* 重置角度 */
	void Reset(void){ __HAL_TIM_SetCompare(this->servoPwmTimer, this->channel, CCR_0_DEG); }
	
	/* 获取当前角度 */
	float getDeg(void){ return degree_now; }
private:
	TIM_HandleTypeDef *servoPwmTimer;
	uint32_t channel;
	const float DEG2CCR;
	const int CCR_0_DEG;
	float degree_now;
};


#endif
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
