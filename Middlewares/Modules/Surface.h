/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Surface.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for surface interfaces.
  *         
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

#ifndef _SURFACE_H_
#define _SURFACE_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "System_DataPool.h"
#include "PID.h"
#include "Servo.h" 
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/


typedef struct
{
  myPID Angle;
	myPID AngularVel;
}Surface_Channel_t;
	
/* 机身姿态的PID控制结构体 */
typedef struct
{
	Surface_Channel_t Roll;
	Surface_Channel_t Pitch;
	Surface_Channel_t Yaw;
}Surface_t;
	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Surface
{
public:
	Surface() : Left_Aileron(&htim4,TIM_CHANNEL_1), Right_Aileron(&htim4,TIM_CHANNEL_3){}
	~Surface(){}
	void PID_Init(void);
	void PID_Update_Angle(float current_roll, float current_pitch, float current_yaw);
	void PID_Update_AngularVel(float current_roll_vel, float current_pitch_vel, float current_yaw_vel);
	void Set_Degree(float las_deg, float ras_deg);
	void Degree_Control(float roll_out, float pitch_out, float yaw_out);
	void Get_Deg(float& las_deg, float& ras_deg);
private:
	/* 机身姿态的控制变量 */
	Surface_t controller;

	/* 定义机体所具有的三个舵机 */
	Servo_Classdef<900, 2100, 110> Left_Aileron;
	Servo_Classdef<900, 2100, 110> Right_Aileron;
};

/* Exported variables --------------------------------------------------------*/
extern Surface surface;
/* Exported function declarations --------------------------------------------*/

#endif

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
