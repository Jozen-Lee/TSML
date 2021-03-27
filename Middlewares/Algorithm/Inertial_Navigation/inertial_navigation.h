/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    inertial_navigation.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for inertial navigation.
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

#ifndef INERTIAL_NAVIGATION_H
#define INERTIAL_NAVIGATION_H

#include "mpl_cal.h"

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "stm32f4xx.h"
/* Private define ------------------------------------------------------------*/

/* 更新状态的时间间隔(ms) */
#define UPDATE_TIME 	1	
	 
/* 积分次数 */	 
#define	 	COUNT_ACC 	3
#define 	COUNT_SPEED 4		

/* Private include -----------------------------------------------------------*/
#include <algorithm>
#include "integral_algorithm.h"
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
typedef struct 
{
	float speed[3];	// 全局下x,y,z轴的速度
	float dis[3];		//全局下x,y,z轴的位移量
	float height;		//全局坐标下的高度
} g_status_t;	

typedef struct
{
	/* 存储数据 */
	float acc[3][COUNT_ACC];
	float speed[3][COUNT_SPEED];
	
	/* 数据读取的时间点数组 */
	float time_acc[COUNT_ACC];
	float time_speed[COUNT_SPEED];	
	
	/* 计数器 */
	uint8_t count_i;
	uint8_t count_j;
} record_t;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Navigation
{
public:
	Navigation() : integ(Lagrange, Parabola, 15)
	{
	 /* 状态量初始化 */
	 for(int i = 0; i < 3; i ++) 
	 {
		 status.dis[i] = 0;
		 status.speed[i] = 0;
	 }
	 status.height = 0;
	 
	 /* 记录量初始化 */
	 record.count_i = 0;
	 record.count_j = 0;
	 for(int i = 0; i < COUNT_ACC; i ++) 
	 {
		 record.time_acc[i] = (i * UPDATE_TIME) * 0.001;
		 for(int j = 0; j < 3; j++) record.acc[j][i] = 0;
	 }
	 
	 for(int i = 0; i < COUNT_SPEED; i ++) 
	 {
		 record.time_speed[i] = COUNT_ACC * i * UPDATE_TIME * 0.001;
		 for(int j = 0; j < 3; j++) record.speed[j][i] = 0;
	 } 	
	}
	Navigation(Interpolation a, Integral b , uint16_t ac, 
							float sppedx, float sppedy, float sppedz,
								float disx, float disy, float disz) : integ(a, b, ac)
	{
		/* 状态初始化 */
		status.speed[0] = sppedx;	 
		status.speed[1] = sppedy;	
		status.speed[2] = sppedz;	

		status.dis[0] = disx;	 
		status.dis[1] = disy;	
		status.dis[2] = disz;	
		
		status.height = disz;
		
		/* 记录量初始化 */
		 record.count_i = 0;
		 record.count_j = 0;
		 for(int i = 0; i < COUNT_ACC; i ++) 
		 {
			 record.time_acc[i] = (i * UPDATE_TIME) * 0.001;
			 for(int j = 0; j < 3; j++) record.acc[j][i] = 0;
		 }
		 
		 for(int i = 0; i < COUNT_SPEED; i ++) 
		 {
			 record.time_speed[i] = COUNT_ACC * i * UPDATE_TIME * 0.001;
			 for(int j = 0; j < 3; j++) record.speed[j][i] = 0;
		 } 		
	}								
	~Navigation(){};
	void Update(const float* acc_x,const float* acc_y,const float* acc_z);
	g_status_t status;	// 存储当前状态的结构体
private:
	
	/* 获取速度 */
	void Get_Speed(const float* acc_x, const float* acc_y, const float* acc_z, const float* t, uint16_t n);

	/* 获取位移量 */
	void Get_Distance(const float* speed_x, const float* speed_y, const float* speed_z, const float* t, uint16_t n);								
	Integral_Lib integ;			// 积分算法
	record_t record;				// 记录过程量的结构体
};

/* Exported variables --------------------------------------------------------*/
extern Navigation nav;

/* Exported function declarations --------------------------------------------*/
void Navigation_Init(void);

#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
