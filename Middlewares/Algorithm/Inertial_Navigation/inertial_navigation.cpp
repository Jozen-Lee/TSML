/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    inertial_navigation.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for inertial navigation.
  * @date    2021-02-01
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-01  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 创建实例对象(可以带参或不带参创建)
				Navigation nav;
         
      -# 更新状态
				nav.Update(&acc_x,const &acc_y,const &acc_z);
    @warning	
      -# 需要`integral_algorithm`支持。
	  
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
#include "inertial_navigation.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Navigation nav;	// 实例对象
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief 获取速度  
 *  @param[in] 	cal 										算法接口
 *  @param[out] sta											用于记录的状态量
 *  @param[in] 	acc_x, acc_y, acc_z 		用于积分的三轴加速度的数值
 *  @param[in] 	t	 											积分时间数组,大小与加速度数组相匹配
 *  @param[in] 	n	 											加速度数组的大小 
 *  @return void
 */
 void Navigation::Get_Speed(const float* acc_x, const float* acc_y, const float* acc_z, const float* t, uint16_t n)
 {
	 status.speed[0] += integ.I(t, acc_x, 0, n);
	 status.speed[1] += integ.I(t, acc_y, 0, n);
	 status.speed[2] += integ.I(t, acc_z, 0, n);
 }
 
/**
 *  @brief 获取位移量  
 *  @param[in] 	cal 										算法接口
 *  @param[out] sta											用于记录的状态量
 *  @param[in] 	acc_x, acc_y, acc_z 		用于积分的三轴加速度的数值
 *  @param[in] 	speed_x,speed_y,speed_z 用于积分的三轴速度的数值
 *  @param[in] 	t	 											积分时间数组,大小与速度数组相匹配
 *  @param[in] 	n	 											速度数组的大小 
 *  @return void
 */
 void Navigation::Get_Distance(const float* speed_x, const float* speed_y, const float* speed_z, const float* t, uint16_t n)
 {
	 status.dis[0] += integ.I(t, speed_x, 0, n);
	 status.dis[1] += integ.I(t, speed_y, 0, n);
	 status.dis[2] += integ.I(t, speed_z, 0, n);
 } 
 
/**
 *  @brief 更新状态 
 *  @param[in] acc_x, acc_y, acc_z 全局坐标轴下的三轴加速度
 *  @return void
 */ 
 void Navigation::Update(const float* acc_x,const float* acc_y,const float* acc_z)
 { 
	 
	 /* 记录加速度 */
	 record.acc[0][record.count_i] = *acc_x;
	 record.acc[1][record.count_i] = *acc_y;
	 record.acc[2][record.count_i] = *acc_z;
	 record.count_i = (1 + record.count_i) % COUNT_ACC;
	 
	 if(record.count_i == 0) 
	 {
		 /* 更新速度 */
		 Get_Speed(record.acc[0], record.acc[1], record.acc[2], record.time_acc, COUNT_ACC);
		 
		 /* 记录速度 */
		 record.speed[0][record.count_j] = status.speed[0];
		 record.speed[1][record.count_j] = status.speed[1];
		 record.speed[2][record.count_j] = status.speed[2];
		 record.count_j = (1 + record.count_j) % COUNT_SPEED;

		 if(record.count_j == 0) 
		 {
			 /* 更新位移量 */
			 Get_Distance(record.speed[0], record.speed[1], record.speed[2], record.time_speed, COUNT_SPEED);
		 }
	 }
	 
 }
  
 
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
