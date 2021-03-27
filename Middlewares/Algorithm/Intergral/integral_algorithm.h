/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    integral_algorithm.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for integral algorithm.
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

#ifndef __INTEGRAL_ALGORITHM_H
#define __INTEGRAL_ALGORITHM_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
/* Private define ------------------------------------------------------------*/
	 									
/* Private include -----------------------------------------------------------*/
#include <algorithm>
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
typedef void (*fitting_func)(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m);	//拟合算法的函数指针
typedef float (*integral_func)(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy);	//积分算法的函数指针

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 拟合算法枚举 */
typedef enum
{
	Lagrange,
	Newton,
	Hermite
}	Interpolation;

/* 积分算法枚举 */
typedef enum
{
	Trapezoid,
	Parabola,
	Vortex,
	Romberg
}	Integral;

typedef struct 
{
	/* 拟合算法 */
	fitting_func fit;
	
	/* 积分算法 */
	integral_func integ;
	
	/* 积分算法精确度 */
	uint16_t ac;
	
} inter_cal_t;

class Integral_Lib
{
public:
	Integral_Lib(Interpolation a = Lagrange, Integral b = Parabola, uint16_t ac = 15);		// 初始化
	~Integral_Lib(){}
	float I(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n);	// 获取积分值
private:
	inter_cal_t cal;	// 算法驱动

	/* 拟合算法 --------------------------------------------------*/
	static void Lagrange_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, 
															const float* out_x, float* out_y, uint16_t m);
	static void Newton_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, 
														const float* out_x, float* out_y, uint16_t m);
	static void Hermite_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, 
														 const float* out_x, float* out_y, uint16_t m);

	/* 积分算法 --------------------------------------------------*/
	static float Trapezoid_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, 
														uint16_t n, uint16_t accuracy);
	static float Parabola_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, 
														uint16_t n, uint16_t accuracy);
	static float Vortex_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, 
														uint16_t n, uint16_t accuracy);
	static float Romberg_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, 
														uint16_t n, uint16_t accuracy);
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif	

#endif	


	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

