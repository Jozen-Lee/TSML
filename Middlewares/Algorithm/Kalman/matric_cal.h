/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    matric.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for Matric Calculate.
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

#ifndef MATRIC_CAL_H
#define MATRIC_CAL_H

#ifdef __cplusplus


/* Includes ------------------------------------------------------------------*/  
#ifdef ARM_MATH_CM4 
  #if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F429xx)
		#include "stm32f4xx.h"
		#include <iterator>	
		#include <algorithm>
    #include <arm_math.h>	
		#include "array.h"
  #endif
#else
struct arm_matrix_instance_f32
{
	int numRows;
	int numCols;
	float* pData;
};
void arm_mat_init_f32(arm_matrix_instance_f32* a, int b, int c, float* _data) {}
void arm_mat_add_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_sub_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_mult_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_trans_f32(const arm_matrix_instance_f32* a, arm_matrix_instance_f32* b) {}
void arm_mat_inverse_f32(const arm_matrix_instance_f32* a, arm_matrix_instance_f32* b) {}
#endif									

/* Exported function declarations --------------------------------------------*/

/**
 * @brief Mat 矩阵类
 * @note 将arm中自带的矩阵结构体封装成C++类
 */
template <int Rows, int Cols>
class Mat
{
public:
	/* 初始化, 默认构造为全零矩阵*/
	
	Mat(void) 
	{ 
		for(int i = 0; i < Rows * Cols; i++) data[i] = 0;
		arm_mat_init_f32(&_mat, Rows, Cols, data);
	}
	
	/* 带初始数值的初始化 */
	Mat(Array<float, Rows * Cols>& obj)
	{
		int i = 0;
		for(float *p = obj.begin(); p != obj.end();p ++) data[i++] = *p;
		arm_mat_init_f32(&_mat, Rows, Cols, data);
	}
	
	Mat(float _data)
	{
		for (int i = 0; i < Rows * Cols; i++) data[i] = _data;
		arm_mat_init_f32(&_mat, Rows, Cols, data);
	}
	
	/* 符号重定义 */
	Mat<Rows, Cols>& operator=(Array<float, Rows* Cols>& obj)
	{
		int i = 0;
		for(float *p = obj.begin(); p != obj.end();p ++) data[i++] = *p;
		return *this;
	}
	
	Mat<Rows, Cols>& operator=(const Mat<Rows, Cols>& src)
	{
		for(int i = 0; i < Rows * Cols; i ++) data[i] = *(src._mat.pData + i); 
		return *this;
	}
	
	

	Mat<Rows, Cols> operator+(const Mat<Rows, Cols>& other)
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_add_f32(&_mat, &other._mat, &result._mat);
		return result;
	}
	
	Mat<Rows, Cols> operator-(const Mat<Rows, Cols>& other)
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_sub_f32(&_mat, &other._mat, &result._mat);
		return result;
	}
	
	Mat<Rows, Cols>& operator+=(const Mat<Rows, Cols>& other)
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_add_f32(&_mat, &other._mat, &result._mat);
		*this = result;
		return *this;
	}

	float& operator[](int idx) { return data[idx]; }
	
	template <int B_Cols> /*!< B.Rows=A.Cols */
	Mat<Rows, B_Cols> operator*(const Mat<Cols, B_Cols>& other)
	{
		auto result = Mat<Rows, B_Cols>();
		arm_mat_mult_f32(&_mat, &other._mat, &result._mat);	
		return result;
	}
	
	Mat<Rows, Cols> operator*(float num)
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_scale_f32(&_mat, num, &result._mat);
		return result;
	}	
	
	Mat<Rows, Cols>& zeros(void)
	{
		for(int i = 0; i < Rows; i ++)
		{
			for(int j = 0; j < Cols; j ++)
			{
				data[i * Cols + j] = 0;
			}
		}
		return *this;
	}
	
	Mat<Rows, Cols> eye(float num = 1)
	{
		Array<float, Rows * Cols> res;
		for(int i = 0; i < Cols; i++) res[i * Cols + i] = num;
		auto m = Mat<Rows, Cols>(res);
		return m;
	}

	Mat<Rows, Cols> diag(Array<float, Rows>& _pData)
	{
		Array<float, Rows * Cols> res;
		for(int i = 0; i < Cols; i++) res[i * Cols + i] = _pData[i];
		auto m = Mat<Rows, Cols>(res);
		return m;
	}
	
	Array<float, Rows* Cols> getData(void) const
	{
		Array<float, Rows* Cols> _data;
		for(int i = 0; i < Rows* Cols; i ++) _data[i] = data[i]; 
		return _data;
	}

	/* 矩阵的转置 */
	Mat<Cols, Rows> trans(void)
	{
		auto result = Mat<Cols, Rows>();
		arm_mat_trans_f32(&_mat, &result._mat);
		return result;
	}
	
	/* 矩阵的逆 */
	Mat<Rows, Cols> inverse(void)
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_inverse_f32(&_mat, &result._mat);
		return result;
	}
	
	static const int cols = Cols;
	static const int rows = Rows;

	arm_matrix_instance_f32 _mat;

protected:
	float data[Cols * Rows];
};

#endif	

#endif

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
