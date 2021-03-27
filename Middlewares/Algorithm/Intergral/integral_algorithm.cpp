/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    integral_algorithm.c
  * @author  LJY 2250017028@qq.com
  * @brief   Code for integral algorithm.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-01-31  <td> 1.0     <td>LJY  			<td>Creator
	* <tr><td>2021-02-22  <td> 1.1     <td>LJY  			<td>Change the frame
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 创建实例对象
	     Integral_Lib inte_lib;		 
        
			-# 获取积分值
				 inte_lib.I(ref_x , ref_y, ref_y_dot, n);
		 
    @warning	
			-# 创建实例时需要选择所需的拟合,积分算法以及准确度, 不选择则采用默认值
      -# 该库参考了数值计算教科书，有不懂的细节可以去书中找答案
			-# 该库分成拟合算法和积分算法两部分
			
	  
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
#include "integral_algorithm.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/********************************************拟合算法************************************************
	* @attention
	* -# 拉格朗日和牛顿拟合都没有用到输入的导数,这个参数只是为了统一接口。如果没有的话,输入0也无妨。
	* -# 在导数较准确的情况下，埃尔米特插值法会更精确。
	**************************************************************************************************/
	
	
/**
 *  @brief 拉格朗日插值拟合法  
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	ref_x 参考点的横坐标数组
 * 	@param[in] 	ref_y 参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组(这里没有用到,只是为了统一接口)
 *	@param[in] 	n 		参考点的个数
 *  @param[in] 	out_x 所求的点横坐标数组
 *  @param[out] out_y 所求点的纵坐标数组
 *  @param[in] 	m 		所求点的个数
 *  @return  void
 */
 void Integral_Lib::Lagrange_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
		int i, j, k;
	  float mol, deno;	//记录每个单项式分子和分母的值
	 
	 /* 先将输出数组的值置零 */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }
	 
	 /* 每个所求点的横坐标代入 */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)次多项式 */
		 for(j = 0; j < n; j ++)
		 {
			 mol = 1.0;
			 deno = 1.0;
			 /* 求解每个单项式的值 */
			 for(k = 0; k < n; k ++)
			 {
				 if(j != k)
				 {
						mol *= out_x[i] - ref_x[k];
						deno *= ref_x[j] - ref_x[k];
				 }
			 }
			 
			 /* 单项式累加 */
			 out_y[i] += ref_y[j] * mol / deno; 
		 }
	 }
 }
 
/**
 *  @brief 牛顿插值拟合法  
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	ref_x 参考点的横坐标数组
 * 	@param[in] 	ref_y 参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组(这里没有用到,只是为了统一接口)
 *	@param[in] 	n 		参考点的个数
 *  @param[in] 	out_x 所求的点横坐标数组
 *  @param[out] out_y 所求点的纵坐标数组
 *  @param[in] 	m 		所求点的个数
 *  @return  void
 */
 void Integral_Lib::Newton_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
	 int i, j, k, l;
	 float mol, deno;	//记录每个差商的分子和分母的值
	 float diff;			//记录每个单项式的差商
	 float single;		//记录每个单项式的值
	
	 /* 先将输出数组的值置零 */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }

	 /* 每个所求点的横坐标代入 */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)次多项式 */
		 for(j = 0; j < n; j ++)
		 { 
			 if(j == 0) out_y[i] += ref_y[j];
			 else
			 {
				 /* 计算每个单项式的差商 */
				 diff = 0;
				 for(k = 0; k <= j; k ++)
				 { 	 
					 mol = ref_y[k];
					 deno = 1.0;
					 for(l = 0; l <= j; l ++)
					 {
						 if(l != k) deno *= ref_x[k] - ref_x[l];
					 }
					 diff += mol / deno;
				 }
				 
				  /* 计算每个单项式 */
				 single = 1.0;
				 for(k = 0; k < j; k ++)
				 {
					 single *= out_x[i] - ref_x[k];
				 }
				 
					/* 单项式累加 */
					out_y[i] += single * diff; 				 
			 }		
		 }
	 }		
 }
 
 /**
 *  @brief 埃尔米特插值拟合法(未测试) 
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @param[in] 	out_x 		所求的点横坐标数组
 *  @param[out] out_y 		所求点的纵坐标数组
 *  @param[in] 	m 				所求点的个数
 *  @return  void
 */
 void Integral_Lib::Hermite_Interpolation(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, const float* out_x, float* out_y, uint16_t m)
 {
	 int i, j, k;
	 float alpha, beta;							//记录单项式中的系数
	 float mol, deno;								//记录拉格朗日插值系数的分子和分母
	 float temp_1, temp_2, temp_3;	//记录系数中的过渡量
	 
	 /* 先将输出数组的值置零 */
	 for(i = 0; i < m; i ++)
	 {
		 out_y[i] = 0;
	 }
	 
	 /* 每个所求点的横坐标代入 */
	 for(i = 0; i < m; i ++)
	 {
		 /* (n-1)次多项式 */
		 for(j = 0; j < n; j ++)
		 {		 
			 /* 计算拉格朗日插值基函数 */
			 mol = 1.0;
			 deno = 1.0;
			 for(k = 0; k < n; k ++)
			 {
				 if(j != k)
				 {
						mol *= out_x[i] - ref_x[k];
						deno *= ref_x[j] - ref_x[k];
				 }
			 }
			 temp_1 = mol / deno;
			 temp_1 *= temp_1;
			 
			 /* 计算系数alpha */
			 temp_2 = 2 * ( out_x[i] - ref_x[j]);
			 temp_3 = 0;
			 for(k = 0; k < n; k ++) 
			 {
				 if(k != j) temp_3 += 1 / (ref_x[j] - ref_x[k]);
			 }
			 temp_2 *= temp_3;
			 alpha = (1 - temp_2) * temp_1;
			 
			 /* 计算系数beta */
			 beta = (out_x[i] - ref_x[j]) * temp_1;
			 
			 /* 单项式累加 */
			 out_y[i] += alpha * ref_y[j] + beta * ref_y_dot[j];
		 }
	 }
 }
 
 
 /********************************************积分算法************************************************
	* @attention
	* -# 由于算法中用到了new来创建数组,存储临时数据,使用时accuracy不能给太大,否则会爆栈。
	* -# 推荐的accuracy值(由于accuracy越大，运算时间越长，具体使用多少要根据情况而定):
				Trapezoid_Integral 20
				Parabola_Integral  15
				Vortex_Integral		 10
				Romberg_Integral   5
	**************************************************************************************************/
 
 /**
 *  @brief 复合梯形求积法 
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	func 			拟合算法的函数指针
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @param[in] 	accuracy	精确度，即将积分区间划分成多少个小区间，取值应 >= 1
 *  @return  		res				积分值
 */
 float Integral_Lib::Trapezoid_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float h;				//积分步长
	 float Tn = 0;	//积分值
	 uint16_t len;	//代入拟合函数中计算的值的个数
	 
	 /* 先判断n值的大小,防止超出数组界限 */
	 if(n <= 0) return 0;
	 else if(n == 1) return ref_y[0];
	 else
	 {
		 /* 计算步长 */
		 h = (ref_x[n-1] - ref_x[0]) / accuracy;
		 
		 /* 计算积分的前半部分 */
		 Tn += h / 2 * (ref_y[0] + ref_y[n-1]);
		 
		 /* 判断精度值大小 */
		 if(accuracy == 1) return Tn;
		 else
		 {
			 /* 计算代入的数组长度 */
			 len = accuracy - 1;
			 
			 /* 创建数组来存储要代入拟合算法中运算的自变量 */
			 float* cal_x = new float[len];
			 for(int i = 0; i < len; i++) cal_x[i] = ref_x[0] + h * (i + 1);
			 
			 /* 创建数组存储运算结果 */
			 float* result = new float[len];
			 
			 /* 运用拟合函数进行运算 */
			 func(ref_x, ref_y, ref_y_dot, n, cal_x, result, len);
			 
			 /* 累加算积分 */
			 for(int i = 0; i < len; i++)
			 {
				 Tn += h * result[i];
			 }
			 
			 /* 清除内存 */
			 delete []cal_x;
			 delete []result;
			 return Tn;
		 }
	 }
 }
 
 
 /**
 *  @brief 复合抛物线求积法 
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	func 			拟合算法的函数指针
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @param[in] 	accuracy 	精确度，即将积分区间划分成多少个小区间
 *  @return  		res				积分值
 */
  float Integral_Lib::Parabola_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Tn, T2n; //利用复合梯形法的递推来求解
	 float Sn = 0;	//积分值
	 Tn = Trapezoid_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 T2n = Trapezoid_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Sn = 4 * T2n / 3 - Tn / 3;
	 return Sn;
 }
 
 /**
 *  @brief 复合科特斯求积法  
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	func 			拟合算法的函数指针
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @param[in] 	accuracy 	精确度，即将积分区间划分成多少个小区间
 *  @return  		res				积分值
 */
 float Integral_Lib::Vortex_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Sn, S2n; //利用复合梯形法的递推来求解
	 float Cn = 0;	//积分值
	 Sn = Parabola_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 S2n = Parabola_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Cn = 16 * S2n / 15 - Sn / 15;
	 return Cn;	 
 }
 
 /**
 *  @brief 龙贝格求积法  
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	func 			拟合算法的函数指针
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @param[in] 	accuracy 	精确度，即将积分区间划分成多少个小区间
 *  @return  		res				积分值
 */
 float Integral_Lib::Romberg_Integral(fitting_func func, const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n, uint16_t accuracy)
 {
	 float Cn, C2n; //利用复合梯形法的递推来求解
	 float Rn = 0;	//积分值
	 Cn = Vortex_Integral(func, ref_x, ref_y, ref_y_dot, n, accuracy);
	 C2n = Vortex_Integral(func, ref_x, ref_y, ref_y_dot, n, 2*accuracy);
	 Rn = 64 * C2n / 63 - Cn / 63;
	 return Rn;	 	 
 } 
 
 /**
 *  @brief 积分算法初始化
 *	@note	在使用积分函数前一定要先初始化
 * 	@param[in] 	void
 *  @return  		void
 */
 Integral_Lib::Integral_Lib(Interpolation a, Integral b , uint16_t ac)
 {
	 /* 选择拟合算法 */
	 switch(a)
	 {
		 case Lagrange: cal.fit = Lagrange_Interpolation; break;
		 case Newton: 	cal.fit = Newton_Interpolation; 	break;
		 case Hermite: 	cal.fit = Hermite_Interpolation; 	break;
	 }
	 cal.fit = Lagrange_Interpolation;
	 
	 /* 选择积分算法 */
	 switch(b)
	 {
		 case Trapezoid: 	cal.integ = Trapezoid_Integral; 	break;
		 case Parabola:	 	cal.integ = Parabola_Integral; 		break;
		 case Vortex:			cal.integ = Vortex_Integral; 			break;
		 case Romberg:		cal.integ = Romberg_Integral; 		break;
	 }
	 
	 /* 设置精确度 */
	 cal.ac = 15;
 }
 
 /**
 *  @brief 积分算法(API函数)
 *	@note	数组大小要和输入的点个数相匹配
 * 	@param[in] 	ref_x 		参考点的横坐标数组
 * 	@param[in] 	ref_y 		参考点的纵坐标数组
 * 	@param[in] 	ref_y_dot 参考点的导数数组
 *	@param[in] 	n 				参考点的个数
 *  @return  		I					积分值
 */
 float Integral_Lib::I(const float* ref_x, const float* ref_y, const float* ref_y_dot, uint16_t n)
 {
	 float I;
	 I = cal.integ(cal.fit, ref_x, ref_y, ref_y_dot, n, cal.ac);
	 return I;
 }

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

