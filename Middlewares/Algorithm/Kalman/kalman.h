/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    kalman.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for kalman filter.
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

#ifndef __KALMAN_H
#define __KALMAN_H 

/* Includes ------------------------------------------------------------------*/  
#ifdef __cplusplus
#include "matric_cal.h" 
	 
/* Private define ------------------------------------------------------------*/								
/* Private include -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilter
{
public:
	/* 初始化 */	
	KalmanFilter(float processErr = 1e-2, float measureErr = 1e-1)
	{
		/* 初始化状态转移矩阵 */
		A = A.eye();

		/* 初始化过程噪声协方差矩阵 */
		Q = Q.eye(processErr);
	
		/* 初始化观测噪声协方差矩阵 */
		R = R.eye(measureErr);

		/**	@brief 初始化后验误差协方差矩阵
			* @note 默认构造假定初始状态不可知，故给予极大的协方差矩阵
			*/
		P = P.eye(1e7);

		/* 初始化测量矩阵和转移矩阵的转置矩阵 */
		HT = H.trans();
		AT = A.trans();
	}

/**
 * @brief 卡尔曼滤波器初始状态设置
 * @note 如果知道初始状态，可以输入给滤波器。否则你可不必调用此函数。
 * @param state 数组，代表各个状态量
 * @param error 一个数字，代表状态量的可信度。可信度越高，数字越小。
 */
	void init(Array<float, StateDim>& state, float error)
	{
		xhat = state;
		P = P.eye(error);
	}
	
	/* 设置过程噪声协方差矩阵 */
	void setQ(Array<float, StateDim>& processErr)
	{
		Q = Q.diag(processErr);
	}

	/* 设置观测噪声协方差矩阵 */
	void setR(Array<float, MeasureDim>& measureErr)
	{
		R = R.diag(measureErr);
	}

	/* 设置转移矩阵 */
	void setA(const Mat<StateDim, StateDim>& _A)
	{
		A = _A;
		AT = A.trans();
	}
	
	/* 设置测量矩阵 */
	void setH(const Mat<MeasureDim, StateDim>& _H)
	{
		H = _H;
		HT = H.trans();
	}
	
	/* 设置控制矩阵 */
	void setB(const Mat<StateDim, ControlDim>& _B)
	{
		B = _B;
	}
	
	/* 预测函数 */
	Array<float, StateDim> predict(const Mat<ControlDim, 1>& control)
	{
		u = control;
		Mat<StateDim, 1> temp_1;
		Mat<StateDim, StateDim> temp_2;
		
		/* 1. 预测状态: xhatminus = A * xhat + B * u */
		xhatminus = A * xhat; temp_1 = B * u; xhatminus += temp_1;

		/* 2. 预测误差协方差: Pminus = A * P * AT + Q */
		temp_2 = A * P; temp_2 = temp_2 * AT; Pminus = temp_2 + Q;

		return xhatminus.getData();
	}
	
	/* 修正函数 */
	Array<float, StateDim> correct(const Mat<MeasureDim, 1>& measurement)
	{
		z = measurement;
		Mat<MeasureDim, MeasureDim> temp_1;
		Mat<MeasureDim, StateDim> temp_2;
		Mat<StateDim, 1> temp_3;
		Mat<MeasureDim, 1> temp_4;
		Mat<StateDim, StateDim> temp_5;
		
		/* 3. 计算卡尔曼增益: K = Pminus * HT * (H * Pminus * HT + R).inverse() */
		K = Pminus * HT; temp_2 = H * Pminus; temp_1 = temp_2 * HT; 
		temp_1 += R; temp_1 = temp_1.inverse(); 
		K = K * temp_1;

		/* 4. 用测量值更新估计: xhat = xhatminus + K * (z - H * xhatminus) */
		temp_4 = H * xhatminus; temp_4 = z - temp_4; 
		temp_3 = K * temp_4; xhat = xhatminus + temp_3;

		/* 5. 更新误差协方差: P = (Q - K * H) * Pminus */
		temp_5 = K * H; temp_5 = Q - temp_5; P = temp_5 * Pminus;

		return xhat.getData();
	}
	
	
	/**
		* @brief 变量定义
		* @note 没有在初始化函数中初始化的矩阵,默认构造为零矩阵 
		*/
	Mat<StateDim, 1> xhat;					//后验状态估计（当前估计值）
	Mat<StateDim, 1> xhatminus;			//先验状态估计（未来预测值）
	Mat<StateDim, StateDim> Q;			//过程噪声协方差（由环境因素引入的噪声）
	Mat<MeasureDim, MeasureDim> R;  //测量噪声协方差（传感器误差）
	Mat<StateDim, StateDim> P;			//后验误差协方差（当前估计误差）
	Mat<StateDim, StateDim> Pminus; //先验误差协方差（未来预测误差）
	Mat<StateDim, MeasureDim> K;		//卡尔曼增益
	Mat<MeasureDim, 1> z;						//测量值（传感器数据）
	Mat<ControlDim, 1> u;						//控制量（控制器数据）
protected:
	Mat<StateDim, StateDim> A;			//状态转换模型（转移矩阵）
	Mat<MeasureDim, StateDim> H;  	//状态变量到测量值的转换矩阵（测量矩阵）
	Mat<StateDim, StateDim> AT;   	//A.trans
	Mat<StateDim, MeasureDim> HT; 	//H.trans
	Mat<StateDim, ControlDim> B;		//控制转换模型（控制矩阵）
};



#endif

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/* Exported function declarations --------------------------------------------*/
void Kalman_Height_Init(void);
float Kalman_Get_Height(void);
	
#ifdef __cplusplus
}
#endif

#endif

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
