/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    kalman.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for kalman filter.
  * @date    2021-02-02
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-02  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
	 * @brief 构造一个新的卡尔曼滤波器实例
	 * 构造完成后，必须按照物理模型手动给 A、B、H 赋值
	 * 
	 * @param stateDim 状态空间维度，即你整个卡尔曼滤波器会接触到多少种物理量。
	 * 这些物理量之间必须是线性的，就是说他们之间的关系只有加减乘除、积分、微分
	 * 例如 位置-速度-加速度 的状态空间，就有3个维度
	 * 
	 * @param measureDim 测量矩阵维度，即有多少个传感器
	 * 例如你有 绝对码盘（角度）-相对码盘（角速度）-陀螺仪（角加速度），就有3个维度
	 * 
	 * @param controlDim 控制矩阵维度，即有多少个控制量可以用进去做预测。
	 * 一般也只有1个，就是你发给电调的电流，它与加速度有关
	 * 
	 * @param processErr 过程噪声
	 * 整个环境中有多少不可控因素，比如向前运动突然撞车，就是一种过程噪声。
	 * 数字越大，噪声越大。 
	 * 
	 * @param measureErr 测量噪声
	 * 传感器引入的噪声。也就是传感器精度。
	 * 数字越大，噪声越大。 
	 
	  @note Q和R调节的原则
		-# Q和R的大小比值决定了预测值和测量值的权重。
				R不变，Q越大，则更信任测量值。Q不变，R越大，则更信任预测值
		-# Q的各个参数大小关系决定了预测数据的可信度，R的各个参数大小关系决定了测量数据的可信度
				相对数值越小，可信度越高
		
		@note 以卡尔曼滤波获取mpu6050的角度为例
		
		-# 确定控制模型,确定各个矩阵的维度
		#define ANGLE_STATEDIM 		2
		#define ANGLE_MEASUREDIM 	1
		#define ANGLE_CONTROLDIM	1
		
		-# 构造卡尔曼滤波器,确定Q和R的值
		KalmanFilter<ANGLE_STATEDIM, ANGLE_MEASUREDIM, ANGLE_CONTROLDIM> Kalman_Angle(5e-3, 5e-1);
		
		-# 根据模型,手动给A,H,B矩阵赋值,若初始状态确定,也可以给初始状态赋值(没有也没关系,它会很快收敛)
		void Kalman_Init(void)
		{
			Array<float , ANGLE_STATEDIM * ANGLE_STATEDIM>  _A{1.0f, -0.005f, 0, 1.0f};
			Mat<ANGLE_STATEDIM, ANGLE_STATEDIM> A(_A);
			Array<float , ANGLE_STATEDIM * ANGLE_MEASUREDIM>  _H{1.0f, 0};	
			Mat<ANGLE_MEASUREDIM, ANGLE_STATEDIM> H(_H);
			Array<float , ANGLE_STATEDIM * ANGLE_CONTROLDIM>	_B{0.005f, 0};
			Mat<ANGLE_STATEDIM, ANGLE_CONTROLDIM> B(_B);
			Array<float , ANGLE_STATEDIM>	state{0, 0};
			Kalman_Angle.setA(A);
			Kalman_Angle.setH(H);
			Kalman_Angle.setB(B);
			Kalman_Angle.init(state, 0.5);
		}

		-# 传入各个传感器处理后的数据,进行卡尔曼滤波,并输出所需要的卡尔曼滤波后的数据
		void Kalman_Get_Angle(float Accel,float Gyro, float* angle)
		{
			Array<float , ANGLE_STATEDIM> res;
			Array<float , ANGLE_CONTROLDIM> control{Gyro};
			Array<float , ANGLE_MEASUREDIM> measure{Accel};
			Kalman_Angle.predict(control);
			res = Kalman_Angle.correct(measure);
			*angle = res[0];
		}
		
		-# 在其他函数中使用以下语句来更新角度
		Accel_Angle = atan2(mpl_var.accel[1], mpl_var.accel[2]) * 180.0f / 3.14f;
		Gyro = mpl_var.gyro[0] * 180.0f / 3.14f;
		Kalman_Get_Angle(Accel_Angle,Gyro,&angle);
	  
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
#include "kalman.h"
#include "mpl_cal.h"
#include "inertial_navigation.h"

/* Private Includes ------------------------------------------------------------*/
#include "System_DataPool.h"

/* Private define ------------------------------------------------------------*/
#define HEIGHT_STATEDIM 		3
#define HEIGHT_MEASUREDIM 	2
#define HEIGHT_CONTROLDIM		1

/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
KalmanFilter<HEIGHT_STATEDIM, HEIGHT_MEASUREDIM, HEIGHT_CONTROLDIM> IMU_BMx_Height;

void Kalman_Height_Init(void)
{
	float sam_frq = 1000.0f;
	float T = 1.0f / sam_frq;
	
	Array<float , HEIGHT_STATEDIM * HEIGHT_STATEDIM>  _A{1.f , T, 0.5f*T*T, 0, 1.f, T, 0 , 0, 1.f};
	Mat<HEIGHT_STATEDIM, HEIGHT_STATEDIM> A(_A);
	Array<float , HEIGHT_STATEDIM * HEIGHT_MEASUREDIM>  _H{1.f, 0, 0, 0, 0, 1.f};	
	Mat<HEIGHT_MEASUREDIM, HEIGHT_STATEDIM> H(_H);
	Array<float , HEIGHT_STATEDIM * HEIGHT_CONTROLDIM>  _B{0, 0, 0};	
	Mat<HEIGHT_STATEDIM, HEIGHT_CONTROLDIM> B(_B);	
	Array<float , HEIGHT_STATEDIM>  _Q{0.01f, 0.0001f, 0.001f};
	Array<float , HEIGHT_MEASUREDIM>  _R{1.f, 5.f};	
	Array<float , HEIGHT_STATEDIM>	state{0, 0, 0};
	/* 初始化卡尔曼运算矩阵 */
	IMU_BMx_Height.setA(A);
	IMU_BMx_Height.setH(H);
	IMU_BMx_Height.setB(B);
	IMU_BMx_Height.setQ(_Q);
	IMU_BMx_Height.setR(_R);
	
	/* 初始化状态 */
	IMU_BMx_Height.init(state, 0.1f);
}

float Kalman_Get_Height(void)
{
	float height = (float)bme280.height; 
	Array<float , HEIGHT_STATEDIM> res;
	Array<float , HEIGHT_CONTROLDIM> control{0};
	Array<float , HEIGHT_MEASUREDIM> measure{height , imu.data.g_com_accel[2]};
	IMU_BMx_Height.predict(control);
	res = IMU_BMx_Height.correct(measure);
	return res[0];	
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
