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
	 * @brief ����һ���µĿ������˲���ʵ��
	 * ������ɺ󣬱��밴������ģ���ֶ��� A��B��H ��ֵ
	 * 
	 * @param stateDim ״̬�ռ�ά�ȣ����������������˲�����Ӵ�����������������
	 * ��Щ������֮����������Եģ�����˵����֮��Ĺ�ϵֻ�мӼ��˳������֡�΢��
	 * ���� λ��-�ٶ�-���ٶ� ��״̬�ռ䣬����3��ά��
	 * 
	 * @param measureDim ��������ά�ȣ����ж��ٸ�������
	 * �������� �������̣��Ƕȣ�-������̣����ٶȣ�-�����ǣ��Ǽ��ٶȣ�������3��ά��
	 * 
	 * @param controlDim ���ƾ���ά�ȣ����ж��ٸ������������ý�ȥ��Ԥ�⡣
	 * һ��Ҳֻ��1���������㷢������ĵ�����������ٶ��й�
	 * 
	 * @param processErr ��������
	 * �����������ж��ٲ��ɿ����أ�������ǰ�˶�ͻȻײ��������һ�ֹ���������
	 * ����Խ������Խ�� 
	 * 
	 * @param measureErr ��������
	 * �����������������Ҳ���Ǵ��������ȡ�
	 * ����Խ������Խ�� 
	 
	  @note Q��R���ڵ�ԭ��
		-# Q��R�Ĵ�С��ֵ������Ԥ��ֵ�Ͳ���ֵ��Ȩ�ء�
				R���䣬QԽ��������β���ֵ��Q���䣬RԽ���������Ԥ��ֵ
		-# Q�ĸ���������С��ϵ������Ԥ�����ݵĿ��Ŷȣ�R�ĸ���������С��ϵ�����˲������ݵĿ��Ŷ�
				�����ֵԽС�����Ŷ�Խ��
		
		@note �Կ������˲���ȡmpu6050�ĽǶ�Ϊ��
		
		-# ȷ������ģ��,ȷ�����������ά��
		#define ANGLE_STATEDIM 		2
		#define ANGLE_MEASUREDIM 	1
		#define ANGLE_CONTROLDIM	1
		
		-# ���쿨�����˲���,ȷ��Q��R��ֵ
		KalmanFilter<ANGLE_STATEDIM, ANGLE_MEASUREDIM, ANGLE_CONTROLDIM> Kalman_Angle(5e-3, 5e-1);
		
		-# ����ģ��,�ֶ���A,H,B����ֵ,����ʼ״̬ȷ��,Ҳ���Ը���ʼ״̬��ֵ(û��Ҳû��ϵ,����ܿ�����)
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

		-# ������������������������,���п������˲�,���������Ҫ�Ŀ������˲��������
		void Kalman_Get_Angle(float Accel,float Gyro, float* angle)
		{
			Array<float , ANGLE_STATEDIM> res;
			Array<float , ANGLE_CONTROLDIM> control{Gyro};
			Array<float , ANGLE_MEASUREDIM> measure{Accel};
			Kalman_Angle.predict(control);
			res = Kalman_Angle.correct(measure);
			*angle = res[0];
		}
		
		-# ������������ʹ��������������½Ƕ�
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

/* Private Includes ------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/



	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
