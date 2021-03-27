/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    mpl_cal.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPL calculate functions.
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
	
#ifndef MPL_CAL_H
#define MPL_CAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/  
#include "main.h"	 
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "hal_outputs.h"
#include "inv_mpu.h"	
#include "inv_mpu_dmp_motion_driver.h"	 
/* Private define ------------------------------------------------------------*/
	
#ifdef __cplusplus
}
#endif	

/* Private include -----------------------------------------------------------*/
#include <algorithm>
#include "my_filters.h"
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class MPL
{
public:
	uint8_t Update_Data(void);																									// ��������
	uint8_t Get_Gyro(float* gx, float* gy, float* gz);													// ���ٶ�
	uint8_t Get_Accel(float* ax, float* ay, float* az);													// ������ϵ�¼��ٶ�(δ��ȥ�������ٶ�)
	uint8_t Get_Linear_Accel(float* gx, float* gy, float* gz);									// ������ϵ�¼��ٶ�(��ȥ�������ٶ�)
	uint8_t Get_Ground_Linear_Accel(float* g_ax, float* g_ay, float* g_az);			// ��������ϵ�¼��ٶ�(��ȥ�������ٶ�)
	uint8_t Get_Compass(float* mx, float* my, float* mz);												// ��ǿ
	uint8_t Get_Quat(float* W, float* X, float* Y, float* Z);										// ��Ԫ��
	uint8_t Get_Heading(float* head);																						// �����(YAW��)
	uint8_t Get_Euler(float* pitch, float* roll, float* yaw);										// ŷ����
	uint8_t Get_DCM(float DCM[][3]);																						// ��ת����
	uint8_t Get_Pos(float* pitch, float* roll, float* yaw);											// ��̬��
	void Acc_Compenssation(const float* pitch, const float* roll, 							// ���ٶȲ���
			const float* yaw, float* g_ax, float* g_ay, float* g_az);
private:	
	MeanFilter<5> accx_filter;
	MeanFilter<5> accy_filter;
	MeanFilter<5> accz_filter;
	void Body_To_World_Accel(float DCM[][3], float b_ax, float b_ay, float b_az, float* g_ax, float* g_ay, float* g_az);		
	
};
/* Exported variables --------------------------------------------------------*/
extern MPL mpl_lib;
/* Exported function declarations --------------------------------------------*/

#endif
	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
	
