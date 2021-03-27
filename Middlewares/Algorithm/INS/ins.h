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

#ifndef INS_H
#define INS_H

#include "mpl_cal.h"

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "stm32f4xx.h"
/* Private define ------------------------------------------------------------*/

/* ����״̬��ʱ����(ms) */
#define UPDATE_TIME 	1	
	 
/* ���ִ��� */	 
#define	 	COUNT_ACC 	3
#define 	COUNT_SPEED 4		

/* Private include -----------------------------------------------------------*/
#include <algorithm>
#include "integral_algorithm.h"
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
typedef struct 
{
	float speed[3];	// ȫ����x,y,z����ٶ�
	float dis[3];		//ȫ����x,y,z���λ����
	float height;		//ȫ�������µĸ߶�
} g_status_t;	

typedef struct
{
	/* �洢���� */
	float acc[3][COUNT_ACC];
	float speed[3][COUNT_SPEED];
	
	/* ���ݶ�ȡ��ʱ������� */
	float time_acc[COUNT_ACC];
	float time_speed[COUNT_SPEED];	
	
	/* ������ */
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
	 /* ״̬����ʼ�� */
	 for(int i = 0; i < 3; i ++) 
	 {
		 status.dis[i] = 0;
		 status.speed[i] = 0;
	 }
	 status.height = 0;
	 
	 /* ��¼����ʼ�� */
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
		/* ״̬��ʼ�� */
		status.speed[0] = sppedx;	 
		status.speed[1] = sppedy;	
		status.speed[2] = sppedz;	

		status.dis[0] = disx;	 
		status.dis[1] = disy;	
		status.dis[2] = disz;	
		
		status.height = disz;
		
		/* ��¼����ʼ�� */
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
	g_status_t status;	// �洢��ǰ״̬�Ľṹ��
private:
	
	/* ��ȡ�ٶ� */
	void Get_Speed(const float* acc_x, const float* acc_y, const float* acc_z, const float* t, uint16_t n);

	/* ��ȡλ���� */
	void Get_Distance(const float* speed_x, const float* speed_y, const float* speed_z, const float* t, uint16_t n);								
	Integral_Lib integ;			// �����㷨
	record_t record;				// ��¼�������Ľṹ��
};

/* Exported variables --------------------------------------------------------*/
extern Navigation nav;

/* Exported function declarations --------------------------------------------*/
void Navigation_Init(void);

#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
