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
      -# ����ʵ������(���Դ��λ򲻴��δ���)
				Navigation nav;
         
      -# ����״̬
				nav.Update(&acc_x,const &acc_y,const &acc_z);
    @warning	
      -# ��Ҫ`integral_algorithm`֧�֡�
	  
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
#include "ins.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Navigation nav;	// ʵ������
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief ��ȡ�ٶ�  
 *  @param[in] 	cal 										�㷨�ӿ�
 *  @param[out] sta											���ڼ�¼��״̬��
 *  @param[in] 	acc_x, acc_y, acc_z 		���ڻ��ֵ�������ٶȵ���ֵ
 *  @param[in] 	t	 											����ʱ������,��С����ٶ�������ƥ��
 *  @param[in] 	n	 											���ٶ�����Ĵ�С 
 *  @return void
 */
 void Navigation::Get_Speed(const float* acc_x, const float* acc_y, const float* acc_z, const float* t, uint16_t n)
 {
	 status.speed[0] += integ.I(t, acc_x, 0, n);
	 status.speed[1] += integ.I(t, acc_y, 0, n);
	 status.speed[2] += integ.I(t, acc_z, 0, n);
 }
 
/**
 *  @brief ��ȡλ����  
 *  @param[in] 	cal 										�㷨�ӿ�
 *  @param[out] sta											���ڼ�¼��״̬��
 *  @param[in] 	acc_x, acc_y, acc_z 		���ڻ��ֵ�������ٶȵ���ֵ
 *  @param[in] 	speed_x,speed_y,speed_z ���ڻ��ֵ������ٶȵ���ֵ
 *  @param[in] 	t	 											����ʱ������,��С���ٶ�������ƥ��
 *  @param[in] 	n	 											�ٶ�����Ĵ�С 
 *  @return void
 */
 void Navigation::Get_Distance(const float* speed_x, const float* speed_y, const float* speed_z, const float* t, uint16_t n)
 {
	 status.dis[0] += integ.I(t, speed_x, 0, n);
	 status.dis[1] += integ.I(t, speed_y, 0, n);
	 status.dis[2] += integ.I(t, speed_z, 0, n);
 } 
 
/**
 *  @brief ����״̬ 
 *  @param[in] acc_x, acc_y, acc_z ȫ���������µ�������ٶ�
 *  @return void
 */ 
 void Navigation::Update(const float* acc_x,const float* acc_y,const float* acc_z)
 { 
	 
	 /* ��¼���ٶ� */
	 record.acc[0][record.count_i] = *acc_x;
	 record.acc[1][record.count_i] = *acc_y;
	 record.acc[2][record.count_i] = *acc_z;
	 record.count_i = (1 + record.count_i) % COUNT_ACC;
	 
	 if(record.count_i == 0) 
	 {
		 /* �����ٶ� */
		 Get_Speed(record.acc[0], record.acc[1], record.acc[2], record.time_acc, COUNT_ACC);
		 
		 /* ��¼�ٶ� */
		 record.speed[0][record.count_j] = status.speed[0];
		 record.speed[1][record.count_j] = status.speed[1];
		 record.speed[2][record.count_j] = status.speed[2];
		 record.count_j = (1 + record.count_j) % COUNT_SPEED;

		 if(record.count_j == 0) 
		 {
			 /* ����λ���� */
			 Get_Distance(record.speed[0], record.speed[1], record.speed[2], record.time_speed, COUNT_SPEED);
		 }
	 }
	 
 }
  
 
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
