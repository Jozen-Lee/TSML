/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Surface.cpp
  * @author  LJY 2244907035@qq.com
  * @brief   Code for surface interfaces.
  * @date    2021-03-02
  * @version 2.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2020-03-02  <td> 1.0     <td>YDX     <td> Creator
	* <tr><td>2021-03-02  <td> 2.0     <td>LJY     <td> Change the frame
  * </table>
  *
  ==============================================================================
                          How to use this driver  
  ==============================================================================
    @note
	    -# provide surface interfaces for missile tasks.
	
    @warning	
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */	
	
/* Includes ------------------------------------------------------------------*/
#include "Surface.h" 
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Surface surface;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  PID��ʼ��
  * @param  void
  * @return void
  */
	void Surface::PID_Init()
{
	/* Roll Channel */
	controller.Roll.Angle.SetPIDParam(1,0,0,0,100);
	controller.Roll.AngularVel.SetPIDParam(1,0,0,0,45);
	
	/* Pitch Channel */
	controller.Pitch.Angle.SetPIDParam(1,0,0,0,100);
	controller.Pitch.AngularVel.SetPIDParam(1,0,0,0,45);
	
	/* Yaw Channel */
	controller.Yaw.Angle.SetPIDParam(1,0,0,0,100);
	controller.Yaw.AngularVel.SetPIDParam(1,0,0,0,45);
}

/**
  * @brief  �Ƕ�PID�ı�������
  * @param[in]  current_roll, current_pitch, current_yaw ��ǰ����̬��
  * @return void
  */
void Surface::PID_Update_Angle(float current_roll, float current_pitch, float current_yaw)
{
	controller.Roll.Angle.Current = current_roll;	
	controller.Pitch.Angle.Current = current_pitch;	
	controller.Yaw.Angle.Current = current_yaw;
}

/**
  * @brief  ���ٶ�PID�ı�������
  * @param[in]  current_roll_vel, current_pitch_vel, current_yaw_vel ��ǰ��������ٶ�
  * @return void
  */
void Surface::PID_Update_AngularVel(float current_roll_vel, float current_pitch_vel, float current_yaw_vel)
{
	controller.Roll.AngularVel.Current = current_roll_vel;
	controller.Pitch.AngularVel.Current = current_pitch_vel;
	controller.Yaw.AngularVel.Current = current_yaw_vel;
}

/**
  * @brief  ���ö���ĽǶ�
  * @param[in]  las_deg: ��������ĽǶ�
  * @param[in]  ras_deg: ��������ĽǶ�
  * @return void
  */
void Surface::Set_Degree(float las_deg, float ras_deg)
{
	Left_Aileron.setDegree(las_deg);
	Right_Aileron.setDegree(ras_deg);
}

/**
  * @brief  ��ȡ����ĽǶ�
  * @param[out]  las_deg: ��������ĽǶ�
  * @param[out]  ras_deg: ��������ĽǶ�
  * @return void
  */
void Surface::Get_Deg(float& las_deg, float& ras_deg)
{
	las_deg = Left_Aileron.getDeg();
	ras_deg = Right_Aileron.getDeg();
}

/**
  * @brief  ���ƶ���Ƕ�
  * @param  roll_out: 	roll��ĽǶ�PID�����
  * @param  pitch_out: 	pitch��ĽǶ�PID�����
  * @param  yaw_out: 		yaw��ĽǶ�PID�����
  * @return void
  * @note   control of pitch channel and roll channel: ELEVON
  *         control of yaw channel:rudder
  */
void Surface::Degree_Control(float roll_out, float pitch_out, float yaw_out)
{
	/* convert the surface degree to the servo degree */
	float LA_Servo_Degree = (roll_out + pitch_out); 
	float RA_Servo_Degree = (-roll_out + pitch_out);
	
	/* set the degree of the servos */
	Set_Degree(LA_Servo_Degree, RA_Servo_Degree);
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
