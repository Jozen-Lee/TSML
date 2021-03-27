/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Missile_INS.pp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for inertial navigation system.
  * @date    2021-02-01
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-01  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
	  
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
#include "Missile_INS.h"
#include "Data_Dealer.h"
#include "Surface.h"
#include "kalman.h"
#include "inertial_navigation.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t Location_Handle;
TaskHandle_t INS_Control_Handle;
/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Location(void *arg);
void Task_INS_Control(void *arg);
/* Function prototypes -------------------------------------------------------*/

/**
* @brief  飞镖计算任务初始化.
*/
void Missile_INS_Task_Init(void)
{
	xTaskCreate(Task_INS_Control,    "Task.INS_Control",    Large_Stack_Size,  NULL, PriorityHigh,    &INS_Control_Handle	);
	xTaskCreate(Task_Location,       "Task.Location",       Large_Stack_Size,  NULL, PriorityHigh,    &Location_Handle		);
}

/**
* @brief  惯导的定位任务.
*/
void Task_Location(void *arg)
{
	/* 任务等待变量 */
	BaseType_t wait = pdFALSE;
	
	/* 卡尔曼滤波器初始化 */
	Kalman_Height_Init();
	
  /* Infinite loop */
  for(;;)
  {
		/* 等待IMU状态更新 */
		wait = xSemaphoreTake(IMU_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* 更新状态量 */
			nav.Update(&imu.data.g_com_accel[0], &imu.data.g_com_accel[1], &imu.data.g_com_accel[2]);
			
//			/* 卡尔曼滤波更新高度 */
//			nav.status.height = Kalman_Get_Height();		
			
//			/* 状态数据存储*/
//			data_storer.Condition_Save();
			
			/* 发布通知,进行惯导控制 */
			xTaskNotifyGive(INS_Control_Handle);
		}
  }
}

/**
* @brief  惯导的控制任务.
*/
void Task_INS_Control(void *arg)
{
  /* Infinite loop */
  for(;;)
  {
		/* 等待通知 */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		
  }	
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
