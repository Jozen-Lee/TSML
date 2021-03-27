/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Missile_LNS.pp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for light navigation system.
  * @date    2021-02-01
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-01  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
	  
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
#include "Missile_LNS.h"
#include "Vision.h"
#include "Data_Dealer.h"
#include "Controler.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t Vision_Handle;
TaskHandle_t LNS_Control_Handle;

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Vision(void *arg);
void Task_LNS_Control(void *arg);
/* Private function prototypes -----------------------------------------------*/

/**
* @brief  飞镖计算任务初始化.
*/
void Missile_LNS_Task_Init(void)
{
	xTaskCreate(Task_LNS_Control,   "Task.LNS_Control",   Large_Stack_Size,  NULL, PriorityHigh,  &LNS_Control_Handle	);
	xTaskCreate(Task_Vision,       	"Task.Vision",      	Large_Stack_Size,  NULL, PriorityHigh,   &Vision_Handle			);
}

/**
* @brief  光导的视觉任务
*/
void Task_Vision(void *arg)
{
	/* 任务等待变量 */
	BaseType_t wait = pdFALSE;
	
  /* Infinite loop */
  for(;;)
  {
		/* 等待图像更新 */
		wait = xSemaphoreTake(Camera_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* 图像处理 */
			vision.Image_Solution(50);
			
			/* 数据存储*/
//			data_dealer.Image_Save();
//			data_dealer.Condition_Save();
			
			/* 发布通知,进行光导控制 */
			xTaskNotifyGive(LNS_Control_Handle);
		}
  }
}

/**
* @brief  光导的控制任务.
*/
void Task_LNS_Control(void *arg)
{
  /* Infinite loop */
  for(;;)
  {
		/* 等待通知 */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
//		controler.Control();
  }	
}
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
