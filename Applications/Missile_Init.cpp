/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Missile_Init.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for Missile Init.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-03-13  <td> 1.0     <td>LJY  			<td>Creator
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
#include "Missile_Init.h"
#include "Service_Devices.h"
#include "Service_Debug.h"
#include "Missile_INS.h"
#include "Missile_LNS.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TaskHandle_t Missile_Waiting_Handle;
TaskHandle_t Missile_InitAll_Handle;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Missile_Waiting(void *arg);
void Missile_InitAll(void *arg);

/* Private function prototypes -----------------------------------------------*/
/**
* @brief  飞镖准备发射
* @param  None.
* @return None.
*/
void Missile_Get_Ready(void)
{
	xTaskCreate(Missile_Waiting,    "Missile.Waiting",   	Normal_Stack_Size, 	NULL, 	PriorityNormal, 			&Missile_Waiting_Handle);
	xTaskCreate(Missile_InitAll,    "Missile.InitAll",    Normal_Stack_Size, 	NULL, 	PriorityNormal, 			&Missile_InitAll_Handle);
}

/**
* @brief  Task for Missile waiting.
*/
void Missile_Waiting(void *arg)
{
	/* Cache for Task */
  TickType_t xLastWakeTime_t;	
	
	uint8_t Init_Flag = 0;	// 初始化标志
	
  /* Pre-Load for task */
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
		/* 更新初始化标志 */
		Init_Flag = 1;
		
		if(Init_Flag == 1)
		{
			/* 发送通知,表明飞镖开始初始化 */
			xTaskNotifyGive(Missile_InitAll_Handle);	
		}
			
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }	
}

/**
* @brief  Task for Missile init.
*/
void Missile_InitAll(void *arg)
{
  for(;;)
  {
		/* 等待通知 */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		
		/* Debug初始化 */
//		Service_Debug_Init();
		
		/* 外设初始化 */
		Service_Devices_Init();
		
		/* 光导初始化 */
		Missile_LNS_Task_Init();
		
		/* 惯导初始化 */
//		Missile_INS_Task_Init();
		
		/* 删除初始化任务 */
		vTaskDelete(Missile_Waiting_Handle);	
		vTaskDelete(Missile_InitAll_Handle);
  }	
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
