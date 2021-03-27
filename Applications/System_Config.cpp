/**
  ******************************************************************************
  * @file   System_config.cpp
  * @brief  Deploy resources,tasks and services in this file.
  ******************************************************************************
  * @note
  *  - Before running your Task you should first include your headers and init- 
  *    ialize used resources in "System_Resource_Init()". This function will be 
  *    called before tasks Start.
  *    
  *  - All tasks should be created in "System_Tasks_Init()", this function will
  *    be called in 'defaultTask()'.
  *
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "System_Config.h"
#include "System_DataPool.h"

/* Service */
#include "Missile_Init.h"

/* User support package & SRML */
#include <SRML.h>
#include <UpperMonitor.h>
/* Private variables ---------------------------------------------------------*/


/*Founctions------------------------------------------------------------------*/
/**
* @brief Load drivers ,modules, and data resources for tasks.
* @note  Edit this function to add Init-functions and configurations.
*/
void System_Resource_Init(void)
{	
	/* Timer Init------------------------*/
  Timer_Init(&htim2, USE_HAL_DELAY);
  
	/* CAN Init--------------------------*/
  
	/* USART Init------------------------*/
  Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE, RecHandle);
  
	/* SPI Init--------------------------*/
  
	/* PWM Init--------------------------*/
	HAL_TIM_Base_Start(&htim4);		
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);	
	
  /* RTOS resources Init --------------*/
	IMU_SemaphoreHandle = xSemaphoreCreateBinary();
	Camera_SemaphoreHandle = xSemaphoreCreateBinary();
	
  /* Other resources Init -------------*/
  
  /* Modules Init ---------------------*/

  /* Service configurations -----------*/
   System_Tasks_Init();
}  


/**
* @brief Load and start User Tasks. 
* @note  Edit this function to add tasks into the activated tasks list.
*/
void System_Tasks_Init(void)
{ 	
  Missile_Get_Ready();
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

