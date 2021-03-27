/**
  **********************************************************************************
  * @file   : Service_Debug.cpp
  * @brief  : Debug support file.This file provides access ports to debug.
  **********************************************************************************
  *  
**/
/* Includes ------------------------------------------------------------------*/
#include "Service_Debug.h"
#include <UpperMonitor.h>
#include <Ano_UpperMonitor.h>

/* Private Includes ----------------------------------------------------------*/
#include "Data_Dealer.h"
#include "bme280_app.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t Debug_Handle;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Debug(void *arg);

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize debug service based on Upper Monitor.
*/
void Service_Debug_Init(void)
{
  
  xTaskCreate(Task_Debug,       /* Task function. */
         "Debug_Service",       /* Task Name. */
       Normal_Stack_Size,       /* Stack depth. */
                    NULL,       /* Task parameter */
						PriorityNormal,     /* Priority */
          &Debug_Handle);       /* Task handle */
}


/**
* @brief  Send the debug meaasge to Upper Monitor(Lowest priority)
* @param  None.
* @return None.
*/
void Task_Debug(void *arg)
{
    /* Cache for Task */
#ifdef USE_LAB_UPPER
	huart1.Init.BaudRate = 115200;
	HAL_UART_Init(&huart1);
#else
	ANO_Upper ano(&huart1);	// 注册一个上位机
	ano.Init();							// 上位机初始化
#endif			
    /* Pre-Load for task */
    for(;;)
    {
      /* User porcess BEGIN. */
			
      /* User process END. */
      
      /* Transmit a message frame. */
#ifdef USE_LAB_UPPER			
	Sent_Contorl(&huart1);
#else
	ano.Send_Data(&imu.data, 0, 0);
#endif			
      
      /*Pass to next ready task*/
      vTaskDelay(10);
    }
}

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
