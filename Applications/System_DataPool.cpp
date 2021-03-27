/**
  ******************************************************************************
  * @file   System_DataPool.c
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#include "System_DataPool.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
/* Semaphores */
SemaphoreHandle_t IMU_SemaphoreHandle;
SemaphoreHandle_t	Camera_SemaphoreHandle;

/* Mutexes */
/* Notifications */

/* Other Resources -----------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart1 */


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/



