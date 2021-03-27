/**
  ******************************************************************************
  * @file   System_DataPool.h
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#ifndef _DATA_POOL_H_
#define _DATA_POOL_H_

/* Includes ------------------------------------------------------------------*/
/* Middlewares & Drivers Support */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stm32f4xx.h>
#include <SRML.h>

/* Devices Support */
#include "MT9V032.h"
#include "SDcard.h"
#include "imu.h"
#include "bme280_app.h"


/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

/* HAL Handlers --------------------------------------------------------------*/

/* TIMER */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

/* USART */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* DCMI */
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
/* Semaphores */
extern SemaphoreHandle_t 	IMU_SemaphoreHandle;
extern SemaphoreHandle_t	Camera_SemaphoreHandle;

/* Mutexes */
/* Notifications */

/* Other Resources -----------------------------------------------------------*/
#define USART1_RX_BUFFER_SIZE 32
//#define USART2_RX_BUFFER_SIZE 64
//#define USART3_RX_BUFFER_SIZE 128
//#define USART4_RX_BUFFER_SIZE 256
//#define USART5_RX_BUFFER_SIZE 512
//#define USART6_RX_BUFFER_SIZE 1024

extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];

/* Devices */
extern MT9V032 mt9v032;
extern BME280 bme280;
extern _MPU6050 imu;
extern SD_CARD SDcard;

#ifndef __UUCOBTypeDef_DEFINED
#define __UUCOBTypeDef_DEFINED
typedef struct
{
  uint8_t  port_num;
  int16_t  len;
  void*    address;
}USART_COB;
#endif

#ifndef __CCOBTypeDef_DEFINED
#define __CCOBTypeDef_DEFINED
/* CAN message data type(Communication Object/标准数据帧) */
typedef struct{
  uint16_t  ID;
  uint8_t   DLC;
  uint8_t   Data[8];
}COB_TypeDef;
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
