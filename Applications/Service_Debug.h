/**
  **********************************************************************************
  * @file   : Service_Debug.h
  * @brief  : Debug support file.This file provides access ports to debug.
  **********************************************************************************
  *  
**/
#ifndef  _SERVICE_DEBUG_H_
#define  _SERVICE_DEBUG_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#ifdef  __cplusplus
extern "C"{
#endif
/* Private macros ------------------------------------------------------------*/
//#define USE_LAB_UPPER 					// 使用实验室上位机
#define USE_ANO_UPPER 						// 使用匿名上位机
	
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern TaskHandle_t Debug_Handle;

/* Exported function declarations --------------------------------------------*/
void Service_Debug_Init(void);

#ifdef  __cplusplus
}
#endif
#endif 
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
