/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Anonymity_UpperMonitor.c
  * @author  LJY
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef __ANO_UPPERMONITOR_H
#define __ANO_UPPERMONITOR_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"
#include <algorithm>	

/* Private include -----------------------------------------------------------*/


/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class ANO_Upper
{
public:
	ANO_Upper(UART_HandleTypeDef* _huart) : huart(_huart){};
	~ANO_Upper(){};
	void Init(void)
	{
		huart->Init.BaudRate = 500000;
		HAL_UART_Init(huart);
	}
	void Send_Data(IMU_Data_t* Data, short csb,int prs);					// 发送数据
private:
	UART_HandleTypeDef* huart;																		// 串口句柄
	void usart_send_char(uint8_t c);															// 串口单字节发送
	uint8_t usart_report(uint8_t fun,uint8_t*data,uint8_t len);		// 串口发送数据
	void send_sensor_data(IMU_Data_t* Data);											// 发送传感器数据
	void send_pos_data(IMU_Data_t* Data, short csb,int prs);			// 发送姿态数据
};

/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

#endif
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
