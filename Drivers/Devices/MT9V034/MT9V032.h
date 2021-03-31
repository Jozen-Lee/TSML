/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    MT9V032.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MT9V032.
  ******************************************************************************
	 ==============================================================================
    @note
					OV7725接线定义：
					------------------------------------
						模块管脚            单片机管脚
						SDA                	A2
						SCL                	A3
						场中断              B7
            行中断              A4
						像素中断            A6

						D0                 	C6
            D1                 	C7
            D2                 	C8
            D3                 	C9
            D4                 	C11
            D5                 	B6
            D6                 	E5
            D7                 	E6

					USB转TTL接线定义:
					------------------------------------
						USB转TTL引脚        单片机引脚
						TX                  A10
						RX                  A9

	 ==============================================================================
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

#ifndef __MT9V032_H
#define __MT9V032_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>
#include "Drivers/Components/drv_uart.h"	 

/* Private define ------------------------------------------------------------*/

/* 定义图像大小 */
#define COL 80
#define ROW 60

/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
/* 摄像头命令枚举 */
typedef enum
{
	INIT = 0,               //摄像头初始化命令
	AUTO_EXP,               //自动曝光命令
	EXP_TIME,               //曝光时间命令
	FPS,                    //摄像头帧率命令
	SET_COL,                //图像列命令
	SET_ROW,                //图像行命令
	LR_OFFSET,              //图像左右偏移命令
	UD_OFFSET,              //图像上下偏移命令
	GAIN,                   //图像偏移命令
	CONFIG_FINISH,          //非命令位，主要用来占位计数

	SET_EXP_TIME = 0XF0,    //单独设置曝光时间命令
	GET_STATUS,             //获取摄像头配置命令
	GET_VERSION,            //固件版本号命令

	SET_ADDR = 0XFE,        //寄存器地址命令
	SET_DATA                //寄存器数据命令
} CMD;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class MT9V032
{
public:
	MT9V032(){}
	~MT9V032(){}
	uint8_t Init(UART_HandleTypeDef* _huart, DCMI_HandleTypeDef* _hdcmi);	// 初始化
	void Usart_Receive_Callback(UART_HandleTypeDef *_huart);							// 串口回调函数，放在串口的中断回调函数中
	void DCMI_Frame_Callback(DCMI_HandleTypeDef *_hdcmi);									// DCMI回调函数，放在DCMI的中断回调函数中
	void Send_Image(UART_HandleTypeDef * _huart);													// 发送图像信息到上位机 
	uint16_t Set_Exposure_Time(uint16_t light);														// 设置曝光时间
	uint8_t image[ROW][COL];																							// 图像缓存
	static const int width = COL;																					// 定义宽度变量，方便外界调用
	static const int height = ROW;																				// 定义高度变量，方便外界调用
	uint32_t framerate;																										// 记录已拍的帧数
private:
	void Get_Config(void);																								// 获取内部配置信息
	uint16_t Get_Version(void);																						// 获取固件版本
	uint16_t Set_Reg(uint8_t addr, uint16_t data);												// 写摄像头内部寄存器
	UART_HandleTypeDef* huart;																						// 串口句柄
	DCMI_HandleTypeDef* hdcmi;																						// DCMI句柄
	uint8_t finish_flag;																									// 摄像头工作完成标志
	uint8_t receive[3];																										// 接收数据的存储区
	uint8_t receive_num;																									// 串口接收数组的位置指针
	uint8_t uart_receive_flag;																						// 标志串口接收完成
	uint8_t rx_buff;																											// 串口接收的数据存储区
};	

/* Exported variables --------------------------------------------------------*/
extern MT9V032 mt9v032;

/* Exported function declarations --------------------------------------------*/


#endif	

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
