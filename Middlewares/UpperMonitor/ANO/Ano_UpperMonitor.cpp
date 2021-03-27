/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Anonymity_UpperMonitor.c
  * @author  LJY 
  * @brief   Code for Upper monitor supported by Jozen in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log：
  * <table
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2021-01-28  <td> 1.0     <td>LJY  					<td>Creator 
  *
  ==============================================================================
                      ##### How to use this driver #####
  ==============================================================================
    @note

    @warning
     

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
	
/***********************上位机调参使用***********************/
/* 在这里extern需要使用的变量和需要包含的头文件 */
#include <Ano_UpperMonitor.h>

/* Includes ------------------------------------------------------------------*/ 
#include "System_Datapool.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  串口1发送1个字符 
  * @param  c:要发送的字符
  */
void ANO_Upper::usart_send_char(uint8_t c)
{
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}; 
	USART1->DR=c;  
} 

/**
  * @brief  传送数据给匿名四轴上位机软件(V2.6版本)
  * @param  fun:  功能字. 0X01~0X1C
	* @param  data: 数据缓存区,最多27字节
	* @param  len:  data区有效数据个数
  * @retval 0, 成功
						1, 失败
  */
uint8_t ANO_Upper::usart_report(uint8_t fun,uint8_t*data,uint8_t len)
{
	uint8_t i;
	uint8_t send_buf[32];																				//存储发送数据
	if(len>=28) return 1;																				//最多27字节数据 
	send_buf[len+4]=0;																					//校验数置零
	send_buf[0]=0XAA;																						//帧头
	send_buf[1]=0XAA;																						//帧头
	send_buf[2]=fun;																						//功能字
	send_buf[3]=len;																						//数据长度
	for(i=0;i<len;i++)send_buf[4+i]=data[i];										//复制数据
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];						//计算校验和	
	for(i=0;i<len+5;i++)usart_send_char(send_buf[i]);						//发送数据到串口
	return 0;
}

/**
  * @brief  发送加速度传感器数据+陀螺仪数据+地磁计数据(传感器帧)
	* @param 	huart: 串口句柄
	* @param  Data:  IMU数据
  * @retval 0, 成功
						1, 失败
  */
void ANO_Upper::send_sensor_data(IMU_Data_t* Data)
{
	uint8_t tbuf[18]; 
	/* 加速度数据 ---------*/
	tbuf[0]=((uint16_t)Data->accel[0]>>8)&0XFF;
	tbuf[1]=(uint16_t)Data->accel[0]&0XFF;
	tbuf[2]=((uint16_t)Data->accel[1]>>8)&0XFF;
	tbuf[3]=(uint16_t)Data->accel[1]&0XFF;
	tbuf[4]=((uint16_t)Data->accel[2]>>8)&0XFF;
	tbuf[5]=(uint16_t)Data->accel[2]&0XFF; 
	
	/* 角速度数据 ---------*/
	tbuf[6]=((uint16_t)Data->gyro[0]>>8)&0XFF;
	tbuf[7]=(uint16_t)Data->gyro[0]&0XFF;
	tbuf[8]=((uint16_t)Data->gyro[1]>>8)&0XFF;
	tbuf[9]=(uint16_t)Data->gyro[1]&0XFF;
	tbuf[10]=((uint16_t)Data->gyro[2]>>8)&0XFF;
	tbuf[11]=(uint16_t)Data->gyro[2]&0XFF;
	
	/* 磁力计数据 ---------*/
	tbuf[12]=((uint16_t)Data->compass[0]>>8)&0XFF;
	tbuf[13]=(uint16_t)Data->compass[0]&0XFF;
	tbuf[14]=((uint16_t)Data->compass[1]>>8)&0XFF;
	tbuf[15]=(uint16_t)Data->compass[1]&0XFF;
	tbuf[16]=((uint16_t)Data->compass[2]>>8)&0XFF;
	tbuf[17]=(uint16_t)Data->compass[2]&0XFF;
	
	/* 发送至上位机 -------*/
	usart_report(0X02,tbuf,18);	//传感器帧,0X02
}	

/**
  * @brief  通过串口上报结算后的姿态数据给电脑(状态帧)
  * @param  Data:  IMU数据, 输入和输出单位为°
	* @param  csb:超声波高度,输入单位为m, 输出单位为cm
	* @param  prs:气压计高度,输入单位为m, 输出单位为mm
  * @retval 0, 成功
						1, 失败
  */
void ANO_Upper::send_pos_data(IMU_Data_t* Data, short csb,	int prs)
{
	uint8_t tbuf[12]; 
	int roll, pitch, yaw, _csb, _prs;
	
	/* 获取姿态角和高度 */
	roll = Data->pos.roll * 100;
	pitch = Data->pos.pitch * 100;
	yaw = Data->pos.yaw * 100;
	_csb = csb * 100;
	_prs = prs * 1000;

	/* 姿态角数据 --------------*/
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	
	/* 高度数据 ---------------*/
	tbuf[6]=(_csb>>8)&0XFF;
	tbuf[7]=_csb&0XFF;
	tbuf[8]=(_prs>>24)&0XFF;
	tbuf[9]=(_prs>>16)&0XFF;
	tbuf[10]=(_prs>>8)&0XFF;
	tbuf[11]=_prs&0XFF;
	
	/* 发送至上位机 -------*/
	usart_report(0X01,tbuf,12);	//状态帧,0X01
}  

/**
  * @brief  发送数据到匿名上位机,MPU数据版本
  * @param  Data:  IMU数据, 输入和输出单位为°
	* @param  csb:超声波高度,输入单位为m, 输出单位为cm
	* @param  prs:气压计高度,输入单位为m, 输出单位为mm
  * @retval void
  */
void ANO_Upper::Send_Data(IMU_Data_t* Data, short csb,	int prs)
{
	/* 发送传感器数据 */
	send_sensor_data(Data);
	
	/* 发送姿态数据 */
	send_pos_data(Data, csb, prs);
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
