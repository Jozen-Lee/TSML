/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    Anonymity_UpperMonitor.c
  * @author  LJY 
  * @brief   Code for Upper monitor supported by Jozen in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log��
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
	
/***********************��λ������ʹ��***********************/
/* ������extern��Ҫʹ�õı�������Ҫ������ͷ�ļ� */
#include <Ano_UpperMonitor.h>

/* Includes ------------------------------------------------------------------*/ 
#include "System_Datapool.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  ����1����1���ַ� 
  * @param  c:Ҫ���͵��ַ�
  */
void ANO_Upper::usart_send_char(uint8_t c)
{
	while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)==RESET){}; 
	USART1->DR=c;  
} 

/**
  * @brief  �������ݸ�����������λ�����(V2.6�汾)
  * @param  fun:  ������. 0X01~0X1C
	* @param  data: ���ݻ�����,���27�ֽ�
	* @param  len:  data����Ч���ݸ���
  * @retval 0, �ɹ�
						1, ʧ��
  */
uint8_t ANO_Upper::usart_report(uint8_t fun,uint8_t*data,uint8_t len)
{
	uint8_t i;
	uint8_t send_buf[32];																				//�洢��������
	if(len>=28) return 1;																				//���27�ֽ����� 
	send_buf[len+4]=0;																					//У��������
	send_buf[0]=0XAA;																						//֡ͷ
	send_buf[1]=0XAA;																						//֡ͷ
	send_buf[2]=fun;																						//������
	send_buf[3]=len;																						//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];										//��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];						//����У���	
	for(i=0;i<len+5;i++)usart_send_char(send_buf[i]);						//�������ݵ�����
	return 0;
}

/**
  * @brief  ���ͼ��ٶȴ���������+����������+�شż�����(������֡)
	* @param 	huart: ���ھ��
	* @param  Data:  IMU����
  * @retval 0, �ɹ�
						1, ʧ��
  */
void ANO_Upper::send_sensor_data(IMU_Data_t* Data)
{
	uint8_t tbuf[18]; 
	/* ���ٶ����� ---------*/
	tbuf[0]=((uint16_t)Data->accel[0]>>8)&0XFF;
	tbuf[1]=(uint16_t)Data->accel[0]&0XFF;
	tbuf[2]=((uint16_t)Data->accel[1]>>8)&0XFF;
	tbuf[3]=(uint16_t)Data->accel[1]&0XFF;
	tbuf[4]=((uint16_t)Data->accel[2]>>8)&0XFF;
	tbuf[5]=(uint16_t)Data->accel[2]&0XFF; 
	
	/* ���ٶ����� ---------*/
	tbuf[6]=((uint16_t)Data->gyro[0]>>8)&0XFF;
	tbuf[7]=(uint16_t)Data->gyro[0]&0XFF;
	tbuf[8]=((uint16_t)Data->gyro[1]>>8)&0XFF;
	tbuf[9]=(uint16_t)Data->gyro[1]&0XFF;
	tbuf[10]=((uint16_t)Data->gyro[2]>>8)&0XFF;
	tbuf[11]=(uint16_t)Data->gyro[2]&0XFF;
	
	/* ���������� ---------*/
	tbuf[12]=((uint16_t)Data->compass[0]>>8)&0XFF;
	tbuf[13]=(uint16_t)Data->compass[0]&0XFF;
	tbuf[14]=((uint16_t)Data->compass[1]>>8)&0XFF;
	tbuf[15]=(uint16_t)Data->compass[1]&0XFF;
	tbuf[16]=((uint16_t)Data->compass[2]>>8)&0XFF;
	tbuf[17]=(uint16_t)Data->compass[2]&0XFF;
	
	/* ��������λ�� -------*/
	usart_report(0X02,tbuf,18);	//������֡,0X02
}	

/**
  * @brief  ͨ�������ϱ���������̬���ݸ�����(״̬֡)
  * @param  Data:  IMU����, ����������λΪ��
	* @param  csb:�������߶�,���뵥λΪm, �����λΪcm
	* @param  prs:��ѹ�Ƹ߶�,���뵥λΪm, �����λΪmm
  * @retval 0, �ɹ�
						1, ʧ��
  */
void ANO_Upper::send_pos_data(IMU_Data_t* Data, short csb,	int prs)
{
	uint8_t tbuf[12]; 
	int roll, pitch, yaw, _csb, _prs;
	
	/* ��ȡ��̬�Ǻ͸߶� */
	roll = Data->pos.roll * 100;
	pitch = Data->pos.pitch * 100;
	yaw = Data->pos.yaw * 100;
	_csb = csb * 100;
	_prs = prs * 1000;

	/* ��̬������ --------------*/
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	
	/* �߶����� ---------------*/
	tbuf[6]=(_csb>>8)&0XFF;
	tbuf[7]=_csb&0XFF;
	tbuf[8]=(_prs>>24)&0XFF;
	tbuf[9]=(_prs>>16)&0XFF;
	tbuf[10]=(_prs>>8)&0XFF;
	tbuf[11]=_prs&0XFF;
	
	/* ��������λ�� -------*/
	usart_report(0X01,tbuf,12);	//״̬֡,0X01
}  

/**
  * @brief  �������ݵ�������λ��,MPU���ݰ汾
  * @param  Data:  IMU����, ����������λΪ��
	* @param  csb:�������߶�,���뵥λΪm, �����λΪcm
	* @param  prs:��ѹ�Ƹ߶�,���뵥λΪm, �����λΪmm
  * @retval void
  */
void ANO_Upper::Send_Data(IMU_Data_t* Data, short csb,	int prs)
{
	/* ���ʹ��������� */
	send_sensor_data(Data);
	
	/* ������̬���� */
	send_pos_data(Data, csb, prs);
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
