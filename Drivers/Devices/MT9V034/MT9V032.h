/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    MT9V032.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MT9V032.
  ******************************************************************************
	 ==============================================================================
    @note
					OV7725���߶��壺
					------------------------------------
						ģ��ܽ�            ��Ƭ���ܽ�
						SDA                	A2
						SCL                	A3
						���ж�              B7
            ���ж�              A4
						�����ж�            A6

						D0                 	C6
            D1                 	C7
            D2                 	C8
            D3                 	C9
            D4                 	C11
            D5                 	B6
            D6                 	E5
            D7                 	E6

					USBתTTL���߶���:
					------------------------------------
						USBתTTL����        ��Ƭ������
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

/* ����ͼ���С */
#define COL 80
#define ROW 60

/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
/* ����ͷ����ö�� */
typedef enum
{
	INIT = 0,               //����ͷ��ʼ������
	AUTO_EXP,               //�Զ��ع�����
	EXP_TIME,               //�ع�ʱ������
	FPS,                    //����ͷ֡������
	SET_COL,                //ͼ��������
	SET_ROW,                //ͼ��������
	LR_OFFSET,              //ͼ������ƫ������
	UD_OFFSET,              //ͼ������ƫ������
	GAIN,                   //ͼ��ƫ������
	CONFIG_FINISH,          //������λ����Ҫ����ռλ����

	SET_EXP_TIME = 0XF0,    //���������ع�ʱ������
	GET_STATUS,             //��ȡ����ͷ��������
	GET_VERSION,            //�̼��汾������

	SET_ADDR = 0XFE,        //�Ĵ�����ַ����
	SET_DATA                //�Ĵ�����������
} CMD;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class MT9V032
{
public:
	MT9V032(){}
	~MT9V032(){}
	uint8_t Init(UART_HandleTypeDef* _huart, DCMI_HandleTypeDef* _hdcmi);	// ��ʼ��
	void Usart_Receive_Callback(UART_HandleTypeDef *_huart);							// ���ڻص����������ڴ��ڵ��жϻص�������
	void DCMI_Frame_Callback(DCMI_HandleTypeDef *_hdcmi);									// DCMI�ص�����������DCMI���жϻص�������
	void Send_Image(UART_HandleTypeDef * _huart);													// ����ͼ����Ϣ����λ�� 
	uint16_t Set_Exposure_Time(uint16_t light);														// �����ع�ʱ��
	uint8_t image[ROW][COL];																							// ͼ�񻺴�
	static const int width = COL;																					// �����ȱ���������������
	static const int height = ROW;																				// ����߶ȱ���������������
	uint32_t framerate;																										// ��¼���ĵ�֡��
private:
	void Get_Config(void);																								// ��ȡ�ڲ�������Ϣ
	uint16_t Get_Version(void);																						// ��ȡ�̼��汾
	uint16_t Set_Reg(uint8_t addr, uint16_t data);												// д����ͷ�ڲ��Ĵ���
	UART_HandleTypeDef* huart;																						// ���ھ��
	DCMI_HandleTypeDef* hdcmi;																						// DCMI���
	uint8_t finish_flag;																									// ����ͷ������ɱ�־
	uint8_t receive[3];																										// �������ݵĴ洢��
	uint8_t receive_num;																									// ���ڽ��������λ��ָ��
	uint8_t uart_receive_flag;																						// ��־���ڽ������
	uint8_t rx_buff;																											// ���ڽ��յ����ݴ洢��
};	

/* Exported variables --------------------------------------------------------*/
extern MT9V032 mt9v032;

/* Exported function declarations --------------------------------------------*/


#endif	

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
