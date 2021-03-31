/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    mpl_cal.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPL calculate functions.
  ******************************************************************************
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

#ifndef __FLASH_H
#define __FLASH_H			    

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include "Drivers/Components/SPI/tutu_driver_spi.h"	 
	 
/* Private define ------------------------------------------------------------*/
	 
/*-------------------------------ָ���---------------------------------------*/
//W25Xϵ��/Qϵ��оƬ�б�	   
#define W25Q80 	0XEF14 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17
#define W25Q256 0XEF18

//���
#define W25X_WriteEnable			0x06 
#define W25X_WriteDisable			0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData					0x03 
#define W25X_FastReadData			0x0B 
#define W25X_FastReadDual			0x3B 
#define W25X_PageProgram			0x02 
#define W25X_BlockErase				0xD8 
#define W25X_SectorErase			0x20 
#define W25X_ChipErase				0xC7 
#define W25X_PowerDown				0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID					0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 

/* Private include -----------------------------------------------------------*/

#include <algorithm>
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
typedef uint8_t (*flash_readwrite_fptr_t)(uint8_t TxData);
typedef void (*flash_delay_fptr_t)(uint32_t period);
/* FLASH�����ṹ�� */
typedef struct
{
	/* CS������Ϣ */
	GPIO_TypeDef* CS_GPIO_PORT;
	uint32_t CS_GPIO_PIN;
	
	/* ��д���� */
	flash_readwrite_fptr_t ReadWriteByte;
	
	/* ��ʱ���� */
	flash_delay_fptr_t delay;
	
} flash_dev;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class myFLASH
{
public:
	uint16_t FLASH_ID;		//FLASH��ID
	
	uint8_t Init(uint16_t type, GPIO_TypeDef *cs_gpiox, uint32_t cs_pinx);						// ��ʼ��
	void Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);	// �����д��
	void Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);					// д��
	void Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   					// ��ȡ
	void Erase_Chip(void);    	  																										// ��Ƭ����
	void Erase_Sector(uint32_t Dst_Addr);																							// ��������
	void PowerDown(void);          																										// �������ģʽ
	void WAKEUP(void);			  																												// ����	
private:
	flash_dev dev;																																		// FLASH������
	uint8_t SPI_FLASH_BUF[4096];																											// д��FLASHʱ�����ݴ�����
	uint16_t ReadID(void);  	    																										// ��ȡFLASH ID
	uint8_t	 ReadSR(void);      					 	 																					// ��ȡ״̬�Ĵ��� 
	void Wait_Busy(void);           																									// �ȴ�����
	void Write_SR(uint8_t sr);  																											// д״̬�Ĵ���
	void Write_Enable(void);  																												// дʹ�� 
	void Write_Disable(void);																													// д����
	void Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);			// д��ҳ
		
};

/* Exported variables --------------------------------------------------------*/
extern myFLASH flash;
/* Exported function declarations --------------------------------------------*/

#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
