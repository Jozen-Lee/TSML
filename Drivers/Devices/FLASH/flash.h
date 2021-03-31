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
	 
/*-------------------------------指令表---------------------------------------*/
//W25X系列/Q系列芯片列表	   
#define W25Q80 	0XEF14 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
#define W25Q128	0XEF17
#define W25Q256 0XEF18

//命令集
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
/* FLASH驱动结构体 */
typedef struct
{
	/* CS引脚信息 */
	GPIO_TypeDef* CS_GPIO_PORT;
	uint32_t CS_GPIO_PIN;
	
	/* 读写函数 */
	flash_readwrite_fptr_t ReadWriteByte;
	
	/* 延时函数 */
	flash_delay_fptr_t delay;
	
} flash_dev;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class myFLASH
{
public:
	uint16_t FLASH_ID;		//FLASH的ID
	
	uint8_t Init(uint16_t type, GPIO_TypeDef *cs_gpiox, uint32_t cs_pinx);						// 初始化
	void Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);	// 不检查写入
	void Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);					// 写入
	void Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   					// 读取
	void Erase_Chip(void);    	  																										// 整片擦除
	void Erase_Sector(uint32_t Dst_Addr);																							// 扇区擦除
	void PowerDown(void);          																										// 进入掉电模式
	void WAKEUP(void);			  																												// 唤醒	
private:
	flash_dev dev;																																		// FLASH的驱动
	uint8_t SPI_FLASH_BUF[4096];																											// 写入FLASH时用于暂存数据
	uint16_t ReadID(void);  	    																										// 读取FLASH ID
	uint8_t	 ReadSR(void);      					 	 																					// 读取状态寄存器 
	void Wait_Busy(void);           																									// 等待空闲
	void Write_SR(uint8_t sr);  																											// 写状态寄存器
	void Write_Enable(void);  																												// 写使能 
	void Write_Disable(void);																													// 写保护
	void Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);			// 写入页
		
};

/* Exported variables --------------------------------------------------------*/
extern myFLASH flash;
/* Exported function declarations --------------------------------------------*/

#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
