/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    mpu6050_config.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for MPU6050 config.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */   

#ifndef __MPU9250_CONFIG_H
#define __MPU9250_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/  
#include "mpu9250.h"
	 
/* Private define ------------------------------------------------------------*/
#define USE_MAG 0		/* 1 使用磁力计 
											 0 不使用磁力计*/
#define MAG_DATA_ADD 4096*4
/* Private include -----------------------------------------------------------*/
#include "mltypes.h"
#include "eMPL_outputs.h"
#include "storage_manager.h"
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/	 
typedef struct _MPUData
{
	/* 温度 */
	short temp;
	
	/* 加速度 */
	float ax;
	float ay;
	float az;
	
	/* 角速度 */
	float gx;
	float gy;
	float gz;
	
	/* 磁力方向 */
	float mx;
	float my;
	float mz;
	
	/* 姿态角 */
	float roll;
	float pitch;
	float yaw;
	
	/* 角速度补偿 */
	float gxoffset;
  float gyoffset;
  float gzoffset;
	
} MPUData_Typedef;


/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern IIC_PIN_Typedef MPU9250_IIC_PIN;
extern MPUData_Typedef MPU9250_Data;

/* Exported function declarations --------------------------------------------*/
uint8_t MPU9250_Init(GPIO_TypeDef *gpiox,uint32_t scl_pinx,uint32_t sda_pinx);
uint8_t MPU_Get_Mag_State(void);
uint8_t MPU_Mag_Init(void);
#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
