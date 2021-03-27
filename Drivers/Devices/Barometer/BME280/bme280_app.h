/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    bme280_app.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for BME280 Application functions.
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

#ifndef __BME280_APP_H
#define __BME280_APP_H

/* Includes ------------------------------------------------------------------*/  
#include "my_filters.h"

#ifdef __cplusplus
#include <algorithm>

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <math.h>
#include "bme280.h"
#include "drv_i2c.h"	 
	 
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class BME280
{
public:
	BME280(){}
	~BME280(){}
	uint8_t Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx);	// 初始化
	uint8_t Update_Data(void);																								// 更新数据
	double Bernoulli_Height(double vel, double pressure);											// 应用伯努利定理算高度 
	struct bme280_data data;																									// 数据存储结构体变量
	double height;																														// 相对高度值(以启动位置为原点)

private:
	double initial_height;																										// 起始高度			
	struct bme280_dev dev;																										// 驱动的结构体变量
	IIC_PIN_Typedef IIC_PIN;																									// IIC接口
	MeanFilter<5> pre_filter;																									// 气压值的滤波器	
	MeanFilter<5> height_filter;																							// 高度滤波器

	/* IIC驱动函数 */
	friend int8_t bme280_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
	friend int8_t bme280_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
	friend void bme280_delay_ms(uint32_t ms);
	
	/* 模式配置 */
	int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);
	int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);

	/* 海拔计算 */
	double Pre_To_Alt(double* pressure);

	/* 计算起始高度 */
	void Cal_Init_Height(void);
};

/* Exported variables --------------------------------------------------------*/	
extern BME280 bme280;
/* Exported function declarations --------------------------------------------*/


#endif	
#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
