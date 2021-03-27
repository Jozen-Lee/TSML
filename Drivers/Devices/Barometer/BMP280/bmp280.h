/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    bmp280.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for BMP280.
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

#ifndef __BMP280_H
#define __BMP280_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"	 
#include "drv_i2c.h"

/* Private define ------------------------------------------------------------*/
#define BMP280_ADDRESS 0xEC  	//器件地址
		 
#define BMP280_I2C_ADDR								(0x76)
#define BMP280_DEFAULT_CHIP_ID				(0x58)
#define BME280_DEFAULT_CHIP_ID				(0x60)
	 
#define BMP280_CHIP_ID								(0xD0)  /* Chip ID Register */
#define BMP280_RST_REG								(0xE0)  /* Softreset Register */
#define BMP280_STAT_REG								(0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG					(0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG							(0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG				(0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG				(0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG			(0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0xFC)  /* Temperature XLSB Reg */

#define BMP280_SLEEP_MODE							(0x00)
#define BMP280_FORCED_MODE						(0x01)
#define BMP280_NORMAL_MODE						(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE															(6)

#define BMP280_OVERSAMP_SKIPPED				(0x00)
#define BMP280_OVERSAMP_1X						(0x01)
#define BMP280_OVERSAMP_2X						(0x02)
#define BMP280_OVERSAMP_4X						(0x03)
#define BMP280_OVERSAMP_8X						(0x04)
#define BMP280_OVERSAMP_16X						(0x05)		
	
/* Private include -----------------------------------------------------------*/
#include "my_filters.h"	 
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/

/* 存储校准数据 */
typedef struct 
{
    uint16_t dig_T1;	/* calibration T1 data */
    int16_t dig_T2; 	/* calibration T2 data */
    int16_t dig_T3; 	/* calibration T3 data */
    int16_t dig_P1;		/* calibration P1 data */
    int16_t dig_P2; 	/* calibration P2 data */
    int16_t dig_P3; 	/* calibration P3 data */
    int16_t dig_P4;	 	/* calibration P4 data */
    int16_t dig_P5; 	/* calibration P5 data */
    int16_t dig_P6; 	/* calibration P6 data */
    int16_t dig_P7; 	/* calibration P7 data */
    int16_t dig_P8; 	/* calibration P8 data */
    int16_t dig_P9; 	/* calibration P9 data */
    int32_t t_fine; 	/* calibration t_fine data */
} BMP280_Calib_t;

/* 存储BMP280获取的数据 */
typedef struct 
{
	float pressure;
	float temperature;
	float asl;
} BMP280_Data_t;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class BMP280
{
public:
	BMP280(): pre_filter(0.2f){}
	~BMP280(){}
	
	uint8_t Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx);
	void Update_Data(void);
	BMP280_Data_t data;							// 数据存储结构体
private:
	IIC_PIN_Typedef IIC_PIN;				// IIC接口
	BMP280_Calib_t  BMP280_Cal;			// 校准数据存取
	int32_t BMP280RawPressure;			// 存储未处理的气压值
	int32_t BMP280RawTemperature;	 	// 存储未处理的温度值
	Lim_MeanFilter<5> pre_filter;		// 气压值的滤波器，初始化在创建BMP280对象的初始化中

	float Pre_To_Alt(float* pressure);
	void BMP280_Get_Data(void);
	uint32_t BMP280_Compensate_T(int32_t adcT);
	uint32_t BMP280_Compensate_P(int32_t adcP);
};

/* Exported variables --------------------------------------------------------*/
extern BMP280 bmp280;	

/* Exported function declarations --------------------------------------------*/

#endif	

#endif	

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
