/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    bme280_app.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for BME280 Application functions.
  * @date    2021-02-18
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-18  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
         e.g:
         bme280.Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
         
      -# 获取气压, 温度, 湿度, 高度等数据
	     调用`bme280.Update_Data()`获取数据
		 
    @warning	
      -# 在气流变化较大的环境下, 气压计的读数不稳定
	  
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

/* Includes ------------------------------------------------------------------*/
#include "bme280_app.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BME280 bme280;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief BME280的延时函数
 *	@param[in] ms 延时时间/毫秒
 *  @return 0 success
						1 fail
 */
__weak void bme280_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

/**
 *  @brief BME280的IIC写函数
 *	@param[in] dev_addr 外设的IIC地址
 *	@param[in] reg_addr 寄存器地址
 *	@param[in] reg_data 要写入的数据
 *	@param[in] len 数据长度
 *  @return 0 success
						1 fail
 */
__weak int8_t bme280_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	uint8_t status;  
	status = IIC_Device_Write_Len(&bme280.IIC_PIN, dev_addr, reg_addr, len, reg_data);
	return status;
}

/**
 *  @brief BME280的IIC读函数
 *	@param[in] dev_addr 外设的IIC地址
 *	@param[in] reg_addr 寄存器地址
 *	@param[in] reg_data 存储数据的数组
 *	@param[in] len 数据长度
 *  @return 0 success
						1 fail
 */
__weak int8_t bme280_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	uint8_t status; 
	status = IIC_Device_Read_Len(&bme280.IIC_PIN, dev_addr, reg_addr, len, reg_data);
	return status;
} 

/**
 *  @brief BME280初始化
 *  @return 0 success
						1 fail
 */
 uint8_t BME280::Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx)
{
	uint8_t res;
	/* 初始化IIC对应端口 */
	IIC_Setting(&IIC_PIN, gpiox, scl_pinx, sda_pinx);
	
	/* 初始化BME280驱动 */
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = bme280_i2c_read;
  dev.write = bme280_i2c_write;
  dev.delay_ms = bme280_delay_ms;
  bme280_init(&dev);
	
	/* 配置工作模式 */
  res = stream_sensor_data_normal_mode(&dev);
	
	/* 计算初始高度 */
	dev.delay_ms(10);
	Cal_Init_Height();
	return res;
}

/**
 *  @brief 获取BME280的数据,存储在bme_data中
 *  @return 0 success
						1 fail
 */
uint8_t BME280::Update_Data(void)
{
	double pre;
	uint8_t res;
	res = bme280_get_sensor_data(BME280_ALL, &data, &dev);
	pre = pre_filter.f(data.pressure);
	data.altitude = Pre_To_Alt(&pre);
	height = height_filter.f(data.altitude - initial_height);
	return res;
}

/**
 *  @brief 配置BME280采样模式为过采样模式
 *  @note 使用该模式需要每次都配置一次模式,并等待一段时间再读取数据
 *	@param[in] dev BME280的驱动
 *  @return 0 success
						1 fail
 */
int8_t BME280::stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;
//  struct bme280_data comp_data;

  dev->settings.osr_h = BME280_OVERSAMPLING_2X;
  dev->settings.osr_p = BME280_OVERSAMPLING_4X;
  dev->settings.osr_t = BME280_OVERSAMPLING_4X;
  dev->settings.filter = BME280_FILTER_COEFF_2;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings(settings_sel, dev);

  /* Continuously stream sensor data */
//  while (1) {
//    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
//    /* Wait for the measurement to complete and print data @25Hz */
//    dev->delay_ms(40);
//    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
//    dev->delay_ms(1000);
//  }
	return rslt;
}

/**
 *  @brief 配置BME280采样模式为正常模式
 *  @note 	室内导航用该模式 
 *	@param[in] dev BME280的驱动
 *  @return 0 success
						1 fail
 */
int8_t BME280::stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;

  
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_1X;
  dev->settings.osr_t = BME280_OVERSAMPLING_1X;
  dev->settings.filter = BME280_FILTER_COEFF_16;
  dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  settings_sel = BME280_OSR_PRESS_SEL;
  settings_sel |= BME280_OSR_TEMP_SEL;
  settings_sel |= BME280_OSR_HUM_SEL;
  settings_sel |= BME280_STANDBY_SEL;
  settings_sel |= BME280_FILTER_SEL;
  rslt = bme280_set_sensor_settings(settings_sel, dev);
  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	return rslt;
}

/**
 *  @brief 将气压值转换为海拔
 *  @param[in] adcP 气压值(hPa)
 *  @return 海拔高度
 */
double BME280::Pre_To_Alt(double* pressure)
{	
    if(*pressure > 0) return 44330 * (pow((101570.0 / *pressure), 0.190295) - 1.0);
    else return 0;
}

/**
 *  @brief 利用伯努利定理计算实际高度
 *  @param[in] vel 气流速度
 *  @param[in] pressure 气压值
 *  @return 实际高度
 */
double BME280::Bernoulli_Height(double vel, double pressure)
{
	double const density = 1.293;	// 空气密度
	double const g = 9.80665;			// 重力加速度
	double C;											// 常数
	double height;								// 返回高度
	static double vel_last = vel;		
	static double pre_last = pressure;
	static double height_last = 0;
	
	/* 更新高度值 */
	C = pre_last + 0.5 * density * pow(vel_last, 2) + density * g * height_last;
	height = (C - pressure - 0.5 * density * pow(vel, 2)) / (density * g);
	
	/* 更新状态量 */
	vel_last = vel;
	pre_last = pressure;
	height_last = height;
	return height;
}

/**
 *  @brief 计算初始高度
 *  @return void
 */
void BME280::Cal_Init_Height(void)
{
	uint8_t const TIMES = 20;
	uint8_t count = TIMES;
	double sum = 0;
	while(count--)
	{
		bme280_get_sensor_data(BME280_ALL, &data, &dev);
		sum += data.pressure;
		dev.delay_ms(2);
	}
	sum /= TIMES;
	initial_height = Pre_To_Alt(&sum);
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
