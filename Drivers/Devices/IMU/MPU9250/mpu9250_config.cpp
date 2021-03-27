/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    mpu9250_config.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPU9250 config.
  * @date    2021-01-30
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2018-10-18  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
	     调用`MPU9250_Init()`进行初始化
         e.g:
         MPU9250_Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
         
      -# 获取角速度和加速度原始数据
	     调用`MPU_Get_Gyroscope()`获取角速度
         e.g:
         if(!MPU_Get_Gyroscope(&MPU9250_IIC_PIN, &MPUData.gx, &MPUData.gy, &MPUData.gz))
         {
            MPUData.gx -= MPUData.gxoffset;
            MPUData.gy -= MPUData.gyoffset;
            MPUData.gz -= MPUData.gzoffset;
         }
			 
			 调用`MPU_Get_Accelerometer()`获取加速度
					 e.g:
					 MPU_Get_Accelerometer(&MPU9250_IIC_PIN,&MPUData.ax,&MPUData.ay,&MPUData.az);

      -# 获取角度
	     调用`mpu_dmp_get_data()`获取角度	 
         e.g:
         mpu_dmp_get_data(&MPUData.roll,&MPUData.pitch,&MPUData.yaw);
		 
			-# 开启磁力计校准
				调用`MPU_Mag_Init()`
    @warning	
      -# 磁力计校准需要`flash`支持,且校准后,获取的航向角会在5度内波动,但不会飘。
			-# 磁力计校准前,单片机启动时mpu的y轴所在位置即为0度位置,校准后,航向角即为y轴与地球正北方向的夹角,与启动位置无关。
      -# 需要`SRML`的`drv_i2c`支持。
      -# 软件IIC的IO口需要配置成：超高速开漏上拉模式。
	  
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
/* Includes ------------------------------------------------------------------*/
#include "mpu9250_config.h"
#include <stdio.h>
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IIC_PIN_Typedef MPU9250_IIC_PIN;
MPUData_Typedef MPU9250_Data;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  MPU9250 initialization
  * @param  gpiox: MPU9250 iic gpio port
  * @param  scl_pinx: MPU9250 iic scl pin
  * @param  sda_pinx: MPU9250 iic sda pin
  * @retval 0:success
  *         1:fail   
  */
//uint8_t MPU9250_Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx)
//{
//    /* MPU6050_IIC initialization */
//		IIC_Setting(&MPU9250_IIC_PIN, gpiox, scl_pinx, sda_pinx);

//    /* MPU9250 initialization */
//    if (mpu_dmp_init() != 0)
//    {
//        HAL_Delay(100);
//        __set_FAULTMASK(1); //reset
//        NVIC_SystemReset();
//    }
//    MPU_Get_Gyroscope_Init(&MPU9250_IIC_PIN, &MPU9250_Data.gxoffset, &MPU9250_Data.gyoffset, &MPU9250_Data.gzoffset);
//    return 0;
//}

#if USE_MAG
#include "flash.h"
/**
  * @brief  获取磁力计校准状态
  * @retval 0,success
  *         1,fail
  */
/* 读取磁力计状态的变量 */
inv_time_t timestamp[1];	
long data[3];
int8_t accuracy[1];
uint8_t MPU_Get_Mag_State(void)
{
	inv_get_sensor_type_euler(data, accuracy, timestamp);
	if(accuracy[0] == 3) return 0;
	else return 1;
}

/**
  * @brief  磁力计校准
	* @param  _flash: FLASH类指针
  * @retval void
  */
uint8_t MPU_Mag_Init(void)
{
	uint32_t out_time = 0;
	const size_t len = 124;	
	uint8_t flag;	
	uint8_t mag_data[len];

	/* 获取磁力计状态 */
	flag = MPU_Get_Mag_State();
	
	if(flag == 0)
	{
		/* 将校准数据写入FLASH */
		inv_save_mpl_states(mag_data, len);
		SPI_Flash_Write(mag_data, MAG_DATA_ADD, len);
		return 0;
	}
	else
	{
		/* 读取FLASH数据, 看是否已有校准数据 */
		SPI_Flash_Read(mag_data, MAG_DATA_ADD, len);
		inv_load_mpl_states(mag_data,len);
		
		/* 循环判断校准 */
		while(out_time < 65535)
		{
			out_time ++;
			mpu_mpl_get_data(&MPU9250_Data.pitch,&MPU9250_Data.roll,&MPU9250_Data.yaw);	//这句话一定要有,否则无法知道是否校准
			flag = MPU_Get_Mag_State();
			if(!flag)
			{
				/* 将校准数据写入FLASH */
				inv_save_mpl_states(mag_data, len);
				SPI_Flash_Write(mag_data, MAG_DATA_ADD, len);
				return 0;
			}
		}
		return 1;
	}
}

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
