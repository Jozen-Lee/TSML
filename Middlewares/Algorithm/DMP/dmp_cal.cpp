/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    dmp_cal.c
  * @author  LJY 2250017028@qq.com
  * @brief   Code for DMP calculate functions.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-03-13  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
		 
    @warning	
      -# 
	  
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
#include "dmp_cal.h"

#include <math.h>
/* Private define ------------------------------------------------------------*/
#define q30  1073741824.0f
#define q16  65536.0f
/* Private variables ---------------------------------------------------------*/
DMP dmp_lib;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief   角速度范围的初始化
 * 	@param[out] gxoffset, gyoffset, gzoffset 三轴角速度原始数据
 *  @return  0 success
						 1 fail
 */
void DMP::Gyroscope_Init(IIC_PIN_Typedef* iic_pin, float* gxoffset, float* gyoffset, float* gzoffset)
{
	unsigned char buf[6];
	short gx,gy,gz=0;
	int i=0,cnt=0,sum_x=0,sum_y=0,sum_z=0;
	
	for(i = 0;i < 1024 ;i++)
	{
		if(IIC_Device_Read_Len(iic_pin, MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf) == 0)
		{
      gx=(short)(((unsigned short int)buf[0]<<8)|buf[1]);
      gy=(short)(((unsigned short int)buf[2]<<8)|buf[3]);
      gz=(short)(((unsigned short int)buf[4]<<8)|buf[5]);
			
			/* 前300次数据不用 */
			if(i>300)
			{
        sum_x += gx;
        sum_y += gy;
				sum_z += gz;
				cnt ++;
			}
		}
	}
	*gxoffset = ((float)sum_x)/((float)cnt);
  *gyoffset = ((float)sum_y)/((float)cnt);
  *gzoffset = ((float)sum_z)/((float)cnt);
}

/**
 *  @brief 获取角速度  
 * 	@param[out] gx,gy,gz 三轴角速度原始数据
 *  @return  0 success
						 1 fail
 */
uint8_t DMP::Get_Gyro(float* gx, float* gy, float* gz)
{
	uint8_t res;
	short data[3];
	res = mpu_get_gyro_reg(data, NULL);
	*gx = data[0]; 
	*gy = data[1]; 
	*gz = data[2]; 	
	return res;
}

/**
 *  @brief 获取加速度  
 * 	@param[out] ax,ay,az 三轴加速度原始数据
 *  @return  0 success
						 1 fail
 */
uint8_t DMP::Get_Accel(float* ax, float* ay, float* az)
{
	uint8_t res;
	short data[3];
	res = mpu_get_accel_reg(data, NULL);
	*ax = data[0]; 
	*ay = data[1]; 
	*az = data[2]; 
	return res;
}

/**
 *  @brief 获取温度  
 * 	@param[out] temp 温度
 *  @return  0 success
						 1 fail
 */
#define TEMP_CONVERSION 1.52587890625e-005f // 1 / 2^16
uint8_t DMP::Get_Tempreture(float* temp)
{
	uint8_t res;
	long data; 
	res = mpu_get_temperature(&data, NULL);
	*temp = data * TEMP_CONVERSION;
	return res;
}

/**
 *  @brief 获取姿态 
 * 	@param[out] pitch，roll，yaw 姿态角
 *  @return  0 success
						 1 fail
 */
uint8_t DMP::Get_Pos(float* pitch, float* roll, float* yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;	 
	
	/* 获取姿态角 */
	if(sensors&INV_WXYZ_QUAT) 
	{
		/* q30格式转换为浮点数 */
		q0 = quat[0] / q30;
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		
		/* 计算得到姿态角 */
	 *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3f;																	
	 *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	
	 *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3f;								
	}
	else return 1;	
	return 0;
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
