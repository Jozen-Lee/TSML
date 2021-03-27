/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    mpl_cal.c
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPL calculate functions.
  * @date    2021-01-30
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-01-30  <td> 1.0     <td>LJY  			<td>Creator
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
		 
    @warning	
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
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
  * <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Data_Storer.h"
#include "Service_Devices.h"
#include "Vision.h"
#include "inertial_navigation.h"
/* Private define ------------------------------------------------------------*/
	
TaskHandle_t Data_CAM_Handle;
TaskHandle_t Data_IMU_Handle;
TaskHandle_t Data_NAV_Handle;

/* Private variables ---------------------------------------------------------*/
extern char data_path[36];	// 存储数据的文件路径
extern char photo_path[36];	// 存储图片的文件路径

/* 生成图片所需变量 */
Inc_Name<5, _BMP> photo("P");
Photo_Creator<ROW, COL> bmp;	
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Data_CAM(void *arg);
void Task_Data_IMU(void *arg);
void Task_Data_NAV(void *arg);

/* Private function prototypes -----------------------------------------------*/

/**
* @brief  飞镖数据存储器初始化
*/
void Data_Storer_Task_Init(void)
{
//	xTaskCreate(Task_Data_CAM,   "Task.Data_CAM",   			Large_Stack_Size,  NULL, PriorityRealtime,   			&Data_CAM_Handle	);
//	xTaskCreate(Task_Data_IMU,   "Task.Data_IMU",   			Large_Stack_Size,  NULL, PriorityHigh,      				&Data_IMU_Handle	);
//	xTaskCreate(Task_Data_NAV,   "Task.Data_NAV",   			Large_Stack_Size,  NULL, PriorityHigh,      				&Data_NAV_Handle	);
}

/**
* @brief  飞镖的相机数据存储任务
*/
void Task_Data_CAM(void *arg)
{
	/* 任务等待变量 */
	BaseType_t wait = pdFALSE;
	
	/* 定义相机的中心点数据的文件名 */
	char point_file[] = "center_point.csv";
	
	/* 定义中心点数据存储区 */
	char data[20];
	uint8_t len;
	
  /* Infinite loop */
  for(;;)
  {
		/* 等待相机数据更新 */
		wait = xSemaphoreTake(Camera_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* 写入图片数据 */
			SDcard.Write(photo_path, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size());
			
			/* 写入中心点数据 */
			len = sprintf(data, "%7.3f,%7.3f\r\n", vision.center.x, vision.center.y);
			SDcard.Write(data_path, point_file, (uint8_t *)data, len);			
		}		
  }	
}
/**
* @brief  飞镖的IMU数据存储任务
*/
void Task_Data_IMU(void *arg)
{
	/* 任务等待变量 */
	BaseType_t wait = pdFALSE;
	
	char accel_file[] = "accel.csv";
	char data[30];
	uint8_t len;
	/* Infinite loop */
  for(;;)
  {	
		/* 等待IMU数据更新 */
		wait = xSemaphoreTake(IMU_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* 写入加速度数据 */
//			len = sprintf(data, "%.2f,%.2f,%.2f\r\n", imu.data.g_com_accel[0], imu.data.g_com_accel[1], imu.data.g_com_accel[2]);
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", imu.data.pos.pitch, imu.data.pos.roll, imu.data.pos.yaw);
			SDcard.Write(data_path, accel_file, (uint8_t *)data, len);		
		}	
  }	
}
/**
* @brief  飞镖的惯导数据存储任务
*/
void Task_Data_NAV(void *arg)
{
	/* 任务等待变量 */
	BaseType_t wait = pdFALSE;
	
	char dis_file[] = "distance.csv";
	char speed_file[] = "speed.csv";
	char data[30];
	uint8_t len;
  /* Infinite loop */
  for(;;)
  {
		/* 等待惯导数据更新 */
		wait = xSemaphoreTake(NAV_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* 写入速度数据 */
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", nav.status.speed[0], nav.status.speed[1], nav.status.speed[2]);
			SDcard.Write(data_path, speed_file, (uint8_t *)data, len);
			
			/* 写入位移数据 */
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", nav.status.dis[0], nav.status.dis[1], nav.status.dis[2]);
			SDcard.Write(data_path, dis_file, (uint8_t *)data, len);		
		}	
  }	
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
