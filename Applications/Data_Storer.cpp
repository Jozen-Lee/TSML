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
      -# ��ʼ��
	     ����`MPU9250_Init()`���г�ʼ��
         e.g:
         MPU9250_Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
         
      -# ��ȡ���ٶȺͼ��ٶ�ԭʼ����
	     ����`MPU_Get_Gyroscope()`��ȡ���ٶ�
         e.g:
         if(!MPU_Get_Gyroscope(&MPU9250_IIC_PIN, &MPUData.gx, &MPUData.gy, &MPUData.gz))
         {
            MPUData.gx -= MPUData.gxoffset;
            MPUData.gy -= MPUData.gyoffset;
            MPUData.gz -= MPUData.gzoffset;
         }
		 
		 ����`MPU_Get_Accelerometer()`��ȡ���ٶ�
         e.g:
         MPU_Get_Accelerometer(&MPU9250_IIC_PIN,&MPUData.ax,&MPUData.ay,&MPUData.az);

      -# ��ȡ�Ƕ�
	     ����`mpu_dmp_get_data()`��ȡ�Ƕ�	 
         e.g:
         mpu_dmp_get_data(&MPUData.roll,&MPUData.pitch,&MPUData.yaw);
		 
    @warning	
      -# ���Ԥ�����`USE_FULL_ASSERT`�������ö��Լ�顣
      -# ��Ҫ`SRML`��`drv_i2c`֧�֡�
      -# ���IIC��IO����Ҫ���óɣ������ٿ�©����ģʽ��
	  
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
extern char data_path[36];	// �洢���ݵ��ļ�·��
extern char photo_path[36];	// �洢ͼƬ���ļ�·��

/* ����ͼƬ������� */
Inc_Name<5, _BMP> photo("P");
Photo_Creator<ROW, COL> bmp;	
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Data_CAM(void *arg);
void Task_Data_IMU(void *arg);
void Task_Data_NAV(void *arg);

/* Private function prototypes -----------------------------------------------*/

/**
* @brief  �������ݴ洢����ʼ��
*/
void Data_Storer_Task_Init(void)
{
//	xTaskCreate(Task_Data_CAM,   "Task.Data_CAM",   			Large_Stack_Size,  NULL, PriorityRealtime,   			&Data_CAM_Handle	);
//	xTaskCreate(Task_Data_IMU,   "Task.Data_IMU",   			Large_Stack_Size,  NULL, PriorityHigh,      				&Data_IMU_Handle	);
//	xTaskCreate(Task_Data_NAV,   "Task.Data_NAV",   			Large_Stack_Size,  NULL, PriorityHigh,      				&Data_NAV_Handle	);
}

/**
* @brief  ���ڵ�������ݴ洢����
*/
void Task_Data_CAM(void *arg)
{
	/* ����ȴ����� */
	BaseType_t wait = pdFALSE;
	
	/* ������������ĵ����ݵ��ļ��� */
	char point_file[] = "center_point.csv";
	
	/* �������ĵ����ݴ洢�� */
	char data[20];
	uint8_t len;
	
  /* Infinite loop */
  for(;;)
  {
		/* �ȴ�������ݸ��� */
		wait = xSemaphoreTake(Camera_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* д��ͼƬ���� */
			SDcard.Write(photo_path, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size());
			
			/* д�����ĵ����� */
			len = sprintf(data, "%7.3f,%7.3f\r\n", vision.center.x, vision.center.y);
			SDcard.Write(data_path, point_file, (uint8_t *)data, len);			
		}		
  }	
}
/**
* @brief  ���ڵ�IMU���ݴ洢����
*/
void Task_Data_IMU(void *arg)
{
	/* ����ȴ����� */
	BaseType_t wait = pdFALSE;
	
	char accel_file[] = "accel.csv";
	char data[30];
	uint8_t len;
	/* Infinite loop */
  for(;;)
  {	
		/* �ȴ�IMU���ݸ��� */
		wait = xSemaphoreTake(IMU_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* д����ٶ����� */
//			len = sprintf(data, "%.2f,%.2f,%.2f\r\n", imu.data.g_com_accel[0], imu.data.g_com_accel[1], imu.data.g_com_accel[2]);
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", imu.data.pos.pitch, imu.data.pos.roll, imu.data.pos.yaw);
			SDcard.Write(data_path, accel_file, (uint8_t *)data, len);		
		}	
  }	
}
/**
* @brief  ���ڵĹߵ����ݴ洢����
*/
void Task_Data_NAV(void *arg)
{
	/* ����ȴ����� */
	BaseType_t wait = pdFALSE;
	
	char dis_file[] = "distance.csv";
	char speed_file[] = "speed.csv";
	char data[30];
	uint8_t len;
  /* Infinite loop */
  for(;;)
  {
		/* �ȴ��ߵ����ݸ��� */
		wait = xSemaphoreTake(NAV_Data_SemaphoreHandle, portMAX_DELAY); 
		if(wait == pdTRUE)
		{
			/* д���ٶ����� */
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", nav.status.speed[0], nav.status.speed[1], nav.status.speed[2]);
			SDcard.Write(data_path, speed_file, (uint8_t *)data, len);
			
			/* д��λ������ */
			len = sprintf(data, "%7.3f,%7.3f,%7.3f\r\n", nav.status.dis[0], nav.status.dis[1], nav.status.dis[2]);
			SDcard.Write(data_path, dis_file, (uint8_t *)data, len);		
		}	
  }	
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
