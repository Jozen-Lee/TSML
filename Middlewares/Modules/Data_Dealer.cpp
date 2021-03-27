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
#include "Data_Dealer.h"
#include "Vision.h"
#include "drv_timer.h"
#include "stdlib.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Data_Dealer data_dealer;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  数据存储器的初始化
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Init(void)
{
	uint8_t res;
	/* 定时器清零 */
	time_stamp = 0;
	
	/* 创建测试数据的存储文件夹 */
	SDcard.Mkdir("0:/", "DATA");
	sprintf(data_path, "%s/", SDcard.Inc_Mkdir("0:/DATA/", "TEST"));
	
	/* 创建测试数据中的图片的存储文件夹 */ 
	SDcard.Mkdir(data_path, "PHOTOS");
	sprintf(photo_path, "%sPHOTOS/", data_path);
	
	/* 文件名初始化 */
	sprintf(point_file, "%s", "center_point.csv");
	sprintf(condition_file, "%s", "condition.csv");
	
	/* 创建中心点存储文件 */
	len = sprintf(data, "%s", "t/ms,center_x,center_y\r\n");
	res = SDcard.Write(data_path, point_file, (uint8_t*)data, len);
	
	/* 创建飞行状态存储文件 */
	len = sprintf(data, "%s", "t/ms,acc_x,acc_y,acc_z,,speed_x,speed_y,speed_z,,dis_x,dis_y,dis_z,,pitch,roll,yaw\r\n");
	res = SDcard.Write(data_path, condition_file, (uint8_t*)data, len);
	
	/* 上位机初始化 */
	ano.Init();
	return res;
}

/**
  * @brief  存储图像信息到SD卡
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Image_Save(void)
{
	uint8_t res;
	/* 写入图片数据 */
	SDcard.Write(photo_path, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size());
	
	/* 写入中心点数据 */
	len = sprintf(data, "%6d,%8.3f,%8.3f\r\n", time_stamp, vision.center.x, vision.center.y);
	res = SDcard.Write(data_path, point_file, (uint8_t *)data, len);
	return res;
}

/**
  * @brief  存储飞行状态信息到SD卡
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Condition_Save(void)
{
	uint8_t res;
	
	/* 写入加速度数据 */
	len = sprintf(data, "\t%6d,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f\r\n", 
								time_stamp, imu.data.g_com_accel[0], imu.data.g_com_accel[1], imu.data.g_com_accel[2],		// 加速度
								nav.status.speed[0], nav.status.speed[1], nav.status.speed[2],														// 速度
								nav.status.dis[0], nav.status.dis[1], nav.status.dis[2],																	// 位移
								imu.data.pos.pitch, imu.data.pos.roll, imu.data.pos.yaw);																	// 姿态角
	res = SDcard.Write(data_path, condition_file, (uint8_t *)data, len);	
	return res;
}

/**
	* @brief  将状态信息发送至上位机
	* @param[in]  test_num 要看的实验序号
  * @return void
  */
void Data_Dealer::View_Condition(uint8_t test_num)
{
	static uint16_t count = 0;
	char temp[10];
	char path[20];
	
	sprintf(path, "%s%d/", "0:/Data/Test_", test_num);
	/** 读取SD卡中的姿态角数据 
		* 第一列字节数: 84
		* 第二列姿态角前的字节数: 121
		* 第二列全部字节数:	158
		*/
	SDcard.Read(path, "condition.csv", (uint8_t*)data, 84 + 101 + count * 132, 35);
	
	if(data[0])
	{
		/* 将姿态角数据存储在存储器中 */
		strncpy(temp, data+1, 8);
		view_data.pos.pitch = atof(temp);
		strncpy(temp, data+11, 8);
		view_data.pos.roll = atof(temp);
		strncpy(temp, data+21, 8);
		view_data.pos.yaw = atof(temp);
		count ++;
		
		/* 发送数据至上位机 */
		ano.Send_Data(&view_data, 0, 0);
	}
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
