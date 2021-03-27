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
  * @brief  ���ݴ洢���ĳ�ʼ��
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Init(void)
{
	uint8_t res;
	/* ��ʱ������ */
	time_stamp = 0;
	
	/* �����������ݵĴ洢�ļ��� */
	SDcard.Mkdir("0:/", "DATA");
	sprintf(data_path, "%s/", SDcard.Inc_Mkdir("0:/DATA/", "TEST"));
	
	/* �������������е�ͼƬ�Ĵ洢�ļ��� */ 
	SDcard.Mkdir(data_path, "PHOTOS");
	sprintf(photo_path, "%sPHOTOS/", data_path);
	
	/* �ļ�����ʼ�� */
	sprintf(point_file, "%s", "center_point.csv");
	sprintf(condition_file, "%s", "condition.csv");
	
	/* �������ĵ�洢�ļ� */
	len = sprintf(data, "%s", "t/ms,center_x,center_y\r\n");
	res = SDcard.Write(data_path, point_file, (uint8_t*)data, len);
	
	/* ��������״̬�洢�ļ� */
	len = sprintf(data, "%s", "t/ms,acc_x,acc_y,acc_z,,speed_x,speed_y,speed_z,,dis_x,dis_y,dis_z,,pitch,roll,yaw\r\n");
	res = SDcard.Write(data_path, condition_file, (uint8_t*)data, len);
	
	/* ��λ����ʼ�� */
	ano.Init();
	return res;
}

/**
  * @brief  �洢ͼ����Ϣ��SD��
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Image_Save(void)
{
	uint8_t res;
	/* д��ͼƬ���� */
	SDcard.Write(photo_path, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size());
	
	/* д�����ĵ����� */
	len = sprintf(data, "%6d,%8.3f,%8.3f\r\n", time_stamp, vision.center.x, vision.center.y);
	res = SDcard.Write(data_path, point_file, (uint8_t *)data, len);
	return res;
}

/**
  * @brief  �洢����״̬��Ϣ��SD��
	* @param void 
  * @return 0 success
  *         1 fail
  */
uint8_t Data_Dealer::Condition_Save(void)
{
	uint8_t res;
	
	/* д����ٶ����� */
	len = sprintf(data, "\t%6d,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f,,\t%8.3f,\t%8.3f,\t%8.3f\r\n", 
								time_stamp, imu.data.g_com_accel[0], imu.data.g_com_accel[1], imu.data.g_com_accel[2],		// ���ٶ�
								nav.status.speed[0], nav.status.speed[1], nav.status.speed[2],														// �ٶ�
								nav.status.dis[0], nav.status.dis[1], nav.status.dis[2],																	// λ��
								imu.data.pos.pitch, imu.data.pos.roll, imu.data.pos.yaw);																	// ��̬��
	res = SDcard.Write(data_path, condition_file, (uint8_t *)data, len);	
	return res;
}

/**
	* @brief  ��״̬��Ϣ��������λ��
	* @param[in]  test_num Ҫ����ʵ�����
  * @return void
  */
void Data_Dealer::View_Condition(uint8_t test_num)
{
	static uint16_t count = 0;
	char temp[10];
	char path[20];
	
	sprintf(path, "%s%d/", "0:/Data/Test_", test_num);
	/** ��ȡSD���е���̬������ 
		* ��һ���ֽ���: 84
		* �ڶ�����̬��ǰ���ֽ���: 121
		* �ڶ���ȫ���ֽ���:	158
		*/
	SDcard.Read(path, "condition.csv", (uint8_t*)data, 84 + 101 + count * 132, 35);
	
	if(data[0])
	{
		/* ����̬�����ݴ洢�ڴ洢���� */
		strncpy(temp, data+1, 8);
		view_data.pos.pitch = atof(temp);
		strncpy(temp, data+11, 8);
		view_data.pos.roll = atof(temp);
		strncpy(temp, data+21, 8);
		view_data.pos.yaw = atof(temp);
		count ++;
		
		/* ������������λ�� */
		ano.Send_Data(&view_data, 0, 0);
	}
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
