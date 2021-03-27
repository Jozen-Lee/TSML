/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    mpl_cal.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MPL calculate functions.
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

#ifndef __DATA_DEALER_H
#define __DATA_DEALER_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"	
#include <algorithm>

/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/
#include "SDcard.h"
#include "imu.h"
#include "MT9V032.h"
#include "inertial_navigation.h"
#include "System_DataPool.h"
#include "Ano_UpperMonitor.h"

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
class Data_Dealer
{
public:
	Data_Dealer():ano(&huart1), photo("P"){}
	~Data_Dealer(){}	
	uint8_t Init(void);
	uint8_t Image_Save(void);
	uint8_t Condition_Save(void);
	void View_Condition(uint8_t test_num);
	char* Get_Data_Path(void){ return data_path;}
	char* Get_Photo_Path(void){ return photo_path;}
	IMU_Data_t view_data;
private:
	/* 注册上位机 */
	ANO_Upper ano;
	
	/* 图像存储变量 */
	Inc_Name<5, _BMP> photo;
	Photo_Creator<ROW, COL> bmp;	

	/* 文件存储路径 */
	char data_path[36];		// 存储数据的文件路径
	char photo_path[36];	// 存储图片的文件路径

	/* 文件名 */
	char point_file[20];
	char condition_file[20];

	/* 临时变量 */
	char data[150];
	uint8_t len;
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern Data_Dealer data_dealer;

/* Exported function declarations --------------------------------------------*/

#endif	

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
