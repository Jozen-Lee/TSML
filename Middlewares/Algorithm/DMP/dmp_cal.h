/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    dmp_cal.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for DMP calculate functions.
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

#ifndef __DMP_CAL_H
#define __DMP_CAL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "Middlewares/DMP_Lib/inv_mpu.h"
#include "Middlewares/DMP_Lib/inv_mpu_dmp_motion_driver.h"

#ifdef __cplusplus
}
#endif	

#ifdef __cplusplus


/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>
/* Private define ------------------------------------------------------------*/
									
/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/
	
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class DMP
{
public:
	uint8_t Get_Gyro(float* gx, float* gy, float* gz);
	uint8_t Get_Accel(float* ax, float* ay, float* az);
	uint8_t Get_Tempreture(float* temp);
	uint8_t Get_Pos(float* pitch, float* roll, float* yaw);
private:		
};
/* Exported variables --------------------------------------------------------*/
extern DMP dmp_lib;
/* Exported function declarations --------------------------------------------*/


#endif	

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
