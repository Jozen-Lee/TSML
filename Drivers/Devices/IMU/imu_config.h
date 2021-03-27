/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    imu_config.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for IMU configuration.
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

#ifndef __IMU_CONFIG_H
#define __IMU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------- IMU型号 -------------------------- */
#define MPU6050 
//#define MPU6500 	 
//#define MPU9150 
//#define MPU9250 	
	
/* -------------------- 工作模式 ------------------------- */
#define DMP_MODE 	
//#define USER_MODE 
	

/* ----------------- 陀螺仪工作频率 ---------------------- */
#define DEFAULT_MPU_HZ  (200)		// 200Hz
	
	
/** 
	* @brief 以下参数只有使用MPU9150和MPU9250的时候需要配置
	*/	
#if defined(MPU9150) || defined(MPU9250)	

/* ----------------- 磁力计工作频率 ---------------------- */
#define COMPASS_READ_MS (200)
	
	
/* ------------------ 磁力计校准 --------------------------*/
//#define USE_MAG_CAIL 
	

/* ------- 磁力计校准数据的FLASH存储地址 ----------------- */
#ifdef USE_MAG_CAIL
	#define MAG_DATA_ADD 4096*4
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
