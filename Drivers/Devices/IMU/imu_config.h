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

/* -------------------- IMU�ͺ� -------------------------- */
#define MPU6050 
//#define MPU6500 	 
//#define MPU9150 
//#define MPU9250 	
	
/* -------------------- ����ģʽ ------------------------- */
#define DMP_MODE 	
//#define USER_MODE 
	

/* ----------------- �����ǹ���Ƶ�� ---------------------- */
#define DEFAULT_MPU_HZ  (200)		// 200Hz
	
	
/** 
	* @brief ���²���ֻ��ʹ��MPU9150��MPU9250��ʱ����Ҫ����
	*/	
#if defined(MPU9150) || defined(MPU9250)	

/* ----------------- �����ƹ���Ƶ�� ---------------------- */
#define COMPASS_READ_MS (200)
	
	
/* ------------------ ������У׼ --------------------------*/
//#define USE_MAG_CAIL 
	

/* ------- ������У׼���ݵ�FLASH�洢��ַ ----------------- */
#ifdef USE_MAG_CAIL
	#define MAG_DATA_ADD 4096*4
#endif

#endif

#ifdef __cplusplus
}
#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
