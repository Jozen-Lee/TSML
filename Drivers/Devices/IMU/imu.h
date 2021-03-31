/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    imu.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for IMU.
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
#ifndef __IMU_H
#define __IMU_H

///* Includes ------------------------------------------------------------------*/  
#ifdef __cplusplus
extern "C" {
#endif
	#include "main.h"
	#include "imu_config.h"
	#include "Drivers/Components/I2C/tutu_drv_i2c.h"	
	#include "Middlewares/DMP_Lib/ml_math_func.h"
	#include "Middlewares/DMP_Lib/inv_mpu.h"
	#include "Middlewares/DMP_Lib/inv_mpu_dmp_motion_driver.h"
#ifdef __cplusplus
}
#endif
/* Private define ------------------------------------------------------------*/
	 
/* Private include -----------------------------------------------------------*/

#ifdef __cplusplus
#include <algorithm>
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/

/* 姿态数据结构体 */
typedef struct 
{
	float pitch;
	float roll;
	float yaw;
} Pos_t;

/* IMU数据结构体 */
typedef struct
{
	float temp;						// 温度
	float gyro[3];				// xyz三轴角速度
	float accel[3];				// 体坐标系下, xyz三轴加速度(未消去重力加速度)
	float compass[3];			// xyz三轴磁强
	float com_accel[3];		// 体坐标系下, xyz三轴加速度(消去重力加速度)
	float g_com_accel[3]; // 世界坐标系下, xyz三轴加速度(消去重力加速度)
	float offset[3];			// 角速度的三轴偏差值
	Pos_t pos;						// 姿态角
} IMU_Data_t;

typedef uint8_t (*imu_init_fptr_t)(void);																													// 初始化函数指针
typedef uint8_t (*imu_update_fptr_t)(IMU_Data_t* imu_data);																				// 更新数据函数指针
typedef uint8_t (*imu_readwrite_fptr_t)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);	 	// IIC读写函数指针 
typedef void (*imu_delay_fptr_t)(uint32_t period);																				 				// 延时函数指针
typedef void (*imu_getms_fptr_t)(unsigned long *time);																						// 获取时间函数指针

/* IMU驱动结构体 */
typedef struct
{
	imu_init_fptr_t init;					// 初始化
	imu_update_fptr_t update;			// 更新数据
	imu_readwrite_fptr_t read;		// 读取
	imu_readwrite_fptr_t write;		// 写入
	imu_delay_fptr_t delay;				// 延时
	imu_getms_fptr_t	getms;			// 获取时间
} imu_dev;

class IMU_Lib
{
public:
	IMU_Lib(){};
	~IMU_Lib(){};
	static uint8_t IMU_User_Init(void);
	static uint8_t IMU_DMP_Init(void);
	static uint8_t IMU_MPL_Init(void);
	static uint8_t IMU_User_Update(IMU_Data_t* imu_data);
	static uint8_t IMU_DMP_Update(IMU_Data_t* imu_data);
	static uint8_t IMU_MPL_Update(IMU_Data_t* imu_data);

private:	
	static uint16_t inv_row_2_scale(const int8_t *row);	// 方向转换
	static uint8_t run_self_test(void);									// 自检
};

class IMU
{
public:
	IMU(){};
	~IMU(){};
	uint8_t IMU_Type;																																		// IMU型号
	IMU_Data_t data;																																		// 数据存储结构体
	imu_dev dev;																																				// 驱动结构体
	IIC_PIN_Typedef IIC_PIN;																														// IIC接口
	uint8_t Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx);						// 初始化
	virtual uint8_t Update(void);																												// 更新数据
	virtual void Dev_Setting();																													// 驱动配置

private:
#ifdef USE_MAG_CAIL
	uint8_t Get_Compass_State(void);																										// 获取磁力计校准状态
	uint8_t Compass_Calibration(void);																									// 磁力计校准
#endif
};	

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

#ifdef MPU6050
class _MPU6050 : public IMU
{
public:
	_MPU6050(){IMU();}
	~_MPU6050(){IMU::~IMU();}
	virtual void Dev_Setting();
};
#endif

#ifdef MPU6500
class _MPU6500 : public IMU
{
public:
	_MPU6500(){IMU();}
	~_MPU6500(){IMU::~IMU();}
	virtual void Dev_Setting();
};
#endif

#ifdef MPU9150
class _MPU9150 : public IMU
{
public:
	_MPU9150(){IMU();}
	~_MPU9150(){IMU::~IMU();}
	virtual void Dev_Setting();
};
#endif

#ifdef MPU9250
class _MPU9250 : public IMU
{
public:
	_MPU9250(){IMU();}
	~_MPU9250(){IMU::~IMU();}	
	virtual void Dev_Setting();
};
#endif

/* Exported variables --------------------------------------------------------*/
extern _MPU6050 imu;
#endif
/* Exported function declarations --------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif
uint8_t dmp_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t dmp_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
void dmp_delay_ms(uint32_t period);
void dmp_get_ms(unsigned long *time);
#ifdef __cplusplus
}
#endif

#endif	
	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
