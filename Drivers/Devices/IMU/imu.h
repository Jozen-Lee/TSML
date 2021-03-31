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
	#include "Drivers/Components/I2C/tutu_drv_i2c.h"
	#include "imu_config.h"
	#include "ml_math_func.h"
	#include "inv_mpu.h"
	#include "inv_mpu_dmp_motion_driver.h"
#ifdef __cplusplus
}
#endif
/* Private define ------------------------------------------------------------*/
	 
/* Private include -----------------------------------------------------------*/

#ifdef __cplusplus
#include <algorithm>
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/

/* ��̬���ݽṹ�� */
typedef struct 
{
	float pitch;
	float roll;
	float yaw;
} Pos_t;

/* IMU���ݽṹ�� */
typedef struct
{
	float temp;						// �¶�
	float gyro[3];				// xyz������ٶ�
	float accel[3];				// ������ϵ��, xyz������ٶ�(δ��ȥ�������ٶ�)
	float compass[3];			// xyz�����ǿ
	float com_accel[3];		// ������ϵ��, xyz������ٶ�(��ȥ�������ٶ�)
	float g_com_accel[3]; // ��������ϵ��, xyz������ٶ�(��ȥ�������ٶ�)
	Pos_t pos;						// ��̬��
} IMU_Data_t;

typedef uint8_t (*imu_init_fptr_t)(void);																													// ��ʼ������ָ��
typedef uint8_t (*imu_update_fptr_t)(IMU_Data_t* imu_data);																				// �������ݺ���ָ��
typedef uint8_t (*imu_readwrite_fptr_t)(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);	 	// IIC��д����ָ�� 
typedef void (*imu_delay_fptr_t)(uint32_t period);																				 				// ��ʱ����ָ��
typedef void (*imu_getms_fptr_t)(unsigned long *time);																						// ��ȡʱ�亯��ָ��

/* IMU�����ṹ�� */
typedef struct
{
	imu_init_fptr_t init;					// ��ʼ��
	imu_update_fptr_t update;			// ��������
	imu_readwrite_fptr_t read;		// ��ȡ
	imu_readwrite_fptr_t write;		// д��
	imu_delay_fptr_t delay;				// ��ʱ
	imu_getms_fptr_t	getms;			// ��ȡʱ��
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
	static uint16_t inv_row_2_scale(const int8_t *row);	// ����ת��
	static uint8_t run_self_test(void);									// �Լ�
};

class IMU
{
public:
	IMU(){};
	~IMU(){};
	uint8_t IMU_Type;																																		// IMU�ͺ�
	IMU_Data_t data;																																		// ���ݴ洢�ṹ��
	imu_dev dev;																																				// �����ṹ��
	IIC_PIN_Typedef IIC_PIN;																														// IIC�ӿ�
	uint8_t Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx);						// ��ʼ��
	virtual uint8_t Update(void);																												// ��������
	virtual void Dev_Setting();																													// ��������

private:
#ifdef USE_MAG_CAIL
	uint8_t Get_Compass_State(void);																										// ��ȡ������У׼״̬
	uint8_t Compass_Calibration(void);																									// ������У׼
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
