/**
	******************************************************************************
	* Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    imu.cpp
	* @author  LJY 2250017028@qq.com
	* @brief   Code for IMU.
	* @date    2021-02-21
	* @version 1.0
	* @par Change Log:
	* <table>
	* <tr><th>Date        <th>Version  <th>Author     <th>Description
	* <tr><td>2021-02-21  <td> 1.0     <td>LJY  			<td>Creator
	* <tr><td>2021-02-23  <td> 1.1     <td>LJY  			<td>Update the code frame
	* </table>
	*
	==============================================================================
															How to use this driver  
	==============================================================================
		@note				 
			-# ��ʼ��
					imu.Init();

			-# ��������,���ݴ洢��imu.data��
					imu.Update();
		@warning	
		-# ��IMU_DMP_MODE��ģʽ��, MPU6050��MPU6500ʹ��DMP�㷨��MPU9150��MPU9250ʹ��MPL�㷨
		-# ʹ��MPL�㷨
			-# ��Ҫ`mpl_cal`��֧��
			-# ��Ҫ��ħ�����м���궨��`EMPL_TARGET_STM32F4`,`EMPL`
		-# ʹ��DMP�㷨
			-# ��Ҫ��ħ�����м���궨��`MOTION_DRIVER_TARGET_MSP430`,`EMPL`
		
		
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
#include "imu.h"
/* DMP�� */
#ifdef IMU_DMP_MODE
	#if defined(MPU9150) || defined(MPU9250) 
		/* ����MPL����� */
		#include "Middlewares/Algorithm/MPL/mpl_cal.h"
		
		/* ����DMP�ٷ�ͷ�ļ� */
		#include "Middlewares/DMP_Lib/mpl.h"
		#include "Middlewares/DMP_Lib/quaternion_supervisor.h"
		#include "Middlewares/DMP_Lib/fusion_9axis.h"
		#include "Middlewares/DMP_Lib/fast_no_motion.h"
		#include "Middlewares/DMP_Lib/gyro_tc.h"
		#include "Middlewares/DMP_Lib/compass_vec_cal.h"
		#include "Middlewares/DMP_Lib/mag_disturb.h"
		#include "Middlewares/DMP_Lib/eMPL_outputs.h"
		#include "Middlewares/DMP_Lib/data_builder.h"	
	#else
		#include "Middlewares/Algorithm/DMP/dmp_cal.h"
	#endif
#endif

/* ������ض���� */
#ifdef IMU_IMU_USER_MODE 
	#if defined(MPU6050) 
		#include "mpu6050.h"
	#elif defined(MPU6500) 
		#include "mpu6500.h"
	#elif defined(MPU9150) 
		#include "mpu9150.h"
	#elif defined(MPU9250) 
		#include "mpu9250.h"	
	#endif
#endif

/* ������У׼����ͷ�ļ� */
#ifdef USE_MAG_CAIL
	#include "storage_manager.h"
	#include "flash.h"
#endif

/* Private define ------------------------------------------------------------*/

#if defined(MPU9250) || defined(MPU9150)
/* �����Ƿ������� */
static signed char gyro_orientation[9] = { 1, 0, 0,
																					 0, 1, 0,
																					 0, 0, 1};

/* �����Ʒ������� */
static signed char comp_orientation[9] = { 0, 1, 0,
																					 1, 0, 0,
																					 0, 0,-1};
#elif defined(MPU6050) || defined(MPU6500)
static signed char gyro_orientation[9] = { 0, 1, 0,
																					 1, 0, 0,
																					 0, 0,-1};
#endif

/* Private variables ---------------------------------------------------------*/																						 
_MPU9250 imu;	
																					 
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
																					 
/**
 *  @brief IMU��IICд����
 *	@param[in] addr �����IIC��ַ
 *	@param[in] reg �Ĵ�����ַ
 *	@param[in] len ���ݳ���
 *	@param[in] data Ҫд�������
 *  @return 0 success
						1 fail
 */
__weak uint8_t dmp_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) 
{
	 uint8_t res;
	 res = IIC_Device_Write_Len(&imu.IIC_PIN, addr, reg, len, data);
	 return res;
}

/**
 *  @brief IMU��IIC������
 *	@param[in] addr �����IIC��ַ
 *	@param[in] reg �Ĵ�����ַ
 *	@param[in] len ���ݳ���
 *	@param[in] data Ҫд�������
 *  @return 0 success
						1 fail
 */
__weak uint8_t dmp_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) 
{
	 uint8_t res;
	 res = IIC_Device_Read_Len(&imu.IIC_PIN, addr, reg, len, data);
	 return res;
}

/**
 *  @brief IMU����ʱ����
 *	@param[in] ms ��ʱʱ��/����
 *  @return 0 success
						1 fail
 */
__weak void dmp_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

 /**
 *  @brief ��ȡʱ��(û������,����Ϊ�պ���)
 *  @param[out] time ��ǰʱ��
 *  @return void
 */ 
void dmp_get_ms(unsigned long *time)
{
// *time=(unsigned long)HAL_GetTick();
}																					 

/**
 *  @brief IMU��ʼ��
 *  @return 0 success
						1 fail
 */
uint8_t IMU::Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx)
{
	uint8_t res;
	
	/* ����IIC�˿ڲ���ʼ�� */
	IIC_Setting(&IIC_PIN, gpiox, scl_pinx, sda_pinx);
	IIC_Init(&IIC_PIN);

	/* �������� */
	Dev_Setting();

	/* ��IMU���г�ʼ�� */
	res = dev.init();
	
	if (res != 0)
	{
			HAL_Delay(100);
			__set_FAULTMASK(1); //reset
			NVIC_SystemReset();
	}
	
	/* �Դ����ƽ���У׼ */
#ifdef USE_MAG_CAIL
	res = Compass_Calibration();
#endif
	dev.delay(1000);
	return res;
}

/**
 *  @brief IMU���ݸ���
 *  @return 0 success
						1 fail
 */
uint8_t IMU::Update(void)
{
	uint8_t res;
	res = dev.update(&data);
//	mpu_mpl_get_data(&data.pos.pitch, &data.pos.roll, &data.pos.yaw);
	return res;
}

/**
 *  @brief ����IMU����
 *  @return void
 */
void IMU::Dev_Setting(void)
{

}

#ifdef USE_MAG_CAIL
/**
	* @brief  ��ȡ������״̬
	* @param[in]  void
	* @return 0 succeed
						1 fail
	*/
uint8_t IMU::Get_Compass_State(void)
{
	inv_time_t timestamp[1];	
	long data[3];
	int8_t accuracy[1];
	inv_get_sensor_type_euler(data, accuracy, timestamp);
	if(accuracy[0] == 3) return 0;
	else return 1;
}


/**
	* @brief  ������У׼
	* @param[in]  void
	* @return 0 succeed
						1 fail
	*/
uint8_t IMU::Compass_Calibration(void)
{
	uint32_t out_time = 0;
	const size_t len = 124;	
	uint8_t flag;	
	uint8_t mag_data[len];

	/* ��ȡ������״̬ */
	flag = Get_Compass_State();
	
	if(flag == 0)
	{
		/* ��У׼����д��FLASH */
		inv_save_mpl_states(mag_data, len);
		flash.Write(mag_data, MAG_DATA_ADD, len);
		return 0;
	}
	else
	{
		/* ��ȡFLASH����, ���Ƿ�����У׼���� */
		flash.Read(mag_data, MAG_DATA_ADD, len);
		inv_load_mpl_states(mag_data,len);
		
		/* ѭ���ж�У׼ */
		while(out_time < 65535)
		{
			out_time ++;
			Update();	//��仰һ��Ҫ��,�����޷�֪���Ƿ�У׼
			flag = Get_Compass_State();
			if(!flag)
			{
				/* ��У׼����д��FLASH */
				inv_save_mpl_states(mag_data, len);
				flash.Write(mag_data, MAG_DATA_ADD, len);
				return 0;
			}
		}
		return 1;
	}
}
#endif
 
/**
 *  @brief IMU�Բ���
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */ 
uint8_t IMU_Lib::run_self_test(void)
{
	int result;
	long gyro[3], accel[3]; 
	
	/* �Բ� */
#if defined(MPU6500) || defined(MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#else
	result = mpu_run_self_test(gyro, accel);
#endif	
	
	/* ����ͨ�� */
	if (result == 0x7) 
	{
		unsigned short accel_sens;
		float gyro_sens;
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}
	else return 1;
}

/**
 *  @brief ����ת��
 *  @param[in] row ��ת������ֵ
 *  @return ת�����scale
 */ 
uint16_t IMU_Lib::inv_row_2_scale(const int8_t *row)
{
	uint16_t b;
	if (row[0] > 0)
			b = 0;
	else if (row[0] < 0)
			b = 4;
	else if (row[1] > 0)
			b = 1;
	else if (row[1] < 0)
			b = 5;
	else if (row[2] > 0)
			b = 2;
	else if (row[2] < 0)
			b = 6;
	else
			b = 7;      // ����
	return b;
} 

#if defined(IMU_DMP_MODE) 

/**
	*  @brief IMU��IIC�ӿں���, ����DMP�㷨����
	*/

	#if defined(MPU9150) || defined(MPU9250)
/**
 *  @brief IMU��MPL��ʼ��
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */ 
uint8_t IMU_Lib::IMU_MPL_Init(void)
{
	unsigned char res=0;
	struct int_param_s int_param;
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	unsigned short compass_fsr;
	
	/* ��ʼ��IMU */
	if(mpu_init(&int_param)==0)		 
	{	 
		/* ��ʼ��MPL */
		res=inv_init_mpl();      
		if(res) return 1;
		
		/* ʹ�ܸ������� */
		inv_enable_quaternion();
		inv_enable_9x_sensor_fusion();
		inv_enable_fast_nomot();
		inv_enable_gyro_tc();
		inv_enable_vector_compass_cal();
		inv_enable_magnetic_disturbance();
		inv_enable_eMPL_outputs();

		/* ����MPL */
		res=inv_start_mpl();    
		if(res)return 1;
		
		/* ��������Ҫ�Ĵ����� */
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS);
		if(res)return 2; 
		
		/* ����FIFO */
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   
		if(res)return 3; 
		
		/* ���ò����� */
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	            
		if(res)return 4; 
		
		/* ���ô����Ʋ����� */
		res=mpu_set_compass_sample_rate(1000/COMPASS_READ_MS);  
		if(res)return 5;
		
		/* ���ø��������� */
		mpu_get_sample_rate(&gyro_rate);
		mpu_get_gyro_fsr(&gyro_fsr);
		mpu_get_accel_fsr(&accel_fsr);
		mpu_get_compass_fsr(&compass_fsr);
		inv_set_gyro_sample_rate(1000000L/gyro_rate);
		inv_set_accel_sample_rate(1000000L/gyro_rate);
		inv_set_compass_sample_rate(COMPASS_READ_MS*1000L);
		inv_set_gyro_orientation_and_scale(
				inv_orientation_matrix_to_scalar(gyro_orientation),(long)gyro_fsr<<15);
		inv_set_accel_orientation_and_scale(
				inv_orientation_matrix_to_scalar(gyro_orientation),(long)accel_fsr<<15);
		inv_set_compass_orientation_and_scale(
				inv_orientation_matrix_to_scalar(comp_orientation),(long)compass_fsr<<15);
						
		/* ����DMP�̼� */        
		res=dmp_load_motion_driver_firmware();		             
		if(res)return 6; 
		
		/* ���������Ƿ��� */
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
		if(res)return 7; 
		
		/* ����DMP���� */
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	            
				DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
				DMP_FEATURE_GYRO_CAL);
		if(res)return 8; 
		
		/* ����DMP�������(��󲻳���200Hz) */
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	
		if(res)return 9;   
		
		/* �Լ� */
		res=run_self_test();		
		if(res)return 10;    
		
		/* ʹ��DMP */
		res=mpu_set_dmp_state(1);	
		if(res)return 11;     
	}
	
	return 0;
}

/**
 *  @brief IMU��MPL�������ݺ���
 *	@param[out] imu_data IMU������
 *  @return 0 success
						1 fail
 */
uint8_t IMU_Lib::IMU_MPL_Update(IMU_Data_t* imu_data)
{
	/* �������� */
	mpl_lib.Update_Data();
	
	/* ��ȡ��̬�� */
	mpl_lib.Get_Pos(&imu_data->pos.pitch, &imu_data->pos.roll, &imu_data->pos.yaw);	
	
	/* ��ȡ���ٶ� */
	mpl_lib.Get_Gyro(&imu_data->gyro[0], &imu_data->gyro[1], &imu_data->gyro[2]);
	
	/* ��ȡ������ϵ�ļ��ٶ� */
	mpl_lib.Get_Accel(&imu_data->accel[0], &imu_data->accel[1], &imu_data->accel[2]);
	
	/* ��ȡ��ǿ */
	mpl_lib.Get_Compass(&imu_data->compass[0], &imu_data->compass[1], &imu_data->compass[2]);
	
	/* ��ȡ������ϵ�����Լ��ٶ� */
	mpl_lib.Get_Linear_Accel(&imu_data->com_accel[0], &imu_data->com_accel[1], &imu_data->com_accel[2]);
	
	/* ���ٶȲ��� */
//		mpl_lib.Acc_Compenssation(&imu_data->pos.pitch, &imu_data->pos.roll, &imu_data->pos.yaw, 
//		&imu_data->com_accel[0], &imu_data->com_accel[1], &imu_data->com_accel[2]);
	
	/* ��ȡ��������ϵ�����Լ��ٶ� */
	mpl_lib.Get_Ground_Linear_Accel(&imu_data->g_com_accel[0], &imu_data->g_com_accel[1], &imu_data->g_com_accel[2]);	

	return 0;
}

#elif defined(MPU6050) || defined(MPU6500)
/**
 *  @brief IMU��DMP��ʼ��
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */ 
uint8_t IMU_Lib::IMU_DMP_Init(void)
{
	uint8_t res=0;
	struct int_param_s int_param;
	
	/* ��ʼ��IMU */
	if(mpu_init(&int_param)==0)	
	{	 
		/* ��������Ҫ�Ĵ����� */
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
		if(res)return 1; 
		
		/* ����FIFO */
		res=mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);
		if(res)return 2; 
		
		/* ���ò����� */
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	
		if(res)return 3; 
		
		/* ����DMP�̼� */
		res=dmp_load_motion_driver_firmware();		
		if(res)return 4; 
		
		/* ���������Ƿ��� */
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
		if(res)return 5; 
		
		/* ����DMP���� */
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	
				DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
				DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		
		/* ����DMP�������(��󲻳���200Hz) */
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);
		if(res)return 7;   
		
		/* �Լ� */
		res=run_self_test();		
		if(res)return 8;   
			
		/* ʹ��DMP */
		res=mpu_set_dmp_state(1);	
		if(res)return 9;     
	}
	else return 10;
	return 0;	
}

/**
 *  @brief IMU��DMP�������ݺ���
 *	@param[out] imu_data IMU������
 *  @return 0 success
						1 fail
 */
uint8_t IMU_Lib::IMU_DMP_Update(IMU_Data_t* imu_data)
{
	/* ��ȡ�¶� */
	dmp_lib.Get_Tempreture(&imu_data->temp);
	
	/* ��ȡ���ٶ� */
	dmp_lib.Get_Gyro(&imu_data->gyro[0], &imu_data->gyro[1], &imu_data->gyro[2]);
	
	/* ��ȡ���ٶ� */
	dmp_lib.Get_Accel(&imu_data->accel[0], &imu_data->accel[1], &imu_data->accel[2]);	

	/* ��ȡ��̬�� */
	dmp_lib.Get_Pos(&imu_data->pos.pitch, &imu_data->pos.roll, &imu_data->pos.yaw);		
	
	return 0;	
}
	#endif /* TYPE */	
	
#else
/**
 *  @brief IMU���û��Զ����ʼ��
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */ 
uint8_t IMU_Lib::IMU_User_Init(void)
{
	return 0;
}

/**
 *  @brief IMU���û��Զ���������ݺ���
 *	@param[out] imu_data IMU������
 *  @return 0 success
						1 fail
 */
uint8_t IMU_Lib::IMU_User_Update(IMU_Data_t* imu_data)
{
	return 0;	
}

#endif	 /* MODE */

#ifdef MPU6050
/**
 *  @brief MPU6050����������
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */
void _MPU6050::Dev_Setting()
{
	#if defined(IMU_USER_MODE)
	 dev.init = IMU_Lib::IMU_User_Init;
	 dev.update = IMU_Lib::IMU_User_Update;
	#else	
	 dev.init = IMU_Lib::IMU_DMP_Init;
	 dev.update = IMU_Lib::IMU_DMP_Update;
	#endif
	
	 dev.delay = dmp_delay_ms;
	 dev.read = dmp_i2c_read;
	 dev.write = dmp_i2c_write;
	 dev.getms = dmp_get_ms;
}
#endif

#ifdef MPU6500
/**
 *  @brief MPU6500����������
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */
void _MPU6500::Dev_Setting()
{
	#if defined(IMU_USER_MODE)
	 dev.init = IMU_Lib::IMU_User_Init;
	 dev.update = IMU_Lib::IMU_User_Update;
	#else	
	 dev.init = IMU_Lib::IMU_DMP_Init;
	 dev.update = IMU_Lib::IMU_DMP_Update;
	#endif
	
	 dev.delay = dmp_delay_ms;
	 dev.read = dmp_i2c_read;
	 dev.write = dmp_i2c_write;
	 dev.getms = dmp_get_ms;
}
#endif

#ifdef MPU9150
/**
 *  @brief MPU9150����������
 *  @param[in] void
 *  @return 0 succeed
						1 fail
 */
void _MPU9150::Dev_Setting()
{
	#if defined(IMU_USER_MODE)
	 dev.init = IMU_Lib::IMU_User_Init;
	 dev.update = IMU_Lib::IMU_User_Update;
	#else	
	 dev.init = IMU_Lib::IMU_MPL_Init;
	 dev.update = IMU_Lib::IMU_MPL_Update;
	#endif
	
	 dev.delay = dmp_delay_ms;
	 dev.read = dmp_i2c_read;
	 dev.write = dmp_i2c_write;
	 dev.getms = dmp_get_ms;
}
#endif

#ifdef MPU9250
/**
 *  @brief MPU9250����������
 *  @param[in] void
 *  @return void
 */
void _MPU9250::Dev_Setting()
{
	#if defined(IMU_USER_MODE)
	 dev.init = IMU_Lib::IMU_User_Init;
	 dev.update = IMU_Lib::IMU_User_Update;
	#else	
	 dev.init = IMU_Lib::IMU_MPL_Init;
	 dev.update = IMU_Lib::IMU_MPL_Update;
	#endif
	
	 dev.delay = dmp_delay_ms;
	 dev.read = dmp_i2c_read;
	 dev.write = dmp_i2c_write;
	 dev.getms = dmp_get_ms;
}
#endif

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
