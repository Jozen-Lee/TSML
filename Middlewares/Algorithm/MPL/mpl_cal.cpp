/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    mpl_cal.cpp
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
      -# ��������
				mpl_lib.Update();
         
      -# ��ȡ����(����̬��Ϊ��)
        mpl_lib.Get_Pos(&pitch, &roll, &yaw);
		 
    @warning	
      -# ��Ҫ`DMP`��֧�֡�
			-# ��Ҫ`my_filters`��֧�֡�
	  
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
#include "mpl_cal.h"
#include "stdlib.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MPL mpl_lib; //ʵ������

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief ��������  
 *	@note	��ȡ����ǰҪ��ʹ�øú�����������
 *  @param[out] void
 *  @param[in] 	void
 *  @return   0 succeed
							1 fail
 */
uint8_t MPL::Update_Data(void)
{
	unsigned long sensor_timestamp;
	short gyro[3], accel_short[3],compass_short[3],sensors;
	unsigned char more;
	long compass[3],accel[3],quat[4],temperature; 
		
	if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more))return 1;	 

	if(sensors&INV_XYZ_GYRO)
	{
			inv_build_gyro(gyro,sensor_timestamp);          //�������ݷ��͸�MPL
			mpu_get_temperature(&temperature,&sensor_timestamp);
			inv_build_temp(temperature,sensor_timestamp);   //���¶�ֵ����MPL��ֻ����������Ҫ�¶�ֵ
	}
	
	if(sensors&INV_XYZ_ACCEL)
	{
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel,0,sensor_timestamp);      //�Ѽ��ٶ�ֵ����MPL
	}
	
	if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) 
	{
			compass[0]=(long)compass_short[0];
			compass[1]=(long)compass_short[1];
			compass[2]=(long)compass_short[2];
			inv_build_compass(compass,0,sensor_timestamp); //�Ѵ�����ֵ����MPL
	}
	inv_execute_on_data();	
	return 0;
}
 
/**
 *  @brief ��ȡ���ٶ�    
 *  @param[out] gx, gy, gz, ������ٶ�,��λΪ(����/��)
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
uint8_t MPL::Get_Gyro(float* gx, float* gy, float* gz)
{
	float data[3];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_gyroscope(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*gx = data[0];
	*gy = data[1];
	*gz = data[2];
	return res;
}

/**
 *  @brief ��ȡ��������ϵ�µļ��ٶ�(δ��ȥ�������ٶ�)    
 *  @param[out] ax, ay, az, ������ٶ�,��λΪ(m/s^2)
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
uint8_t MPL::Get_Accel(float* ax, float* ay, float* az)
{
	float data[3];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_accelerometer(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*ax = data[0];
	*ay = data[1];
	*az = data[2];
	return res;
}

/**
 *  @brief ��ȡ��������ϵ�µļ��ٶ�(��ȥ�������ٶ�)    
 *  @param[out] ax, ay, az, ������ٶ�,��λΪ(m/s^2)
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
uint8_t MPL::Get_Linear_Accel(float* ax, float* ay, float* az)
{
	float data[3];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_linear_acceleration(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*ax = accx_filter.f(data[0]);
	*ay = accy_filter.f(data[1]);
	*az = accz_filter.f(data[2]);
	return res;
}

/**
 *  @brief ��ȡ����������   
 *  @param[out] mx, my, mz, ���᳡ǿ��С
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
#define MAG_CONVERSION 1.52587890625e-005f  //  1 / 2^16
uint8_t MPL::Get_Compass(float* mx, float* my, float* mz)
{
	long data[3];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_compass(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*mx = data[0] * MAG_CONVERSION;
	*my = data[1] * MAG_CONVERSION;
	*mz = data[2] * MAG_CONVERSION;
	return res;
}

/**
 *  @brief ��ȡ��Ԫ�� 
 *  @param[out] W,X,Y,Z ��Ԫ����ֵ
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
#define QUAT_CONVERSION 9.313225746e-010f //1/2^30
uint8_t MPL::Get_Quat(float* W, float* X, float* Y, float* Z)
{
	long data[4];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_quat(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*W = data[0] * QUAT_CONVERSION;
	*X = data[1] * QUAT_CONVERSION;
	*Y = data[2] * QUAT_CONVERSION;
	*Z = data[3] * QUAT_CONVERSION;
	return res;
}

/**
 *  @brief ��ȡ��Ԫ���������õķ�λ��(Yaw��) 
 *  @param[out] head �����
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
#define HEAD_CONVERSION 1.52587890625e-005f  //  1 / 2^16
uint8_t MPL::Get_Heading(float* head)
{
	long data[1];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_heading(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*head = data[0] * HEAD_CONVERSION;
	return res;
}

/**
 *  @brief ��ȡŷ����  
 *  @param[out] pitch, roll, yaw, ŷ����
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
#define EULER_CONVERSION 1.52587890625e-005f  //  1 / 2^16
uint8_t MPL::Get_Euler(float* pitch, float* roll, float* yaw)
{
	
	long data[3];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_euler(data, accuracy, timestamp)) res = 1;
	else res = 0;
	*pitch = data[0]*EULER_CONVERSION;
	*roll = data[1]*EULER_CONVERSION;
	*yaw = data[2]*EULER_CONVERSION;
	return res;
}

/**
 *  @brief ��ȡ��ת����  
 *  @param[out] DCM ��ת����,��СΪ3*3
 *  @param[in] void
 *  @return 1 ���ݸ���
 */
#define DCM_CONVERSION 	9.313225746e-010f //1/2^30
uint8_t MPL::Get_DCM(float DCM[][3])
{
	long data[9];
	uint8_t res;
	int8_t accuracy[1];
	inv_time_t timestamp[1];
	if(inv_get_sensor_type_rot_mat(data, accuracy, timestamp)) res = 1;
	else res = 0;
	for(int i = 0; i < 3; i ++)
	{
		for(int j = 0; j < 3; j ++) 
		{
			DCM[i][j] = data[i*3+j]*DCM_CONVERSION;
		}
	}
	return res;
}

/**
 *  @brief ��ȡ��̬�� 
 *  @param[out] pitch, roll, raw: ��̬��
 *  @param[in] void
 *  @return 0 �ɹ� ����ʧ��
 */
#define q16  65536.0f // longתfloatʱ�ĳ��� 
uint8_t MPL::Get_Pos(float* pitch, float* roll, float* yaw)
{
	inv_time_t timestamp;
	long data[9];
	int8_t accuracy;
	inv_get_sensor_type_euler(data,&accuracy,&timestamp);
	
	*roll  = (data[0]/q16);
	*pitch = -(data[1]/q16);
	*yaw   = -data[2] / q16;
	return 0;
}

/**
 *  @brief ��������ϵ�µļ��ٶȵ�ȫ������ϵ��ת��
 *  @param[in] DCM ��ת����
 *  @param[in] b_ax, b_ay, b_az ��������ϵ�µ�������ٶ�
 *  @param[out] g_ax, g_ay, g_az ȫ������ϵ�µ�������ٶ�
 *  @return void
 */
void MPL::Body_To_World_Accel(float DCM[][3], float b_ax, float b_ay, float b_az, float* g_ax, float* g_ay, float* g_az)
{
	/* ���þ������� */
	*g_ax = DCM[0][0] * b_ax + DCM[0][1] * b_ay + DCM[0][2] * b_az;
	*g_ay = DCM[1][0] * b_ax + DCM[1][1] * b_ay + DCM[1][2] * b_az;
	*g_az = DCM[2][0] * b_ax + DCM[2][1] * b_ay + DCM[2][2] * b_az;
}

/**
 *  @brief ��ȡȫ������ϵ�µļ��ٶ�
 *  @param[out] g_ax, g_ay, g_az ȫ������ϵ�µ�������ٶ�
 *  @return 1 ���ݸ���
 */
uint8_t MPL::Get_Ground_Linear_Accel(float* g_ax, float* g_ay, float* g_az)
{
	uint8_t res;
	float b_ax, b_ay, b_az;
	float DCM[3][3];
	
	/* ��ȡ������ٶ� */
	res = Get_Linear_Accel(&b_ax, &b_ay, &b_az);
	
	/* ��ȡ��ת���� */
	Get_DCM(DCM);
	
	/* ��ȡȫ������ϵ�µļ��ٶ� */
	Body_To_World_Accel(DCM, b_ax, b_ay, b_az, g_ax, g_ay, g_az);
	return res;
}

/**
 *  @brief ���ٶȲ�������
 *  @note ����ʹ��mpl�㷨���õ�ȫ�ּ��ٶ�û����ȫ�������������ٶȵ�Ӱ��,
					�����ڲ�ͬ�ĽǶ���,���õ�������ٶȾ��о�̬���,�˺�������������̬���
 *  @param[in] pitch, roll, raw: ��̬��
 *  @param[out] g_ax, g_ay, g_az ȫ������ϵ�µĲ���������ٶ�
 *  @return void
 */
void MPL::Acc_Compenssation(const float* pitch, const float* roll, const float* yaw, float* g_ax, float* g_ay, float* g_az)
{
	/* z�Ჹ�� */
	if(*roll <= 90) 
	{
		/* pitch -90��-> 0 -> 90��--> 0 -> 0.2 -> 0 */
		if(*pitch <= 0) *g_az -= (90.0f + *pitch) * 0.3f / 90.0f;
		else *g_az -= (90.0f - *pitch) * 0.25f / 90.0f;
	}
	else
	{
		/* pitch -90��-> 0 -> 90��--> 0 -> -0.1 -> 0 */
		if(*pitch <= 0) *g_az += (90.0f + *pitch) * 0.1f / 90.0f;
		else *g_az += (90.0f - *pitch) * 0.1f / 90.0f;		
	}
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
