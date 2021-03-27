/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    mpu9250.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for mpu9250.
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
      -# ʹ��˵����`mpu9250_config.cpp`��
			
		@warning
			-# ���ʹ����`mpu_dmp_init()`,�������ٵ���`MPU_Init()`
	  
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
#include "mpu9250.h"

/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  ��ʼ��MPU9250
  * @param  void
  * @retval 0,�ɹ� 
  *         ����,�������
  */
unsigned char MPU_Init(IIC_PIN_Typedef *iic_pin)
{
		uint8_t res=0;
    IIC_Init(iic_pin);     																										//��ʼ��IIC����
    IIC_Device_Write_Byte(iic_pin, MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);			//��λMPU9250
    HAL_Delay(100);  																													//��ʱ100ms
	
    IIC_Device_Write_Byte(iic_pin, MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);			//����MPU9250
    MPU_Set_Gyro_Fsr(iic_pin,3);					        														//�����Ǵ�����,��2000dps
	  MPU_Set_Accel_Fsr(iic_pin,0);					       	 														//���ٶȴ�����,��2g
    MPU_Set_Rate(iic_pin,50);						       	 															//���ò�����50Hz
	
    IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_INT_EN_REG,0X00);   				//�ر������ж�
	  IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);				//I2C��ģʽ�ر�
	  IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);					//�ر�FIFO
	  IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);				//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
	
    res=IIC_Device_Read_Byte(iic_pin,MPU9250_ADDR,MPU_DEVICE_ID_REG);  				//��ȡMPU6500��ID
    if(res==MPU6500_ID) //����ID��ȷ
    {
        IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		    MPU_Set_Rate(iic_pin,50);						       														//���ò�����Ϊ50Hz   
    }else return 1;
 
    res=IIC_Device_Read_Byte(iic_pin,AK8963_ADDR,MAG_WIA);    								//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        IIC_Device_Write_Byte(iic_pin,AK8963_ADDR,MAG_CNTL1,0X11);						//����AK8963Ϊ���β���ģʽ
    }else return 1;

    return 0;
}


/**
  * @brief  ����MPU9250�����Ǵ����������̷�Χ
  * @param  fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Gyro_Fsr(IIC_PIN_Typedef *iic_pin,unsigned char fsr)
{
	return IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ
}

/**
  * @brief  ����MPU9250���ٶȴ����������̷�Χ
  * @param  fsr:0,��2g;1,��4g;2,��8g;3,��16g
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Accel_Fsr(IIC_PIN_Typedef *iic_pin,unsigned char fsr)
{
	return IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ
}

/**
  * @brief  ����MPU6050�����ֵ�ͨ�˲���
  * @param  lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_LPF(IIC_PIN_Typedef *iic_pin,unsigned short int lpf)
{
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}

/**
  * @brief  ����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
  * @param  rate:4-8000(Hz)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Rate(IIC_PIN_Typedef *iic_pin,unsigned short int rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=IIC_Device_Write_Byte(iic_pin,MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//���ò���Ƶ�ʷ�Ƶ��
 	return MPU_Set_LPF(iic_pin,rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}


/**
  * @brief  ��ȡ�¶�ֵ
  * @param  void
  * @retval �¶�ֵ       
  */
short MPU_Get_Temperature(IIC_PIN_Typedef *iic_pin)
{
    uint8_t buf[2]; 
		short raw;
		float temp;
		IIC_Device_Read_Len(iic_pin,MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp;
}

/**
  * @brief  �õ�������ֵ�������ٶ�ֵ
  * @param  gx,gy,gz:������x,y,z���ԭʼ����(������)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Get_Gyroscope(IIC_PIN_Typedef * iic_pin,float *gx,float *gy,float *gz)
{
  unsigned char buf[6],res;
	res=IIC_Device_Read_Len(iic_pin,MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(short)(((unsigned short int)buf[0]<<8)|buf[1]);
		*gy=(short)(((unsigned short int)buf[2]<<8)|buf[3]);
		*gz=(short)(((unsigned short int)buf[4]<<8)|buf[5]);	
	}
  return res;
}

/**
  * @brief  �õ����ٶ�ֵ
  * @param  ax,ay,az:������x,y,z��Ķ���
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Get_Accelerometer(IIC_PIN_Typedef *iic_pin, float *ax,float *ay,float *az)
{
  unsigned char buf[6],res;
	res=IIC_Device_Read_Len(iic_pin,MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
    return res;
}

/**
  * @brief  �õ�������ֵ(ԭʼֵ),��MPL������������޷�������ȡ��������Ϣ
  * @param  mx,my,mz:������x,y,z���ԭʼ����(������)
  * @retval 0,success
  *         1,fail
  */
unsigned char buf[6];
unsigned char MPU_Get_Magnetometer(IIC_PIN_Typedef * iic_pin,float *mx,float *my,float *mz)
{
    unsigned char res;  
	res=IIC_Device_Read_Len(iic_pin,AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
	} 	
    IIC_Device_Write_Byte(iic_pin,AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}

/**
  * @brief  ���ٶȲ���ֵ�ĳ�ʼ��
	* @param  gxoffset,gyoffset,gzoffset:������ٶȵĲ���ֵ
  * @retval void
  */
void MPU_Get_Gyroscope_Init(IIC_PIN_Typedef * iic_pin,float *gxoffset, float *gyoffset, float *gzoffset)
{
	unsigned char buf[6];
	short gx,gy,gz=0;
	int i=0,cnt=0,sum_x=0,sum_y=0,sum_z=0;
	
	for(i = 0;i < 1024 ;i++)
	{
		if(IIC_Device_Read_Len(iic_pin,MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf)==0)
		{
      gx=(short)(((unsigned short int)buf[0]<<8)|buf[1]);
      gy=(short)(((unsigned short int)buf[2]<<8)|buf[3]);
      gz=(short)(((unsigned short int)buf[4]<<8)|buf[5]);
			if(i>300)// ǰ300�����ݲ���
			{
        sum_x += gx;
        sum_y += gy;
				sum_z += gz;
				cnt ++;
			}
		}
	}
	*gxoffset = ((float)sum_x)/((float)cnt);
  *gyoffset = ((float)sum_y)/((float)cnt);
  *gzoffset = ((float)sum_z)/((float)cnt);
}

///**
//  * @brief MPU���ݸ���
//	* @param  void
//  * @retval void
//  */
//void MPU_Update(void)
//{
//	/* ����IMU���� ---------------------------------------------------------------------------------------------*/
//	if(mpu_mpl_get_data(&MPU9250_Data.pitch,&MPU9250_Data.roll,&MPU9250_Data.yaw) == 0)
//	{

//		/* ��ȡ���ٶ����� -----------------------------*/
//		if(!MPU_Get_Gyroscope(&MPU9250_IIC_PIN, &MPU9250_Data.gx, &MPU9250_Data.gy, &MPU9250_Data.gz))
//		{
//			 MPU9250_Data.gx -= MPU9250_Data.gxoffset;
//			 MPU9250_Data.gy -= MPU9250_Data.gyoffset;
//			 MPU9250_Data.gz -= MPU9250_Data.gzoffset;
//		}
//	 
//		/* ��ȡ���ٶ����� -----------------------------*/
//		MPU_Get_Accelerometer(&MPU9250_IIC_PIN,&MPU9250_Data.ax,&MPU9250_Data.ay,&MPU9250_Data.az);
//		
//		/* ��ȡ���������� -----------------------------*/
//		MPU_Get_Magnetometer(&MPU9250_IIC_PIN,&MPU9250_Data.mx,&MPU9250_Data.my,&MPU9250_Data.mz);
//		
//		/* ��ȡ�¶� */
//		MPU9250_Data.temp = MPU_Get_Temperature(&MPU9250_IIC_PIN);
//	}	
//}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
