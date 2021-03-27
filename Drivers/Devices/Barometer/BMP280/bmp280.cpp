/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    bmp280.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for BMP280.
  * @date    2021-02-21
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-02  <td> 1.0     <td>LJY  			<td>Creator
	* <tr><td>2021-02-21  <td> 1.1     <td>LJY  			<td>Change the code frame
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# ��ʼ��
	     ����`BMP280::Init()`���г�ʼ��
         e.g:
         bmp280.Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
         
      -# ��ȡ����
				bmp280.Update_Data();
		 
    @warning	
      -# ʹ��BME280��ȡ����ǰҪ�ȳ�ʼ��
	  
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
#include "bmp280.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
/*bmp280 ��ѹ���¶ȹ����� ����ģʽ*/
#define BMP280_PRESSURE_OSR				(BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR		(BMP280_OVERSAMP_8X)
#define BMP280_MODE								(BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)

/* Private variables ---------------------------------------------------------*/
BMP280 bmp280;	//ʵ������

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief BMP280��ʼ��
 *  @param[in] gpiox GPIO���
 *  @param[in] scl_pinx, sda_pinx IIC�Ķ�ӦIO��
 *  @return void
 */
 uint8_t BMP280::Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx)
{	
	uint8_t res=0;
	/* ��ʼ��IIC��Ӧ�˿� */
	IIC_Setting(&IIC_PIN, gpiox, scl_pinx, sda_pinx);
	
	/* ��ȡBMP��ID */
	res = IIC_Device_Read_Byte(&IIC_PIN, BMP280_I2C_ADDR, BMP280_CHIP_ID);
	
	/* ���ID��ȷ */
	if(res == BMP280_DEFAULT_CHIP_ID)
	{
		/* ��ȡУ׼���� */
		IIC_Device_Read_Len(&IIC_PIN, BMP280_I2C_ADDR, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&BMP280_Cal);
		
		/* ����ģʽ */
		IIC_Device_Write_Byte(&IIC_PIN, BMP280_I2C_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE);
		
		/*����IIR�˲�*/
		IIC_Device_Write_Byte(&IIC_PIN, BMP280_I2C_ADDR, BMP280_CONFIG_REG, 5<<2);		
		return 0;
	}
  else return 1;  
	
}

/**
 *  @brief ��ȡ��ѹֵ(hPa),�¶�ֵ(��),���θ߶�(m)
 *  @note ���ݴ洢��bmp280.data��
 *  @return void
 */
void BMP280::Update_Data(void)
{
  float tem , pre; //�¶�ֵ����ѹֵ
	
	/* ��ȡ��ѹֵ */
	BMP280_Get_Data();

	tem = BMP280_Compensate_T(BMP280RawTemperature)/100.0;		
	pre = BMP280_Compensate_P(BMP280RawPressure)/25600.0;		

	/* ����˲������ѹֵ */
	data.pressure = pre_filter.f(pre);
	
	/* ���ת������¶�ֵ */
	data.temperature = (float)tem;
	
	/* ��ѹת���ɺ��� */	
	data.asl=Pre_To_Alt(&data.pressure);	
}

/**
 *  @brief ����ѹֵת��Ϊ����
 *  @param[in] adcP ��ѹֵ(hPa)
 *  @return ���θ߶�
 */
float BMP280::Pre_To_Alt(float* pressure)
{	
    if(*pressure > 0) return 44330.0f * (pow((101570.0f / *pressure), 0.190295f) - 1.0f);
    else return 0;
}

/**
 *  @brief ��ȡ��ѹֵ
 *  @param[in] void
 *  @return void
 */
void BMP280::BMP280_Get_Data(void)
{
    uint8_t data[BMP280_DATA_FRAME_SIZE];

    /* ��ȡ����*/
		IIC_Device_Read_Len(&IIC_PIN, BMP280_I2C_ADDR, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
	
		/* ������ѹ���¶����� */
    BMP280RawPressure = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
    BMP280RawTemperature = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
}

/**
 *  @brief ��ȡ�¶Ȳ���ֵ,��λΪ���϶�,����Ϊ0.01��,���ֵ "5123" == 51.23��
 *  @note BMP280_Cal.t_fine �д洢���¶�ֵ
 *  @param[in] adcT �¶ȵ�ADCת��ֵ
 *  @return T �¶�
 */
uint32_t BMP280::BMP280_Compensate_T(int32_t adcT)
{
    int32_t var1, var2, T;

    var1 = ((((adcT >> 3) - ((int32_t)BMP280_Cal.dig_T1 << 1))) * ((int32_t)BMP280_Cal.dig_T2)) >> 11;
    var2  = (((((adcT >> 4) - ((int32_t)BMP280_Cal.dig_T1)) * ((adcT >> 4) - ((int32_t)BMP280_Cal.dig_T1))) >> 12) * ((int32_t)BMP280_Cal.dig_T3)) >> 14;
    BMP280_Cal.t_fine = var1 + var2;
	
    T = (BMP280_Cal.t_fine * 5 + 128) >> 8;

    return T;
}

/**
 *  @brief ��ȡ��ѹֵ,��λΪPa,�ҷ���Q24.8�ĸ�������ʽ
 *  @note ���ֵ"24674867" -->  24674867/256 = 96386.2 Pa = 963.862 hPa
 *  @param[in] adcP ��ѹ��ADCת��ֵ
 *  @return P ��ѹ
 */
uint32_t BMP280::BMP280_Compensate_P(int32_t adcP)
{
    int64_t var1, var2, p;
	
    var1 = ((int64_t)BMP280_Cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)BMP280_Cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)BMP280_Cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)BMP280_Cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)BMP280_Cal.dig_P3) >> 8) + ((var1 * (int64_t)BMP280_Cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)BMP280_Cal.dig_P1) >> 33;
	
    if (var1 == 0) return 0;
    p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)BMP280_Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)BMP280_Cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)BMP280_Cal.dig_P7) << 4);
    return (uint32_t)p;
}





	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
