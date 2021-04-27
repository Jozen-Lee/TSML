/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
	* @file    sk6812.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for SK6812MINI.
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

#ifndef __SK6812_H
#define __SK6812_H	 

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>

/* Private define ------------------------------------------------------------*/
#define RGB_NUM 	24		//RGB��ռ��Ŀ
#define LED_LOW 	26		//0�źŵ�ռ�ձ�
#define LED_HIGH 	54		//1�źŵ�ռ�ձ�									

/* Private include -----------------------------------------------------------*/
#include "tim.h"

/* Private macros ------------------------------------------------------------*/
extern uint32_t Temp_Buffer[420][RGB_NUM];
extern uint32_t Reset_RGB[70];
/* Private type --------------------------------------------------------------*/

/* ��ɫ��ṹ�� */
typedef struct RGB
{	
	uint8_t red;
	uint8_t green;
	uint8_t blue;
}color;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
template<uint16_t NUM>
class LED
{
public:
	LED(TIM_HandleTypeDef *htimer, DMA_HandleTypeDef *_hdma, uint32_t _channel)
	: ledPwmTimer(htimer), hdma(_hdma), Channel(_channel){}
		
	/**@brief ����
	*@param[in] data ��ɫ����
	*@return 0 success
					 1 fail
	*/
	uint8_t ON(uint16_t data[NUM][RGB_NUM])
	{	
		uint16_t i, j;
		uint8_t res;
		
		/* ���ڶ�ʱ��2�ļĴ���Ϊ32λ,���Էֿ����� */
		if(ledPwmTimer->Instance == TIM2)
		{
			for(i = 0; i < NUM; i ++)
			{
				for(j = 0; j < RGB_NUM; j ++) Temp_Buffer[i][j] = data[i][j];
			}
			res = Set((uint32_t *)Temp_Buffer, NUM * RGB_NUM);
		}
		else 
		{
			res = Set((uint32_t *)data, NUM * RGB_NUM);
		}
		Reset();
		return res;
	}
	
	/**@brief ���
	*@param[in] data ��ɫ����,����Ҫ��ֵ
	*@return 0 success
					 1 fail
	*/
	uint8_t OFF(uint16_t data[NUM][RGB_NUM])
	{
		uint16_t i, j;
		uint8_t res;
		
		/* ���ڶ�ʱ��2�ļĴ���Ϊ32λ,���Էֿ����� */
		if(ledPwmTimer->Instance == TIM2)
		{
			for(i = 0; i < NUM; i ++)
			{
				for(j = 0; j < RGB_NUM; j ++) Temp_Buffer[i][j] = LED_LOW;
			}
			res = Set((uint32_t *)Temp_Buffer, NUM * RGB_NUM);
		}		
		else
		{
			for(i = 0; i < NUM; i ++)
			{
				for(j = 0; j < RGB_NUM; j ++) data[i][j] = LED_LOW;
			}		
			res = Set((uint32_t *)data, NUM * RGB_NUM);
		}
		Reset();
		return res;
	}	

private:
	TIM_HandleTypeDef* ledPwmTimer;
	DMA_HandleTypeDef* hdma;
	uint32_t Channel;
	
	/**@brief ������ɫ����
		*@param void
		*@return 0 success
						 1 fail
	*/
	uint8_t Set(uint32_t* data, uint32_t num)
	{
		if(!Data_Transmit(data, num)) return 0;
		else return 1;	
	}

	/**@brief ������������
		*@param void
		*@return 0 success
						 1 fail
	*/
	uint8_t Reset(void)
	{
		if(!Data_Transmit(Reset_RGB,70)) return 0;
		else return 1;
	}

	/**@brief ���ݴ���
		*@param[in]	buffer 	���黺��
		*@param[in] num			������Ŀ
		*@return 0 success
						 1 fail
	*/
	uint8_t Data_Transmit(uint32_t* buffer, uint32_t num)
	{	
		/* ��ʼ�������� */
		while(HAL_TIM_PWM_Start_DMA(ledPwmTimer, Channel, buffer, num) != HAL_OK);
		
		/* �ȴ�������� */
		while(HAL_DMA_GetState(hdma) != HAL_DMA_STATE_READY);
		
		/* ��ͨ����DMAҪ�ر� */
		if(hdma == &hdma_tim2_ch2_ch4)
		while(HAL_TIM_PWM_Stop_DMA(ledPwmTimer, Channel) != HAL_OK);
		
		return 0;
	}	
};
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
#endif	

#ifdef __cplusplus
 extern "C" {
#endif
void led_test(void);
#ifdef __cplusplus
}
#endif

	

#endif
