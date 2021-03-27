/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    MT9V032.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for MT9V032.
  * @date    2021-02-24
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-24  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note		
			-# �������蹦�ܵĲ�ͬ,�ض���DCMI֡�жϻص�����
				e.g:
				
				-# DCMI֡�жϻص�����
				void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
				{
					mt9v032.DCMI_Frame_Callback(hdcmi);
				}
				
			-# ��ʼ��
				mt9v032.Init(&huart2, &hdcmi);
			
			-# ��ͼ���͵���λ��
				mt9v032.Send_Image(&huart1);
				
    @warning	
      -# ʹ�ø�������Ҫ��CubeMX�п���`DCMI`��`USART`����
	  
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

/* Includes ------------------------------------------------------------------ */
#include "MT9V032.h"

/* Private define ------------------------------------------------------------ */

/** @brief    	�����ж��¼��ص�����
	* @param[in]  huart ���ھ��
	* @return     void
	*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   mt9v032.Usart_Receive_Callback(huart);
}

/* ��Ҫ���õ�����ͷ������ */
int16_t MT9V032_CFG[CONFIG_FINISH][2]=
{
    {AUTO_EXP,          0},   //�Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
    //һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
    {EXP_TIME,          300}, //�ع�ʱ��          ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ 300
    {FPS,               300},	//ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS 
    {SET_COL, 					COL},	//ͼ��������        ��Χ1-752     K60�ɼ���������188
    {SET_ROW,  					ROW},	//ͼ��������        ��Χ1-480
    {LR_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
    {UD_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
    {GAIN,              48},  //ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�
    {INIT,              0}    //����ͷ��ʼ��ʼ��
};

/* ������ͷ�ڲ���ȡ������������ */
int16_t GET_CFG[CONFIG_FINISH-1][2]=
{
    {AUTO_EXP,          0},   //�Զ��ع�����
    {EXP_TIME,          0},   //�ع�ʱ��
    {FPS,               0},   //ͼ��֡��
    {SET_COL,           0},   //ͼ��������
    {SET_ROW,           0},   //ͼ��������
    {LR_OFFSET,         0},   //ͼ������ƫ����
    {UD_OFFSET,         0},   //ͼ������ƫ����
    {GAIN,              0},   //ͼ������
};

/* Private variables --------------------------------------------------------- */
MT9V032 mt9v032;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @brief  		����ͷ��ʼ������
	* @note				������ͷ���õ�����ȫ�����ᱣ��������ͷ��51��Ƭ����eeprom��, ��ʹ��Set_Exposure_Time���õ��ع��ʲ��ᱣ�浽eeprom��
	*	@param[in]  _huart ���ھ��	
	*	@param[in]  _hdcmi DCMI���	
	*	@return   void 
	*/
uint8_t MT9V032::Init(UART_HandleTypeDef* _huart, DCMI_HandleTypeDef* _hdcmi)
{
	/* �ڲ��������� */
	framerate = 0;
	finish_flag = 0;
	receive_num = 0;
	uart_receive_flag = 1;
	
	/* ������� */
	huart = _huart;
	hdcmi = _hdcmi;

	/* ���ò��� */
	uint16_t temp, i;
	uint8_t  send_buffer[4];
	
	/* ʹ�ܴ��ڽ����ж� */
	HAL_UART_Receive_IT(huart, &rx_buff, 1);
	HAL_Delay(1000);
	
	/* ���ý��ձ�־λ */
	uart_receive_flag = 0;

	//��ʼ��������ͷ�����³�ʼ��
	for(i = 0; i < CONFIG_FINISH; i++)
	{
			send_buffer[0] = 0xA5;
			send_buffer[1] = MT9V032_CFG[i][0];
			temp = MT9V032_CFG[i][1];
			send_buffer[2] = temp>>8;
			send_buffer[3] = (uint8_t)temp;
			HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);
			HAL_Delay(10);
	}

	/* �ȴ�����ͷ��ʼ���ɹ� */
	while(!uart_receive_flag)	HAL_Delay(1);

	uart_receive_flag = 0;

	while((0xff != receive[1]) || (0xff != receive[2]));

	/* ��ȡ���ñ��ڲ鿴�����Ƿ���ȷ */
	Get_Config();
	
	/* ����֡�жϣ�һ֡�ж�һ�� */
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME); 	
	
	/* ʹ��DCMI_DMA�������ɼ�ͼ�� */
	HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_CONTINUOUS,(uint32_t)image, ROW * COL/4);	
	
	return 0;
}

/** @brief  �����ж��¼��ص�����
	*	@note   ��������MT9V032(���������ͷ)�ͽ������������ͷ����
	*	@param      void	
	*	@return     void 
	*/
void MT9V032::Usart_Receive_Callback(UART_HandleTypeDef *_huart)
{
	if(_huart->Instance == huart->Instance)
  {
		receive[receive_num] = rx_buff;
		receive_num++;

		/* �������ݼ���,ȷ��������ͷ������ */
		if(receive_num==1 && receive[0]!=0XA5)  receive_num = 0;
		
		/* ������� */
		if(3 == receive_num)
		{
			receive_num = 0;
			uart_receive_flag = 1;
		}

		HAL_UART_Receive_IT(huart, &rx_buff, 1);
	}
}

/** @brief  DCMI֡�ж��¼��ص�����
	*	@param      void	
	*	@return     void 
	*/
void MT9V032::DCMI_Frame_Callback(DCMI_HandleTypeDef *_hdcmi)
{
	framerate ++;
}

void MT9V032::Send_Image(UART_HandleTypeDef * _huart)
{
	/* Э��ͷ */
	uint8_t send_buffer[4] = {0x00,0xff,0x01,0x01};
	uint8_t i;
	
	/* ���ͼ��������󣬽��ְ�����,��65535�ֽ�Ϊ��λ����ָ� */
	/* �ָ�pack_num���� */
	uint8_t pack_num = ROW*COL >> 16; 
	
	/* β�������� */
	uint16_t pack_tail = ROW*COL & 0xFFFF;  
	
	HAL_UART_Transmit(_huart, send_buffer, 4, 0xffff);

	
	/* �ȷ���pack_num * 65535�ֽ��� */
	for(i=0; i<pack_num; i++)
	{
			HAL_UART_Transmit(_huart, (uint8_t *)image + i*0xFFFF, 0xFFFF, 0xffff); 
	}
	
	/* Ȼ����β�� */
	HAL_UART_Transmit(_huart, (uint8_t *)image + ROW*COL - pack_tail, pack_tail, 0xffff); 	
}

/** @brief  ��ȡ����ͷ�ڲ�������Ϣ
	*	@param 	void
	* @return void  
	*/
void MT9V032::Get_Config(void)
{
	uint16_t temp, i;
	uint8_t  send_buffer[4];

	/* �����ȡ�������� */
	for(i = 0; i < CONFIG_FINISH-1; i++)
	{
		/* ��������ͷ */
		send_buffer[0] = 0xA5;
		
		/* ���÷��͵��������� */
		send_buffer[1] = GET_STATUS;
		
		/* д��Ҫ���͵����� */
		temp = GET_CFG[i][0];
		send_buffer[2] = temp>>8;
		send_buffer[3] = (uint8_t)temp;

		HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

		//�ȴ����ܻش�����
		while(!uart_receive_flag)	HAL_Delay(1);

		uart_receive_flag = 0;

		GET_CFG[i][1] = receive[1]<<8 | receive[2];
	}
}

/** @brief  ��ȡ����ͷ�̼��汾
	*	@param 	void
	* @return �汾��  
	*/
uint16_t MT9V032::Get_Version(void)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* ��������ͷ */
	send_buffer[0] = 0xA5;
	
	/* ���÷��͵��������� */
	send_buffer[1] = GET_STATUS;
	
	/* д��Ҫ���͵����� */
	temp = GET_VERSION;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	/* �ȴ����ܻش����� */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

/** @brief  		��������ͷ�ع�ʱ��
	*	@param[in] 	�����ع�ʱ��Խ��ͼ��Խ��������ͷ�յ������ݷֱ��ʼ�FPS��������ع�ʱ��������õ����ݹ�����ô����ͷ��������������ֵ
	* @return 		��ǰ�ع�ֵ������ȷ���Ƿ���ȷд��  
	*/
uint16_t MT9V032::Set_Exposure_Time(uint16_t light)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* ��������ͷ */
	send_buffer[0] = 0xA5;
	
	/* ���÷��͵��������� */
	send_buffer[1] = SET_EXP_TIME;
	
	/* д��Ҫ���͵����� */
	temp = light;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	/* �ȴ����ܻش����� */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

/** @brief  		������ͷ�ڲ��Ĵ�������д����
	*	@param[in] 	addr  ����ͷ�ڲ��Ĵ�����ַ
	*	@param[in]	data  ��Ҫд�������
	* @return 		�Ĵ�����ǰ���ݣ�����ȷ���Ƿ�д��ɹ� 
	*/
uint16_t MT9V032::Set_Reg(uint8_t addr, uint16_t data)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* ��������ͷ */
	send_buffer[0] = 0xA5;
	
	/* ���÷��͵��������� */
	send_buffer[1] = SET_ADDR;
	
	/* д��Ҫ���͵����� */
	temp = addr;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	HAL_Delay(10);
	
	/* ��������ͷ */
	send_buffer[0] = 0xA5;
	
	/* ���÷��͵��������� */
	send_buffer[1] = SET_DATA;
	
	/* д��Ҫ���͵����� */
	temp = data;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);
	
	/* �ȴ����ܻش����� */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
