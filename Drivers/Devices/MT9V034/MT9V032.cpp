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
			-# 根据所需功能的不同,重定义DCMI帧中断回调函数
				e.g:
				
				-# DCMI帧中断回调函数
				void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
				{
					mt9v032.DCMI_Frame_Callback(hdcmi);
				}
				
			-# 初始化
				mt9v032.Init(&huart2, &hdcmi);
			
			-# 将图像发送到上位机
				mt9v032.Send_Image(&huart1);
				
    @warning	
      -# 使用该外设需要在CubeMX中开启`DCMI`和`USART`功能
	  
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

/** @brief    	串口中断事件回调函数
	* @param[in]  huart 串口句柄
	* @return     void
	*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   mt9v032.Usart_Receive_Callback(huart);
}

/* 需要配置到摄像头的数据 */
int16_t MT9V032_CFG[CONFIG_FINISH][2]=
{
    {AUTO_EXP,          0},   //自动曝光设置      范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    //一般情况是不需要开启这个功能，因为比赛场地光线一般都比较均匀，如果遇到光线非常不均匀的情况可以尝试设置该值，增加图像稳定性
    {EXP_TIME,          300}, //曝光时间          摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值 300
    {FPS,               300},	//图像帧率          摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS 
    {SET_COL, 					COL},	//图像列数量        范围1-752     K60采集不允许超过188
    {SET_ROW,  					ROW},	//图像行数量        范围1-480
    {LR_OFFSET,         0},   //图像左右偏移量    正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {UD_OFFSET,         0},   //图像上下偏移量    正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {GAIN,              48},  //图像增益          范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    {INIT,              0}    //摄像头开始初始化
};

/* 从摄像头内部获取到的配置数据 */
int16_t GET_CFG[CONFIG_FINISH-1][2]=
{
    {AUTO_EXP,          0},   //自动曝光设置
    {EXP_TIME,          0},   //曝光时间
    {FPS,               0},   //图像帧率
    {SET_COL,           0},   //图像列数量
    {SET_ROW,           0},   //图像行数量
    {LR_OFFSET,         0},   //图像左右偏移量
    {UD_OFFSET,         0},   //图像上下偏移量
    {GAIN,              0},   //图像增益
};

/* Private variables --------------------------------------------------------- */
MT9V032 mt9v032;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @brief  		摄像头初始化函数
	* @note				对摄像头配置的数据全部都会保存在摄像头上51单片机的eeprom中, 而使用Set_Exposure_Time配置的曝光率不会保存到eeprom中
	*	@param[in]  _huart 串口句柄	
	*	@param[in]  _hdcmi DCMI句柄	
	*	@return   void 
	*/
uint8_t MT9V032::Init(UART_HandleTypeDef* _huart, DCMI_HandleTypeDef* _hdcmi)
{
	/* 内部变量配置 */
	framerate = 0;
	finish_flag = 0;
	receive_num = 0;
	uart_receive_flag = 1;
	
	/* 句柄设置 */
	huart = _huart;
	hdcmi = _hdcmi;

	/* 设置参数 */
	uint16_t temp, i;
	uint8_t  send_buffer[4];
	
	/* 使能串口接收中断 */
	HAL_UART_Receive_IT(huart, &rx_buff, 1);
	HAL_Delay(1000);
	
	/* 设置接收标志位 */
	uart_receive_flag = 0;

	//开始配置摄像头并重新初始化
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

	/* 等待摄像头初始化成功 */
	while(!uart_receive_flag)	HAL_Delay(1);

	uart_receive_flag = 0;

	while((0xff != receive[1]) || (0xff != receive[2]));

	/* 获取配置便于查看配置是否正确 */
	Get_Config();
	
	/* 开启帧中断，一帧中断一次 */
	__HAL_DCMI_ENABLE_IT(hdcmi, DCMI_IT_FRAME); 	
	
	/* 使能DCMI_DMA，连续采集图像 */
	HAL_DCMI_Start_DMA(hdcmi, DCMI_MODE_CONTINUOUS,(uint32_t)image, ROW * COL/4);	
	
	return 0;
}

/** @brief  串口中断事件回调函数
	*	@note   用于配置MT9V032(总钻风摄像头)和接收总钻风摄像头数据
	*	@param      void	
	*	@return     void 
	*/
void MT9V032::Usart_Receive_Callback(UART_HandleTypeDef *_huart)
{
	if(_huart->Instance == huart->Instance)
  {
		receive[receive_num] = rx_buff;
		receive_num++;

		/* 接收数据检验,确定是摄像头的数据 */
		if(receive_num==1 && receive[0]!=0XA5)  receive_num = 0;
		
		/* 接收完成 */
		if(3 == receive_num)
		{
			receive_num = 0;
			uart_receive_flag = 1;
		}

		HAL_UART_Receive_IT(huart, &rx_buff, 1);
	}
}

/** @brief  DCMI帧中断事件回调函数
	*	@param      void	
	*	@return     void 
	*/
void MT9V032::DCMI_Frame_Callback(DCMI_HandleTypeDef *_hdcmi)
{
	framerate ++;
}

void MT9V032::Send_Image(UART_HandleTypeDef * _huart)
{
	/* 协议头 */
	uint8_t send_buffer[4] = {0x00,0xff,0x01,0x01};
	uint8_t i;
	
	/* 如果图像数组过大，将分包发送,以65535字节为单位将其分割 */
	/* 分割pack_num个包 */
	uint8_t pack_num = ROW*COL >> 16; 
	
	/* 尾包数据量 */
	uint16_t pack_tail = ROW*COL & 0xFFFF;  
	
	HAL_UART_Transmit(_huart, send_buffer, 4, 0xffff);

	
	/* 先发送pack_num * 65535字节数 */
	for(i=0; i<pack_num; i++)
	{
			HAL_UART_Transmit(_huart, (uint8_t *)image + i*0xFFFF, 0xFFFF, 0xffff); 
	}
	
	/* 然后发送尾包 */
	HAL_UART_Transmit(_huart, (uint8_t *)image + ROW*COL - pack_tail, pack_tail, 0xffff); 	
}

/** @brief  获取摄像头内部配置信息
	*	@param 	void
	* @return void  
	*/
void MT9V032::Get_Config(void)
{
	uint16_t temp, i;
	uint8_t  send_buffer[4];

	/* 逐个读取配置数据 */
	for(i = 0; i < CONFIG_FINISH-1; i++)
	{
		/* 设置数据头 */
		send_buffer[0] = 0xA5;
		
		/* 设置发送的数据类型 */
		send_buffer[1] = GET_STATUS;
		
		/* 写入要发送的数据 */
		temp = GET_CFG[i][0];
		send_buffer[2] = temp>>8;
		send_buffer[3] = (uint8_t)temp;

		HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

		//等待接受回传数据
		while(!uart_receive_flag)	HAL_Delay(1);

		uart_receive_flag = 0;

		GET_CFG[i][1] = receive[1]<<8 | receive[2];
	}
}

/** @brief  获取摄像头固件版本
	*	@param 	void
	* @return 版本号  
	*/
uint16_t MT9V032::Get_Version(void)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* 设置数据头 */
	send_buffer[0] = 0xA5;
	
	/* 设置发送的数据类型 */
	send_buffer[1] = GET_STATUS;
	
	/* 写入要发送的数据 */
	temp = GET_VERSION;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	/* 等待接受回传数据 */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

/** @brief  		设置摄像头曝光时间
	*	@param[in] 	设置曝光时间越大图像越亮，摄像头收到后会根据分辨率及FPS计算最大曝光时间如果设置的数据过大，那么摄像头将会设置这个最大值
	* @return 		当前曝光值，用于确认是否正确写入  
	*/
uint16_t MT9V032::Set_Exposure_Time(uint16_t light)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* 设置数据头 */
	send_buffer[0] = 0xA5;
	
	/* 设置发送的数据类型 */
	send_buffer[1] = SET_EXP_TIME;
	
	/* 写入要发送的数据 */
	temp = light;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	/* 等待接受回传数据 */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

/** @brief  		对摄像头内部寄存器进行写操作
	*	@param[in] 	addr  摄像头内部寄存器地址
	*	@param[in]	data  需要写入的数据
	* @return 		寄存器当前数据，用于确认是否写入成功 
	*/
uint16_t MT9V032::Set_Reg(uint8_t addr, uint16_t data)
{
	uint16_t temp;
	uint8_t  send_buffer[4];
	
	/* 设置数据头 */
	send_buffer[0] = 0xA5;
	
	/* 设置发送的数据类型 */
	send_buffer[1] = SET_ADDR;
	
	/* 写入要发送的数据 */
	temp = addr;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);

	HAL_Delay(10);
	
	/* 设置数据头 */
	send_buffer[0] = 0xA5;
	
	/* 设置发送的数据类型 */
	send_buffer[1] = SET_DATA;
	
	/* 写入要发送的数据 */
	temp = data;
	send_buffer[2] = temp>>8;
	send_buffer[3] = (uint8_t)temp;

	HAL_UART_Transmit(huart, send_buffer, 4, 0xffff);
	
	/* 等待接受回传数据 */
	while(!uart_receive_flag);

	uart_receive_flag = 0;

	return ((uint16_t)(receive[1]<<8) | receive[2]);
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
