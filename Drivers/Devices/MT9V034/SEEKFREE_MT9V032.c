/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @brief      		�����(�Ҷ�����ͷ)����
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		MDK 5.17
 * @Target core		STM32F407ZGT6
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-10-11
 * @note
					���߶��壺
					------------------------------------
					ģ��ܽ�            ��Ƭ���ܽ�
					SDA                 A2
					SCL                 A3
					���ж�              B7
					���ж�              A4
					�����ж�            A6

					D0                  C6
					D1                  C7
					D2                  C8
					D3                  C9
					D4                  C11
					D5                  B6
					D6                  E5
					D7                  E6

					------------------------------------
                    USBתTTL���߶���
					USBתTTL����        ��Ƭ������
					TX                  A10
					RX                  A9

					Ĭ�Ϸֱ���              �鿴SEEKFREE_MT9V032.h �ļ��ڵĺ궨�� COL ROW
					Ĭ��FPS                 50֡
 ********************************************************************************************************************/

#include "SEEKFREE_MT9V032.h"
#define MT9V032_COF_UART    &huart2
uint8_t image[ROW][COL];      //ͼ������
uint8_t finish_flag_032 = 0;
uint8_t receive[3];
uint8_t receive_num = 0;
uint8_t uart_receive_flag = 1;
uint8_t rx_buff;

//��Ҫ���õ�����ͷ������
int16_t MT9V032_CFG[CONFIG_FINISH][2]=
{
    {AUTO_EXP,          0},   //�Զ��ع�����      ��Χ1-63 0Ϊ�ر� ����Զ��ع⿪��  EXP_TIME�������õ����ݽ����Ϊ����ع�ʱ�䣬Ҳ�����Զ��ع�ʱ�������
    //һ������ǲ���Ҫ����������ܣ���Ϊ�������ع���һ�㶼�ȽϾ��ȣ�����������߷ǳ������ȵ�������Գ������ø�ֵ������ͼ���ȶ���
    {EXP_TIME,          200}, //�ع�ʱ��          ����ͷ�յ�����Զ����������ع�ʱ�䣬������ù���������Ϊ�������������ع�ֵ 300
    {FPS,               2},  //ͼ��֡��          ����ͷ�յ�����Զ���������FPS���������������Ϊ������������FPS 100
    {SET_COL,           COL}, //ͼ��������        ��Χ1-752     K60�ɼ���������188
    {SET_ROW,           ROW}, //ͼ��������        ��Χ1-480
    {LR_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ188 376 752ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
    {UD_OFFSET,         0},   //ͼ������ƫ����    ��ֵ ��ƫ��   ��ֵ ��ƫ��  ��Ϊ120 240 480ʱ�޷�����ƫ��    ����ͷ��ƫ�����ݺ���Զ��������ƫ�ƣ�������������ü�����������ƫ��
    {GAIN,              32},  //ͼ������          ��Χ16-64     ����������ع�ʱ��̶�������¸ı�ͼ�������̶�


    {INIT,              0}    //����ͷ��ʼ��ʼ��
};

//������ͷ�ڲ���ȡ������������
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


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����������ͷ������2�ж��¼��ص�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//  @note       //��������MT9V032(���������ͷ)�ͽ������������ͷ����
                //�ú�����stm32f4xx_hal_uart.c�ļ��е�UART_Receive_IT�����н��лص�
//-------------------------------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        receive[receive_num] = rx_buff;
        receive_num++;

        if(1==receive_num && 0XA5!=receive[0])  receive_num = 0;
        if(3 == receive_num)
        {
					receive_num = 0;
					uart_receive_flag = 1;
        }

        HAL_UART_Receive_IT(&huart2,&rx_buff,1);
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����ͷ�ڲ�������Ϣ
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				���øú���ǰ���ȳ�ʼ��uart2
//-------------------------------------------------------------------------------------------------------------------
void get_config(void)
{
    uint16_t temp, i;
    uint8_t  send_buffer[4];

    for(i=0; i<CONFIG_FINISH-1; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = GET_STATUS;
        temp = GET_CFG[i][0];
        send_buffer[2] = temp>>8;
        send_buffer[3] = (uint8_t)temp;

        HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);

        //�ȴ����ܻش�����
        while(!uart_receive_flag)	HAL_Delay(1);


        uart_receive_flag = 0;

        GET_CFG[i][1] = receive[1]<<8 | receive[2];

    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����ͷ�̼��汾
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				���øú���ǰ���ȳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
uint16_t get_version(void)
{
    uint16_t temp;
    uint8_t  send_buffer[4];
    send_buffer[0] = 0xA5;
    send_buffer[1] = GET_STATUS;
    temp = GET_VERSION;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8_t)temp;

    HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);

    uart_receive_flag = 0;

    return ((uint16_t)(receive[1]<<8) | receive[2]);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������������ͷ�ع�ʱ��
//  @param      light   �����ع�ʱ��Խ��ͼ��Խ��������ͷ�յ������ݷֱ��ʼ�FPS��������ع�ʱ��������õ����ݹ�����ô����ͷ��������������ֵ
//  @return     uint16_t  ��ǰ�ع�ֵ������ȷ���Ƿ���ȷд��
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint16_t set_exposure_time(uint16_t light)
{
    uint16_t temp;
    uint8_t  send_buffer[4];

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_EXP_TIME;
    temp = light;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8_t)temp;

    HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);

    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];
    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ͷ�ڲ��Ĵ�������д����
//  @param      addr    ����ͷ�ڲ��Ĵ�����ַ
//  @param      data    ��Ҫд�������
//  @return     uint16_t  �Ĵ�����ǰ���ݣ�����ȷ���Ƿ�д��ɹ�
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint16_t set_mt9v032_reg(uint8_t addr, uint16_t data)
{
    uint16_t temp;
    uint8_t  send_buffer[4];

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_ADDR;
    temp = addr;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8_t)temp;

    HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);
    HAL_Delay(10);

    send_buffer[0] = 0xA5;
    send_buffer[1] = SET_DATA;
    temp = data;
    send_buffer[2] = temp>>8;
    send_buffer[3] = (uint8_t)temp;

    HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);

    //�ȴ����ܻش�����
    while(!uart_receive_flag);

    uart_receive_flag = 0;

    temp = receive[1]<<8 | receive[2];

    return temp;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ͷ��ʼ��
//  @return     void
//  @since      v1.1
//  Sample usage:	ʹ��DCMI�ӿڲɼ�����ͷͼ��
//-------------------------------------------------------------------------------------------------------------------
void mt9v032_init(void)
{
    //���ò���    ������ο�ʹ���ֲ�
    uint16_t temp, i;
    uint8_t  send_buffer[4];
    //ʹ�ܴ���2�����ж�
    HAL_UART_Receive_IT(&huart2,&rx_buff,1);
    HAL_Delay(1000);
    uart_receive_flag = 0;

    //��ʼ��������ͷ�����³�ʼ��
    for(i=0; i<CONFIG_FINISH; i++)
    {
        send_buffer[0] = 0xA5;
        send_buffer[1] = MT9V032_CFG[i][0];
        temp = MT9V032_CFG[i][1];
        send_buffer[2] = temp>>8;
        send_buffer[3] = (uint8_t)temp;
        HAL_UART_Transmit(MT9V032_COF_UART,send_buffer,4,0xffff);
        HAL_Delay(10);
    }

    //�ȴ�����ͷ��ʼ���ɹ�
    while(!uart_receive_flag)	HAL_Delay(1);

    uart_receive_flag = 0;

    while((0xff != receive[1]) || (0xff != receive[2]));
    //���ϲ��ֶ�����ͷ���õ�����ȫ�����ᱣ��������ͷ��51��Ƭ����eeprom��
    //����set_exposure_time�����������õ��ع����ݲ��洢��eeprom��

    //��ȡ���ñ��ڲ鿴�����Ƿ���ȷ
    get_config();
    //ʹ��DCMI�ж�
    __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME); 				//����֡�жϣ�һ֡�ж�һ��
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS,(uint32_t)image, ROW * COL/4);	//ʹ��DCMI_DMA�������ɼ�ͼ��
}
