/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @brief      		总钻风(灰度摄像头)历程
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		MDK 5.17
 * @Target core		STM32F407ZGT6
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-10-11
 * @note
					OV7725接线定义：
					------------------------------------
						模块管脚            单片机管脚
						SDA                A2
						SCL                A3
						场中断              B7
                        行中断              A4
						像素中断            A6

						D0                 C6
                        D1                 C7
                        D2                 C8
                        D3                 C9
                        D4                 C11
                        D5                 B6
                        D6                 E5
                        D7                 E6

					------------------------------------
                    USB转TTL接线定义
                        USB转TTL引脚        单片机引脚
                        TX                  A10
                        RX                  A9

					默认分辨率              查看SEEKFREE_MT9V032.h 文件内的宏定义 COL ROW
					默认FPS                 50帧
 ********************************************************************************************************************/

#ifndef _SEEKFREE_MT9V032_H
#define _SEEKFREE_MT9V032_H


#include "main.h"
#include "System_DataPool.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COL     60//图像宽度    范围1-752    STM32可采集752*480分辨率，不过需要外部SRAM，内部的RAM不足以存储一副图像
#define ROW     45//图像高度    范围1-480


//摄像头命令枚举
typedef enum
{
	INIT = 0,               //摄像头初始化命令
	AUTO_EXP,               //自动曝光命令
	EXP_TIME,               //曝光时间命令
	FPS,                    //摄像头帧率命令
	SET_COL,                //图像列命令
	SET_ROW,                //图像行命令
	LR_OFFSET,              //图像左右偏移命令
	UD_OFFSET,              //图像上下偏移命令
	GAIN,                   //图像偏移命令
	CONFIG_FINISH,          //非命令位，主要用来占位计数

	SET_EXP_TIME = 0XF0,    //单独设置曝光时间命令
	GET_STATUS,             //获取摄像头配置命令
	GET_VERSION,            //固件版本号命令

	SET_ADDR = 0XFE,        //寄存器地址命令
	SET_DATA                //寄存器数据命令
} CMD;


extern uint8_t mt9v032_finish_flag;  //一场图像采集完成标志位
extern uint8_t image[ROW][COL];      //内部使用
extern uint8_t finish_flag_032;


void mt9v032_init(void);
void mt9v032_cof_uart_interrupt(void);
void seekfree_sendimg_032(void);

#ifdef __cplusplus
}
#endif
#endif
