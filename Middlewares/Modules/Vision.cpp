/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Vision.cpp
  * @author  YDX 2244907035@qq.com
  * @brief   Code for vision interfaces.
  * @date    2020-03-02
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2020-03-02  <td> 1.0     <td>YDX     <td>
  * </table>
  *
  ==============================================================================
                          How to use this driver  
  ==============================================================================
    @note
	    -# provide vision interfaces for missile tasks.
	    -# 图像坐标规定：左上角顶点为原点，向下为x轴正方向，向右为y轴正方向
		
    @warning	
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */	
	
/* Includes ------------------------------------------------------------------*/
#include "Vision.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Vision vision;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  图像的中值滤波器
  * @param[in]  src 	源图像数组
  * @param[in]  dst 	目标数组
	* @param filter_row 中值滤波的行数
	* @param filter_col 中值滤波的列数
  * @return void
  */
void Vision::Image_MedianFilter(uint8_t *src,uint8_t *dst)
{		
    uint8_t *lpSrc;			              
	uint8_t *lpDst;		                 	
	uint8_t aValue[filter_row*filter_col]; 	//指向滤波器数组的指针
	uint8_t	i,j;
	uint8_t fi,fj;		                
	for(i = (filter_row-1)/2;i < ROW - (filter_row-1)/2;i++)
	{
		for(j = (filter_col-1)/2;j < COL - (filter_col-1)/2;j++)
		{
			for(fi = 0; fi < filter_row; fi++)
			{
				for(fj = 0; fj < filter_col; fj++)
				{
					lpSrc = src + i*COL + j;  		 											//指向源图像
					aValue[fi * filter_row + fj] = *lpSrc;
				}
			}
			lpDst = dst + i*COL + j;					 											//指向目标图像
			*lpDst = Get_Median(aValue, filter_row * filter_col);		//获取中值
		}
	}
}
 
/**
  * @brief  获取中值
  * @param[in] bArray  			待处理的数组
  * @param[in] filter_len		数组长度
  * @return void
  */
uint8_t Vision::Get_Median(uint8_t *bArray,uint8_t filter_len)
{
	uint8_t	i,j;			
	uint8_t bTemp;
	
	// 用冒泡法对数组进行排序
	for (j = 0; j < filter_len - 1; j ++)
	{
		for (i = 0; i < filter_len - j - 1; i ++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// 互换
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}
	
	// 计算中值
	if ((filter_len & 1) > 0)
	{
		// 数组有奇数个元素，返回中间一个元素
		bTemp = bArray[(filter_len - 1) / 2];
	}
	else
	{
		// 数组有偶数个元素，返回中间两个元素平均值
		bTemp = (bArray[filter_len / 2 - 1] + bArray[filter_len / 2]) / 2;
	}
	return bTemp;
}
 
/**
  * @brief  二值化
  * @param  threshold 二值化的阈值
  * @return void
  */
void Vision::Image_Binaryzation(uint8_t threshold)	
{
	uint8_t i,j;
    for(i=0;i<ROW;i++)
    { 
        for(j=0;j<COL;j++)
        { 
            if(mt9v032.image[i][j] >= threshold)
				BW_Image[i][j] = White;
			else
				BW_Image[i][j] = Black;
        }
    }
}

/**
  * @brief  标记光源的中心点(灰色十字)
  * @param[in] arr 图像数组 
  * @param[in] threshold 二值化阈值
  * @return void
  */
void Vision::Sign_Center(uint8_t arr[][COL], uint8_t threshold)
{
	arr[(uint8_t)center.x][(uint8_t)center.y] = 128;
	if((uint8_t)center.x > 0 && (uint8_t)center.x < ROW - 1)
	{
		arr[(uint8_t)center.x-1][(uint8_t)center.y] = 128;
		arr[(uint8_t)center.x+1][(uint8_t)center.y] = 128;
	}
			if((uint8_t)center.y > 0 && (uint8_t)center.y < COL - 1)
	{
		arr[(uint8_t)center.x][(uint8_t)center.y-1] = 128;
		arr[(uint8_t)center.x][(uint8_t)center.y+1] = 128;
	}
}

/**
  * @brief  获取二值化图像, 并更新光源中心位置
  * @param[in] arr 图像数组 
  * @param[in]  threshold 二值化的阈值
  * @retval void
  */
void Vision::Image_Solution(uint8_t threshold)
{
    uint8_t i,j;
	
		/* 获取二值化图像 */
    for(i=0;i<ROW;i++)
    { 
        for(j=0;j<COL;j++)
        { 
					if(mt9v032.image[i][j] >= threshold)
					{
						BW_Image[i][j] = White;
						Whitepixels.Sum_x += j;
						Whitepixels.Sum_y += i;
						Whitepixels.Sum++;             //白点个数
					}
					else
						BW_Image[i][j] = Black;
				}
    }
		
		/* 查找中心点 */
    if(Whitepixels.Sum != 0)
    {
        center.x = (float)Whitepixels.Sum_x / Whitepixels.Sum;
        center.y = (float)Whitepixels.Sum_y / Whitepixels.Sum;
    }
		else
		{
			center.x = COL / 2;
			center.y = ROW / 2;
		}
		
		/* 清空中间变量 */
    Whitepixels.Sum_x = 0;
    Whitepixels.Sum_y = 0;
    Whitepixels.Sum = 0;
		
		/* 标记中心点 */
		Sign_Center(BW_Image, threshold);
}

/**
  * @brief  图像压缩
  * @param[in]  src 源图像数组
  * @param[in]  dst 目标数组
  * @return void
  */
void Vision::Image_Compression(uint8_t *dst)
{
    uint8_t i,j,k;
    uint8_t temdst;
    for(i=0;i<ROW;i++)
    {
        for(j=0;j<COL;j=j+8)
        {
            temdst = 0;
            for(k=0;k<8;k++)
            {
                if(mt9v032.image[i][j+k] ==1)
                    temdst |=(0x01<<(7-k));
            }
            *dst = temdst; 
            dst++;
        }
    }
}

/**
  * @brief  将图像数据存储到SD卡中
  * @param 	void
  * @return void
  */
void Vision::Inf_Save(void)
{
	if(file_flag == 0)
	{
		/* 创建一级文件夹 */
		sprintf(path_1, "%s/", SDcard.Mkdir("0:/", "PHOTOS"));
		
		/* 创建二级文件夹 */
		sprintf(path_2, "%s/", SDcard.Inc_Mkdir(path_1, "TEST"));	
		
		file_flag = 1;		
	}
	else 
	{
		SDcard.Write(path_2, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size()); 
	}
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/  

