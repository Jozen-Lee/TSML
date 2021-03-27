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
	    -# ͼ������涨�����ϽǶ���Ϊԭ�㣬����Ϊx������������Ϊy��������
		
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
  * @brief  ͼ�����ֵ�˲���
  * @param[in]  src 	Դͼ������
  * @param[in]  dst 	Ŀ������
	* @param filter_row ��ֵ�˲�������
	* @param filter_col ��ֵ�˲�������
  * @return void
  */
void Vision::Image_MedianFilter(uint8_t *src,uint8_t *dst)
{		
    uint8_t *lpSrc;			              
	uint8_t *lpDst;		                 	
	uint8_t aValue[filter_row*filter_col]; 	//ָ���˲��������ָ��
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
					lpSrc = src + i*COL + j;  		 											//ָ��Դͼ��
					aValue[fi * filter_row + fj] = *lpSrc;
				}
			}
			lpDst = dst + i*COL + j;					 											//ָ��Ŀ��ͼ��
			*lpDst = Get_Median(aValue, filter_row * filter_col);		//��ȡ��ֵ
		}
	}
}
 
/**
  * @brief  ��ȡ��ֵ
  * @param[in] bArray  			�����������
  * @param[in] filter_len		���鳤��
  * @return void
  */
uint8_t Vision::Get_Median(uint8_t *bArray,uint8_t filter_len)
{
	uint8_t	i,j;			
	uint8_t bTemp;
	
	// ��ð�ݷ��������������
	for (j = 0; j < filter_len - 1; j ++)
	{
		for (i = 0; i < filter_len - j - 1; i ++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// ����
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}
	
	// ������ֵ
	if ((filter_len & 1) > 0)
	{
		// ������������Ԫ�أ������м�һ��Ԫ��
		bTemp = bArray[(filter_len - 1) / 2];
	}
	else
	{
		// ������ż����Ԫ�أ������м�����Ԫ��ƽ��ֵ
		bTemp = (bArray[filter_len / 2 - 1] + bArray[filter_len / 2]) / 2;
	}
	return bTemp;
}
 
/**
  * @brief  ��ֵ��
  * @param  threshold ��ֵ������ֵ
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
  * @brief  ��ǹ�Դ�����ĵ�(��ɫʮ��)
  * @param[in] arr ͼ������ 
  * @param[in] threshold ��ֵ����ֵ
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
  * @brief  ��ȡ��ֵ��ͼ��, �����¹�Դ����λ��
  * @param[in] arr ͼ������ 
  * @param[in]  threshold ��ֵ������ֵ
  * @retval void
  */
void Vision::Image_Solution(uint8_t threshold)
{
    uint8_t i,j;
	
		/* ��ȡ��ֵ��ͼ�� */
    for(i=0;i<ROW;i++)
    { 
        for(j=0;j<COL;j++)
        { 
					if(mt9v032.image[i][j] >= threshold)
					{
						BW_Image[i][j] = White;
						Whitepixels.Sum_x += j;
						Whitepixels.Sum_y += i;
						Whitepixels.Sum++;             //�׵����
					}
					else
						BW_Image[i][j] = Black;
				}
    }
		
		/* �������ĵ� */
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
		
		/* ����м���� */
    Whitepixels.Sum_x = 0;
    Whitepixels.Sum_y = 0;
    Whitepixels.Sum = 0;
		
		/* ������ĵ� */
		Sign_Center(BW_Image, threshold);
}

/**
  * @brief  ͼ��ѹ��
  * @param[in]  src Դͼ������
  * @param[in]  dst Ŀ������
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
  * @brief  ��ͼ�����ݴ洢��SD����
  * @param 	void
  * @return void
  */
void Vision::Inf_Save(void)
{
	if(file_flag == 0)
	{
		/* ����һ���ļ��� */
		sprintf(path_1, "%s/", SDcard.Mkdir("0:/", "PHOTOS"));
		
		/* ���������ļ��� */
		sprintf(path_2, "%s/", SDcard.Inc_Mkdir(path_1, "TEST"));	
		
		file_flag = 1;		
	}
	else 
	{
		SDcard.Write(path_2, photo.up(), bmp.Bmp_Encode((uint8_t*)mt9v032.image), bmp.Get_Size()); 
	}
}

	/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/  

