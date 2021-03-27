/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    SDcard.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for SDcard.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _SDCARD_H_
#define _SDCARD_H_

#ifdef __cplusplus
		 
/* Includes ------------------------------------------------------------------*/	
#include "main.h"
#include <string.h>
#include "stdio.h"
#include "ff.h"
	
/* Private macros ------------------------------------------------------------*/
#define F_Sync_Times 30        //synchronization threshold	

	
/* Private type --------------------------------------------------------------*/

/* ͼ������ѹ�������� */
#define BI_RGB	 			0  //û��ѹ��.RGB 5,5,5.
#define BI_RLE8 			1  //ÿ������8���ص�RLEѹ�����룬ѹ����ʽ��2�ֽ����(�ظ����ؼ�������ɫ����)��
#define BI_RLE4 			2  //ÿ������4���ص�RLEѹ�����룬ѹ����ʽ��2�ֽ����
#define BI_BITFIELDS 	3  //ÿ�����صı�����ָ�������������

/* �ļ����͵�ö�� */
typedef enum 
{
	_BMP,
	_JPG,
	_TXT,
	_CSV
} FILE_TYPE;
const char * Get_Type(FILE_TYPE type);

/* RGB�� */ 
typedef __packed struct 
{
    uint8_t Blue ;   	// ָ����ɫǿ��
    uint8_t Green ;		// ָ����ɫǿ�� 
    uint8_t Red ;	  	// ָ����ɫǿ�� 
		uint8_t Reserved;	// ����
}RGB;

/* BMP��Ϣͷ */
typedef __packed struct
{
    uint32_t biSize ;		   			//˵��BITMAPINFOHEADER�ṹ����Ҫ��������
    long  biWidth ;		   				//˵��ͼ��Ŀ�ȣ�������Ϊ��λ 
    long  biHeight ;	   				//˵��ͼ��ĸ߶ȣ�������Ϊ��λ 
    uint16_t  biPlanes ;	   		//ΪĿ���豸˵��λ��������ֵ�����Ǳ���Ϊ1 
    uint16_t  biBitCount ;	   	//˵��������/���أ���ֵΪ1��4��8��16��24����32
    uint32_t biCompression ;  	//˵��ͼ������ѹ�������͡���ֵ����������ֵ֮һ��
																/*BI_RGB��û��ѹ����
																	BI_RLE8��ÿ������8���ص�RLEѹ�����룬ѹ����ʽ��2�ֽ����(�ظ����ؼ�������ɫ����)��  
																	BI_RLE4��ÿ������4���ص�RLEѹ�����룬ѹ����ʽ��2�ֽ����
																	BI_BITFIELDS��ÿ�����صı�����ָ�������������*/
    uint32_t biSizeImage ;			//˵��ͼ��Ĵ�С�����ֽ�Ϊ��λ������BI_RGB��ʽʱ��������Ϊ0  
    long  biXPelsPerMeter ;			//˵��ˮƽ�ֱ��ʣ�������/�ױ�ʾ
    long  biYPelsPerMeter ;			//˵����ֱ�ֱ��ʣ�������/�ױ�ʾ
    uint32_t biClrUsed ;	  	 	//˵��λͼʵ��ʹ�õĲ�ɫ���е���ɫ������
    uint32_t biClrImportant ; 	//˵����ͼ����ʾ����ҪӰ�����ɫ��������Ŀ�������0����ʾ����Ҫ�� 
} INFO_HEADER;

/* BMP�ļ�ͷ */
typedef __packed struct
{
    uint16_t  bfType ;    	//�ļ���־.ֻ��'BM',����ʶ��BMPλͼ����
    uint32_t  bfSize ;	  	//�ļ���С,ռ�ĸ��ֽ�
    uint16_t  bfReserved1 ;	//����
    uint16_t  bfReserved2 ;	//����
    uint32_t  bfOffBits ;  	//���ļ���ʼ��λͼ����(bitmap data)��ʼ֮��ĵ�ƫ����
} FILE_HEADER ;

/* λͼ��Ϣͷ */
typedef __packed struct
{ 
	FILE_HEADER bmfHeader;
	INFO_HEADER bmiHeader;  
	RGB Colors[256];	
} BITMAP_INFO; 

class BMP_CONFIG
{
public:
	BMP_CONFIG(long height, long width);
	~BMP_CONFIG(){}
	BITMAP_INFO* Get_Data(void){ return &hbmp;}
	static const uint32_t SIZE = sizeof(BITMAP_INFO);
private:
	BITMAP_INFO hbmp;	
};


template<int HEIGHT, int WIDTH>
class Photo_Creator
{
public:
	Photo_Creator() : config(HEIGHT, WIDTH)
	{ 
		BITMAP_INFO* config_data = config.Get_Data();
		for(int i = 0; i < BMP_CONFIG::SIZE; i ++)
		{
			buffer[i] = ((uint8_t *)config_data)[i];
		}
	}
	~Photo_Creator(){}
		
	/* BMP���뺯�� */
	uint8_t* Bmp_Encode(uint8_t *data)
	{
		/* ����BMPͼ��������ǵ��õģ���ͼ����ʾ�ĵ�һ�������һ�����ݣ�����Ҫ���� */
		for(int i = HEIGHT - 1; i >= 0; i--)
		{
				for(int j = 0; j < WIDTH; j++)
				{
						buffer[BMP_CONFIG::SIZE + i*WIDTH+j]=data[(HEIGHT-1-i)*WIDTH+j];
				}			
		}
		return buffer;
	}
	
	/* �������ݳ��� */
	uint32_t Get_Size(void){ return SIZE; }
	
private:
	static const uint32_t SIZE = BMP_CONFIG::SIZE + WIDTH * HEIGHT;
	uint8_t buffer[SIZE];																	// �����Ƭ��Ϣ	
	BMP_CONFIG config;																		// BMPͷ������Ϣ
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template<uint8_t TAIL_NUM, FILE_TYPE TYPE>	// β�������ֳ���
class Inc_Name
{
public:
	Inc_Name(){}
	~Inc_Name(){}
		
	/* ��ʼ�� */
	Inc_Name(const TCHAR* _name)
	{
		num = 0;
		sprintf(init_name, "%s%n", _name, (int *)MAX_SIZE);
	}
	
	/* ��ȡ��ʼ���� */
	TCHAR* Get_Init_Name(void) { return init_name; }
	
	/* �����ļ��� */
	TCHAR* up(void)
	{
		sprintf(out_name, "%s_%0*d%n.%s", init_name, TAIL_NUM, num, (int *)MAX_SIZE, Get_Type(TYPE));
		num ++;
		return out_name;
	}
	
private:	
	static const int MAX_SIZE = 36; // �ļ�����󳤶�
	int num;												// ��ǰ���ļ���β׺
	TCHAR init_name[MAX_SIZE];			// Ҫ�������ļ���
	TCHAR out_name[MAX_SIZE];				// ������ļ���
};

class SD_CARD
{
public:
	SD_CARD(){}
	~SD_CARD(){}
	uint8_t Init(void);																																													// ��ʼ��
	const TCHAR* Mkdir(const TCHAR* addr, const TCHAR* name);																										// �����ļ���
	const TCHAR* Inc_Mkdir(const TCHAR* addr, const TCHAR* name);																								// �����ļ���(�ļ���β׺����)
	uint8_t Write(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint16_t len);													// �����ǵش洢����
	uint8_t Write_Cover(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint16_t len);										// ����ʽ�洢����
	uint8_t Read(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint32_t offset, uint16_t len);					// ��ȡ����
private:
	static const int MAX_SIZE = 100;
	char Path_Name[MAX_SIZE];		// �ļ����洢��
	FATFS fatfs;								// �ļ�����ϵͳ����
	FIL fil;			    					// �ļ�����
	FRESULT rc;			    				// �洢����ı���
	UINT bw;                		// ��д���ֽ���
	UINT br;                		// �Ѷ����ֽ���
};

/* Exported variables --------------------------------------------------------*/
extern SD_CARD SDcard;
/* Exported function declarations --------------------------------------------*/


#endif 

#endif /* _SD_CARD_H_ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
