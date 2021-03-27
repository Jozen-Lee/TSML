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

/* 图象数据压缩的类型 */
#define BI_RGB	 			0  //没有压缩.RGB 5,5,5.
#define BI_RLE8 			1  //每个象素8比特的RLE压缩编码，压缩格式由2字节组成(重复象素计数和颜色索引)；
#define BI_RLE4 			2  //每个象素4比特的RLE压缩编码，压缩格式由2字节组成
#define BI_BITFIELDS 	3  //每个象素的比特由指定的掩码决定。

/* 文件类型的枚举 */
typedef enum 
{
	_BMP,
	_JPG,
	_TXT,
	_CSV
} FILE_TYPE;
const char * Get_Type(FILE_TYPE type);

/* RGB表 */ 
typedef __packed struct 
{
    uint8_t Blue ;   	// 指定蓝色强度
    uint8_t Green ;		// 指定绿色强度 
    uint8_t Red ;	  	// 指定红色强度 
		uint8_t Reserved;	// 保留
}RGB;

/* BMP信息头 */
typedef __packed struct
{
    uint32_t biSize ;		   			//说明BITMAPINFOHEADER结构所需要的字数。
    long  biWidth ;		   				//说明图象的宽度，以象素为单位 
    long  biHeight ;	   				//说明图象的高度，以象素为单位 
    uint16_t  biPlanes ;	   		//为目标设备说明位面数，其值将总是被设为1 
    uint16_t  biBitCount ;	   	//说明比特数/象素，其值为1、4、8、16、24、或32
    uint32_t biCompression ;  	//说明图象数据压缩的类型。其值可以是下述值之一：
																/*BI_RGB：没有压缩；
																	BI_RLE8：每个象素8比特的RLE压缩编码，压缩格式由2字节组成(重复象素计数和颜色索引)；  
																	BI_RLE4：每个象素4比特的RLE压缩编码，压缩格式由2字节组成
																	BI_BITFIELDS：每个象素的比特由指定的掩码决定。*/
    uint32_t biSizeImage ;			//说明图象的大小，以字节为单位。当用BI_RGB格式时，可设置为0  
    long  biXPelsPerMeter ;			//说明水平分辨率，用象素/米表示
    long  biYPelsPerMeter ;			//说明垂直分辨率，用象素/米表示
    uint32_t biClrUsed ;	  	 	//说明位图实际使用的彩色表中的颜色索引数
    uint32_t biClrImportant ; 	//说明对图象显示有重要影响的颜色索引的数目，如果是0，表示都重要。 
} INFO_HEADER;

/* BMP文件头 */
typedef __packed struct
{
    uint16_t  bfType ;    	//文件标志.只对'BM',用来识别BMP位图类型
    uint32_t  bfSize ;	  	//文件大小,占四个字节
    uint16_t  bfReserved1 ;	//保留
    uint16_t  bfReserved2 ;	//保留
    uint32_t  bfOffBits ;  	//从文件开始到位图数据(bitmap data)开始之间的的偏移量
} FILE_HEADER ;

/* 位图信息头 */
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
		
	/* BMP编码函数 */
	uint8_t* Bmp_Encode(uint8_t *data)
	{
		/* 由于BMP图像对于行是倒置的，即图像显示的第一行是最后一行数据，所以要倒置 */
		for(int i = HEIGHT - 1; i >= 0; i--)
		{
				for(int j = 0; j < WIDTH; j++)
				{
						buffer[BMP_CONFIG::SIZE + i*WIDTH+j]=data[(HEIGHT-1-i)*WIDTH+j];
				}			
		}
		return buffer;
	}
	
	/* 返回数据长度 */
	uint32_t Get_Size(void){ return SIZE; }
	
private:
	static const uint32_t SIZE = BMP_CONFIG::SIZE + WIDTH * HEIGHT;
	uint8_t buffer[SIZE];																	// 存放照片信息	
	BMP_CONFIG config;																		// BMP头配置信息
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template<uint8_t TAIL_NUM, FILE_TYPE TYPE>	// 尾部的数字长度
class Inc_Name
{
public:
	Inc_Name(){}
	~Inc_Name(){}
		
	/* 初始化 */
	Inc_Name(const TCHAR* _name)
	{
		num = 0;
		sprintf(init_name, "%s%n", _name, (int *)MAX_SIZE);
	}
	
	/* 获取初始名称 */
	TCHAR* Get_Init_Name(void) { return init_name; }
	
	/* 更新文件名 */
	TCHAR* up(void)
	{
		sprintf(out_name, "%s_%0*d%n.%s", init_name, TAIL_NUM, num, (int *)MAX_SIZE, Get_Type(TYPE));
		num ++;
		return out_name;
	}
	
private:	
	static const int MAX_SIZE = 36; // 文件名最大长度
	int num;												// 当前的文件名尾缀
	TCHAR init_name[MAX_SIZE];			// 要递增的文件名
	TCHAR out_name[MAX_SIZE];				// 输出的文件名
};

class SD_CARD
{
public:
	SD_CARD(){}
	~SD_CARD(){}
	uint8_t Init(void);																																													// 初始化
	const TCHAR* Mkdir(const TCHAR* addr, const TCHAR* name);																										// 创建文件夹
	const TCHAR* Inc_Mkdir(const TCHAR* addr, const TCHAR* name);																								// 创建文件夹(文件名尾缀递增)
	uint8_t Write(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint16_t len);													// 不覆盖地存储数据
	uint8_t Write_Cover(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint16_t len);										// 覆盖式存储数据
	uint8_t Read(const TCHAR* addr, const TCHAR* name, uint8_t* buffer, uint32_t offset, uint16_t len);					// 读取数据
private:
	static const int MAX_SIZE = 100;
	char Path_Name[MAX_SIZE];		// 文件名存储区
	FATFS fatfs;								// 文件管理系统对象
	FIL fil;			    					// 文件对象
	FRESULT rc;			    				// 存储结果的变量
	UINT bw;                		// 已写的字节数
	UINT br;                		// 已读的字节数
};

/* Exported variables --------------------------------------------------------*/
extern SD_CARD SDcard;
/* Exported function declarations --------------------------------------------*/


#endif 

#endif /* _SD_CARD_H_ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
