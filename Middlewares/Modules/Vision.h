/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Vision.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for vision interfaces.
  *         
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

#ifndef  _VISION_H_
#define  _VISION_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "System_DataPool.h" 

/* Private define ------------------------------------------------------------*/
#define Black 0
#define White 1	

/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/

/* 像素点信息存储结构体 */
typedef struct
{
	uint32_t Sum_x;
	uint32_t Sum_y;    
	uint32_t Sum;    
}Whitepixels_t;

/* 中心点数据 */
typedef struct
{
	float x;
	float y;    
}Center_Point;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Vision
{
public:
	Vision(): photo("PHOTO"){}
	~Vision(){}
	void Image_Solution(uint8_t threshold);	
	void Image_Compression(uint8_t *dst);
	void Inf_Save(void);
	Center_Point center;
private:
	void Image_Binaryzation(uint8_t threshold);	
	uint8_t Get_Median(uint8_t *bArray,uint8_t filter_len);	
	void Sign_Center(uint8_t arr[][COL], uint8_t threshold);
	void Image_MedianFilter(uint8_t *src,uint8_t *dst);
	static const uint8_t filter_row = 3;
	static const uint8_t filter_col = 3;
	Whitepixels_t Whitepixels;
	uint8_t BW_Image[ROW][COL];	

	/* SD卡存储图片所需变量 */
	Inc_Name<5, _BMP> photo;
	Photo_Creator<ROW, COL> bmp;
	char path_1[36];
	char path_2[36];
	uint8_t file_flag = 0;

};
/* Exported variables --------------------------------------------------------*/
extern Vision vision;
/* Exported function declarations --------------------------------------------*/
#endif

#ifdef  __cplusplus
extern "C"{
#endif

#ifdef  __cplusplus
}
#endif

#endif  

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
