/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    flash.cpp
  * @author  LJY 2250017028@qq.com
  * @brief   Code for extern FLASH.
  * @date    2021-02-21
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2021-02-21  <td> 1.0     <td>LJY  			<td>Creator
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
	     flash.Init(W25Q256, GPIOF, GPIO_PIN_6);	
			
			-# 读写数据
				flash.Write(pBuffer, WriteAddr, NumByteToWrite);
				flash.Read(pBuffer, WriteAddr, NumByteToWrite);
				
    @warning	
      -# 	4Kbytes为一个Sector, 16个扇区为1个Block
	  
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

/* Includes ------------------------------------------------------------------*/
#include "flash.h" 
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
myFLASH flash;  //实例对象

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void FLASH_CS_LOW(GPIO_TypeDef* FLASH_CS_GPIO ,uint32_t FLASH_CS_PIN) {HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_RESET);}
void FLASH_CS_HIGH(GPIO_TypeDef* FLASH_CS_GPIO ,uint32_t FLASH_CS_PIN) {HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);}

/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief FLASH初始化
 *	@param[in] gpiox CS端口的GPIOx
 *  @param[in] cs_pinx CS端口的GPIO_PIN
 *  @return 0 succeed
						1 fail
 */
uint8_t myFLASH::Init(uint16_t type, GPIO_TypeDef *cs_gpiox, uint32_t cs_pinx)
{
	
	/* 初始化驱动 */
	dev.CS_GPIO_PORT = cs_gpiox;
	dev.CS_GPIO_PIN	 = cs_pinx;
	dev.ReadWriteByte = SPI_ReadWriteByte;
	
	/* 获取FLASH的ID */
	FLASH_ID = ReadID();
	if(type == FLASH_ID) return 0;
	else return 1;
}

/**
 *  @brief 读取FLASH的状态寄存器
 *	@param[in] void
 *  @return byte 数据
 */
 uint8_t myFLASH::ReadSR(void)   
{  
	uint8_t byte=0;   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* 发送读取状态寄存器命令 */
	dev.ReadWriteByte(W25X_ReadStatusReg);  

	/* 读取一个字节 */	
	byte=dev.ReadWriteByte(0Xff);                
  FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	return byte;   
} 

/**
 *  @brief 写FLASH的状态寄存器
 *	@note 只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写
 *  @return void
 */
void myFLASH::Write_SR(uint8_t sr)   
{    
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* 发送写取状态寄存器命令 */
	dev.ReadWriteByte(W25X_WriteStatusReg); 
  
	/* 写入一个字节 */
	dev.ReadWriteByte(sr);               	
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
}   

/**
 *  @brief FLASH写使能	
 *	@note 将WEL置位  
 *  @return void
 */  
void myFLASH::Write_Enable(void)   
{  
		FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* 发送写使能 */
    dev.ReadWriteByte(W25X_WriteEnable);    
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
} 

/**
 *  @brief FLASH写禁止		
 *	@note 将WEL清零   
 *  @return void
 */  
void myFLASH::Write_Disable(void)   
{  
		FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* 发送写禁止指令 */
    dev.ReadWriteByte(W25X_WriteDisable);     
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
} 	

/**
 *  @brief 读取芯片ID 		  
 *  @return 芯片ID:0XEF14
 */ 
uint16_t myFLASH::ReadID(void)
{
	uint16_t Temp = 0;	  
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);	

	/* 发送读取ID命令 */
	dev.ReadWriteByte(0x90);	    
	dev.ReadWriteByte(0x00); 	    
	dev.ReadWriteByte(0x00); 	    
	dev.ReadWriteByte(0x00); 	 			   
	Temp|=dev.ReadWriteByte(0xFF)<<8;  
	Temp|=dev.ReadWriteByte(0xFF);
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	return Temp;
}   	

/**
 *  @brief 擦除整个芯片
 *	@note 整片擦除时间:
					W25X16:25s 
					W25X32:40s 
					W25X64:40s 
 *  @return void
 */ 
void myFLASH::Erase_Chip(void)   
{           
	/* 设置WEL寄存器 */
	Write_Enable();                   
	Wait_Busy();   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
	
	/* 发送片擦除命令 */
	dev.ReadWriteByte(W25X_ChipErase);         
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);

	/* 等待芯片擦除结束 */
	Wait_Busy();   				   				
}   

/**
 *  @brief 擦除一个扇区
 *	@note 最小时间：150ms
 *  @param[in] Dst_Addr 扇区地址 0~2047 for W25Q64
 *  @return void
 */ 
void myFLASH::Erase_Sector(uint32_t Dst_Addr)   
{   
	Dst_Addr*=4096;
	/* 设置WEL寄存器 */
	myFLASH::Write_Enable();  
	Wait_Busy();   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);   
	
	/* 发送扇区擦除指令 */
	dev.ReadWriteByte(W25X_SectorErase);  

	/* 发送24bit地址 */
	dev.ReadWriteByte((uint8_t)((Dst_Addr)>>16));     
	dev.ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
	dev.ReadWriteByte((uint8_t)Dst_Addr);  
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN); 

	/* 等待擦除完成 */
	Wait_Busy();   				   
}  

/**
 *  @brief 等待空闲
 *  @return void
 */
void myFLASH::Wait_Busy(void)   
{   
	while ((ReadSR()&0x01)==0x01);   // 等待BUSY位清空
} 

/**
 *  @brief 进入掉电模式
 *  @return void
 */
void myFLASH::PowerDown(void)   
{ 
  	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* 发送掉电命令 */
    dev.ReadWriteByte(W25X_PowerDown);        
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
		
		/* 等待TRES1 */
    dev.delay(3);                           	 
}  

/**
 *  @brief 唤醒
 *  @return void
 */
void myFLASH::WAKEUP(void)   
{ 
  	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* 发送取消掉电命令 */
    dev.ReadWriteByte(W25X_ReleasePowerDown);        
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);   
		
		/* 等待TRES1 */
    dev.delay(3);                              
}  

/**
 *  @brief 读取FLASH数据
 *	@note 在指定地址开始读取指定长度的数据
 *	@param[in] pBuffer 				数据存储区
 *	@param[in] ReadAddr 			开始读取的地址(24bit)
 *	@param[in] NumByteToRead 	要读取的字节数(最大65535)
 *  @return void
 */  
void myFLASH::Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* 发送读取命令 */
	dev.ReadWriteByte(W25X_ReadData);   

	/* 发送24bit地址 */
	dev.ReadWriteByte((uint8_t)((ReadAddr)>>16));     
	dev.ReadWriteByte((uint8_t)((ReadAddr)>>8));   
	dev.ReadWriteByte((uint8_t)ReadAddr);   
	for(i=0;i<NumByteToRead;i++)
	{ 
		/* 循环读数 */
    pBuffer[i]=dev.ReadWriteByte(0XFF);     
  }    	      
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
}  

/**
 *  @brief 写入FLASH数据
 *	@note 在指定地址开始写入指定长度的数据
 *  @warning 该函数带擦除操作
 *	@param[in] pBuffer 				数据存储区
 *	@param[in] WriteAddr 			开始写入的地址(24bit)
 *	@param[in] NumByteToWrite 	要写入的字节数(最大65535)
 *  @return void
 */  		   
void myFLASH::Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    

	secpos=WriteAddr/4096;//扇区地址 
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小   

	/* 不大于4096个字节 */
	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;
	while(1) 
	{	
		/* 读出整个扇区的内容 */
		Read(SPI_FLASH_BUF,secpos*4096,4096);
		
		/* 校验数据 */
		for(i=0;i<secremain;i++)
		{
			/* 需要擦除 */
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)break; 	  
		}
		
		/* 需要擦除 */
		if(i<secremain)
		{
			/* 擦除这个扇区 */
			Erase_Sector(secpos);
			
			/* 复制 */
			for(i=0;i<secremain;i++)	   
			{
				SPI_FLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			
			/* 写入整个扇区 */
			Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096); 

		}
		
		/* 写已经擦除了的,直接写入扇区剩余区间 */
		else Write_NoCheck(pBuffer,WriteAddr,secremain); 	
		
		/* 写入结束 */
		if(NumByteToWrite==secremain)break;
		
		/* 写入未结束 */
		else
		{
			/* 扇区地址增1, 偏移位置为0 */
			secpos++;
			secoff=0; 
			
			/* 数据指针和写地址偏移 */
		  pBuffer+=secremain;  
			WriteAddr+=secremain;   
			
			/* 字节数递减 */
		  NumByteToWrite-=secremain;			
			
			/* 下一个扇区还是写不完 */
			if(NumByteToWrite>4096)secremain=4096;	
			
			/* 下一个扇区可以写完 */
			else secremain=NumByteToWrite;			
		}	 
	};	 	 
}

/**
 *  @brief 在一页(0~65535)内写入少于256个字节的数据
 *	@param[in] pBuffer 				数据存储区
 *	@param[in] WriteAddr 			开始写入的地址(24bit)
 *	@param[in] NumByteToWrite 	要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	
 *  @return void
 */  		 
void myFLASH::Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
  Write_Enable();                  
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN); 
		
	/* 发送写入页的指令 */
	SPI_ReadWriteByte(W25X_PageProgram);    

	/* 发送24bit地址 */
	SPI_ReadWriteByte((uint8_t)((WriteAddr)>>16));    
	SPI_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
	SPI_ReadWriteByte((uint8_t)WriteAddr);   
	
	/* 循环写数 */
	for(i=0;i<NumByteToWrite;i++)SPI_ReadWriteByte(pBuffer[i]); 
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
	Wait_Busy();					   
} 

/**
 *  @brief 无检验写入FLASH数据
 *	@note 在指定地址开始写入指定长度的数据
 *  @warning 必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
 *	@param[in] pBuffer 				数据存储区
 *	@param[in] WriteAddr 			开始写入的地址(24bit)
 *	@param[in] NumByteToWrite 	要写入的字节数(最大65535)
 *  @return void
 */ 
void myFLASH::Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
	
	/* 单页剩余的字节数	*/
	pageremain=256 - WriteAddr % 256; 	 	    
	
	/* 不大于256个字节 */
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;
	while(1)
	{	   
		Write_Page(pBuffer,WriteAddr,pageremain);
		
		/* 写入结束 */
		if(NumByteToWrite==pageremain)break;
	 	else 
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			/* 减去已经写入了的字节数 */
			NumByteToWrite-=pageremain;		

			/* 一次可以写入256个字节 */
			if(NumByteToWrite>256)pageremain=256;

			/* 不够256个字节 */
			else pageremain=NumByteToWrite; 	  
		}
	};	    
} 

 /************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
