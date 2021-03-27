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
      -# ��ʼ��
	     flash.Init(W25Q256, GPIOF, GPIO_PIN_6);	
			
			-# ��д����
				flash.Write(pBuffer, WriteAddr, NumByteToWrite);
				flash.Read(pBuffer, WriteAddr, NumByteToWrite);
				
    @warning	
      -# 	4KbytesΪһ��Sector, 16������Ϊ1��Block
	  
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
myFLASH flash;  //ʵ������

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void FLASH_CS_LOW(GPIO_TypeDef* FLASH_CS_GPIO ,uint32_t FLASH_CS_PIN) {HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_RESET);}
void FLASH_CS_HIGH(GPIO_TypeDef* FLASH_CS_GPIO ,uint32_t FLASH_CS_PIN) {HAL_GPIO_WritePin(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO_PIN_SET);}

/* Private function prototypes -----------------------------------------------*/

/**
 *  @brief FLASH��ʼ��
 *	@param[in] gpiox CS�˿ڵ�GPIOx
 *  @param[in] cs_pinx CS�˿ڵ�GPIO_PIN
 *  @return 0 succeed
						1 fail
 */
uint8_t myFLASH::Init(uint16_t type, GPIO_TypeDef *cs_gpiox, uint32_t cs_pinx)
{
	
	/* ��ʼ������ */
	dev.CS_GPIO_PORT = cs_gpiox;
	dev.CS_GPIO_PIN	 = cs_pinx;
	dev.ReadWriteByte = SPI_ReadWriteByte;
	
	/* ��ȡFLASH��ID */
	FLASH_ID = ReadID();
	if(type == FLASH_ID) return 0;
	else return 1;
}

/**
 *  @brief ��ȡFLASH��״̬�Ĵ���
 *	@param[in] void
 *  @return byte ����
 */
 uint8_t myFLASH::ReadSR(void)   
{  
	uint8_t byte=0;   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* ���Ͷ�ȡ״̬�Ĵ������� */
	dev.ReadWriteByte(W25X_ReadStatusReg);  

	/* ��ȡһ���ֽ� */	
	byte=dev.ReadWriteByte(0Xff);                
  FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	return byte;   
} 

/**
 *  @brief дFLASH��״̬�Ĵ���
 *	@note ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д
 *  @return void
 */
void myFLASH::Write_SR(uint8_t sr)   
{    
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* ����дȡ״̬�Ĵ������� */
	dev.ReadWriteByte(W25X_WriteStatusReg); 
  
	/* д��һ���ֽ� */
	dev.ReadWriteByte(sr);               	
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
}   

/**
 *  @brief FLASHдʹ��	
 *	@note ��WEL��λ  
 *  @return void
 */  
void myFLASH::Write_Enable(void)   
{  
		FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* ����дʹ�� */
    dev.ReadWriteByte(W25X_WriteEnable);    
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
} 

/**
 *  @brief FLASHд��ֹ		
 *	@note ��WEL����   
 *  @return void
 */  
void myFLASH::Write_Disable(void)   
{  
		FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* ����д��ָֹ�� */
    dev.ReadWriteByte(W25X_WriteDisable);     
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
} 	

/**
 *  @brief ��ȡоƬID 		  
 *  @return оƬID:0XEF14
 */ 
uint16_t myFLASH::ReadID(void)
{
	uint16_t Temp = 0;	  
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);	

	/* ���Ͷ�ȡID���� */
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
 *  @brief ��������оƬ
 *	@note ��Ƭ����ʱ��:
					W25X16:25s 
					W25X32:40s 
					W25X64:40s 
 *  @return void
 */ 
void myFLASH::Erase_Chip(void)   
{           
	/* ����WEL�Ĵ��� */
	Write_Enable();                   
	Wait_Busy();   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
	
	/* ����Ƭ�������� */
	dev.ReadWriteByte(W25X_ChipErase);         
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);

	/* �ȴ�оƬ�������� */
	Wait_Busy();   				   				
}   

/**
 *  @brief ����һ������
 *	@note ��Сʱ�䣺150ms
 *  @param[in] Dst_Addr ������ַ 0~2047 for W25Q64
 *  @return void
 */ 
void myFLASH::Erase_Sector(uint32_t Dst_Addr)   
{   
	Dst_Addr*=4096;
	/* ����WEL�Ĵ��� */
	myFLASH::Write_Enable();  
	Wait_Busy();   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);   
	
	/* ������������ָ�� */
	dev.ReadWriteByte(W25X_SectorErase);  

	/* ����24bit��ַ */
	dev.ReadWriteByte((uint8_t)((Dst_Addr)>>16));     
	dev.ReadWriteByte((uint8_t)((Dst_Addr)>>8));   
	dev.ReadWriteByte((uint8_t)Dst_Addr);  
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN); 

	/* �ȴ�������� */
	Wait_Busy();   				   
}  

/**
 *  @brief �ȴ�����
 *  @return void
 */
void myFLASH::Wait_Busy(void)   
{   
	while ((ReadSR()&0x01)==0x01);   // �ȴ�BUSYλ���
} 

/**
 *  @brief �������ģʽ
 *  @return void
 */
void myFLASH::PowerDown(void)   
{ 
  	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* ���͵������� */
    dev.ReadWriteByte(W25X_PowerDown);        
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
		
		/* �ȴ�TRES1 */
    dev.delay(3);                           	 
}  

/**
 *  @brief ����
 *  @return void
 */
void myFLASH::WAKEUP(void)   
{ 
  	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
		/* ����ȡ���������� */
    dev.ReadWriteByte(W25X_ReleasePowerDown);        
		FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);   
		
		/* �ȴ�TRES1 */
    dev.delay(3);                              
}  

/**
 *  @brief ��ȡFLASH����
 *	@note ��ָ����ַ��ʼ��ȡָ�����ȵ�����
 *	@param[in] pBuffer 				���ݴ洢��
 *	@param[in] ReadAddr 			��ʼ��ȡ�ĵ�ַ(24bit)
 *	@param[in] NumByteToRead 	Ҫ��ȡ���ֽ���(���65535)
 *  @return void
 */  
void myFLASH::Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
 	uint16_t i;   
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
	
	/* ���Ͷ�ȡ���� */
	dev.ReadWriteByte(W25X_ReadData);   

	/* ����24bit��ַ */
	dev.ReadWriteByte((uint8_t)((ReadAddr)>>16));     
	dev.ReadWriteByte((uint8_t)((ReadAddr)>>8));   
	dev.ReadWriteByte((uint8_t)ReadAddr);   
	for(i=0;i<NumByteToRead;i++)
	{ 
		/* ѭ������ */
    pBuffer[i]=dev.ReadWriteByte(0XFF);     
  }    	      
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);
}  

/**
 *  @brief д��FLASH����
 *	@note ��ָ����ַ��ʼд��ָ�����ȵ�����
 *  @warning �ú�������������
 *	@param[in] pBuffer 				���ݴ洢��
 *	@param[in] WriteAddr 			��ʼд��ĵ�ַ(24bit)
 *	@param[in] NumByteToWrite 	Ҫд����ֽ���(���65535)
 *  @return void
 */  		   
void myFLASH::Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    

	secpos=WriteAddr/4096;//������ַ 
	secoff=WriteAddr%4096;//�������ڵ�ƫ��
	secremain=4096-secoff;//����ʣ��ռ��С   

	/* ������4096���ֽ� */
	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;
	while(1) 
	{	
		/* ������������������ */
		Read(SPI_FLASH_BUF,secpos*4096,4096);
		
		/* У������ */
		for(i=0;i<secremain;i++)
		{
			/* ��Ҫ���� */
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)break; 	  
		}
		
		/* ��Ҫ���� */
		if(i<secremain)
		{
			/* ����������� */
			Erase_Sector(secpos);
			
			/* ���� */
			for(i=0;i<secremain;i++)	   
			{
				SPI_FLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			
			/* д���������� */
			Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096); 

		}
		
		/* д�Ѿ������˵�,ֱ��д������ʣ������ */
		else Write_NoCheck(pBuffer,WriteAddr,secremain); 	
		
		/* д����� */
		if(NumByteToWrite==secremain)break;
		
		/* д��δ���� */
		else
		{
			/* ������ַ��1, ƫ��λ��Ϊ0 */
			secpos++;
			secoff=0; 
			
			/* ����ָ���д��ַƫ�� */
		  pBuffer+=secremain;  
			WriteAddr+=secremain;   
			
			/* �ֽ����ݼ� */
		  NumByteToWrite-=secremain;			
			
			/* ��һ����������д���� */
			if(NumByteToWrite>4096)secremain=4096;	
			
			/* ��һ����������д�� */
			else secremain=NumByteToWrite;			
		}	 
	};	 	 
}

/**
 *  @brief ��һҳ(0~65535)��д������256���ֽڵ�����
 *	@param[in] pBuffer 				���ݴ洢��
 *	@param[in] WriteAddr 			��ʼд��ĵ�ַ(24bit)
 *	@param[in] NumByteToWrite 	Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	
 *  @return void
 */  		 
void myFLASH::Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
  Write_Enable();                  
	FLASH_CS_LOW(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN); 
		
	/* ����д��ҳ��ָ�� */
	SPI_ReadWriteByte(W25X_PageProgram);    

	/* ����24bit��ַ */
	SPI_ReadWriteByte((uint8_t)((WriteAddr)>>16));    
	SPI_ReadWriteByte((uint8_t)((WriteAddr)>>8));   
	SPI_ReadWriteByte((uint8_t)WriteAddr);   
	
	/* ѭ��д�� */
	for(i=0;i<NumByteToWrite;i++)SPI_ReadWriteByte(pBuffer[i]); 
	FLASH_CS_HIGH(dev.CS_GPIO_PORT, dev.CS_GPIO_PIN);  
	Wait_Busy();					   
} 

/**
 *  @brief �޼���д��FLASH����
 *	@note ��ָ����ַ��ʼд��ָ�����ȵ�����
 *  @warning ����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
 *	@param[in] pBuffer 				���ݴ洢��
 *	@param[in] WriteAddr 			��ʼд��ĵ�ַ(24bit)
 *	@param[in] NumByteToWrite 	Ҫд����ֽ���(���65535)
 *  @return void
 */ 
void myFLASH::Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 			 		 
	uint16_t pageremain;	   
	
	/* ��ҳʣ����ֽ���	*/
	pageremain=256 - WriteAddr % 256; 	 	    
	
	/* ������256���ֽ� */
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;
	while(1)
	{	   
		Write_Page(pBuffer,WriteAddr,pageremain);
		
		/* д����� */
		if(NumByteToWrite==pageremain)break;
	 	else 
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			/* ��ȥ�Ѿ�д���˵��ֽ��� */
			NumByteToWrite-=pageremain;		

			/* һ�ο���д��256���ֽ� */
			if(NumByteToWrite>256)pageremain=256;

			/* ����256���ֽ� */
			else pageremain=NumByteToWrite; 	  
		}
	};	    
} 

 /************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
