/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_timer.h
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Code for Timer Management in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
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
	
#ifndef _TUTU_DRIVER_SPI_H
#define _TUTU_DRIVER_SPI_H

#ifdef  __cplusplus
extern "C"{
#endif
	
#include "main.h"

uint8_t SPI_ReadWriteByte(uint8_t TxData);
	
#ifdef  __cplusplus
}
#endif

#endif
