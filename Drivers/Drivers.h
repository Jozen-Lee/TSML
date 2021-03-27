/**
  ******************************************************************************
  * @file    Drivers.h
  * @brief   Header to include all Drivers.
  * @author  TuTu 2250017028@qq.com
  * @version 0.0.1
  ******************************************************************************
  * This library inherit the architechure of TSML and is open source for all developers.
  * If you find any mistakes, plz criticize and correct them.
  * 
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  * 
  *                          License Agreement
  *                           For TuTu Studio
  * 
  * Copyright (c) 2021 - ~, TuTu Studio, all rights reserved.
  * 
  * This file includes all of the headers of TSML.
  * 
  * Before using this library, plz make sure that u have read the README document
  * carefully,  
  *    @note
  *     - Plz do not modifiy this file(Except for developer).
  *     - Plz remember to update the version number.
  */
#pragma once
/** @addtogroup Drivers
  * @{
  */
#include <twml_config.h>
/* Devices header begin */
#if USE_TWML_BMP280
#include "Devices/Barometer/BMP280/bmp280.h"
#endif
#if USE_TWML_BME280
#include "Devices/Barometer/BME280/bme280_app.h"
#endif
#if USE_TWML_FLASH
#include "Devices/FLASH/flash.h"
#endif
#if USE_TSML_IMU
#include "Devices/IMU/imu.h"
#endif
#if USE_TSML_MT9V032
#include "Devices/MT9V034/MT9V032.h"
#endif
#if USE_TSML_SDCARD
#include "Devices/SDcard/SDcard.h"
#endif
#if USE_TSML_SERVO
#include "Devices/Servo/Servo.h"
#endif
/* Devices header end */

/* Components header begin */
#if USE_TSML_I2C
#include "Components/I2C/tutu_drv_i2c.h"
#endif
#if USE_TSML_SPI
#include "Components/SPI/tutu_driver_spi.h"
#endif
/* Components header end */

/**
  * @}
  */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
