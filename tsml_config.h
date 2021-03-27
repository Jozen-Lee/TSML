#ifndef __TSML_CONFIG_H__
#define __TSML_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Drivers ----------------------------------------------------*/
#define USE_TWML_BMP280                     0
#define USE_TWML_BME280                     0
#define USE_TWML_FLASH                      0
#define USE_TSML_IMU                        0
#define USE_TSML_MT9V032                    0
#define USE_TSML_SDCARD                     0
#define USE_TSML_SERVO                      0
  
#define USE_TSML_I2C                        0
#define USE_TSML_SPI                        0
	
/* Middlewares -----------------------------------------------*/
#define USE_TSML_DMP                        0
#define USE_TSML_MPL                        0
#define USE_TSML_FILTERS                    0
#define USE_TSML_KALMAN                     0
#define USE_TSML_INS                        0
#define USE_TSML_INTERGRAL                  0
  
#define USE_TSML_LAB_UM                     0
#define USE_TSML_ANO_UM                     0

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT(C) TuTu Studio **************************/
