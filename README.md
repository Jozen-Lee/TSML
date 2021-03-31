## TSML: TuTu Studio Middlewares Layer Library

<Font><font size="5">**煜个头头工作室嵌入式软件中间件层库**</Font>

![](https://img.shields.io/badge/当前版本-V1.0-blue.svg)

## Summary 简介

该库基于SRML库的框架进行编写，编写的目的是为了将已经写过的封装库进行统一管理，方便之后的开发，避免重复造轮子，同时方便代码的交流和传承。库的内容大部分为自主封装，也有一部分是为了内容的完整性，从其他库中复制而来（所有的版权信息都完整保留）。由于本人能力有限，库的包容性有待考量，如果在不同的编译环境遇到了奇奇怪怪的BUG，欢迎来找我一起探讨解决(2250017028@qq.com)



### How to Deploy 配置方法

- __获取 TSML 库__
  
  - 参见：[添加子模块的远程仓库](#添加子模块的远程仓库)
- __添加 TSML 库到崭新的 Stm32 工程中__
  - __单个IDE工程__
    - 包含 TSML.h
    - 将本库放置于工程根目录中
    - 编译
  - __多个IDE工程（开发中）__
    
    - 包含 TSML.h
  - [__MDK工程__](https://git.scutbot.cn/Embedded/20_Project_Template.git)
    - 包含 TSML.h
    - 在根目录中添加名为tsml_config.h`的头文件,并在项目包含目录中添加该文件所在的目录
    - 在tsml_config.h`中定义各模块是否参与编译（内容参考[附件1](#附件1)，附件更新可能有延迟）
    - 按照文件目录结构将库文件添加到工程中
    - 编译
    
    

### How to Develop 维护方法

- __Git仓库管理__

  - 添加子模块的远程仓库

  ```bash
  $ git submodule add https://git.scutbot.cn/13726346614/TSML.git
  ```

  - clone已有的工程时拉取TSML的代码

  ```bash
  $ git submodule init
  $ git submodule update
  ```

  - 更新 TSML 库

  ```bash
  $ cd TSML/
  $ git pull origin master
  ```

  - 推送更改到 TSML 库

  ```bash
  $ cd TSML/
  $ git push origin dev
  ```



### 附件1

```c
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

#define USE_TSML_LAB_UM						0
#define USE_TSML_ANO_UM                     0

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) TuTu Studio **************************/
```

