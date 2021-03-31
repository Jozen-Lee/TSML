/**
  ******************************************************************************
  * @file    Middlewares.h
  * @brief   Header to include all Middlewares.
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
/** @addtogroup Middlewares
  * @{
  */
#include <tsml_config.h>
/* Algorithms header begin */
#if USE_TSML_DMP
#include "Algorithm/DMP/dmp_cal.h"
#endif
#if USE_TSML_MPL
#include "Algorithm/MPL/mpl_cal.h"
#endif
#if USE_TSML_FILTERS
#include "Algorithm/Filters/tutu_filters.h"
#endif
#if USE_TSML_KALMAN
#include "Algorithm/Kalman/kalman.h"
#endif
#if USE_TSML_INS
#include "Algorithm/INS/ins.h"
#endif
#if USE_TSML_INTERGRAL
#include "Algorithm/Intergral/integral_algorithm.h"
#endif
/* Algorithms header end */

/* UpperMonitor header begin */
#if USE_TSML_LAB_UM
#include "UpperMonitor/SCUT_LAB/UpperMonitor.h"
#endif
#if USE_TSML_ANO_UM
#include "UpperMonitor/ANO/Ano_UpperMonitor.h"
#endif
/* UpperMonitor header end */

/**
  * @}
  */
/************************ COPYRIGHT(C) TuTu Studio **************************/
