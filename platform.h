/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       platform.h
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *  Last modified by Marc0 Xu on Jan 06 2018 
 */
 
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#ifndef __GNUC__
#define inline
#endif

/*!
 * Platform definition
 */

#define SX12xxEiger                                 1


/*!
 * Platform choice. Please uncoment the PLATFORM define and choose your platform
 * or add/change the PLATFORM definition on the compiler Defines option
 */
#define PLATFORM                                    SX12xxEiger

#if( PLATFORM == SX12xxEiger )
/*!
 * Radio choice. Please uncomment the wanted radio and comment the others
 * or add/change wanted radio definition on the compiler Defines option
 */
//#define USE_SX1232_RADIO
//#define USE_SX1272_RADIO
#define USE_SX1278_RADIO
//#define USE_SX1243_RADIO
#endif
/*!
 * Module choice. There are three existing module with the SX1278.
 * Please set the connected module to the value 1 and set the others to 0
 */
#ifdef USE_SX1278_RADIO
#define MODULE_SX1278RISYM                         1

    #include "gpio.h"
    #include "spi.h"
    #include "CMSIS_OS.h"
    #include <stdbool.h>
    #include <stdio.h>
    #define USE_UART                               0
    #define   SX1278_DEBUG                          0 
    #define bool _Bool

#endif

#endif // __PLATFORM_H__
