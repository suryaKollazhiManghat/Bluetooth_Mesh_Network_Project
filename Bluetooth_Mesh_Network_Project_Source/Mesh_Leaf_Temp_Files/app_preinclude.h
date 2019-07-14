/*! *********************************************************************************
 * \defgroup app
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * \file app_preinclude.h
 * This file is the app configuration file which is pre included.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _APP_PREINCLUDE_H_
#define _APP_PREINCLUDE_H_

/*! *********************************************************************************
 * 	Board Configuration
 ********************************************************************************** */
 /* Defines the number of available keys for the keyboard module */
#define gKBD_KeysCount_c        2

/* Specifies the number of physical LEDs on the target board */
#define gLEDsOnTargetBoardCnt_c 4

/* Define Clock Configuration */
#define CLOCK_INIT_CONFIG 		CLOCK_RUN_48_24

/*! *********************************************************************************
 * 	Framework Configuration
 ********************************************************************************** */
/* Defines Num of Serial Manager interfaces */
#define gSerialManagerMaxInterfaces_c   1

/* Defines Size for Timer Task*/
#define gTmrTaskStackSize_c  784

/* Defines pools by block size and number of blocks. Must be aligned to 4 bytes.*/
#define PoolsDetails_c \
         _block_size_  32  _number_of_blocks_    6 _eol_  \
         _block_size_  64  _number_of_blocks_    3 _eol_  \
         _block_size_ 128  _number_of_blocks_    4 _eol_  \
         _block_size_ 512  _number_of_blocks_    4 _eol_

/* Defines number of timers needed by the application */
#define gTmrApplicationTimers_c         7

/* Defines number of timers needed by the protocol stack */
#define gTmrStackTimers_c               5

/* Set this define TRUE if the PIT frequency is an integer number of MHZ */
#define gTMR_PIT_FreqMultipleOfMHZ_d    0

/* Enables / Disables the precision timers platform component */
#define gTimestamp_Enabled_d            0

/* Enable/Disable Low Power Timer */
#define gTMR_EnableLowPowerTimers       1

/* Enable/Disable PowerDown functionality in PwrLib */
#define cPWR_UsePowerDownMode           0

/* Enable/Disable BLE Link Layer DSM */
#define cPWR_BLE_LL_Enable              1

/* Default Deep Sleep Mode*/
#define cPWR_DeepSleepMode              3

/* Disable all pins when entering Low Power */           
#define APP_DISABLE_PINS_IN_LOW_POWER   1
           
/* Default deep sleep duration in ms */ 
#define cPWR_DeepSleepDurationMs        30000

/* Number of slots(625us) before the wake up instant before which the hardware needs to exit from deep sleep mode. */           
#define cPWR_BLE_LL_OffsetToWakeupInstant 3          

/* Enables / Disables the DCDC platform component */
#define gDCDC_Enabled_d                 1          

/* Default DCDC Mode used by the application */           
#define APP_DCDC_MODE                   gDCDC_Mode_Buck_c

/* Default DCDC Battery Level Monitor interval */
#define APP_DCDC_VBAT_MONITOR_INTERVAL  600000
             
/*! *********************************************************************************
 * 	RTOS Configuration
 ********************************************************************************** */
/* Defines the RTOS used */
#define FSL_RTOS_FREE_RTOS

/* Defines the usage of the FreeRTOS IDLE task hook to enter low power */           
#define configUSE_IDLE_HOOK     1           

/* Defines main task stack size */
#define gMainThreadStackSize_c  1024
           
/* Defines total heap size used by the OS */
#define gTotalHeapSize_c        8000
           
/*! *********************************************************************************
 * 	BLE Stack Configuration
 ********************************************************************************** */




/* Time between the beginning of two consecutive advertising PDU's */
#define mcAdvertisingPacketInterval_c     0x02    /* 1.25 msec */

/* Offset to the first instant register. */
#define mcOffsetToFirstInstant_c        0x00      /* 625usec */           
                      
#ifndef BD_ADDR_ID
#define BD_ADDR_ID      112
#endif
     
#define BD_ADDR             0x08,BD_ADDR_ID,BD_ADDR_ID,0x9F,0x04,0x00


#endif /* _APP_PREINCLUDE_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */
