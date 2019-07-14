/*! *********************************************************************************
* \addtogroup Temperature Sensor
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
* \file app.c
* This file is the source file for the Temperature Sensor application
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
/* Framework / Drivers */
#include "RNG_Interface.h"
#include "Keyboard.h"
#include "LED.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "Panic.h"
#include "app.h"
#include "board.h"

#include "mesh_interface.h"

#if gAppLightBulb_d
#include "mesh_light_server.h"
#elif gAppLightSwitch_d
#include "mesh_light_client.h"
#endif

#if gAppTempSensor_d
#include "mesh_temperature_server.h"
#endif

#include <stdio.h>
#include <stdarg.h>
#include "SerialManager.h"
#include "MemManager.h"

#include "fsl_i2c.h"
#include "pin_mux.h"

#include "fsl_common.h"
#include "fsl_port.h"




/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define ADDRESS 9000
#define mAppMaxResponseDelay_ms    500

#define SHELL_CB_SIZE                 128
#define SHELL_MAX_COMMANDS            20

#define CUSTOM_CMD_SOURCE				0
#define CUSTOM_CMD_DEST					1
#define CUSTOM_CMD_FUNC					2
#define CUSTOM_CMD_POLL_ITVL_0			3
#define CUSTOM_CMD_POLL_ITVL_1			4
#define CUSTOM_CMD_POLL_ITVL_2			5
#define CUSTOM_CMD_POLL_ITVL_3			6
#define CUSTOM_CMD_POWER_CTRL			7
#define CUSTOM_CMD_VAL_ID				8
#define CUSTOM_CMD_VAL_0				9
#define CUSTOM_CMD_VAL_1				10
#define CUSTOM_CMD_VAL_2				11
#define CUSTOM_CMD_VAL_3				12

#define CUSTOM_CMD_TEMP_ID				1
#define CUSTOM_CMD_LIGHT_ID				2

#define CUSTOM_CMD_SYS_AWAKE			1
#define CUSTOM_CMD_SYS_SLEEP			2

#define CUSTOM_CMD_START_DATA			1
#define CUSTOM_CMD_STOP_DATA			2

#define LIGHT_I2C_ADDR					(uint8_t)(0x88)

#define BOARD_ACCEL_I2C_BASEADDR I2C1

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
#if gAppLightBulb_d
static bool_t mLightState = FALSE;
static tmrTimerID_t mLightReportTimerId;
static uint32_t     mLightReportInterval_sec;
static tmrTimerID_t mLightRandomWaitTimerId;
#endif

#if gAppTempSensor_d
static tmrTimerID_t mTemperatureReportTimerId;
static uint32_t     mTemperatureReportInterval_sec;
#endif

static uint8_t        interfaceId;
bool_t IsTimerStarted = FALSE;

static tmrTimerID_t mCustomReportTimerId;
static uint32_t     mCustomReportInterval_sec;

uint32_t Temp_Read_Val = 0;
uint32_t Light_Read_Val = 0;

i2c_master_handle_t g_m_handle;
volatile bool completionFlag = false;
volatile bool nakFlag = false;

osaEventId_t          mAppEvent;

static uint32_t UartIpBuffCount = 0;
static uint8_t UartIpData[10];

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static void AppConfig();

static meshResult_t MeshGenericCallback
(
    meshGenericEvent_t* pEvent
);

#if gAppLightBulb_d
static void EnableLightReports(bool_t enable);
static void LightReportTimerCallback(void* param);
static void RandomLightWait(uint32_t maxMs);
static meshResult_t MeshLightServerCallback
(
    meshLightServerEvent_t* pEvent
);
static void UpdateLightUI(bool_t lightOn);
#endif

#if gAppTempSensor_d
static void EnableTemperatureReports(bool_t enable);
static void TemperatureReportTimerCallback(void* param);
static meshResult_t MeshTemperatureServerCallback
(
    meshTemperatureServerEvent_t* pEvent
);
#endif

static void CustomReportTimerCallback(void* param);

uint8_t gState;

uint16_t debug_printf(char * format,...)
{
    va_list ap;
    uint16_t n;
    char *pStr = (char*)MEM_BufferAlloc(SHELL_CB_SIZE);

    if(!pStr)
        return 0;

    va_start(ap, format);
    n = vsnprintf(pStr, SHELL_CB_SIZE, format, ap);
    //va_end(ap); /* follow MISRA... */
    Serial_SyncWrite(interfaceId, (uint8_t*)pStr, n);
    MEM_BufferFree(pStr);
    return n;
}

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/


static void UartRxCallBack(void *pData)
{
    uint8_t pressedKey;
    uint16_t count;
    uint16_t byte_count;

	Serial_RxBufferByteCount(interfaceId, &count);

    Serial_Read(interfaceId, &UartIpData[UartIpBuffCount], count, &byte_count);

   // for(int i=UartIpBuffCount;i<count+UartIpBuffCount;i++)
    	//debug_printf("%d %x %c\r\n",UartIpData[i],UartIpData[i],UartIpData[i]);

    UartIpBuffCount+=count;

    if(UartIpData[UartIpBuffCount-1] == 0x0A)
    {
    	uint32_t sum=0;
    	uint32_t mult=10;
    	sum+=(UartIpData[UartIpBuffCount-2]-48);
    	//debug_printf("0 sum = %d\r\n",sum);
    	for(int i=UartIpBuffCount-3;i>=0;i--)
    	{
    		//for(int j=UartIpBuffCount-3;j>i;mult*=10,j--);
    		sum+=((UartIpData[i]-48) * mult);
    		mult*=10;
    		//debug_printf("%d %d sum = %d mult = %d\r\n",i,(UartIpData[i]-48),sum,mult);
    	}
    	Light_Read_Val = sum;
    	UartIpBuffCount = 0;
    }

    //debug_printf("UartRxCallBack ++ Temp = %d count  = %d Datacount %d\r\n",Temp_Read_Val,count,UartIpBuffCount);
    debug_printf("UartRxCallBack ++ Light Read val = %d\r\n",Light_Read_Val);

}

void BleApp_Init(void)
{    
    BOARD_InitAdc();
    
    RNG_Init();
    
    SerialManager_Init();

    Serial_InitInterface(&interfaceId, APP_SERIAL_INTERFACE_TYPE, APP_SERIAL_INTERFACE_INSTANCE);
    Serial_SetBaudRate(interfaceId, gUARTBaudRate115200_c);
    Serial_SetRxCallBack(interfaceId, UartRxCallBack, NULL);

    uint8_t rngSeed[20] = { BD_ADDR_ID };
    RNG_SetPseudoRandomNoSeed(rngSeed);
    mCustomReportInterval_sec = 5;

    //I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);

    debug_printf("BLE Light Switch ID: %d Multicast Addr: %d - 0x%x\n\r", BD_ADDR_ID,ADDRESS,(uint32_t)(ADDRESS));

}

void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:    
        {
            AppConfig();
        }
        break;        

        case gInternalError_c:
        {
            Led2On();
            panic(0,0,0,0);
        }
        break;

        default: 
            break;
    }
}

void BleApp_HandleKeys(key_event_t events)
{
    switch (events)
    {       
        case gKBD_EventPressPB1_c:
            /* Fallthrough */
        //case gKBD_EventPressPB2_c:
        {
        	debug_printf("Switch Pressed: 4\n\r");

/*
#if gAppLightBulb_d
            mLightState = !mLightState;
            UpdateLightUI(mLightState);
            MeshLightServer_PublishState(mLightState);
#elif gAppLightSwitch_d
            MeshLightClient_PublishToggleLight();
#endif
            meshAddress_t destination = GetMeshAddressFromId(11);
            meshCustomData_t CustomData;
            CustomData.aData[0] = 0xFD;
            CustomData.aData[1] = 0xFC;
            CustomData.dataLength = 2;
            Mesh_SendCustomData(destination,&CustomData);

*/

		    if (!IsTimerStarted)
		    {
		        TMR_StartIntervalTimer
		        (
		            mCustomReportTimerId,
		            1000 * mCustomReportInterval_sec, // 1000 * reqd seconds
					CustomReportTimerCallback,
		            NULL
		        );
		        IsTimerStarted = TRUE;
		        debug_printf("Start report timer interval: %d\n\r",mCustomReportInterval_sec);
		    }

        }
        break;

        //case gKBD_EventPressPB3_c:
            /* Fallthrough */
        //case gKBD_EventPressPB4_c:
        case gKBD_EventPressPB2_c:
        {
        	debug_printf("Switch Pressed: 3\n\r");
		    if (IsTimerStarted)
		    {
		    	TMR_StopTimer(mCustomReportTimerId);
		    	IsTimerStarted = FALSE;
		    	debug_printf("Stop report timer\n\r");
		    }
#if 0
#if gAppTempSensor_d
            int16_t tempCelsius = BOARD_GetTemperature();
            MeshTemperatureServer_PublishTemperature(tempCelsius);
#endif            
#endif
		}
        break;
        
        default:
            break;
    }
}

/*
#define CUSTOM_CMD_SOURCE				0
#define CUSTOM_CMD_DEST					1
#define CUSTOM_CMD_POLL_ITVL_0			2
#define CUSTOM_CMD_POLL_ITVL_1			3
#define CUSTOM_CMD_POLL_ITVL_2			4
#define CUSTOM_CMD_POLL_ITVL_3			5
#define CUSTOM_CMD_POWER_CTRL			6
#define CUSTOM_CMD_VAL_ID				7
#define CUSTOM_CMD_VAL_0				8
#define CUSTOM_CMD_VAL_1				9
#define CUSTOM_CMD_VAL_2				10
#define CUSTOM_CMD_VAL_3				11

#define CUSTOM_CMD_TEMP_ID				1
#define CUSTOM_CMD_LIGHT_ID				2
#define CUSTOM_CMD_SYS_AWAKE			1
#define CUSTOM_CMD_SYS_SLEEP			2
 */

static void CustomReportTimerCallback(void* param)
{

    meshAddress_t destination = GetMeshAddressFromId(22);
    meshCustomData_t CustomData;
    CustomData.aData[CUSTOM_CMD_SOURCE] = BD_ADDR_ID;
    CustomData.aData[CUSTOM_CMD_DEST] = GetIdFromMeshAddress(destination);

    CustomData.aData[CUSTOM_CMD_POLL_ITVL_0] = (uint8_t)(mCustomReportInterval_sec & 0xFF);
    CustomData.aData[CUSTOM_CMD_POLL_ITVL_1] = (uint8_t)((mCustomReportInterval_sec >> 8) & 0xFF);
    CustomData.aData[CUSTOM_CMD_POLL_ITVL_2] = (uint8_t)((mCustomReportInterval_sec >> 16) & 0xFF);
    CustomData.aData[CUSTOM_CMD_POLL_ITVL_3] = (uint8_t)((mCustomReportInterval_sec >> 24) & 0xFF);

    CustomData.aData[CUSTOM_CMD_POWER_CTRL] = CUSTOM_CMD_SYS_AWAKE;
    /*
    CustomData.aData[CUSTOM_CMD_VAL_ID] = CUSTOM_CMD_TEMP_ID;
    CustomData.aData[CUSTOM_CMD_VAL_0] = (uint8_t)(Temp_Read_Val & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_1] = (uint8_t)((Temp_Read_Val >> 8) & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_2] = (uint8_t)((Temp_Read_Val >> 16) & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_3] = (uint8_t)((Temp_Read_Val >> 24) & 0xFF);
*/

    CustomData.aData[CUSTOM_CMD_VAL_ID] = CUSTOM_CMD_LIGHT_ID;
    CustomData.aData[CUSTOM_CMD_VAL_0] = (uint8_t)(Light_Read_Val & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_1] = (uint8_t)((Light_Read_Val >> 8) & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_2] = (uint8_t)((Light_Read_Val >> 16) & 0xFF);
    CustomData.aData[CUSTOM_CMD_VAL_3] = (uint8_t)((Light_Read_Val >> 24) & 0xFF);

    CustomData.dataLength = 13;
    Mesh_SendCustomData(destination,&CustomData);
    debug_printf("Custom data Sent to: %d\n\r",GetIdFromMeshAddress(destination));
	debug_printf("Data is: ");
    for(int i = 0; i<CustomData.dataLength && i<gMeshMaxAppCustomDataSize_c;
    		i++)
    {
    	debug_printf("0x%x ", CustomData.aData[i]);
    }
    debug_printf("\r\n");
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static void AppConfig()
{      
#if gAppLightBulb_d    
    mLightReportTimerId = TMR_AllocateTimer();
    mLightRandomWaitTimerId = TMR_AllocateTimer();
    MeshLightServer_RegisterCallback(MeshLightServerCallback);
#endif
    
#if gAppTempSensor_d
    mTemperatureReportTimerId = TMR_AllocateTimer();
    MeshTemperatureServer_RegisterCallback(MeshTemperatureServerCallback);
#endif
    
    MeshNode_Init(MeshGenericCallback);
}

static meshResult_t MeshGenericCallback
(
    meshGenericEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshInitComplete_c:
            {
                if (!pEvent->eventData.initComplete.deviceIsCommissioned)
                {                                       
                    /* This will trigger this callback again, but "deviceIsCommissioned" will be TRUE. */
                    MeshNode_Commission(&gRawCommData);
                    debug_printf("Device Commissioned for first time: %d\n\r", pEvent->eventData.initComplete.deviceIsCommissioned);
                }
                else
                {
                	StopLed1Flashing();
                	StopLed2Flashing();
                	StopLed3Flashing();
                	StopLed4Flashing();
                	Led3On();
#if gAppLightBulb_d
                    Mesh_SetRelayState(TRUE);
#else
                    Mesh_SetRelayState(FALSE);
                    Mesh_SetPublishAddress(gMeshProfileLighting_c,ADDRESS);
                    debug_printf("Device Commissioned: %d\n\r", pEvent->eventData.initComplete.deviceIsCommissioned);
#endif
                }
            }               
            break;

/*
#define CUSTOM_CMD_SOURCE				0
#define CUSTOM_CMD_DEST					1
#define CUSTOM_CMD_POLL_ITVL_0			2
#define CUSTOM_CMD_POLL_ITVL_1			3
#define CUSTOM_CMD_POLL_ITVL_2			4
#define CUSTOM_CMD_POLL_ITVL_3			5
#define CUSTOM_CMD_POWER_CTRL			6
#define CUSTOM_CMD_VAL_ID				7
#define CUSTOM_CMD_VAL_0				8
#define CUSTOM_CMD_VAL_1				9
#define CUSTOM_CMD_VAL_2				10
#define CUSTOM_CMD_VAL_3				11

#define CUSTOM_CMD_TEMP_ID				1
#define CUSTOM_CMD_LIGHT_ID				2
#define CUSTOM_CMD_SYS_AWAKE			1
#define CUSTOM_CMD_SYS_SLEEP			2
 */
        case gMeshCustomDataReceived_c:
			{
				debug_printf("\r\n -> Received Custom Data: Source: %d\r\n",
						GetIdFromMeshAddress(pEvent->eventData.customDataReceived.source));
				debug_printf("\r\nData is: ");
                for(int i = 0; i<pEvent->eventData.customDataReceived.data.dataLength && i<gMeshMaxAppCustomDataSize_c;
                		i++)
                {
                	debug_printf("0x%x ",pEvent->eventData.customDataReceived.data.aData[i]);
                }
                debug_printf("\r\n");

				if((pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_SOURCE] == 22) // Bulb is source
						&& (pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_DEST] == BD_ADDR_ID)) // Light is dest
				{
	                mCustomReportInterval_sec = (uint32_t)(
	                		(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_3]<<24) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_2]<<16) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_1]<<8) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_0]));

					debug_printf("\r\nSource: %d Dest: %d Poll Time: %d\r\n",
							pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_SOURCE],
							pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_DEST],
							mCustomReportInterval_sec);

				    if (IsTimerStarted)
				    {
				    	TMR_StopTimer(mCustomReportTimerId);

				        TMR_StartIntervalTimer
				        (
				            mCustomReportTimerId,
				            1000 * mCustomReportInterval_sec, // 1000 * reqd seconds
							CustomReportTimerCallback,
				            NULL
				        );
				        IsTimerStarted = TRUE;
				    }
				}


			}
			break;

        default:
            {
                /* Ignore */
            }
            break;

    }
    return gMeshSuccess_c;
}

/*
*
* Mesh Profile Callbacks
*
*/

#if gAppLightBulb_d
static meshResult_t MeshLightServerCallback
(
    meshLightServerEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshLightToggleCommand_c:
            {
                mLightState = !mLightState;
                UpdateLightUI(mLightState);
                RandomLightWait(mAppMaxResponseDelay_ms);
            }
            break;
            
        case gMeshLightGetCommand_c:
            {
                MeshLightServer_SendState(pEvent->eventData.getCommand.source, mLightState);
            }
            break;
            
        case gMeshLightSetCommand_c:
            {
                mLightState = pEvent->eventData.setCommand.lightState;
                UpdateLightUI(mLightState);
                RandomLightWait(mAppMaxResponseDelay_ms);
            }
            break;
            
        case gMeshLightGetReportCommand_c:
            {
                MeshLightServer_SendPeriodicReportState
                (
                    pEvent->eventData.getReportCommand.source,
                    TMR_IsTimerActive(mLightReportTimerId),
                    mLightReportInterval_sec
                );
            }
            break;
            
        case gMeshLightSetReportCommand_c:
            {
                mLightReportInterval_sec = pEvent->eventData.setReportCommand.intervalSeconds;
                EnableLightReports(pEvent->eventData.setReportCommand.enable);
                MeshLightServer_SendPeriodicReportState
                (
                    pEvent->eventData.getReportCommand.source,
                    pEvent->eventData.setReportCommand.enable,
                    mLightReportInterval_sec
                );
            }
            break;
            
        default:
            {
                /* Ignore */
            }
            break;
    }
    return gMeshSuccess_c;
}
#endif /* gAppLightBulb_d */

#if gAppTempSensor_d
static meshResult_t MeshTemperatureServerCallback
(
    meshTemperatureServerEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshTemperatureGetCommand_c:
            {
                int16_t tempCelsius = BOARD_GetTemperature();
            	debug_printf("MeshTemperatureServerCallback: Temp GET Event Source: %d, Temp = %d\n\r"
            			,pEvent->eventData.getCommand.source,tempCelsius);
                MeshTemperatureServer_SendTemperature(pEvent->eventData.getCommand.source, tempCelsius);
            } 
            break;
            
        case gMeshTemperatureSetReportCommand_c:
            {
            	debug_printf("MeshTemperatureServerCallback: SET Report Event Source: %d , Enable: %d  Report Intvl: %d\n\r"
            			,pEvent->eventData.setReportCommand.source,pEvent->eventData.setReportCommand.enable,
							pEvent->eventData.setReportCommand.intervalSeconds);
                mTemperatureReportInterval_sec = pEvent->eventData.setReportCommand.intervalSeconds;
                EnableTemperatureReports(pEvent->eventData.setReportCommand.enable);
                MeshTemperatureServer_SendPeriodicReportState
                (
                    pEvent->eventData.getReportCommand.source,
                    pEvent->eventData.setReportCommand.enable,
                    mTemperatureReportInterval_sec
                );
            } 
            break;
            
        case gMeshTemperatureGetReportCommand_c:
            {
            	debug_printf("MeshTemperatureServerCallback: GET Report Event Source: %d , Report Intvl: %d\n\r"
            			,pEvent->eventData.getReportCommand.source,mTemperatureReportInterval_sec);
                MeshTemperatureServer_SendPeriodicReportState
                (
                    pEvent->eventData.getReportCommand.source,
                    TMR_IsTimerActive(mTemperatureReportTimerId),
                    mTemperatureReportInterval_sec
                );
            } 
            break;
            
        default:
            {
                /* Ignore */
            }
            break;
    }
    return gMeshSuccess_c;
}
#endif /* gAppTempSensor_d */

/*
*
* Report handling functions
*
*/

#if gAppLightBulb_d
static void EnableLightReports(bool_t enable)
{
    if (enable)
    {
        TMR_StartIntervalTimer
        (
            mLightReportTimerId,
            1000 * mLightReportInterval_sec,
            LightReportTimerCallback,
            NULL
        );
    }
    else
    {
        TMR_StopTimer(mLightReportTimerId);
    }
}

static void LightReportTimerCallback(void* param)
{
    MeshLightServer_PublishState(mLightState);
}

static void RandomLightWait(uint32_t maxMs)
{
    uint16_t random;
    RNG_GetPseudoRandomNo((uint8_t*) &random, 2, NULL);
    
    uint32_t interval_ms = (uint32_t) random * maxMs / 0x0000ffff;
    
    TMR_StartSingleShotTimer(mLightRandomWaitTimerId, interval_ms, LightReportTimerCallback, NULL);
}

static void UpdateLightUI(bool_t lightOn)
{
    /* Insert code here to update UI to indicate Light State ON/OFF. */
}
#endif /* gAppLightBulb_d */

#if gAppTempSensor_d
static void EnableTemperatureReports(bool_t enable)
{
    if (enable)
    {
        TMR_StartIntervalTimer
        (
            mTemperatureReportTimerId,
            1000 * mTemperatureReportInterval_sec,
            TemperatureReportTimerCallback,
            NULL
        );
    }
    else
    {
        TMR_StopTimer(mTemperatureReportTimerId);
    }
}
static void TemperatureReportTimerCallback(void* param)
{
    int16_t tempCelsius = BOARD_GetTemperature();
    MeshTemperatureServer_PublishTemperature(tempCelsius);
}

#endif /* gAppTempSensor_d */

/*! *********************************************************************************
* @}
********************************************************************************** */
