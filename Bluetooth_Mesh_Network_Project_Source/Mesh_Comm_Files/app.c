/*! *********************************************************************************
* \addtogroup Temperature Collector
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
* \file app.c
* This file is the source file for the Temperature Collector application
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
#include "shell.h"
#include "Panic.h"
#include "PWR_Interface.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"

/* Profile / Services */
#include "mesh_interface.h"
#include "mesh_config_client.h"
#include "mesh_light_client.h"
#include "mesh_temperature_client.h"

#include "string.h"
#include "stdlib.h"
#include "ApplMain.h"
#include "app.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

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

#define UART_TX_IND_GPIO GPIOA
#define UART_TX_IND_GPIO_PIN 18U

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
/* Timers */
static tmrTimerID_t mAppTimerId;

static bool_t mLog = TRUE;

static bool_t 	mDataTxStatus = FALSE;
static uint32_t mDataPollRate = 10;
static uint32_t mTempSenPollRate = 5;
static uint32_t mLightSenPollRate = 5;
static bool_t 	mTempSenPowSt = TRUE;
static bool_t 	mLightSenPowSt = TRUE;

uint32_t mTempLatVal = 0;
uint32_t mLightLatVal = 0;

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

static meshResult_t MeshConfigClientCallback
(
    meshConfigClientEvent_t* pEvent
);

static meshResult_t MeshLightClientCallback
(
    meshLightClientEvent_t* pEvent
);

static meshResult_t MeshTemperatureClientCallback
(
    meshTemperatureClientEvent_t* pEvent
);

int8_t ShellMesh_Publish(uint8_t argc, char * argv[]);
int8_t ShellMesh_Subscribe(uint8_t argc, char * argv[]);
int8_t ShellMesh_Relay(uint8_t argc, char * argv[]);
int8_t ShellMesh_Ttl(uint8_t argc, char * argv[]);
int8_t ShellMesh_Light(uint8_t argc, char * argv[]);
int8_t ShellMesh_Demo(uint8_t argc, char * argv[]);
int8_t ShellMesh_Log(uint8_t argc, char * argv[]);
int8_t ShellMesh_DataTransfer(uint8_t argc, char * argv[]);
int8_t ShellMesh_DataPollRate(uint8_t argc, char * argv[]);
int8_t ShellMesh_SenPollRate(uint8_t argc, char * argv[]);
int8_t ShellMesh_SenPower(uint8_t argc, char * argv[]);

void delay(uint32_t count);

const cmd_tbl_t mMeshPublishCmd =
{
    .name = "pub",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_Publish,
    .help = "Usage:\r\n"
        ">>> pub get ID\r\n"
        ">>> pub set ID address\r\n",
    .usage = "Get/set publishing addresses for a Switch ID."
};
const cmd_tbl_t mMeshSubscribeCmd =
{
    .name = "sub",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_Subscribe,
    .help = "Usage:\r\n"
        ">>> sub get ID\r\n"
        ">>> sub add ID address\r\n"
        ">>> sub rem ID address\r\n",
    .usage = "Get/add/remove subscription addresses for a Light ID."
};
const cmd_tbl_t mMeshRelayCmd =
{
    .name = "relay",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_Relay,
    .help = "Usage:\r\n"
        ">>> relay get ID\r\n"
        ">>> relay set ID value\r\n",
    .usage = "Get/set Relay state for a node ID."
};
const cmd_tbl_t mMeshTtlCmd =
{
    .name = "ttl",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_Ttl,
    .help = "Usage:\r\n"
        ">>> ttl get ID\r\n"
        ">>> ttl set ID value\r\n",
    .usage = "Get/set TTL value for a node ID."
};
const cmd_tbl_t mMeshLightCmd =
{
    .name = "light",
    .maxargs = 3,
    .repeatable = 1,
    .cmd = ShellMesh_Light,
    .help = "Usage:\r\n"
        ">>> light on\r\n"
        ">>> light off\r\n"
        ">>> light toggle\r\n"
        ">>> light on id\r\n"
        ">>> light off id\r\n"
        ">>> light toggle id\r\n",
    .usage = "Send light commands."
};

const cmd_tbl_t mMeshLogCmd =
{
    .name = "l",
    .maxargs = 1,
    .repeatable = 1,
    .cmd = ShellMesh_Log,
    .help = "Usage:\r\n"
        ">>> l\r\n",
    .usage = "Toggles the logging of incoming packets ON and OFF."
};

const cmd_tbl_t mMeshCustomDataTxCmd =
{
    .name = "datatx",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_DataTransfer,
    .help = "Usage:\r\n"
        ">>> datatx get\r\n"
        ">>> datatx set start\r\n"
		">>> datatx set stop\r\n",
    .usage = "Start/Stop Data transfer to cloud."
};

const cmd_tbl_t mMeshCustomDataPollRateCmd =
{
    .name = "datapollrt",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_DataPollRate,
    .help = "Usage:\r\n"
    	">>> datapollrt get\r\n"
        ">>> datapollrt set time_val_in_seconds\r\n"
    	">>> datapollrt set 5\r\n",
    .usage = "Set Data poll rate time in seconds."
};

const cmd_tbl_t mMeshCustomSenPollRateCmd =
{
    .name = "senpollrt",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_SenPollRate,
    .help = "Usage:\r\n"
        ">>> senpollrt get sen_type\r\n"
    	">>> senpollrt set sen_type time_val_in_seconds\r\n"
    	">>> senpollrt get temp\r\n"
    	">>> senpollrt set temp time_val_in_seconds\r\n"
       	">>> senpollrt get light\r\n"
       	">>> senpollrt set light time_val_in_seconds\r\n",
    .usage = "Set Sensor data poll rate time in seconds."
};

const cmd_tbl_t mMeshCustomSenPower =
{
    .name = "senpow",
    .maxargs = 4,
    .repeatable = 1,
    .cmd = ShellMesh_SenPower,
    .help = "Usage:\r\n"
    	">>> senpow get sen_type\r\n"
    	">>> senpow set sen_type sleep/wake\r\n"
    	">>> senpow get temp\r\n"
    	">>> senpow get light\r\n"
        ">>> senpow set temp wake\r\n"
    	">>> senpow set temp sleep\r\n"
        ">>> senpow set light wake\r\n"
        ">>> senpow set light sleep\r\n",
    .usage = "Set Sensor Power status."
};

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{      
        /* UI */
    shell_init("BLE MESH >>> ");
    shell_register_function((cmd_tbl_t *)&mMeshPublishCmd);
    shell_register_function((cmd_tbl_t *)&mMeshSubscribeCmd);
    shell_register_function((cmd_tbl_t *)&mMeshRelayCmd);
    shell_register_function((cmd_tbl_t *)&mMeshTtlCmd);
    shell_register_function((cmd_tbl_t *)&mMeshLightCmd);
    shell_register_function((cmd_tbl_t *)&mMeshLogCmd);
    shell_register_function((cmd_tbl_t *)&mMeshCustomDataTxCmd);
    shell_register_function((cmd_tbl_t *)&mMeshCustomDataPollRateCmd);
    shell_register_function((cmd_tbl_t *)&mMeshCustomSenPollRateCmd);
    shell_register_function((cmd_tbl_t *)&mMeshCustomSenPower);
#if 0
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;

    PORT_SetPinConfig(UART_TX_IND_GPIO, UART_TX_IND_GPIO_PIN, &i2c_pin_config);

	GPIO_PinInit(UART_TX_IND_GPIO, UART_TX_IND_GPIO_PIN, &pin_config);
	uint32_t test = 100;

	GPIO_WritePinOutput(UART_TX_IND_GPIO, UART_TX_IND_GPIO_PIN, 0U);
#endif

}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{ 
    switch (events)
    {
        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
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

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void AppConfig()
{
    mAppTimerId = TMR_AllocateTimer();
	
    MeshConfigClient_RegisterCallback(MeshConfigClientCallback);
    MeshLightClient_RegisterCallback(MeshLightClientCallback);
    MeshTemperatureClient_RegisterCallback(MeshTemperatureClientCallback);
    MeshCommissioner_Init(MeshGenericCallback);
    
}

void delay(uint32_t count)
{
    uint32_t i = 0;
    for (i = 0; i < count; i++)
    {
        __NOP();
    }
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
                shell_write("\r\n\r\nMesh Commissioner initialization complete!\r\n");
                shell_refresh();

            	StopLed1Flashing();
            	StopLed2Flashing();
            	StopLed3Flashing();
            	StopLed4Flashing();
            	Led4On();
            }               
            break;
            
        case gMeshCustomDataReceived_c:
			{
				/*
                shell_printf("\r\n -> Received Custom Data: Source: %d\r\n",
                		pEvent->eventData.customDataReceived.source);
                shell_printf("Data is: ");
                for(int i = 0; i<pEvent->eventData.customDataReceived.data.dataLength && i<gMeshMaxAppCustomDataSize_c;
                		i++)
                {
                	shell_printf("0x%x ",pEvent->eventData.customDataReceived.data.aData[i]);
                }
                shell_printf("\r\n");
				*/
				if(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_SOURCE] == 22) // Comm is source
				{

					 mDataPollRate = (uint32_t)(
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_3]<<24) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_2]<<16) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_1]<<8) |
							(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_POLL_ITVL_0]));

					if(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_ID] == CUSTOM_CMD_TEMP_ID)
					{
						mTempLatVal = (uint32_t)(
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_3]<<24) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_2]<<16) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_1]<<8) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_0]));

						shell_printf("Received Temp is: %d\r\n",mTempLatVal);

					}
					else if(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_ID] == CUSTOM_CMD_LIGHT_ID)
					{
						mLightLatVal = (uint32_t)(
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_3]<<24) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_2]<<16) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_1]<<8) |
								(pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_0]));

						shell_printf("Received Light is: %d\r\n",mLightLatVal);

					}
					else
					{
						shell_printf("Invalid Val type received: %d",pEvent->eventData.customDataReceived.data.aData[CUSTOM_CMD_VAL_ID] );
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

static meshResult_t MeshConfigClientCallback
(
    meshConfigClientEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshConfigReceivedPublishAddress_c:
            {                
                if (mLog)
                {
                    if (pEvent->eventData.receivedPublishAddress.profileId == gMeshProfileLighting_c)
                    {
                        uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedPublishAddress.source);
                        shell_printf("\r\n -> Received Publish Status: Node %d publishes Light Toggle on 0x%04X. \r\n", 
                                     id, pEvent->eventData.receivedPublishAddress.address);
                        shell_refresh();
                    }
                }
            }
            break;
            
        case gMeshConfigReceivedRelayState_c:
            {
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedRelayState.source);
                    shell_printf("\r\n -> Received Relay Status: Node %d has relay state %d. \r\n", 
                                 id, pEvent->eventData.receivedRelayState.relayEnabled);
                    shell_refresh();
                }
            }
            break;
            
        case gMeshConfigReceivedSubscriptionList_c:
            {                
                if (mLog)
                {
                    if (pEvent->eventData.receivedSubscriptionList.profileId == gMeshProfileLighting_c)
                    {
                        uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedSubscriptionList.source);
                        uint8_t numAddresses = pEvent->eventData.receivedSubscriptionList.listSize;
                        meshAddress_t* aAddresses = pEvent->eventData.receivedSubscriptionList.aAddressList;
                        shell_printf("\r\n -> Received Subscription List: Node %d is subscribed to Light State messages to 0x%04X", id, aAddresses[0]);
                        uint8_t index = 1;                                            
                        while (index < numAddresses)
                        {
                            shell_printf(", 0x%04X", aAddresses[index++]);
                        }
                        shell_printf(". \r\n");
                        shell_refresh();
                    }
                }
            }
            break;
            
        case gMeshConfigReceivedTtl_c:
            {
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedTtl.source);
                    shell_printf("\r\n -> Received TTL Status: Node %d has TTL %d. \r\n", 
                                 id, pEvent->eventData.receivedTtl.ttl);
                    shell_refresh();
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

static meshResult_t MeshLightClientCallback
(
    meshLightClientEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshLightReceivedLightState_c:
            {                
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedLightState.source);
                    if (pEvent->eventData.receivedLightState.lightOn)
                    {
                        shell_printf("\r\n -> Received Light State: Light %d is ON. \r\n", id);
                    }
                    else
                    {
                        shell_printf("\r\n -> Received Light State: Light %d is OFF. \r\n", id);
                    }
                    shell_refresh();
                }
            }
            break;
            
        case gMeshLightReceivedReportState_c:
            {
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedReportState.source);
                    shell_printf("\r\n -> Received Light report from ID: %d \r\nReport status: %d\r\nInterval: %d\r\n",
                                 id,
                                 pEvent->eventData.receivedReportState.reportOn,
								 pEvent->eventData.receivedReportState.intervalSeconds);
                    shell_refresh();
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

static meshResult_t MeshTemperatureClientCallback
(
    meshTemperatureClientEvent_t* pEvent
)
{
    switch (pEvent->eventType)
    {
        case gMeshTemperatureReceivedTemperature_c:
            {                
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedTemperature.source);
                    shell_printf("\r\n -> Received Temp measurement from ID %d: %d degrees Celsius. \r\n",
                                 id,
                                 pEvent->eventData.receivedTemperature.tempCelsius);
                    shell_refresh();
                }
            }
            break;
            
        case gMeshTemperatureReceivedReportState_c:
            {
                if (mLog)
                {
                    uint8_t id = GetIdFromMeshAddress(pEvent->eventData.receivedReportState.source);
                    shell_printf("\r\n -> Received Temp report from ID: %d \r\nReport status: %d\r\nInterval: %d\r\n",
                                 id,
                                 pEvent->eventData.receivedReportState.reportOn,
								 pEvent->eventData.receivedReportState.intervalSeconds);
                    shell_refresh();
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
   
int8_t ShellMesh_Publish(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }
    
    meshResult_t result;
    if (argc == 3)
    {
        if (strcmp(argv[1], "get"))
        {
            return CMD_RET_USAGE;
        }
        int8_t id = atoi(argv[2]);
        meshAddress_t destination = GetMeshAddressFromId(id);
        result = MeshConfigClient_GetPublishAddress(destination, gMeshProfileLighting_c);
        if (result == gMeshSuccess_c)
        {
            shell_printf("< Publish Get command sent to ID %d >", id);
       }
        else
        {
            shell_printf("< Cannot send command - Error code: 0x%04x >", result);
        }
    }
    else /* argc == 4 */
    {
        if (!strcmp(argv[1], "set"))
        {
            int8_t id = atoi(argv[2]);
            int16_t add = atoi(argv[3]);
            meshAddress_t destination = GetMeshAddressFromId(id);
            result = MeshConfigClient_SetPublishAddress(destination, gMeshProfileLighting_c, add);
            if (result == gMeshSuccess_c)
            {
                shell_printf("< Publish Set command sent to ID %d >", id);
            }
            else
            {
                shell_printf("< Cannot send command - Error code: 0x%04x >", result);
            }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_Subscribe(uint8_t argc, char * argv[])
{
        if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }
    
    meshResult_t result;
    if (argc == 3)
    {
        if (strcmp(argv[1], "get"))
        {
            return CMD_RET_USAGE;
        }
        int8_t id = atoi(argv[2]);
        meshAddress_t destination = GetMeshAddressFromId(id);
        result = MeshConfigClient_GetSubscriptionList(destination, gMeshProfileLighting_c);
        if (result == gMeshSuccess_c)
        {
            shell_printf("< Subscription List Get command sent to %d >", id);
        }
        else
        {
            shell_printf("< Cannot send command - Error code: 0x%04x >", result);
        }
    }
    else /* argc == 4 */
    {
        if (!strcmp(argv[1], "add"))
        {
            int8_t id = atoi(argv[2]);
            int16_t add = atoi(argv[3]);
            if (id != 0)
            {
                meshAddress_t destination = GetMeshAddressFromId(id);
                result = MeshConfigClient_Subscribe(destination, gMeshProfileLighting_c, add);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< Subscribe command sent to ID %d >", id);
                }
                else
                {
                    shell_printf("< Cannot send command - Error code: 0x%04x >", result);
                }
            }
            else
            {
                result = Mesh_Subscribe(gMeshProfileLighting_c, add);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< Subscribed local node to 0x%04X >", add);
                }
                else
                {
                    shell_printf("< Could not subscribe local node - Error code: 0x%04x >", result);
                }
            }
            
        }
        else if (!strcmp(argv[1], "rem"))
        {
            int8_t id = atoi(argv[2]);
            int16_t add = atoi(argv[3]);
            if (id != 0)
            {
                meshAddress_t destination = GetMeshAddressFromId(id);
                result = MeshConfigClient_Unsubscribe(destination, gMeshProfileLighting_c, add);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< Unsubscribe command sent to ID %d >", id);
                }
                else
                {
                    shell_printf("< Cannot send command - Error code: 0x%04x >", result);
                }
            }
            else
            {
                result = Mesh_Unsubscribe(gMeshProfileLighting_c, add);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< Unsubscribed local node from 0x%04X >", add);
                }
                else
                {
                    shell_printf("< Could not unsubscribe local node - Error code: 0x%04x >", result);
                }
            }            
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_Relay(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }
    
    meshResult_t result;
    if (argc == 3)
    {
        if (strcmp(argv[1], "get"))
        {
            return CMD_RET_USAGE;
        }
        int8_t id = atoi(argv[2]);
        if (id != 0)
        {
            meshAddress_t destination = GetMeshAddressFromId(id);
            result = MeshConfigClient_GetRelayState(destination);
            if (result == gMeshSuccess_c)
            {
                shell_printf("< Relay Get command sent to %d >", id);
            }
            else
            {
                shell_printf("< Cannot send command - Error code: 0x%04x >", result);
            }
        }
        else
        {
            bool_t relayEnabled;
            Mesh_GetRelayState(&relayEnabled);
            shell_printf("< Commissioner has relay state %d >", relayEnabled);
            result = gMeshSuccess_c;
        }
    }
    else /* argc == 4 */
    {
        if (!strcmp(argv[1], "set"))
        {
            int8_t id = atoi(argv[2]);
            uint8_t state = atoi(argv[3]);
            if (state > 1)
            {
                shell_printf("< Please use Relay state value 1 for ON and 0 for OFF >");
                return CMD_RET_FAILURE;
            }
            if (id != 0)
            {
                meshAddress_t destination = GetMeshAddressFromId(id);
                result = MeshConfigClient_EnableRelay(destination, state);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< Relay Set command sent to ID %d >", id);
                }
                else
                {
                    shell_printf("< Cannot send command - Error code: 0x%04x >", result);
                }
            }
            else
            {
                Mesh_SetRelayState(state);
                shell_printf("< Commissioner's relay state set to %d >", state);
                result = gMeshSuccess_c;
            }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_Ttl(uint8_t argc, char * argv[])
{
        if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }
    
    meshResult_t result;
    if (argc == 3)
    {
        if (strcmp(argv[1], "get"))
        {
            return CMD_RET_USAGE;
        }
        int8_t id = atoi(argv[2]);
        if (id != 0)
        {
            meshAddress_t destination = GetMeshAddressFromId(id);
            result = MeshConfigClient_GetTtl(destination);
            if (result == gMeshSuccess_c)
            {
                shell_printf("< TTL Get command sent to %d >", id);
            }
            else
            {
                shell_printf("< Cannot send command - Error code: 0x%04x >", result);
            }
        }
        else
        {
            uint8_t ttl;
            Mesh_GetTtl(&ttl);
            shell_printf("< Commissioner has TTL %d >", ttl);
            result = gMeshSuccess_c;
        }
    }
    else /* argc == 4 */
    {
        if (!strcmp(argv[1], "set"))
        {
            int8_t id = atoi(argv[2]);
            uint8_t ttl = atoi(argv[3]);
            if (ttl > 63)
            {
                shell_printf("< TTL can have a maximum value of 63 >");
                return CMD_RET_FAILURE;
            }
            if (id != 0)
            {
                meshAddress_t destination = GetMeshAddressFromId(id);
                result = MeshConfigClient_SetTtl(destination, ttl);
                if (result == gMeshSuccess_c)
                {
                    shell_printf("< TTL Set command sent to ID %d >", id);
                }
                else
                {
                    shell_printf("< Cannot send command - Error code: 0x%04x >", result);
                }
            }
            else
            {
                Mesh_SetTtl(ttl);
                shell_printf("< Commissioner's TTL set to %d >", ttl);
                result = gMeshSuccess_c;
            }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_Light(uint8_t argc, char * argv[])
{
    if (argc != 2 && argc != 3)
    {
        return CMD_RET_USAGE;
    }
    
    meshAddress_t destination;
    if  (argc == 2)
    {
        destination = gBroadcastAddress_c;
    }
    else
    {
        int8_t id = atoi(argv[2]);
        destination = GetMeshAddressFromId(id);
    }
    
    meshResult_t result = gMeshSuccess_c;
    if (!strcmp(argv[1], "on"))
    {
        MeshLightClient_SetLightState(destination, TRUE);
        shell_printf("< Light ON sent >");
    }
    else if (!strcmp(argv[1], "off"))
    {
        MeshLightClient_SetLightState(destination, FALSE);
        shell_printf("< Light OFF sent >");
    }
    else if (!strcmp(argv[1], "toggle"))
    {
        MeshLightClient_ToggleLight(destination);
        shell_printf("< Light TOGGLE sent >");
    }
    else
    {
        return CMD_RET_USAGE;
    }
             
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_Log(uint8_t argc, char * argv[])
{
    if (argc > 1)
    {
        return CMD_RET_USAGE;
    }
    
    meshResult_t result = gMeshSuccess_c; 
    
    mLog = !mLog;
    if (mLog)
    {
        shell_printf("\r\n Packet log is ON.");
    }
    else
    {
        shell_printf("\r\n Packet log is OFF.");
    }
    
    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_DataTransfer(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 2)
    {
        return CMD_RET_USAGE;
    }

    meshResult_t result;
    if (argc == 2)
    {
        if (!strcmp(argv[1], "get"))
        {
        	if(mDataTxStatus)
        		shell_printf("\r\nData transfer is ONGOING ");
        	else
        		shell_printf("\r\nData transfer is STOPPED ");

        	result = gMeshSuccess_c;
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "set"))
        {
			if (!strcmp(argv[2], "start"))
			{

				meshAddress_t destination = GetMeshAddressFromId(22);
				meshCustomData_t CustomData;
				CustomData.aData[CUSTOM_CMD_SOURCE] = 0;
				CustomData.aData[CUSTOM_CMD_DEST] = 22;

				CustomData.aData[CUSTOM_CMD_FUNC] = CUSTOM_CMD_START_DATA;

				CustomData.aData[CUSTOM_CMD_POLL_ITVL_0] = (uint8_t)(mDataPollRate & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_1] = (uint8_t)((mDataPollRate >> 8) & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_2] = (uint8_t)((mDataPollRate >> 16) & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_3] = (uint8_t)((mDataPollRate >> 24) & 0xFF);

				CustomData.aData[CUSTOM_CMD_POWER_CTRL] = CUSTOM_CMD_SYS_AWAKE;
				CustomData.dataLength = 8;
				Mesh_SendCustomData(destination,&CustomData);

				mDataTxStatus = TRUE;
				shell_printf("\r\nData transfer Started ");


				result = gMeshSuccess_c;
			}
			else if (!strcmp(argv[2], "stop"))
			{

				meshAddress_t destination = GetMeshAddressFromId(22);
				meshCustomData_t CustomData;
				CustomData.aData[CUSTOM_CMD_SOURCE] = 0;
				CustomData.aData[CUSTOM_CMD_DEST] = 22;

				CustomData.aData[CUSTOM_CMD_FUNC] = CUSTOM_CMD_STOP_DATA;
				/*
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_0] = (uint8_t)(mDataPollRate & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_1] = (uint8_t)((mDataPollRate >> 8) & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_2] = (uint8_t)((mDataPollRate >> 16) & 0xFF);
				CustomData.aData[CUSTOM_CMD_POLL_ITVL_3] = (uint8_t)((mDataPollRate >> 24) & 0xFF);
				*/
				CustomData.aData[CUSTOM_CMD_POWER_CTRL] = CUSTOM_CMD_SYS_SLEEP;
				CustomData.dataLength = 8;
				Mesh_SendCustomData(destination,&CustomData);

				mDataTxStatus = FALSE;
				shell_printf("\r\nData transfer Stopped ");


				result = gMeshSuccess_c;
			}
	        else
	        {
	            return CMD_RET_USAGE;
	        }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else
    {
        return CMD_RET_USAGE;
    }


    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_SenPollRate(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }

    meshResult_t result;
    if (argc == 3)
    {
        if (!strcmp(argv[1], "get"))
        {
            if (!strcmp(argv[2], "temp"))
            {
            	shell_printf("\r\nTemp Sensor poll rate is %d ",mTempSenPollRate);
            	//get data from relay
            }
            else if (!strcmp(argv[2], "light"))
            {
            	shell_printf("\r\nLight Sensor poll rate is %d ",mLightSenPollRate);
            	//get data from relay
            }
            else
            {
                return CMD_RET_USAGE;
            }

        	result = gMeshSuccess_c;
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else if (argc == 4)
    {
        if (!strcmp(argv[1], "set"))
        {
            if (!strcmp(argv[2], "temp"))
            {
            	mTempSenPollRate = (uint32_t)(atoi(argv[3]));
            	shell_printf("\r\nTemp Sensor poll rate set to %d ",mTempSenPollRate);
            	//get data from relay
            }
            else if (!strcmp(argv[2], "light"))
            {
            	mLightSenPollRate = (uint32_t)(atoi(argv[3]));
            	shell_printf("\r\nLight Sensor poll rate set to %d ",mLightSenPollRate);
            	//get data from relay
            }
            else
            {
                return CMD_RET_USAGE;

            }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else
    {
        return CMD_RET_USAGE;
    }


    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}

int8_t ShellMesh_DataPollRate(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 2)
    {
        return CMD_RET_USAGE;
    }

    meshResult_t result;
    if (argc == 2)
    {
        if (!strcmp(argv[1], "get"))
        {
        	shell_printf("\r\nData Poll rate is: %d ",mDataPollRate);
        	result = gMeshSuccess_c;
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "set"))
        {
        	mDataPollRate = (uint32_t)(atoi(argv[2]));

			meshAddress_t destination = GetMeshAddressFromId(22);
			meshCustomData_t CustomData;
			CustomData.aData[CUSTOM_CMD_SOURCE] = 0;
			CustomData.aData[CUSTOM_CMD_DEST] = 22;

			CustomData.aData[CUSTOM_CMD_FUNC] = CUSTOM_CMD_START_DATA;

			CustomData.aData[CUSTOM_CMD_POLL_ITVL_0] = (uint8_t)(mDataPollRate & 0xFF);
			CustomData.aData[CUSTOM_CMD_POLL_ITVL_1] = (uint8_t)((mDataPollRate >> 8) & 0xFF);
			CustomData.aData[CUSTOM_CMD_POLL_ITVL_2] = (uint8_t)((mDataPollRate >> 16) & 0xFF);
			CustomData.aData[CUSTOM_CMD_POLL_ITVL_3] = (uint8_t)((mDataPollRate >> 24) & 0xFF);

			//CustomData.aData[CUSTOM_CMD_POWER_CTRL] = CUSTOM_CMD_SYS_AWAKE;
			CustomData.dataLength = 8;
			Mesh_SendCustomData(destination,&CustomData);

        	//Start Reset Timer here
        	shell_printf("\r\nData Poll rate Set to: %d ",mDataPollRate);
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else
    {
        return CMD_RET_USAGE;
    }


    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}


int8_t ShellMesh_SenPower(uint8_t argc, char * argv[])
{
    if (argc > 4 || argc < 3)
    {
        return CMD_RET_USAGE;
    }

    meshResult_t result;
    if (argc == 3)
    {
        if (!strcmp(argv[1], "get"))
        {
            if (!strcmp(argv[2], "temp"))
            {
            	if(mTempSenPowSt)
            		shell_printf("\r\nTemp Sensor Power status is ON ");
            	else
            		shell_printf("\r\nTemp Sensor Power status is OFF ");
            	//get data from relay
            }
            else if (!strcmp(argv[2], "light"))
            {
            	if(mLightSenPowSt)
            		shell_printf("\r\nLight Sensor Power status is ON ");
            	else
            		shell_printf("\r\nLight Sensor Power status is OFF ");
            	//get data from relay
            }
            else
            {
                return CMD_RET_USAGE;
            }

        	result = gMeshSuccess_c;
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else if (argc == 4)
    {
        if (!strcmp(argv[1], "set"))
        {
            if (!strcmp(argv[2], "temp"))
            {
                if (!strcmp(argv[3], "wake"))
                {
                	mTempSenPowSt = TRUE;
                	shell_printf("\r\nTemp Sensor is in WAKE mode ");
                	//set data to relay
                }
                else if (!strcmp(argv[3], "sleep"))
                {
                	mTempSenPowSt = FALSE;
                	shell_printf("\r\nTemp Sensor is in SLEEP mode ");
                	//set data to relay
                }
                else
                {
                    return CMD_RET_USAGE;

                }
            }
            else if (!strcmp(argv[2], "light"))
            {
                if (!strcmp(argv[3], "wake"))
                {
                	mLightSenPowSt = TRUE;
                	shell_printf("\r\nLight Sensor is in WAKE mode ");
                	//set data to relay
                }
                else if (!strcmp(argv[3], "sleep"))
                {
                	mLightSenPowSt = FALSE;
                	shell_printf("\r\nLight Sensor is in SLEEP mode ");
                	//set data to relay
                }
                else
                {
                    return CMD_RET_USAGE;

                }
            }
            else
            {
                return CMD_RET_USAGE;

            }
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }
    else
    {
        return CMD_RET_USAGE;
    }


    if (result == gMeshSuccess_c)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_FAILURE;
    }
}
/*! *********************************************************************************
* @}
********************************************************************************** */
