/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2021, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/timer/GPTimerCC26XX.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <board.h>

#include "simple_peripheral.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or 
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with 
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_PUBLIC

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 104=130ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     104

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 300=3s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          300

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_ACCEPT_ALL // GAP_UPDATE_REQ_PASS_TO_APP

// How often to read current current RPA (in ms)
// #define SP_READ_RPA_EVT_PERIOD               3000

// Delay (in ms) after connection establishment before sending a parameter update requst
#define SP_SEND_PARAM_UPDATE_DELAY           6000

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   644
#endif

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_CCFG_CHANGE_EVT                   10

#define SP_ADV_EVT                           3

#define SP_CONN_EVT                          9



// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SP_HTIMER_EVT                        Event_Id_29

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT | SP_QUEUE_EVT | SP_HTIMER_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30
#define RSSI_1M_THRSHLD           -40
#define RSSI_S2_THRSHLD           -50
#define RSSI_S8_THRSHLD           -60
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         	    connHandle;                        // Connection Handle                   // pointer to clock struct
  int8_t           	    rssiArr[SP_MAX_RSSI_STORE_DEPTH];
  uint8_t          	    rssiCntr;
  int8_t           	    rssiAvg;
  bool             	    phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          	    currPhy;
  uint8_t          	    rqPhy;
  uint8_t          	    phyRqFailCnt;                      // PHY change request count
  bool             	    isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Memory to pass RPA read event ID to clock handler
//spClockEventData_t argRpaRead =
//{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
// static List_List paramUpdateList;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Peripheral";

// Advertisement data
static uint8_t advertData[] =
{
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Scan Response Data
static uint8_t scanRspData[] =
{
  // complete name
  17,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'P',
  'e',
  'r',
  'i',
  'p',
  'h',
  'e',
  'r',
  'a',
  'l',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

static volatile uint8_t ccfg = 0;
static uint8_t charValue4[SIMPLEPROFILE_CHAR4_LEN] = {0};

static GPTimerCC26XX_Handle hTimer;

void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // interrupt callback code goes here. Minimize processing in interrupt.
    uint32_t* pValue = (uint32_t*)charValue4;
    (*pValue)++;
    Event_post(syncEvent, SP_HTIMER_EVT);
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_processCharConfigChangeEvt(uint8_t config);

static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
void SimplePeripheral_charConfigChangeCB(uint8_t config);
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);

static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);

static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
{
  SimplePeripheral_charValueChangeCB, // Simple GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;

  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the SimpleProfile Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;

    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN,
                               charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();

  GPTimerCC26XX_Params params;
  GPTimerCC26XX_Params_init(&params);
  params.width          = GPT_CONFIG_16BIT;
  params.mode           = GPT_MODE_PERIODIC;
  params.direction      = GPTimerCC26XX_DIRECTION_UP;
  params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
  hTimer = GPTimerCC26XX_open(Board_GPTIMER0A, &params); // Board_GPTIMER0A
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimplePeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      if (events & SP_HTIMER_EVT)
      {
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN,
                                   charValue4);
      }
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
//            if (pPUC->status != SUCCESS)
//            {
//              Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
//                             "PHY Change failure");
//            }
//            else
//            {
//              // Only symmetrical PHY is supported.
//              // rxPhy should be equal to txPhy.
//              Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
//                             "PHY Updated to %s",
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value");
//            }

            SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    // Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    // Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg)
{
  bool dealloc = TRUE;

  switch (pMsg->event)
  {
    case SP_CHAR_CHANGE_EVT:
      SimplePeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_CCFG_CHANGE_EVT:
      SimplePeripheral_processCharConfigChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_CONN_EVT:
      SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Temporary memory for advertising parameters for set #1. These will be copied
        // by the GapAdv module
        GapAdv_params_t advParamLegacy = GAPADV_PARAMS_LEGACY_SCANN_CONN;

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParamLegacy,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advertData), advertData);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanRspData), scanRspData);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

#if defined(BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              SP_READ_RPA_EVT_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
#endif // PRIVACY_1_2_CFG
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
//      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
//                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);
      }

      if (numActive < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      // Display_clearLine(dispHandle, SP_ROW_CONNECTION);

      GPTimerCC26XX_stop(hTimer);
      ccfg = 0;

      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charConfigChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
void SimplePeripheral_charConfigChangeCB(uint8_t config)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = config;

    if (SimplePeripheral_enqueueMsg(SP_CCFG_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue;

  switch(paramId)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      // Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Char 1: %d", (uint16_t)newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      // Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Char 3: %d", (uint16_t)newValue);
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharConfigChangeEvt(uint8_t config)
{
  if (config == (uint8_t)GATT_CLIENT_CFG_NOTIFY)
  {
    Types_FreqHz  freq;
    BIOS_getCpuFreq(&freq);
    // GPTimerCC26XX_Value loadVal = freq.lo / 1000 - 1; //47999
    GPTimerCC26XX_Value loadVal = freq.lo / 20 - 1; //4799999
    GPTimerCC26XX_setLoadValue(hTimer, loadVal);
    GPTimerCC26XX_registerInterrupt(hTimer, timerCallback, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(hTimer);

    ccfg = config;
  }
  else if (config == (uint8_t)GATT_CFG_NO_OPERATION)
  {
    GPTimerCC26XX_stop(hTimer);

    ccfg = config;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool SimplePeripheral_doSetConnPhy(uint8 index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = SimplePeripheral_getConnIndex(menuConnHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    // Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
    return FALSE;
  }


  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    SimplePeripheral_stopAutoPhyChange(connList[connIndex].connHandle);

    SimplePeripheral_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

//    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "PHY preference: %s",
//                   TBM_GET_ACTION_DESC(&spMenuConnPhy, index));
  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    SimplePeripheral_startAutoPhyChange(menuConnHandle);
  }

  return status;
}
/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
      break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Get index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  // If auto phy change is enabled
  if (connList[connIndex].isAutoPHYEnable == TRUE)
  {
    // Read the RSSI
    HCI_ReadRssiCmd(pReport->handle);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool SimplePeripheral_doSelectConn(uint8_t index)
{
  return (true);
}

/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == CONNHANDLE_ALL))
    {
      connList[i].connHandle = CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];  

      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        SIMPLEPERIPHERAL_ASSERT(index < MAX_NUM_BLE_CONNS);

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = SP_PHY_NONE;
          uint8_t phyRqS = SP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != SP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != SP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

//        Display_printf(dispHandle, SP_ROW_RSSI, 0,
//                       "RSSI:%d dBm, AVG RSSI:%d dBm",
//                       (uint32_t)(rssi),
//                       connList[index].rssiAvg);

	  } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
//        Display_printf(dispHandle, SP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
//                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
*********************************************************************/
