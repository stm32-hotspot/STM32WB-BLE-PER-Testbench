/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
  *****************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"
#include "custom_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];

}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

}BleApplicationContext_t;
/* USER CODE BEGIN PTD */
typedef PACKED(struct)
{
  uint8_t radio_mode;
  
  uint8_t tx_power;
  uint8_t tx_frequency;
  uint8_t tx_length_of_test_data;
  uint8_t tx_packet_payload;
  uint8_t tx_phy;
  
  uint8_t rx_frequency;
  uint8_t rx_phy;
  uint8_t rx_modulation_index;
  uint16_t number_of_packets;
  
  uint8_t reserved[2];
}config_params_t; 

typedef enum {
  TX_PWR = 0, 
  TX_FREQ,
  TX_DATA_LENGTH,   //length in bytes of payload data in each packet
  TX_PACKET_PAYLOAD,
  TX_PHY,
  RX_FREQ, 
  RX_PHY,
  RX_MODULATION, 
  RX_NUM_OF_PACKETS_LSB,    
  RX_NUM_OF_PACKETS_MSB    
} DTMParamsE;


uint8_t TimerMeasurement_Id;   
uint8_t TimerStartPER_Id;      

uint16_t Num_of_Packets_Received; 
float per_test_timeout;

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */
/* USER CODE BEGIN PD */
//#define PER_TEST_TIMEOUT               (0.949*1000*1000/CFG_TS_TICK_VAL)  /* Expected time to receive 1500 Packets of 37 Bytes is 938ms, per Core Specification version 5.x, chapter “4.1.6 LE Test Packet Interval” of Volume 6, Part F, + 11ms of system set up time*/
#define START_PER_TIMEOUT              (1*1000*1000/CFG_TS_TICK_VAL)  /**< 1s */  
#define BD_ADDR_SIZE_LOCAL              6
#define FUDGE_FACTOR                    0.0115f    /* Fudge factor to fine tune receive window to receive expected number of packets, 11.5ms measured for a data length of 37 bytes */
#define DEFAULT_NUM_OF_TEST_PACKETS             1500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RADIO_MODE_TX           0
#define RADIO_MODE_RX           1
#define RADIO_MODE_NO        0xFF
#define EXT_PA_ENABLED       0x01
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];



PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;

Custom_App_ConnHandle_Not_evt_t handleNotification;

/**
 * Advertising Data
 */

/* USER CODE BEGIN PV */
typedef PACKED(struct)
{
  uint8_t length;
  
  const uint8_t type;
  
  uint8_t version;
  
  uint8_t id;
  
  uint8_t index;

  uint16_t packets_received;
  
  float per; // [STM] packet error rate
  
  int16_t rssi;
  
  uint8_t public_device_addr[BD_ADDR_SIZE_LOCAL];
  
} adv_beacon_t; // [STM] check the URL, slide Test Bench PER / GAP Advertising Packet
// https://stmicroelectronics.sharepoint.com/:p:/r/sites/AMEMCUFAERFTeam/_layouts/15/Doc.aspx?sourcedoc=%7B4D44C5AB-65FA-4631-9B91-BCC48CBAE729%7D&file=STM32WB_PER_Testbench_v1.1.pptx

static adv_beacon_t m_adv_beacon = {
  .length = sizeof(adv_beacon_t) - 1,
  .type = AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
  .version = 0x01,
  .id = 0x83, // [STM] for PER testbench
  .index = 0,
};

static config_params_t m_config_params;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void * pPayload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
static void Adv_Request(APP_BLE_ConnStatus_t New_Status);
static void Adv_Cancel(void);
static void PER_Test_Config(void);
static void Start_Tx(void);
static void Stop_Tx(void);
static void Start_Rx(void);
static void Stop_Rx(void);
static void Measure_PER( void );
static void PER_APP_Measure_PER(void);
static void PER_APP_Start_Rx(void);
static void Read_RSSI(int16_t *rssi_dbm);
static void adv_config(void);


/* USER CODE BEGIN PFP */
static void per_set_default_params(config_params_t * p_param);
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init( void )
{
/* USER CODE BEGIN APP_BLE_Init_1 */
  
/* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_SLAVE_SCA,
     CFG_BLE_MASTER_SCA,
     CFG_BLE_LSE_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG}
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  if (SHCI_C2_BLE_Init( &ble_init_cmd_packet ) != SHCI_Success)
  {
    Error_Handler();
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_CONFIG_ID, UTIL_SEQ_RFU, PER_Test_Config);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_START_TX_ID, UTIL_SEQ_RFU, Start_Tx);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_STOP_TX_ID, UTIL_SEQ_RFU, Stop_Tx);

  UTIL_SEQ_RegTask( 1<<CFG_TASK_ADV_CONFIG_ID, UTIL_SEQ_RFU, adv_config);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_MEASURE_PER_ID, UTIL_SEQ_RFU, Measure_PER);

  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_START_RX_ID, UTIL_SEQ_RFU, Start_Rx);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_PER_TEST_STOP_RX_ID, UTIL_SEQ_RFU, Stop_Rx);

  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(TimerMeasurement_Id), hw_ts_SingleShot, PER_APP_Measure_PER);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(TimerStartPER_Id), hw_ts_SingleShot, PER_APP_Start_Rx);

  per_set_default_params(&m_config_params);

  PER_Test_Config();

  /**
   * Start to Advertise to be connected by a Client
   */
  Adv_Request(APP_BLE_FAST_ADV);
/* USER CODE END APP_BLE_Init_2 */
  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  evt_blecore_aci *blecore_evt;

  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;

  /* PAIRING */
  aci_gap_numeric_comparison_value_event_rp0 *evt_numeric_value;
  aci_gap_pairing_complete_event_rp0 *pairing_complete;
  uint32_t numeric_value;
  /* PAIRING */

  /* USER CODE BEGIN SVCCTL_App_Notification */
  
  /* USER CODE END SVCCTL_App_Notification */

  switch (event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      hci_disconnection_complete_event_rp0 *disconnection_complete_event;
      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) event_pckt->data;

      if (disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

        APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH CLIENT \n");
      }

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);

      /**
       * SPECIFIC to Custom Template APP
       */
      handleNotification.Custom_Evt_Opcode = CUSTOM_DISCON_HANDLE_EVT;
      handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
      Custom_APP_Notification(&handleNotification);
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */
    }

    break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
    {
      meta_evt = (evt_le_meta_event*) event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("\r\n\r** CONNECTION UPDATE EVENT WITH CLIENT \n");

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_complete_event_rp0 *connection_complete_event;

          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;

          APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n", connection_complete_event->Connection_Handle);
          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
          /**
           * SPECIFIC to Custom Template APP
           */
          handleNotification.Custom_Evt_Opcode = CUSTOM_CONN_HANDLE_EVT;
          handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          Custom_APP_Notification(&handleNotification);
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */

          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
        }
        break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

        /* USER CODE BEGIN META_EVT */

        /* USER CODE END META_EVT */

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }
    }
    break; /* HCI_LE_META_EVT_CODE */

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blecore_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */
      /**
       * SPECIFIC to Custom Template APP
       */
        case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0 )
          mutex = 1;
#endif
      /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */

      /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */
      break;
        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
        APP_DBG_MSG("\r\n\r** ACI_GAP_PROC_COMPLETE_VSEVT_CODE \n");
        /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

        /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */
#if(RADIO_ACTIVITY_EVENT != 0)
        case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
        /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/

        /* USER CODE END RADIO_ACTIVITY_EVENT*/
          break; /* ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE */
#endif

        /* PAIRING */
        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
         APP_DBG_MSG("\r\n\r** ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE \n");
        break;

        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
            aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, CFG_FIXED_PIN);
        break;

        case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
            evt_numeric_value = (aci_gap_numeric_comparison_value_event_rp0 *)blecore_evt->data;
            numeric_value = evt_numeric_value->Numeric_Value;
            APP_DBG_MSG("numeric_value = %ld\n", numeric_value);
            aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES);
        break;

        case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
            pairing_complete = (aci_gap_pairing_complete_event_rp0*)blecore_evt->data;
            APP_DBG_MSG("BLE_CTRL_App_Notification: ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE, pairing_complete->Status = %d\n",pairing_complete->Status);
        break;
        /* PAIRING */

      /* USER CODE BEGIN BLUE_EVT */

      /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

      default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
    return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/

/* USER CODE END FD*/
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE };

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);


  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;
  role |= GAP_PERIPHERAL_ROLE;

  if (role > 0)
  {
    const char *name = CFG_GAP_DEVICE_NAME;

    aci_gap_init(role,
                 0,
                 CFG_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle,
                 &gap_dev_name_char_handle,
                 &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }
  /**
   * Initialize Default PHY
   */
  hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                         CFG_SC_SUPPORT,
                                         CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         CFG_BLE_ADDRESS_TYPE
                                         );

  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }
}


static void Adv_Request(APP_BLE_ConnStatus_t New_Status)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  BleApplicationContext.Device_Connection_Status = New_Status;
  uint8_t adv_type = New_Status == APP_BLE_FAST_ADV ? ADV_TYPE : ADV_NONCONN_IND;

  uint8_t radio_mode = m_config_params.radio_mode;  /* Set the radio mode set by user in mobile app */
  uint8_t name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'D', 'T', 'M', '-', 'S', 'T'};
  uint8_t sz = sizeof(name);

  switch(radio_mode){
    case RADIO_MODE_TX:
      name[5] = 'T';
      name[6] = 'X';
      break;

    case RADIO_MODE_RX:
      name[5] = 'R';
      name[6] = 'X';
      break;

    default:
      break;
  }


  /* Start Fast or Low Power Advertising */
  ret = aci_gap_set_discoverable(adv_type,
                                 CFG_FAST_CONN_ADV_INTERVAL_MIN,
                                 CFG_FAST_CONN_ADV_INTERVAL_MAX,
                                 CFG_BLE_ADDRESS_TYPE,
                                 ADV_FILTER,
                                 sz,
                                 name,
                                 0, 0, 0, 0);

  ret |= aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

  /* Update Advertising data */
  if(radio_mode == RADIO_MODE_RX && New_Status == APP_BLE_LP_ADV){
    ret |= aci_gap_update_adv_data(m_adv_beacon.length + 1, (uint8_t*) &m_adv_beacon);
  }

  if (ret == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("%s successfully Start %s Advertising \r\n",
                &name[1], New_Status == APP_BLE_FAST_ADV ? "Fast" : "Beacon");
  }
  else
  {
    APP_DBG_MSG("Advertising Failed , result: %d \r\n", ret);
  }

  return;
}

const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

/**
 * Public Address with the ST company ID
 * bit[47:24] : 24bits (OUI) equal to the company ID
 * bit[23:16] : Device ID.
 * bit[15:0] : The last 16bits from the UDN
 * Note: In order to use the Public Address in a final product, a dedicated
 * 24bits company ID (OUI) shall be bought.
 */
    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)device_id;
    bd_addr_udn[3] = (uint8_t)(company_id & 0x000000FF);
    bd_addr_udn[4] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x00FF0000) >> 16 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }
  }

  return bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* Timer Server Handlers */
void PER_APP_Measure_PER(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_MEASURE_PER_ID, CFG_SCH_PRIO_0);  
}

void PER_APP_Start_Rx(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_START_RX_ID, CFG_SCH_PRIO_0);  
}

/* Tasks */
static void PER_Test_Config()
{
  const uint8_t sz = sizeof(m_config_params) - sizeof(m_config_params.reserved);
  uint8_t * p_ptr = (uint8_t *)&m_config_params;
  uint8_t tx_data_length = m_config_params.tx_length_of_test_data;
  
  uint8_t msg[64] = {0,};
  float packet_interval;

  APP_DBG_MSG("PER_Test_Config for %02X:%02X:%02X:%02X:%02X:%02X\r\n",
              bd_addr_udn[5],
              bd_addr_udn[4],
              bd_addr_udn[3],
              bd_addr_udn[2],
              bd_addr_udn[1],
              bd_addr_udn[0]);  

  /* Nothing to do here, just print values for now, the new configuration parameters 
     will be set in Start_Tx and Start_Rx tasks */
  for(uint8_t k = 0; k < sz; k++){
    sprintf((char *)msg, "%s %02X", msg, p_ptr[k]);
  }
  APP_DBG_MSG("%s\n", msg);

  /* Select PER Test Interval based on Test Data Length, per Core Specification version 5.x, 
     chapter “4.1.6 LE Test Packet Interval” of Volume 6, Part F */
  if (tx_data_length <= 37)
  {
    packet_interval = 0.000625f;  /* 625ms */
  }
  else if ((tx_data_length >=  38) && (tx_data_length <= 115))
  {
    packet_interval = 0.001250f;  /* 1250ms */ 
  }
  else if ((tx_data_length >= 116) && (tx_data_length <= 193))
  {
    packet_interval = 0.001875f;  /* 1875ms */   
  }
  else if (tx_data_length >= 194)
  {
    packet_interval = 0.002500f;  /* 2500ms */        
  }
  else
  {
    APP_DBG_MSG("Undefined Tx Data Length for packet interval selection \n");      
  }
  
  /* Calculate PER Test Timeout (receive window) based on packet interval and number of test packet. 
     Ex. PER Test Timeout is 938ms to receive 1500 Packets of 37 data bytes */ 
  per_test_timeout = (m_config_params.number_of_packets * packet_interval + FUDGE_FACTOR);  /* timeout value for TimerMeasurement_Id virtual timer */ 
                                     
}



static void Start_Tx(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  APP_DBG_MSG("Start DTM Tx \n");    
  
  /* HCI Reset to synchronise BLE Stack */
  hci_reset();
  
  HAL_Delay(10);

  HAL_GPIO_WritePin(EXT_PA_TX_PORT, EXT_PA_TX_PIN, GPIO_PIN_SET);  /* Set EXT_PA_TX pin during DTM Tx */

  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);  /* Turn on Green LED2 on NUCLEO-WB55RG */
  
  aci_hal_set_tx_power_level(1, m_config_params.tx_power);  
  
  ret = hci_le_enhanced_transmitter_test(m_config_params.tx_frequency,   
                                         m_config_params.tx_length_of_test_data,   
                                         m_config_params.tx_packet_payload,  
                                         m_config_params.tx_phy);   
 
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("HCI_LE_TRANSMITTER_TEST_V2, Not Successful \n");      
  }
  
}

static void Stop_Tx(void)
{
  uint16_t val = 0;
  
  APP_DBG_MSG("Stop DTM Tx \n");  
    
  /* Release EXT_PA_TX PB0 pin when not in DTM mode */
  GPIO_InitTypeDef gpio;
  
  __HAL_RCC_GPIOB_CLK_ENABLE( );
  /* Configure GPIO pin (PB0) in AF6 mode (EXT_PA_TX) to control CTX */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Alternate = 6;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = EXT_PA_TX_PIN;
  HAL_GPIO_Init( EXT_PA_TX_PORT, &gpio );
  
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);  /* Turn on Green LED2 on NUCLEO-WB55RG */
    
  hci_le_test_end(&val);   /* Stop DTM Tx mode */ 
  
  /* Start normal advertising */ 
  Ble_Hci_Gap_Gatt_Init();
  SVCCTL_Init();  
  Adv_Request(APP_BLE_FAST_ADV);  
}


static void adv_config(void)
{
  /* Advertise the PER results for 1 second or enough for phone to pick it during 
     its scan. Then restart a new PER test */

  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);  /* Turn off Green LED2 on NUCLEO-WB55RG when not in DTM Rx mode */
  
  Ble_Hci_Gap_Gatt_Init();
  
  Adv_Request(APP_BLE_LP_ADV); /* Special advertising for DTX-RX */
    
  HW_TS_Start(TimerStartPER_Id, START_PER_TIMEOUT);   /* 1s timeout */
  
}


static void Start_Rx(void)
{
  APP_DBG_MSG("Start DTM Rx \n");     
  
  hci_reset();
  
  HAL_GPIO_WritePin(EXT_PA_TX_PORT, EXT_PA_TX_PIN, GPIO_PIN_RESET);  /* Reset EXT_PA_TX pin during DTM Rx */
 
  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);  /* Turn on Green LED2 on NUCLEO-WB55RG */
  
  HW_TS_Start(TimerMeasurement_Id, (uint32_t)(per_test_timeout*1000*1000/CFG_TS_TICK_VAL));   
  
  /* Start DTM Rx mode */
  hci_le_enhanced_receiver_test(m_config_params.rx_frequency,   
                                m_config_params.rx_phy,   
                                m_config_params.rx_modulation_index);  
         
}



static void Stop_Rx(void)
{
  APP_DBG_MSG("Stop DTM Rx \n");   
  
  /* Release EXT_PA_TX PB0 pin when not in DTM mode */
  GPIO_InitTypeDef gpio;  
  
  __HAL_RCC_GPIOB_CLK_ENABLE( );
  /* Configure GPIO pin (PB0) in AF6 mode (EXT_PA_TX) to control CTX */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Alternate = 6;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = GPIO_PIN_0;
  HAL_GPIO_Init( GPIOB, &gpio );
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  /* Turn off Green LED2 on NUCLEO-WB55RG */
   
  /* Stop PER Test related timers */
  HW_TS_Stop(TimerMeasurement_Id); 
  HW_TS_Stop(TimerStartPER_Id);   

  /* Start normal advertising */ 
  Ble_Hci_Gap_Gatt_Init();
  SVCCTL_Init();  
  Adv_Request(APP_BLE_FAST_ADV);    
}

static void Read_RSSI(int16_t *rssi_dbm)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t rssi_level[3];

  uint32_t rssi_int16;
  uint32_t reg_agc;    

  ret = aci_hal_read_raw_rssi(rssi_level);
  
  if (ret == BLE_STATUS_SUCCESS)
  {
    /* Extract the data rssi_int16 + agc */
    rssi_int16 = ((rssi_level[0]) | (rssi_level[1]<<8)) & 0xFFFFU;  /* First 2 bytes contain Rssi measured on the received signal */
    reg_agc = (rssi_level[2]) & 0xffU;  /* Third byte is the AGC value used for the RX */
    /* check if rssi is too low */
    if((rssi_int16 == 0U) || (reg_agc > 0xbU))
    {
        *rssi_dbm = 127 ;
    }
    else
    {
        *rssi_dbm = (int32_t)reg_agc * 6 - 127 ;
        while(rssi_int16 > 30U)
        {
            *rssi_dbm += 6 ;
            rssi_int16 = (rssi_int16 >> 1) ;
        }
        *rssi_dbm += (int32_t)(uint32_t)((417U*rssi_int16 + 18080U)>>10) ;
    }
  }
  else
  {
    APP_DBG_MSG("aci_hal_read_raw_rssi: 0x%X \n", ret);    
  }
  
}

static void Measure_PER(void)
{
  int16_t rssi_dbm;
  uint16_t diff;
  float per; 

  APP_DBG_MSG("Measure_PER \n");   
  
  hci_le_test_end(&Num_of_Packets_Received);      

  /* Since this implementation is based on using a precalculated timing windows for 
     a certain number of packets of certain length, it is possible that the timing 
     is not 100% accurate. Add to that the Virtual Timer's error, and we can end 
     up with a slightly larger or smaller rx window than is needed to receive exactly 
     the set number of packets expected. The error noticed during testing was of 
     +/- 2 packets, which will only be observed when the two Tx and Rx boards are 
     sitting right next to each other (in other words perfect conditions). In any 
     case, +/-2 packets in a 1500 packet test results in a negligible 0.13% PER 
     error, thus can be ignored. To prevent confusion, the PER sent to the mobile 
     app will be truncated to 0 in case more than the expected number of packets */
  
  if (Num_of_Packets_Received > m_config_params.number_of_packets)
  {
    Num_of_Packets_Received = m_config_params.number_of_packets;
  }
  
  diff = m_config_params.number_of_packets - Num_of_Packets_Received;
  per = 100.0f * ((float)diff / (float)m_config_params.number_of_packets);

  Read_RSSI(&rssi_dbm);
    
  APP_DBG_MSG("INDEX %d\tNumber of Packets Received: %d  per: %f  rssi: %d\n",
		  m_adv_beacon.index++,
		  Num_of_Packets_Received,
		  per,
		  rssi_dbm);
  
  /* Update the advertisement packet with latest measurements */
  m_adv_beacon.packets_received = Num_of_Packets_Received;
  m_adv_beacon.per = per;
  m_adv_beacon.rssi = rssi_dbm;
  
  memcpy(m_adv_beacon.public_device_addr, BleGetBdAddress(), sizeof(m_adv_beacon.public_device_addr));
  
  Num_of_Packets_Received = 0;
  
  /* Start Task to start advertising the new measurement results */
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_CONFIG_ID, CFG_SCH_PRIO_0);
}

static void per_set_default_params(config_params_t * p_param)
{
  p_param->radio_mode = RADIO_MODE_NO; // not configured
  
  /* Init to default tx config params */
  p_param->tx_power                     = 0x1F;         /* +6dBm max power */
  p_param->tx_frequency                 = 0x00;         /* 2402 MHz (ch37) */
  p_param->tx_length_of_test_data       = 0x25;         /* 37 Bytes */
  p_param->tx_packet_payload            = 0x00;         /* Pseudo-Random bit sequence 9 */
  p_param->tx_phy                       = 0x01;         /* LE 1M PHY */
  
  /* Init to default rx config params */
  p_param->rx_frequency                 = 0x00;         /* 2402 MHz (ch37) */
  p_param->rx_phy                       = 0x01;         /* LE 1M PHY */
  p_param->rx_modulation_index          = 0x00;         /* Modulation Index */
  
  p_param->number_of_packets            = DEFAULT_NUM_OF_TEST_PACKETS;
  
  APP_DBG_MSG("Default config init complete\r\n");
}

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 *SPECIFIC FUNCTIONS FOR CUSTOM
 *
 *************************************************************/
static void Adv_Cancel( void )
{
/* USER CODE BEGIN Adv_Cancel_1 */

/* USER CODE END Adv_Cancel_1 */

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_SERVER)

  {

    tBleStatus result = 0x00;

    result = aci_gap_set_non_discoverable();

    BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
    if (result == BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  \r\n\r");APP_DBG_MSG("** STOP ADVERTISING **  \r\n\r");
    }
    else
    {
      APP_DBG_MSG("** STOP ADVERTISING **  Failed \r\n\r");
    }

  }

/* USER CODE BEGIN Adv_Cancel_2 */

/* USER CODE END Adv_Cancel_2 */
  return;
}


/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */
void APP_BLE_Key_Button1_Action(void)
{  
  /* SW1: Start DTM Tx or Rx, according to the configured radio mode */
  switch(m_config_params.radio_mode){
    case RADIO_MODE_TX:
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_START_TX_ID, CFG_SCH_PRIO_0);
      break;
      
    case RADIO_MODE_RX:
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_START_RX_ID, CFG_SCH_PRIO_0);
      break;
    
    default:
      APP_DBG_MSG("DTM board is not configued as RX/TX board yet\r\n\r");
      break;
  }
}

void APP_BLE_Key_Button2_Action(void)
{
  /* SW2: Stop DTM Tx or Rx, according to the configured radio mode */  
  switch(m_config_params.radio_mode){
    case RADIO_MODE_TX:
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_STOP_TX_ID, CFG_SCH_PRIO_0);
      break;
      
    case RADIO_MODE_RX:
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_STOP_RX_ID, CFG_SCH_PRIO_0);
      break;
    
    default:
      APP_DBG_MSG("DTM board is not configued as RX/TX board yet\r\n\r");
      break;
  }    
}

void APP_BLE_Key_Button3_Action(void)
{

}

void APP_BLE_set_config_params(uint8_t * p_data, uint8_t sz)
{
  memcpy(&m_config_params, p_data, sz);
}

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;
  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);

      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
