/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
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
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ble.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* My_P2P_Server */
  uint8_t               Srvtoclient_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

//extern TP_Flow_Status_t TpFlowStatus;  

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
  /* My_P2P_Server */

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
  
  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

  /* My_P2P_Server */
    case CUSTOM_STM_TEST_START_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TEST_START_READ_EVT */

      /* USER CODE END CUSTOM_STM_TEST_START_READ_EVT */
      break;

    case CUSTOM_STM_TEST_START_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TEST_START_WRITE_NO_RESP_EVT */
      APP_DBG_MSG("\r\n\r** CUSTOM_STM_TEST_START_WRITE_NO_RESP_EVT \r\n");
      
      /* Copy the received configuration data into local buffer */
      APP_BLE_set_config_params(pNotification->DataTransfered.pPayload,
                                pNotification->DataTransfered.Length);

      /* Start task to reconfigure the parameters for the respective Tx and Rx DTM modes */
      UTIL_SEQ_SetTask( 1<<CFG_TASK_PER_TEST_CONFIG_ID, CFG_SCH_PRIO_0);        
      
      /* USER CODE END CUSTOM_STM_TEST_START_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SRVTOCLIENT_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SRVTOCLIENT_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_SRVTOCLIENT_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SRVTOCLIENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SRVTOCLIENT_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_SRVTOCLIENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SRVTOCLIENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SRVTOCLIENT_NOTIFY_DISABLED_EVT */
        
      /* USER CODE END CUSTOM_STM_SRVTOCLIENT_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch(pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}



/* USER CODE BEGIN FD */

/* USER CODE END FD */



/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/


/* USER CODE END FD_LOCAL_FUNCTIONS*/
