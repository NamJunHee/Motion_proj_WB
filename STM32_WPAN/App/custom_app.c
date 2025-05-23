/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/custom_app.c
 * @author  MCD Application Team
 * @brief   Custom Example Application (Server)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	/* mySVC */
	uint8_t Mycharnotify_Notification_Status;
	/* USER CODE BEGIN CUSTOM_APP_Context_t */

	/* USER CODE END CUSTOM_APP_Context_t */

	uint16_t ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

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

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* mySVC */
static void Custom_Mycharnotify_Update_Char(void);
static void Custom_Mycharnotify_Send_Notification(void);

/* USER CODE BEGIN PFP */
void myTask_IMU(void) {

	//nomal_power_code
//	LSM9DS1_ReadGyro();
//	LSM9DS1_ReadAccel();
//	LSM9DS1_ReadMag();
}

void myTask_BLE(void) {

	int16_t accel_x_int = (int16_t) (accel_x * 1000);
	int16_t accel_y_int = (int16_t) (accel_y * 1000);
	int16_t accel_z_int = (int16_t) (accel_z * 1000);

	int16_t gyro_x_int = (int16_t) (gyro_x * 1000);
	int16_t gyro_y_int = (int16_t) (gyro_y * 1000);
	int16_t gyro_z_int = (int16_t) (gyro_z * 1000);

	int16_t mag_x_int = (int16_t) (mag_x * 100);
	int16_t mag_y_int = (int16_t) (mag_y * 100);
	int16_t mag_z_int = (int16_t) (mag_z * 100);

//	int16_t mag_x_int = (int16_t) (mx2 * 100);
//	int16_t mag_y_int = (int16_t) (my2 * 100);
//	int16_t mag_z_int = (int16_t) (mz2 * 100);

	int16_t q0_int = (int16_t) (q0 * 10000);
	int16_t q1_int = (int16_t) (q1 * 10000);
	int16_t q2_int = (int16_t) (q2 * 10000);
	int16_t q3_int = (int16_t) (q3 * 10000);

	UpdateCharData[0] = (uint8_t) (accel_x_int >> 8);
	UpdateCharData[1] = (uint8_t) (accel_x_int & 0xFF);
	UpdateCharData[2] = (uint8_t) (accel_y_int >> 8);
	UpdateCharData[3] = (uint8_t) (accel_y_int & 0xFF);
	UpdateCharData[4] = (uint8_t) (accel_z_int >> 8);
	UpdateCharData[5] = (uint8_t) (accel_z_int & 0xFF);

	UpdateCharData[6] = (uint8_t) (gyro_x_int >> 8);
	UpdateCharData[7] = (uint8_t) (gyro_x_int & 0xFF);
	UpdateCharData[8] = (uint8_t) (gyro_y_int >> 8);
	UpdateCharData[9] = (uint8_t) (gyro_y_int & 0xFF);
	UpdateCharData[10] = (uint8_t) (gyro_z_int >> 8);
	UpdateCharData[11] = (uint8_t) (gyro_z_int & 0xFF);

	UpdateCharData[12] = (uint8_t) (q0_int >> 8);
	UpdateCharData[13] = (uint8_t) (q0_int & 0xFF);
	UpdateCharData[14] = (uint8_t) (q1_int >> 8);
	UpdateCharData[15] = (uint8_t) (q1_int & 0xFF);
	UpdateCharData[16] = (uint8_t) (q2_int >> 8);
	UpdateCharData[17] = (uint8_t) (q2_int & 0xFF);
	UpdateCharData[18] = (uint8_t) (q3_int >> 8);
	UpdateCharData[19] = (uint8_t) (q3_int & 0xFF);

	UpdateCharData[20] = (uint8_t) (mag_x_int >> 8);
	UpdateCharData[21] = (uint8_t) (mag_x_int & 0xFF);
	UpdateCharData[22] = (uint8_t) (mag_y_int >> 8);
	UpdateCharData[23] = (uint8_t) (mag_y_int & 0xFF);
	UpdateCharData[24] = (uint8_t) (mag_z_int >> 8);
	UpdateCharData[25] = (uint8_t) (mag_z_int & 0xFF);

	Custom_Mycharnotify_Update_Char();

	UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK_IMU, CFG_SCH_PRIO_0);
//	UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK_BLE, CFG_SCH_PRIO_0);
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(
		Custom_STM_App_Notification_evt_t *pNotification) {
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

	/* USER CODE END CUSTOM_STM_App_Notification_1 */
	switch (pNotification->Custom_Evt_Opcode) {
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

	/* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

	/* mySVC */
	case CUSTOM_STM_MYCHARWRITE_WRITE_EVT:
		/* USER CODE BEGIN CUSTOM_STM_MYCHARWRITE_WRITE_EVT */

		/* USER CODE END CUSTOM_STM_MYCHARWRITE_WRITE_EVT */
		break;

	case CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT:
		/* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT */

		/* USER CODE END CUSTOM_STM_MYCHARNOTIFY_NOTIFY_ENABLED_EVT */
		break;

	case CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT:
		/* USER CODE BEGIN CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT */

		/* USER CODE END CUSTOM_STM_MYCHARNOTIFY_NOTIFY_DISABLED_EVT */
		break;

	case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
		/* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

		/* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
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

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification) {
	/* USER CODE BEGIN CUSTOM_APP_Notification_1 */

	/* USER CODE END CUSTOM_APP_Notification_1 */

	switch (pNotification->Custom_Evt_Opcode) {
	/* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

	/* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
	case CUSTOM_CONN_HANDLE_EVT:
		/* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

		/* USER CODE END CUSTOM_CONN_HANDLE_EVT */
		break;

	case CUSTOM_DISCON_HANDLE_EVT:
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

void Custom_APP_Init(void) {
	/* USER CODE BEGIN CUSTOM_APP_Init */

	/* USER CODE END CUSTOM_APP_Init */
	return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* mySVC */
__USED void Custom_Mycharnotify_Update_Char(void) /* Property Read */
{
	uint8_t updateflag = 0;

	/* USER CODE BEGIN Mycharnotify_UC_1*/
	updateflag = 1;
	/* USER CODE END Mycharnotify_UC_1*/

	if (updateflag != 0) {
		Custom_STM_App_Update_Char(CUSTOM_STM_MYCHARNOTIFY,
				(uint8_t*) UpdateCharData);
	}

	/* USER CODE BEGIN Mycharnotify_UC_Last*/

	/* USER CODE END Mycharnotify_UC_Last*/
	return;
}

void Custom_Mycharnotify_Send_Notification(void) /* Property Notification */
{
	uint8_t updateflag = 0;

	/* USER CODE BEGIN Mycharnotify_NS_1*/

	/* USER CODE END Mycharnotify_NS_1*/

	if (updateflag != 0) {
		Custom_STM_App_Update_Char(CUSTOM_STM_MYCHARNOTIFY,
				(uint8_t*) NotifyCharData);
	}

	/* USER CODE BEGIN Mycharnotify_NS_Last*/

	/* USER CODE END Mycharnotify_NS_Last*/

	return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
