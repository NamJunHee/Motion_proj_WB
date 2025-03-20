/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define MAGBUFFSIZE 650 // Freescale's lib needs at least 392

extern uint8_t magcal_flag;

extern uint8_t gyro_buffer[6];
extern uint8_t accel_buffer[6];
extern uint8_t mag_buffer[6];

extern int16_t gyroX, gyroY, gyroZ;
extern int16_t accelX, accelY, accelZ;
extern int16_t magX, magY, magZ;

extern float accel_x, accel_y, accel_z;
extern float gyro_x, gyro_y, gyro_z;
extern float mag_x, mag_y, mag_z;

extern float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, mx2, my2, mz2;
extern volatile float q0, q1, q2, q3;

typedef enum{
	GYRO_SENSOR,
	ACCEL_SENSOR,
	MAG_SENSOR
}SensorType;

extern SensorType CurrentSensor;

typedef struct {
    float V[3];                  // current hard iron offset x, y, z, (uT)
    float invW[3][3];            // current inverse soft iron matrix
    float B;                     // current geomagnetic field magnitude (uT)
    float FourBsq;               // current 4*B*B (uT^2)
    float FitError;              // current fit error %
    float FitErrorAge;           // current fit error % (grows automatically with age)
    float trV[3];                // trial value of hard iron offset z, y, z (uT)
    float trinvW[3][3];          // trial inverse soft iron matrix size
    float trB;                   // trial value of geomagnetic field magnitude in uT
    float trFitErrorpc;          // trial value of fit error %
    float A[3][3];               // ellipsoid matrix A
    float invA[3][3];            // inverse of ellipsoid matrix A
    float matA[10][10];          // scratch 10x10 matrix used by calibration algorithms
    float matB[10][10];          // scratch 10x10 matrix used by calibration algorithms
    float vecA[10];              // scratch 10x1 vector used by calibration algorithms
    float vecB[4];               // scratch 4x1 vector used by calibration algorithms
    int8_t ValidMagCal;          // integer value 0, 4, 7, 10 denoting both valid calibration and solver used
    int16_t BpFast[3][MAGBUFFSIZE];   // uncalibrated magnetometer readings
    int8_t  valid[MAGBUFFSIZE];        // 1=has data, 0=empty slot
    int16_t MagBufferCount;           // number of magnetometer readings
} MagCalibration_t;
extern MagCalibration_t magcal;

typedef struct {
	float x;
	float y;
	float z;
	//int valid;
} Point_t;


extern int interrupt_cnt;
extern int MagCal_Run(void);
extern float quality_surface_gap_error(void);
extern void apply_calibration(int16_t rawx, int16_t rawy, int16_t rawz, Point_t *out);

extern void f3x3matrixAeqI(float A[][3]);
extern void fmatrixAeqI(float *A[], int16_t rc);
extern void f3x3matrixAeqScalar(float A[][3], float Scalar);
extern void f3x3matrixAeqAxScalar(float A[][3], float Scalar);
extern void f3x3matrixAeqMinusA(float A[][3]);
extern void f3x3matrixAeqInvSymB(float A[][3], float B[][3]);
extern float f3x3matrixDetA(float A[][3]);
extern void fmatrixAeqInvA(float *A[], int8_t iColInd[], int8_t iRowInd[], int8_t iPivot[], int8_t isize);
extern void fmatrixAeqRenormRotA(float A[][3]);

extern int magcal_type;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define LSM9DS1_ADDR (0x6A << 1)
#define MAG_ADDR 	 (0x1E << 1)

#define WHO_AM_I_REG 0x0F

#define CTRL_REG1_G  0x10
#define CTRL_REG3_G  0x12

#define CTRL_REG5_XL 0x1F
#define CTRL_REG6_XL 0x20
#define CTRL_REG7_XL 0x21

#define CTRL_REG1_M  0x20
#define CTRL_REG3_M  0x22
#define CTRL_REG4_M  0x23

#define CTRL_REG4_G  0x1E
#define CTRL_REG7_G  0x16
#define CTRL_REG8    0x22
#define CTRL_REG9    0x23

#define OUT_X_G      0x18
#define OUT_X_XL     0x28
#define OUT_X_M      0x28

#define OUT_X_L_G    0x18
#define OUT_X_H_G    0x19
#define OUT_Y_L_G    0x1A
#define OUT_Y_H_G    0x1B
#define OUT_Z_L_G    0x1C
#define OUT_Z_H_G    0x1D

#define ACCEL_SENSITIVITY_2G 61.0f
#define MAG_SENSITIVITY_4GAUSS 0.14f
#define GYRO_SENSITIVITY_245DPS 8.75f

#define betaDef 0.5f

#define UT_PER_COUNT 0.1F
#define DEG_PER_SEC_PER_COUNT 0.0625F  // = 1/16

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void LSM9DS1_ReadGyro_DMA(void);
void LSM9DS1_ReadAccel_DMA(void);
void LSM9DS1_ReadMag_DMA(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYS_WKUP2_Pin GPIO_PIN_13
#define SYS_WKUP2_GPIO_Port GPIOC
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_0
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_6
#define STLINK_RX_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
