/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_stm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t gyro_buffer[6] = { 0, 0, 0, 0, 0, 0 };
uint8_t accel_buffer[6] = { 0, 0, 0, 0, 0, 0 };
uint8_t mag_buffer[6] = { 0, 0, 0, 0, 0, 0 };

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
int16_t magX = 0;
int16_t magY = 0;
int16_t magZ = 0;

float accel_x = 0;
float accel_y = 0;
float accel_z = 0;

float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;

float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
float gyroX_avg = 0, gyroY_avg = 0, gyroZ_avg = 0;
float gyroX_current = 0, gyroY_current = 0, gyroZ_current = 0;

float mag_x = 0;
float mag_y = 0;
float mag_z = 0;

SensorType CurrentSensor = GYRO_SENSOR;

int DAM_Callback_cnt = 0;

int ReadGryo_cnt = 0;
int ReadAccel_cnt = 0;
int ReadMag_cnt = 0;

float recipNorm;
float s0, s1, s2, s3;
float qDot1, qDot2, qDot3, qDot4;
float hx, hy;
float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2,
		_2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2,
		q2q3, q3q3;

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1, mx2, my2, mz2;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float deltat = 0.0f;
volatile float beta = betaDef;

uint32_t lastUpdate = 0;
uint32_t micro_time = 0;
uint32_t Now = 0;

float gxyz[3], gxyz1[3], axyz[3], axyz1[3], mxyz[3], axyz2[3];
//float result_V[3];

//float hardIron_x = -14.73;
//float hardIron_y = -33.35;
//float hardIron_z = 0.49;

float hardIron_x = -4.81;
float hardIron_y = -12.49;
float hardIron_z = 7.03;

//float softIron_cali[3][3] = { { 1.001, 0.033, -0.006 }, { 0.033, 0.968, 0.000 },
//		{ -0.006, -0.000, 1.033 } };

float softIron_cali[3][3] = { { 1.004, 0.090, -0.006 },
		{ 0.090, 0.980, -0.000 }, { -0.006, -0.000, 1.023 } };

uint8_t magcal_flag = 1;
static int magcal_counter = 0;

MagCalibration_t magcal;

float test_val = 0;
uint16_t mag_raw[3] = { 0, 0, 0 };

int32_t rawx, rawy, rawz;
int32_t dx, dy, dz;
float x, y, z;
uint64_t distsq, minsum = 0xFFFFFFFFFFFFFFFFull;
static int runcount = 0;
int i, j, minindex = 0;
Point_t point;
float gaps = 99, field, error, errormax;

int choose_flag = 0;
const double DEG2RAD = M_PI / 180.0;

uint32_t waitcount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_RF_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LSM9DS1_WriteRegister(uint8_t addr, uint8_t reg, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
	HAL_MAX_DELAY);
}

uint8_t LSM9DS1_ReadRegister(uint8_t addr, uint8_t reg) {
	uint8_t value;
	HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,
	HAL_MAX_DELAY);
	return value;
}

void LSM9DS1_Init() {
	//sensor reset
	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG8, 0x05);
	HAL_Delay(10);

	//low-power setting
	//gyro setting
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG1_G, 0x40); //gyro ODR 59.5 Hz(low-power mode), full scale 245dps(basic)
	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG1_G, 0x60); //ODR_G = 119Hz, FS = 245dps
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG1_G, 0x6C); //ODR_G = 119Hz, FS = 2000dps
	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG3_G, 0x80); //gyro low-power mode activate
	HAL_Delay(10);

	//accel setting
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG6_XL, 0x20); //accel ODR 10Hz, full scale 2g(basic)
	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG6_XL, 0x60); // ODR_XL = 119Hz, FS = ±2g
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG6_XL, 0x58); // ODR_XL = 119Hz, FS = ±4g
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG6_XL, 0x68); // ODR_XL = 119Hz, FS = ±8g
	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG7_XL, 0x00); //accel low-power mode activate
	HAL_Delay(10);

	//mag setting
//	LSM9DS1_WriteRegister(MAG_ADDR, CTRL_REG1_M, 0x10); // mag ODR 10Hz, temp-comp activate
	LSM9DS1_WriteRegister(MAG_ADDR, CTRL_REG1_M, 0x7C); // OM = 11 (ultra-high), DO = 110 (80Hz)
	LSM9DS1_WriteRegister(MAG_ADDR, CTRL_REG3_M, 0x00); // mag Continuous-conversion mode activate
	HAL_Delay(10);

	//not low-power setting
	//gyro setting
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG1_G, 0xE0); // gyro ODR 952Hz, full scale 245dps(basic)
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG3_G, 0x00); // gyro low-power mode deactivate
//	HAL_Delay(10);
//
//	//accel setting
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG6_XL, 0x60); // accel ODR 952Hz, full scale 2g(basic)
//	LSM9DS1_WriteRegister(LSM9DS1_ADDR, CTRL_REG7_XL, 0x80); // accel high-resolution mode activate
//	HAL_Delay(10);
//
//	//mag setting
//	LSM9DS1_WriteRegister(MAG_ADDR, CTRL_REG1_M, 0x70); // mag ODR 80Hz, temp-comp activate
//	LSM9DS1_WriteRegister(MAG_ADDR, CTRL_REG3_M, 0x00); // mag Continuous-conversion mode activate
//	HAL_Delay(10);
}

uint32_t micros(void) {
	return micro_time = __HAL_TIM_GET_COUNTER(&htim2);
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float vector_dot(float a[3], float b[3]) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
	float mag = sqrt(vector_dot(a, a));
	a[0] /= mag;
	a[1] /= mag;
	a[2] /= mag;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
		float az, float mx, float my, float mz) {

	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2
				+ _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1
				+ my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1
				+ _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
				- _2bz * q2
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (-_2bx * q3 + _2bz * q1)
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ _2bx * q2
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
				- 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
				+ _2bz * q3
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (_2bx * q2 + _2bz * q0)
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ (_2bx * q3 - _4bz * q1)
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
				- 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
				+ (-_4bx * q2 - _2bz * q0)
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (_2bx * q1 + _2bz * q3)
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ (_2bx * q0 - _4bz * q2)
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
				+ (-_4bx * q3 + _2bz * q1)
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (-_2bx * q0 + _2bz * q2)
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ _2bx * q1
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	q0 += qDot1 * (deltat);
	q1 += qDot2 * (deltat);
	q2 += qDot3 * (deltat);
	q3 += qDot4 * (deltat);

	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//low_power_code
void LSM9DS1_ReadGyro_DMA(void) {
	HAL_I2C_Mem_Read_DMA(&hi2c1, LSM9DS1_ADDR, OUT_X_G | 0x80,
	I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6);
}

void LSM9DS1_ReadAccel_DMA(void) {
	HAL_I2C_Mem_Read_DMA(&hi2c1, LSM9DS1_ADDR, OUT_X_XL | 0x80,
	I2C_MEMADD_SIZE_8BIT, accel_buffer, 6);
}

void LSM9DS1_ReadMag_DMA(void) {
	HAL_I2C_Mem_Read_DMA(&hi2c1, MAG_ADDR, OUT_X_M | 0x80, I2C_MEMADD_SIZE_8BIT,
			mag_buffer, 6);
}

void magcal_Init(void) {
	memset(&magcal, 0, sizeof(magcal));
	magcal.V[2] = 80.0f;
	magcal.invW[0][0] = 1.0f;
	magcal.invW[1][1] = 1.0f;
	magcal.invW[2][2] = 1.0f;
	magcal.FitError = 100.0f;
	magcal.FitErrorAge = 100.0f;
	magcal.B = 50.0f;
}

static int chunk_i = 0;
static int chunk_j = 0;

void apply_calibration(int16_t rawx, int16_t rawy, int16_t rawz, Point_t *out) {
	float x, y, z;

	x = ((float) rawx * UT_PER_COUNT) - magcal.V[0];
	y = ((float) rawy * UT_PER_COUNT) - magcal.V[1];
	z = ((float) rawz * UT_PER_COUNT) - magcal.V[2];
	out->x = x * magcal.invW[0][0] + y * magcal.invW[0][1]
			+ z * magcal.invW[0][2];
	out->y = x * magcal.invW[1][0] + y * magcal.invW[1][1]
			+ z * magcal.invW[1][2];
	out->z = x * magcal.invW[2][0] + y * magcal.invW[2][1]
			+ z * magcal.invW[2][2];
}

static int choose_discard_magcal(void) {
	choose_flag = 1;
	// When enough data is collected (gaps error is low), assume we
	// have a pretty good coverage and the field stregth is known.
	gaps = quality_surface_gap_error();
	if (gaps < 25.0f) {
		// occasionally look for points farthest from average field strength
		// always rate limit assumption-based data purging, but allow the
		// rate to increase as the angular coverage improves.
		if (gaps < 1.0f)
			gaps = 1.0f;
		if (++runcount > (int) (gaps * 10.0f)) {
			j = MAGBUFFSIZE;
			errormax = 0.0f;
			for (i = 0; i < MAGBUFFSIZE; i++) {
				rawx = magcal.BpFast[0][i];
				rawy = magcal.BpFast[1][i];
				rawz = magcal.BpFast[2][i];
//				apply_calibration(rawx, rawy, rawz, &point);
				x = point.x;
				y = point.y;
				z = point.z;
				field = sqrtf(x * x + y * y + z * z);
				// if magcal.B is bad, things could go horribly wrong
				error = fabsf(field - magcal.B);
				if (error > errormax) {
					errormax = error;
					j = i;
				}
			}
			runcount = 0;
			if (j < MAGBUFFSIZE) {
				//printf("worst error at %d\n", j);
				return j;
			}
		}
	} else {
		runcount = 0;
	}

//	for (i = 0; i < MAGBUFFSIZE; i++)
//	{
//		for (j = i + 1; j < MAGBUFFSIZE; j++) {
//			dx = magcal.BpFast[0][i] - magcal.BpFast[0][j];
//			dy = magcal.BpFast[1][i] - magcal.BpFast[1][j];
//			dz = magcal.BpFast[2][i] - magcal.BpFast[2][j];
//			distsq = (int64_t) dx * (int64_t) dx;
//			distsq += (int64_t) dy * (int64_t) dy;
//			distsq += (int64_t) dz * (int64_t) dz;
//			if (distsq < minsum) {
//				minsum = distsq;
//				minindex = (random() & 1) ? i : j;
//			}
//		}
//	}

	for (int cnt = 0; cnt < 10; cnt++) {
		if (chunk_i >= MAGBUFFSIZE) {
			chunk_i = 0;
			chunk_j = 0;
			break;
		}

		if (chunk_j >= MAGBUFFSIZE) {
			chunk_i++;
			chunk_j = chunk_i + 1;
			continue;
		}

		dx = magcal.BpFast[0][chunk_i] - magcal.BpFast[0][chunk_j];
		dy = magcal.BpFast[1][chunk_i] - magcal.BpFast[1][chunk_j];
		dz = magcal.BpFast[2][chunk_i] - magcal.BpFast[2][chunk_j];
		distsq = (int64_t) dx * (int64_t) dx + (int64_t) dy * (int64_t) dy
				+ (int64_t) dz * (int64_t) dz;

		if (distsq < minsum) {
			minsum = distsq;
			minindex = (random() & 1) ? chunk_i : chunk_j;
		}

		chunk_j++;
	}

	return minindex;
}

static void add_magcal_data(const int16_t *data) {

	int i;

	// first look for an unused caldata slot
	for (i = 0; i < MAGBUFFSIZE; i++) {
		if (!magcal.valid[i])
			break;
	}

	if (i >= MAGBUFFSIZE) {
		i = choose_discard_magcal();
		if (i < 0 || i >= MAGBUFFSIZE) {
			i = random() % MAGBUFFSIZE;
		}
	}

	// add it to the cal buffer
	magcal.BpFast[0][i] = data[0];
	magcal.BpFast[1][i] = data[1];
	magcal.BpFast[2][i] = data[2];
	magcal.valid[i] = 1;
}

bool is_mag_moving(float magX, float magY, float magZ) {
	static float prevX = 0.0f, prevY = 0.0f, prevZ = 0.0f;
	static int initialized = 0;

	if (!initialized) {
		prevX = magX;
		prevY = magY;
		prevZ = magZ;
		initialized = 1;
		return true;  // 초기에는 무조건 움직인 것으로 간주
	}

	float deltaX = magX - prevX;
	float deltaY = magY - prevY;
	float deltaZ = magZ - prevZ;

	float deltaMag = sqrtf(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

	// 현재 값을 저장
	prevX = magX;
	prevY = magY;
	prevZ = magZ;

	return deltaMag > MAG_MOVEMENT_THRESHOLD;
}

void process_imu_data(int16_t raw_magX, int16_t raw_magY, int16_t raw_magZ) {

	// 자력계 값 변환 (단위: uT)
	float magX = raw_magX * MAG_SENSITIVITY_4GAUSS;
	float magY = raw_magY * MAG_SENSITIVITY_4GAUSS;
	float magZ = raw_magZ * MAG_SENSITIVITY_4GAUSS;

	// 자력계 변화 기반 움직임 감지
	if (is_mag_moving(magX, magY, magZ)) {
		mag_raw[0] = raw_magX;
		mag_raw[1] = raw_magY;
		mag_raw[2] = raw_magZ;

		add_magcal_data(mag_raw);

		// waitcount 방식 캘리브레이션
		waitcount++;
		if (waitcount >= 20) {
			if (MagCal_Run()) {
				// 캘리브레이션 성공 시 추가 작업 가능
			}
			waitcount = 0;
		}
	}

	//	mag_raw[0] = raw_magX;
//	mag_raw[1] = raw_magY;
//	mag_raw[2] = raw_magZ;
//
//	add_magcal_data(mag_raw);
//
//	waitcount++;
//
//	if (waitcount >= 5) {
//	    if (MagCal_Run()) {
//	    }
//	    waitcount = 0;
//	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

//	DAM_Callback_cnt++;

	if (hi2c->Instance == I2C1) {
		if (CurrentSensor == GYRO_SENSOR) {
			gyroX = (int16_t) ((gyro_buffer[1] << 8) | gyro_buffer[0]);
			gyroY = (int16_t) ((gyro_buffer[3] << 8) | gyro_buffer[2]);
			gyroZ = (int16_t) ((gyro_buffer[5] << 8) | gyro_buffer[4]);

			gyroX_current = gyroX * GYRO_SENSITIVITY_245DPS * DEG2RAD;
			gyroY_current = gyroY * GYRO_SENSITIVITY_245DPS * DEG2RAD;
			gyroZ_current = gyroZ * GYRO_SENSITIVITY_245DPS * DEG2RAD;

//			gyroX_current = gyroX * GYRO_SENSITIVITY_245DPS * DEG2RAD / 1000.0f;
//			gyroY_current = gyroY * GYRO_SENSITIVITY_245DPS * DEG2RAD / 1000.0f;
//			gyroZ_current = gyroZ * GYRO_SENSITIVITY_245DPS * DEG2RAD / 1000.0f;

//			gyroX_current = gyroX * (245.0f * (M_PI / 180.0f) / 32768.0f); // rad/s
//			gyroY_current = gyroY * (245.0f * (M_PI / 180.0f) / 32768.0f); // rad/s
//			gyroZ_current = gyroZ * (245.0f * (M_PI / 180.0f) / 32768.0f); // rad/s

//			gyroX_current = gyroX * GYRO_SENSITIVITY_245DPS ;
//			gyroY_current = gyroY * GYRO_SENSITIVITY_245DPS ;
//			gyroZ_current = gyroZ * GYRO_SENSITIVITY_245DPS ;

			if (DAM_Callback_cnt < 1000) {

				gyroX_sum += gyroX_current;
				gyroY_sum += gyroY_current;
				gyroZ_sum += gyroZ_current;

				DAM_Callback_cnt++;

			} else {

				DAM_Callback_cnt = 10000;

				gyroX_avg = gyroX_sum / 1000.0f;
				gyroY_avg = gyroY_sum / 1000.0f;
				gyroZ_avg = gyroZ_sum / 1000.0f;

				gyro_x = gyroX_current - gyroX_avg;
				gyro_y = gyroY_current - gyroY_avg;
				gyro_z = gyroZ_current - gyroZ_avg;

				gxyz[0] = gyro_x;
				gxyz[1] = gyro_y;
				gxyz[2] = gyro_z;

				CurrentSensor = ACCEL_SENSOR;
				LSM9DS1_ReadAccel_DMA();
			}

		} else if (CurrentSensor == ACCEL_SENSOR) {
			accelX = (int16_t) ((accel_buffer[1] << 8) | accel_buffer[0]);
			accelY = (int16_t) ((accel_buffer[3] << 8) | accel_buffer[2]);
			accelZ = (int16_t) ((accel_buffer[5] << 8) | accel_buffer[4]);

//			accel_x = accelX * ACCEL_SENSITIVITY_2G * 0.001f;
//			accel_y = accelY * ACCEL_SENSITIVITY_2G * 0.001f;
//			accel_z = accelZ * ACCEL_SENSITIVITY_2G * 0.001f;

			accel_x = accelX * (2.0f * 9.81f / 32768.0f); // 2G = 2 * 9.81 m/s², 16비트 = 32768
			accel_y = accelY * (2.0f * 9.81f / 32768.0f);
			accel_z = accelZ * (2.0f * 9.81f / 32768.0f);

			axyz[0] = accel_x;
			axyz[1] = accel_y;
			axyz[2] = accel_z;

			axyz2[0] = axyz[0];
			axyz2[1] = axyz[1];
			axyz2[2] = axyz[2];

			axyz[0] = axyz[0] * 1 / 9.8;
			axyz[1] = axyz[1] * 1 / 9.8;
			axyz[2] = axyz[2] * 1 / 9.8;

			vector_normalize(axyz);

			CurrentSensor = MAG_SENSOR;
			LSM9DS1_ReadMag_DMA();

		} else if (CurrentSensor == MAG_SENSOR) {
			magX = (int16_t) ((mag_buffer[1] << 8) | mag_buffer[0]);
			magY = (int16_t) ((mag_buffer[3] << 8) | mag_buffer[2]);
			magZ = (int16_t) ((mag_buffer[5] << 8) | mag_buffer[4]);

//			mag_x = magX * MAG_SENSITIVITY_4GAUSS * 0.1f;
//			mag_y = magY * MAG_SENSITIVITY_4GAUSS * 0.1f;
//			mag_z = magZ * MAG_SENSITIVITY_4GAUSS * 0.1f;

			mag_x = magX * (4.0f / 32768.0f) * 100;  // uT
			mag_y = magY * (4.0f / 32768.0f) * 100;  // uT
			mag_z = magZ * (4.0f / 32768.0f) * 100;  // uT

			mx1 = mag_x - hardIron_x;
			my1 = mag_y - hardIron_y;
			mz1 = mag_z - hardIron_z;

			float corrected_V[] = { mx1, my1, mz1 };
			float result_V[3];

			result_V[0] = corrected_V[0] * softIron_cali[0][0]
					+ corrected_V[1] * softIron_cali[0][1]
					+ corrected_V[2] * softIron_cali[0][2];
			result_V[1] = corrected_V[0] * softIron_cali[1][0]
					+ corrected_V[1] * softIron_cali[1][1]
					+ corrected_V[2] * softIron_cali[1][2];
			result_V[2] = corrected_V[0] * softIron_cali[2][0]
					+ corrected_V[1] * softIron_cali[2][1]
					+ corrected_V[2] * softIron_cali[2][2];

			mx2 = result_V[0];
			my2 = result_V[1];
			mz2 = result_V[2];

			mxyz[0] = mx2;
			mxyz[1] = my2;
			mxyz[2] = mz2;

			vector_normalize(mxyz);

			axyz1[0] = -axyz[0];
			gxyz1[0] = -gxyz[0];

			Now = micros();
			deltat = (Now - lastUpdate) * 1.0e-6 * 5.0f;
			;
			lastUpdate = Now;

			MadgwickAHRSupdate(gxyz1[0], gxyz[1], gxyz[2], axyz1[0], axyz[1],
					axyz[2], mxyz[0], mxyz[1], mxyz[2]);

			UTIL_SEQ_SetTask(1 << CFG_TASK_MY_TASK_BLE, CFG_SCH_PRIO_0);
			CurrentSensor = GYRO_SENSOR;

		} else if (CurrentSensor = MAG_SENSOR_CAL) {

			DAM_Callback_cnt = 0;

			magX = (int16_t) ((mag_buffer[1] << 8) | mag_buffer[0]);
			magY = (int16_t) ((mag_buffer[3] << 8) | mag_buffer[2]);
			magZ = (int16_t) ((mag_buffer[5] << 8) | mag_buffer[4]);

			mag_x = magX * MAG_SENSITIVITY_4GAUSS;
			mag_y = magY * MAG_SENSITIVITY_4GAUSS;
			mag_z = magZ * MAG_SENSITIVITY_4GAUSS;

			//magcal algorithm
			process_imu_data(mag_x, mag_y, mag_z);
//			magcal_counter++;

//			if (magcal_counter >= 20) {
//				MagCal_Run();
//				magcal_counter = 0;
//			}

			for (i = 0; i < MAGBUFFSIZE; i++) {
				if (magcal.valid[i]) {

					apply_calibration(magcal.BpFast[0][i], magcal.BpFast[1][i],
							magcal.BpFast[2][i], &point);
					quality_update(&point);
				}
			}

			if (gaps <= 1.2 && magcal.FitError < 2.1) {

				hardIron_x = magcal.V[0];
				hardIron_y = magcal.V[1];
				hardIron_z = magcal.V[2];

				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						softIron_cali[i][j] = magcal.invW[i][j];
					}
				}
				CurrentSensor = GYRO_SENSOR;
				LSM9DS1_ReadGyro_DMA();
			} else {
				LSM9DS1_ReadMag_DMA();
			}
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
	MX_APPE_Config();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* IPCC initialisation */
	MX_IPCC_Init();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	MX_TIM2_Init();
	MX_RF_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init code for STM32_WPAN */
	MX_APPE_Init();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	LSM9DS1_Init();
	HAL_Delay(10);

	HAL_TIM_Base_Start(&htim2);

	magcal_Init();

	while (1) {

		/* USER CODE END WHILE */
		MX_APPE_Process();

		/* USER CODE BEGIN 3 */

		//low_power_code
		if (CurrentSensor == MAG_SENSOR_CAL) {
			LSM9DS1_ReadMag_DMA();
		} else {
			LSM9DS1_ReadGyro_DMA();
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2
			| RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS
			| RCC_PERIPHCLK_RFWAKEUP;
	PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
	PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN Smps */

	/* USER CODE END Smps */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00B07CB4;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */
//	__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);
	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IPCC Initialization Function
 * @param None
 * @retval None
 */
static void MX_IPCC_Init(void) {

	/* USER CODE BEGIN IPCC_Init 0 */

	/* USER CODE END IPCC_Init 0 */

	/* USER CODE BEGIN IPCC_Init 1 */

	/* USER CODE END IPCC_Init 1 */
	hipcc.Instance = IPCC;
	if (HAL_IPCC_Init(&hipcc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IPCC_Init 2 */

	/* USER CODE END IPCC_Init 2 */

}

/**
 * @brief RF Initialization Function
 * @param None
 * @retval None
 */
static void MX_RF_Init(void) {

	/* USER CODE BEGIN RF_Init 0 */

	/* USER CODE END RF_Init 0 */

	/* USER CODE BEGIN RF_Init 1 */

	/* USER CODE END RF_Init 1 */
	/* USER CODE BEGIN RF_Init 2 */

	/* USER CODE END RF_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
	hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/** Enable the WakeUp
	 */
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 99;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xffffffff;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	/* DMAMUX1_OVR_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD2_Pin | LD3_Pin | LD1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | LD3_Pin | LD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_USB;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : B2_Pin B3_Pin */
	GPIO_InitStruct.Pin = B2_Pin | B3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
	GPIO_InitStruct.Pin = STLINK_RX_Pin | STLINK_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
