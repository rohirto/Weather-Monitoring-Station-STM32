/**
  ******************************************************************************
  * @file           : sensors.h
  * @brief          : Header for all Sensors related stuff
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"
	
/* Sensors Defines */
	/* Comment if Sensor not in Use */
#define DHT11
#define GP2Y101
#define BMP280
	
	
	
#ifdef DHT11
/* DHT 11 Settings */


void DHT11_Start (void);
HAL_StatusTypeDef DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
HAL_StatusTypeDef DHT11_Get_Data (float *Temperature, float *Humidity);
HAL_StatusTypeDef DHT_Waiton_ExpectedState(uint8_t );
#define DHT_TIMER &htim2
#endif
	
#ifdef GP2Y101
/* GP2Y101 Settings */
#define NO_OF_CHANNELS	1
#define GP2Y101_ANALOG	&hadc1
#define GP2Y101_TIMER 	&htim3
#define SAMPLINGTIME		280			/* In Microseconds */
#define DELTATIME				40
#define SLEEPTIME				9680		/* 280+40+9680 = Total Time */
float GP2Y101_GetData( void );
#endif

#ifdef BMP280

#include <stdbool.h>

/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88

#define BMP280_RESET_VALUE     0xB6

/**
 * BMP280 or BME280 address is 0x77 if SDO pin is high, and is 0x76 if
 * SDO pin is low.
 */

#define BMP280_I2C_ADDRESS_0  0x76
#define BMP280_I2C_ADDRESS_1  0x77

#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */
#define BME280_CHIP_ID  0x60 /* BME280 has chip-id 0x60 */

/**
 * Mode of BMP280 module operation.
 * Forced - Measurement is initiated by user.
 * Normal - Continues measurement.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,
    BMP280_MODE_FORCED = 1,
    BMP280_MODE_NORMAL = 3
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED = 0,          /* no measurement  */
    BMP280_ULTRA_LOW_POWER = 1,  /* oversampling x1 */
    BMP280_LOW_POWER = 2,        /* oversampling x2 */
    BMP280_STANDARD = 3,         /* oversampling x4 */
    BMP280_HIGH_RES = 4,         /* oversampling x8 */
    BMP280_ULTRA_HIGH_RES = 5    /* oversampling x16 */
} BMP280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 = 0,      /* stand by time 0.5ms */
    BMP280_STANDBY_62 = 1,      /* stand by time 62.5ms */
    BMP280_STANDBY_125 = 2,     /* stand by time 125ms */
    BMP280_STANDBY_250 = 3,     /* stand by time 250ms */
    BMP280_STANDBY_500 = 4,     /* stand by time 500ms */
    BMP280_STANDBY_1000 = 5,    /* stand by time 1s */
    BMP280_STANDBY_2000 = 6,    /* stand by time 2s BMP280, 10ms BME280 */
    BMP280_STANDBY_4000 = 7,    /* stand by time 4s BMP280, 20ms BME280 */
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;


typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;

    I2C_HandleTypeDef* i2c;

    bmp280_params_t params;

    uint8_t  id;        /* Chip ID */

} BMP280_HandleTypedef;

/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
void bmp280_init_default_params(bmp280_params_t *params);

/**
 * Initialize BMP280 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns true on success otherwise false.
 *
 * The I2C address is assumed to have been initialized in the dev, and
 * may be either BMP280_I2C_ADDRESS_0 or BMP280_I2C_ADDRESS_1. If the I2C
 * address is unknown then try initializing each in turn.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
bool bmp280_init(BMP280_HandleTypedef *dev, bmp280_params_t *params);

/**
 * Start measurement in forced mode.
 * The module remains in forced mode after this call.
 * Do not call this method in normal mode.
 */
bool bmp280_force_measurement(BMP280_HandleTypedef *dev);

/**
 * Check if BMP280 is busy with measuring temperature/pressure.
 * Return true if BMP280 is busy.
 */
bool bmp280_is_measuring(BMP280_HandleTypedef *dev);

/**
 * Read compensated temperature and pressure data:
 *
 *  Temperature in degrees Celsius times 100.
 *
 *  Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity as a fixed point 22 bit interger and 10 bit fraction format.
 */
bool bmp280_read_fixed(BMP280_HandleTypedef *dev, int32_t *temperature,
                       uint32_t *pressure, uint32_t *humidity);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */
bool bmp280_read_float(BMP280_HandleTypedef *dev, float *temperature,
                       float *pressure, float *humidity);
#endif
/* Externs */
extern SemaphoreHandle_t xTimerMutex; 
extern SemaphoreHandle_t xI2CMutex;
	
/* Function Prototypes */
void DHT11_Task(void* pvParams);
void GP2Y101_Task(void *pvParams); 
void BMP280_Task(void *pvParams); 
HAL_StatusTypeDef GP2Y101_Init(ADC_HandleTypeDef* hadc);
void bmp280_init_default_params(bmp280_params_t *params);
HAL_StatusTypeDef BMP_Read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len);
HAL_StatusTypeDef BMP_Read_data16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value);
HAL_StatusTypeDef BMP280_WriteRegister8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value); 
HAL_StatusTypeDef BMP280_Init(void);	
HAL_StatusTypeDef BMP280_ReadCalibrationDdata(BMP280_HandleTypedef *dev);
HAL_StatusTypeDef BMP280_ReadFloat(BMP280_HandleTypedef *dev, float *temperature, float *pressure);	
HAL_StatusTypeDef BMP280_ReadFloat24(BMP280_HandleTypedef *dev, float *temperature);
HAL_StatusTypeDef BMP_Read_data24(BMP280_HandleTypedef *dev, uint8_t addr, int32_t *value);
#endif
	
	
	
	