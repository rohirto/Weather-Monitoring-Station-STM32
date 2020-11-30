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
	
	
/* Sensors Defines */
	/* Comment if Sensor not in Use */
#define DHT11
#ifdef DHT11
/* DHT 11 Settings */
#include "stm32f1xx_hal.h"
void DHT_Delay (uint16_t time);
void DHT11_Start (void);
HAL_StatusTypeDef DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
HAL_StatusTypeDef DHT11_Get_Data (float *Temperature, float *Humidity);
#endif
/* Externs */

	
/* Function Prototypes */
void DHT11_Task(void* pvParams);
	
	
#endif

	