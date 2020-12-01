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

/* Externs */
extern SemaphoreHandle_t xTimerMutex; 
	
/* Function Prototypes */
void DHT11_Task(void* pvParams);
void GP2Y101_Task(void *pvParams);
HAL_StatusTypeDef GP2Y101_Init(ADC_HandleTypeDef* hadc);
	
	
#endif

	