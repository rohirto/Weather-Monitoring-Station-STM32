/**
  ******************************************************************************
  * @file           : debug.h
  * @brief          : Header for UART Debug file also other debugging related stuff
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
#ifndef __DEBUG_H
#define __DEBUG_H

/* Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* User Defines */
#define DEBUG		1 	/* Enables DEBUG on UART1 Port */

/* Externs */
extern QueueHandle_t xDebugQueue;
extern uint8_t pcDebugBuffer[50];

/* Function Prototypes */
void prvDebug_Task (void* pvParameters);
void Debug_Print(unsigned char *pcMessage);
#ifdef __cplusplus
extern "C" {
#endif
	
#endif

	