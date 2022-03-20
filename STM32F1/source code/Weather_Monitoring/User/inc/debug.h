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
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "croutine.h"
#include "list.h"
#include "FreeRTOSConfig.h"

/* User Defines */
#define DEBUG		1 	/* Enables DEBUG on UART1 Port */


/* User Macros */
/* This Macro takes the debug queue mutex, writes to debug queue and then releases the mutex */
#define Debug_Mutex()           		xSemaphoreTake(xDebugQueueMutex, 0);\
                                    /* Store the address of string buffer (to be printed) in pointer variable */\
                                    debug_messagePtr = pcDebugBuffer;\
                                    /* Write to Queue*/\
                                    xQueueSendToBack(xDebugQueue, &debug_messagePtr, 0 );\
                                    /* Give back the Mutex */\
                                    xSemaphoreGive( xDebugQueueMutex );
/* Externs */
extern QueueHandle_t xDebugQueue;
extern uint8_t pcDebugBuffer[50]; 
extern SemaphoreHandle_t xDebugQueueMutex; 
extern SemaphoreHandle_t xUART1Mutex;
/* Function Prototypes */
void prvDebug_Task (void* pvParameters);
//void System_Status(void *pvParameters);
void Debug_Print(unsigned char *pcMessage);
#ifdef __cplusplus
extern "C" {
#endif
	
#endif

	