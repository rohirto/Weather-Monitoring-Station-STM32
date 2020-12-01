/**********************************************************************************
  
*@file		debug.c
*@author	rohirto
*@version	1.0
*@date 		30/11/2020
*@brief 	UART Debug Messages are dumped through this file 
**************************************************************************************************************
**/

/* Includes */
#include "main.h"
#include "debug.h"


/* Global Variables */
uint8_t pcDebugBuffer[50];
QueueHandle_t xDebugQueue; 
SemaphoreHandle_t xDebugQueueMutex; 
SemaphoreHandle_t xUART1Mutex;

/* Exten handles */
extern UART_HandleTypeDef huart1;

/* Function PTs */
void Debug_Print(unsigned char *);

#ifdef DEBUG
void prvDebug_Task (void* pvParameters)
{
	unsigned char* pcMessage;
	for(;;)
    {
			if(xQueueReceive(xDebugQueue, &pcMessage, 0) == pdPASS)
      {
				/* Look up the event code */
				Debug_Print( pcMessage );
				
				/* Lower its priority*/
			  // vTaskPrioritySet(NULL, 1);
      }
			else
			{
					/* Lower its priority*/
					//vTaskPrioritySet(NULL, 1);
			}
        
    }
}

/**
* @brief Debug_Print 
* @param pointer to message string
* @retval None
* Function to print debug messages sent from different tasks
* 
*/ 
void Debug_Print(unsigned char *pcMessage) 
{
	/* Mutex to protect printing */
    xSemaphoreTake(xUART1Mutex, portMAX_DELAY);
	//if(huart1.gState == HAL_UART_STATE_READY)
	
		HAL_UART_Transmit(&huart1, (uint8_t*) pcMessage, strlen((const char *)pcMessage), 1000);
    //huart1.uartBytereceived = RESET;
	/* Give back the Mutex */
    xSemaphoreGive( xUART1Mutex ); 
	
}
#endif
