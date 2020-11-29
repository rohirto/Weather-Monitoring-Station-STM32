/**********************************************************************************
  
*@file		freertos.c
*@author	rohirto
*@version	1.0
*@date 		29/11/2020
*@brief 	All the tasks and Buisness Logic are implemented here
**************************************************************************************************************
**/
/* Includes */
#include "main.h"
#include "sensors.h"
#include "debug.h"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* Exten handles */
extern UART_HandleTypeDef huart1;
/* Function Prototypes */
void RTOS(void);  /* Function call from the main */
void togglePin(void* pvParameters);

/* Private Global Variables */
//uint8_t tx_string[] = "Hello World";


/**
* @brief RTOS entry code 
* @param none
* @retval none
* It is called from main after intialization of all peripherals.
* RTOS tasks are created and scheduler is started in this function
*/
void RTOS()
{
	xDebugQueue = xQueueCreate( 10, 4);
	
	if( (xDebugQueue != NULL))
	{
		/* Simple blinky Example */
		xTaskCreate(togglePin, "TogglePin", 128, NULL, 2, NULL); 
		xTaskCreate(DHT11_Get_data , "DHT11Data", 400, NULL, 2, NULL);  
		xTaskCreate(prvDebug_Task, "Debug Task" ,128, NULL, 2, NULL); 
		
		
		
		/* Start The Scheduler */
		vTaskStartScheduler();
	}
}


/**
  * @brief Toggle Function FreeRTOS
  * @retval None
  */
void togglePin(void* pvParameters)
{
	#ifdef DEBUG
	uint8_t* debug_messagePtr;
	#endif
	for(;;)
	{
		HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
		vTaskDelay(pdMS_TO_TICKS(5000));
		//HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_string, sizeof(tx_string));
	} 
}
