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
extern TIM_HandleTypeDef htim2, htim3;
/* Function Prototypes */
void RTOS(void);  /* Function call from the main */
void togglePin(void* pvParameters);

/* Private Global Variables */
//uint8_t tx_string[] = "Hello World";

/* Task Handler */
xTaskHandle DHT11_Task_Handler;
xSemaphoreHandle DHT_SEM;
xSemaphoreHandle GP2Y101_SEM;
SemaphoreHandle_t xTimerMutex;
xTaskHandle GP2Y101_Task_Handler;

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
	xDebugQueueMutex = xSemaphoreCreateMutex();
	xUART1Mutex = xSemaphoreCreateMutex();	
	xTimerMutex = xSemaphoreCreateMutex();
	
	if( (xDebugQueue != NULL) & (xDebugQueueMutex != NULL) & (xUART1Mutex != NULL) &
			(xTimerMutex != NULL))
	{
		DHT_SEM = xSemaphoreCreateBinary();
		/* Simple blinky Example */
		xTaskCreate(togglePin, "TogglePin", 128, NULL, 2, NULL); 
		xTaskCreate(DHT11_Task , "DHT11", 128, NULL, 2, &DHT11_Task_Handler);  
		xTaskCreate(prvDebug_Task, "Debug Task" ,128, NULL, 1, NULL); 
		xTaskCreate(GP2Y101_Task, "GP2Y101", 128, NULL, 2,  &GP2Y101_Task_Handler); 
		/* Start the DHT11 Timer*/
		HAL_TIM_Base_Start(&htim2);
		HAL_TIM_Base_Start(&htim3);
			
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
	for(;;)
	{
		HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
		vTaskDelay(pdMS_TO_TICKS(5000));
	} 
}
