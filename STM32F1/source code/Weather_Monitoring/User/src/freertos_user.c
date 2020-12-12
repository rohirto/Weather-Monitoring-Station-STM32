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
xTaskHandle BMP280_Task_Handler;
xSemaphoreHandle DHT_SEM;
xSemaphoreHandle GP2Y101_SEM;
SemaphoreHandle_t xMu_ADC1_MB;
SemaphoreHandle_t xTimerMutex;
xTaskHandle GP2Y101_Task_Handler;
SemaphoreHandle_t xI2CMutex;
BaseType_t xStatus;
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
	xI2CMutex = xSemaphoreCreateMutex();
	xMu_ADC1_MB = xSemaphoreCreateMutex();
	unsigned char* pcMessage = pcDebugBuffer;
	
	if( (xDebugQueue != NULL) & (xDebugQueueMutex != NULL) & (xUART1Mutex != NULL) &
			(xTimerMutex != NULL) & (xI2CMutex != NULL) & (xMu_ADC1_MB != NULL))
	{
		/* Simple blinky Example */
//		if(xTaskCreate(togglePin, "TogglePin", 128, NULL, 2, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
//		{
//			sprintf((char *)pcDebugBuffer, "Toogle Task failed  \n");
//		}	
		if(xTaskCreate(DHT11_Task , "DHT11", 128, NULL, 2, &DHT11_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "DHT11_Task failed  \n");
		}
		else if(xTaskCreate(BMP280_Task, "BMP280", 128, NULL, 2, &BMP280_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "BMP280 failed  \n");
		}
		else if(xTaskCreate(prvDebug_Task, "Debug Task" ,128, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "Debug Task failed  \n");
		}
		else if(xTaskCreate(GP2Y101_Task, "GP2Y101", 128, NULL, 2,  &GP2Y101_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "GP2Y101 failed  \n");
		}
		
		else
		{
			sprintf((char *)pcDebugBuffer, "All tasks created successfully!  \n");
		}
		HAL_UART_Transmit(&huart1,(uint8_t*) pcMessage, strlen((const char *)pcMessage), 100);
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
