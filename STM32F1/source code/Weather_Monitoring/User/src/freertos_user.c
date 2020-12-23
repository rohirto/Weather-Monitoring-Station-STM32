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
#include "mqtt_client.h"
#include "UartRingBuffer_multi.h"

/* FreeRTOS Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

/* Exten handles */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2; 
extern TIM_HandleTypeDef htim3;
extern ESP_handle wifi_module;
/* Function Prototypes */
void RTOS(void);  /* Function call from the main */
void togglePin(void* pvParameters);
void vApplicationIdleHook(void); 

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
xTaskHandle CommCallback_Task_Handler;
xTaskHandle SendData_Handler;
xTaskHandle Error_Task_Handler;
xTaskHandle Publish_Task_Handler;
extern SemaphoreHandle_t xUART3Mutex;
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
	xUART3Mutex = xSemaphoreCreateMutex();	
	xTimerMutex = xSemaphoreCreateMutex();
	xI2CMutex = xSemaphoreCreateMutex();
	xMu_ADC1_MB = xSemaphoreCreateMutex();
	unsigned char* pcMessage = pcDebugBuffer;
	
	if( (xDebugQueue != NULL) & (xDebugQueueMutex != NULL) & (xUART1Mutex != NULL) &
			(xTimerMutex != NULL) & (xI2CMutex != NULL) & (xMu_ADC1_MB != NULL) & (xUART3Mutex != NULL))
	{
		/* Simple blinky Example */
//		if(xTaskCreate(MQTT_Ping_Task, "PingTask", 64, NULL, 1, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
//		{
//			sprintf((char *)pcDebugBuffer, "Ping Task failed \n");
//		}	
		if(xTaskCreate(DHT11_Task , "DHT11", 128, NULL, 1, &DHT11_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "DHT11_Task failed  \n");
		}
		else if(xTaskCreate(CommCBTask,"CommCB",1024, NULL, 1, &CommCallback_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "CommCB Failed  \n");
		}
		else if(xTaskCreate(TCP_SendData_Task,"SendData", 64, NULL, 1, &SendData_Handler)== errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "Update Cntxt Failed  \n");
		}
		else if(xTaskCreate(BMP280_Task, "BMP280", 128, NULL, 0, &BMP280_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "BMP280 failed  \n");
		}
		else if(xTaskCreate(prvDebug_Task, "Debug Task" ,128, NULL, 0, NULL) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "Debug Task failed  \n");
		}
		else if(xTaskCreate(GP2Y101_Task, "GP2Y101", 128, NULL, 0,  &GP2Y101_Task_Handler) == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "GP2Y101 failed  \n");
		}
		
		else if(xTaskCreate(MQTT_Init_Task,"MQTTInit", 512, NULL, 2, NULL)== errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "MQTT Init Failed  \n");
		}
		else if(xTaskCreate(MQTT_Publish_Task,"PublishTask", 128, NULL, 2, &Publish_Task_Handler)== errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
		{
			sprintf((char *)pcDebugBuffer, "PublishTaskFailed\n");
		}
//		else if(xTaskCreate(ESP_Error_Handler_Task,"ErrorHanle", 64, NULL, 2, &Error_Task_Handler)== errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY)
//		{
//			sprintf((char *)pcDebugBuffer, "ErrorHandlerCreateFailed\n");
//		}
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


///**
//  * @brief Toggle Function FreeRTOS
//  * @retval None
//  */
//void togglePin(void* pvParameters)
//{
//	for(;;)
//	{
//		HAL_GPIO_TogglePin(BUILTIN_LED_GPIO_Port, BUILTIN_LED_Pin);
//		vTaskDelay(pdMS_TO_TICKS(5000));
//	} 
//}
/**
  * @brief Idle task Hook Function
  * @retval None
  */
void vApplicationIdleHook(void)
{
	/* When Data is available from ESP the create a Event */
	if(IsDataAvailable(device_uart) && (wifi_module.send_data_flag == false))
	{
		xTaskNotify(CommCallback_Task_Handler,0,eNoAction);			
	}
	if(wifi_module.error_flag == true)
	{
		/* Notify Error Handler */
		//xTaskNotify(Error_Task_Handler,0,eNoAction);
	}
	if((wifi_module.send_data_flag == true) && (wifi_module.tcp_connected == true))
	{
		if(wifi_module.command_code == CIPSEND_ENTERDATA)
		{
			/* Notify Send Data Task */
			xTaskNotify(SendData_Handler,0,eNoAction);
		}			
	}
	if(wifi_module.tcp_connected == true && (wifi_module.command_code == IDLE || wifi_module.command_code == CIPSEND_RECV) && 
		 wifi_module.error_flag == false && wifi_module.respone_expected == false )
	{
		wifi_module.recieve_flag = true;
	}
	else
	{
		wifi_module.recieve_flag = false;
	}
	if(wifi_module.mqtt_connected == true && wifi_module.error_flag != true 
		&& wifi_module.publish_flag == true)
		{
			xTaskNotify(Publish_Task_Handler,0,eNoAction);
		}
	if(wifi_module.mqtt_connected == true && wifi_module.error_flag != true
		&& wifi_module.publish_indx == IDLE_INDEX) 
	{
		wifi_module.publish_indx = TEMPERATURE_INDEX;
	}
//			 
//		if(wifi_module. == 0) /* If in Data mode */
//	{
//		xTaskNotify(SendData_Handler,0,eNoAction); 
//	}

//		xTaskNotify(CommCallback_Task_Handler,0,eNoAction); 
//	}
//	
	
}