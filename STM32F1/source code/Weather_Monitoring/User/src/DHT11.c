/**********************************************************************************
  
*@file		DHT11.c
*@author	rohirto
*@version	1.0
*@date 		29/11/2020
*@brief 	DHT11 Sensor library. According to the datasheet
					it is based on Single Bus Transfer. GPIO init already done.
**************************************************************************************************************
**/
/* Includes */
#include "main.h"
#include "sensors.h"
#include "FreeRTOS.h"
#include "task.h"



/* Global Variables */
float Temperature = 0, Humidity = 0;


void DHT11_Get_data(void* pvParameters);


/**
  * @brief Configure DHT11 in Output mode 
  * @param None
  * @retval None 
	*
  */
void Set_Pin_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : DHT11_DATAIN_Pin */
  GPIO_InitStruct.Pin = DHT11_DATAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_DATAIN_GPIO_Port, &GPIO_InitStruct);
}
/**
  * @brief Configure DHT11 in Input mode 
  * @param None
  * @retval None 
	*
  */
void Set_Pin_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : DHT11_DATAIN_Pin */
  GPIO_InitStruct.Pin = DHT11_DATAIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_DATAIN_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief DHT11 Check response
  * @param None
  * @retval HAL Status  
	*
	* Checks if Sensor is connected or not.
	* HAL_OK: Connected 
	
  */
HAL_StatusTypeDef DHT11_Check_Response(void)
{
	int i = 0;
	/* Wait for 40us */
	/* 48MHz x 40us = 1920 Ticks */
	for( i = 0 ; i < 1920; i++);
	/* DHT11 will send ACK*/
	if(!(HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)))
	{
		/* Again wait 80 us*/
		for( i = 0 ; i < 3840 ; i++);
		if(HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_SET)
		{
			/* OK */
		}
		else
		{
			/* Not OK */
			return HAL_ERROR;
		}
		/* Wait for Pin to go low */
		while(HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_SET);
		
		
	}
	return HAL_OK;
}

/**
  * @brief DHT11 Read Data function
  * @param None
  * @retval the read 8 bit data  
	*
	* Data Frame from DHT-11 
	* The output given out by the data pin will be in the order of 
	* 8bit humidity integer data + 
	* 8bit the Humidity decimal data +
	* 8 bit temperature integer data + 
	* 8bit fractional temperature data +
	* 8 bit parity bit.
	
  */
uint8_t DHT11_Read(void)
{
	uint8_t i,j;
	int a;
	for(j = 0; j<8; j++)
	{
		/* wait for pin to go high */
		while(HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_RESET);
		/* 40 us delay */
		for( a = 0 ; a < 1920; a++);
		
		/* If Pin is low state*/
		if(HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_RESET)
		{
			i&= ~(1<<(7-j));  //if pin low, Write 0
		}
		else
		{
			i|= (1<<(7-j));	//Pin high, WRITE 1 
		}
		
		while((HAL_GPIO_ReadPin(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)) == GPIO_PIN_SET);
	}
	return i;
		
}

void DHT11_Start (void)
{
	int i;
	Set_Pin_Output ();  // set the pin as output
	HAL_GPIO_WritePin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin, 0);   // pull the pin low
	for(i = 0; i < 960000; i++);
    HAL_GPIO_WritePin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin, 1);   // pull the pin high
	for(i = 0; i < 960000; i++);
	Set_Pin_Input();    // set as input
}

/**
  * @brief DHT11 Get Data function
  * @param void Pointer
  * @retval none 
	* It is a task of RTOS schedulling
	*/
void DHT11_Get_data(void* pvParameters)
{
	uint8_t humidity_byte1,humidity_byte2, temperature_byte1, temperature_byte2, checksum;
	uint16_t TEMP, HUM, SUM;
	for(;;)
	{
		DHT11_Start();
		
		if(DHT11_Check_Response() == HAL_OK)
		{
			/* Get Humidity and Temperature Data */
			humidity_byte1 = DHT11_Read();
			humidity_byte2 = DHT11_Read();
			temperature_byte1 = DHT11_Read();
			temperature_byte2 = DHT11_Read();
			checksum = DHT11_Read();
			
			TEMP = ((temperature_byte1<<8)|temperature_byte2);
			HUM = ((humidity_byte1<<8)|humidity_byte2);
			
			Temperature = (float) (TEMP/10.0);
			Humidity = (float) (HUM/10.0);
		}
		else
		{
			/* Error from DHT11 */
			
		}
		
		#ifdef DEBUG
		sprintf((char *)pcDebugBuffer, "Temperature: %f\n", Temperature);  
		debug_messagePtr = pcDebugBuffer;
		xQueueSendToBack(xDebugQueue, &debug_messagePtr, 0 );
		#endif
	}
	
}