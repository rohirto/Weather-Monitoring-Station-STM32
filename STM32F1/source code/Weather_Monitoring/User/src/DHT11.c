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
#include "debug.h"
#include "semphr.h"

extern TIM_HandleTypeDef htim2;


/* Global Variables */
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM;
uint16_t TEMP, RH;
float Temperature, Humidity;


/* RTOS */
extern xSemaphoreHandle DHT_SEM;
/**
  * @brief DHT11 FreeRTOS sampling task
  * @param void pointer
  * @retval none 
	* RTOS task for sampling DHT11 data and sending it
	*/
void DHT11_Task(void* pvParams)
{
	int indx = 1;
	uint8_t* debug_messagePtr;
	for(;;)
	{
		//if(xSemaphoreTake(DHT_SEM, 2500) != pdTRUE)
		if(0)
		{
			sprintf((char *)pcDebugBuffer, "Semaphore DHT Failed");
			Debug_Mutex();  /* Sends the data to the Debug Queue */ 
		}
		else
		{
			if(DHT11_Get_Data(&Temperature, &Humidity) == HAL_OK) 
			{
				sprintf((char*)pcDebugBuffer, "%d. Temp = %f C\t RH = %f \n", indx, Temperature, Humidity);
				Debug_Mutex();  /* Sends the data to the Debug Queue */ 
				indx++;
			}
			else
			{
				sprintf((char*)pcDebugBuffer, "Get Data Error");
				Debug_Mutex();  /* Sends the data to the Debug Queue */ 
			}
				
		}
		//xSemaphoreGive(DHT_SEM);
		/* Run Task for every 5 secs */
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}


/**
  * @brief DHT Delay 
  * @param None
  * @retval None 
	*	DHT11 requires microseconds delay, Using Timer 2 as a delay Generator
	* Timer 2: Clock Source 48MHz, Prescaler: 48 Thus 1 tick of 1us 
  */
void DHT_Delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(DHT_TIMER, 0);
	while ((__HAL_TIM_GET_COUNTER(DHT_TIMER))<time);
}


/**
  * @brief Configure DHT11 in Output mode 
  * @param GPIO config struct, GPIO Port and Pin
  * @retval None 
	*
  */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief Configure DHT11 in Input mode 
  * @param GPIO config struct, GPIO Port and Pin
  * @retval None 
	*
  */
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief DHT11 Check response
  * @param None
  * @retval HAL Status  
	*
	* Checks if Sensor is connected or not.
	* HAL_OK: Connected 
	
  */
HAL_StatusTypeDef DHT11_Check_Response (void) 
{
	HAL_StatusTypeDef Response = HAL_ERROR;
	DHT_Delay(40); 
	if (!(HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)))
	{
		DHT_Delay(80);
		if ((HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin))) Response = HAL_OK;
		else Response = HAL_ERROR;
	}
	//while ((HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)));	// wait for the pin to go low
	if(DHT_Waiton_ExpectedState(0) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return Response;
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
uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		//while (!(HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)));   // wait for the pin to go high
		if(DHT_Waiton_ExpectedState(1) == HAL_ERROR)
		{
			return 0xFF;
		}
		DHT_Delay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		//while ((HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin)));  // wait for the pin to go low
		if(DHT_Waiton_ExpectedState(0) == HAL_ERROR)
		{
			return 0xFF;
		}
	}
	return i;
}

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_DATAIN_GPIO_Port , DHT11_DATAIN_Pin);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin, GPIO_PIN_RESET);   // pull the pin low
	DHT_Delay(18000);   // wait for 18ms
  HAL_GPIO_WritePin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin, GPIO_PIN_SET);   // pull the pin high
  DHT_Delay(20);   // wait for 30us
	Set_Pin_Input(DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin);    // set as input
}

/**
  * @brief DHT11 Get Data function
  * @param Pass by reference values of Temperature and Humidity
  * @retval none 
	* It is a task of RTOS schedulling
	*/
HAL_StatusTypeDef DHT11_Get_Data (float *Temperature, float *Humidity)
{
  DHT11_Start ();
	if (DHT11_Check_Response () == HAL_OK)
	{
		Rh_byte1 = DHT11_Read ();
		Rh_byte2 = DHT11_Read ();
		Temp_byte1 = DHT11_Read ();
		Temp_byte2 = DHT11_Read ();
		SUM = DHT11_Read();
		if ((SUM | Rh_byte1 | Rh_byte2 | Temp_byte1 | Temp_byte2 | SUM) == 0xFF)
		{
			return HAL_ERROR;
		}
		if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
		{
			TEMP = (Temp_byte1 << 8) |Temp_byte2;
		   RH = (Rh_byte1 << 8) |Rh_byte2;
		}

		else 
		{
			return HAL_ERROR;
		}
	}
	else
	{
		return HAL_ERROR;
	}

    *Temperature = (TEMP >> 8) + ((TEMP & 0x00FF) * 0.1);
    *Humidity = (RH >> 8) + ((RH & 0x00FF) * 0.1);;
		
		#ifdef DEBUG

		#endif
		
		return HAL_OK;
}
	
/**
  * @brief DHT11 Wait on Expected State of Pin
  * @param Expected State HIGH OR LOW
  * @retval Status if expected State occured or not 
	* While loops without any breaks might be dangerous
	*/
HAL_StatusTypeDef DHT_Waiton_ExpectedState(uint8_t ExpectedState)
{
	/* Expect The Pin to be in LOW State */
	if(ExpectedState == 0)
	{
		int i = 9999;
		while((HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_SET))
		{
			/* Wait till expected state is achieved */
			/* If Time out return HAL ERROR */
			i--;
			if( i == 0)
			{
				return HAL_ERROR;
			}
			
		}
		return HAL_OK;
	}
	else
	{
		/* Expect The Pin to be on High State*/
		int i = 9999;
		while((HAL_GPIO_ReadPin (DHT11_DATAIN_GPIO_Port, DHT11_DATAIN_Pin) == GPIO_PIN_RESET))
		{
			/* Wait till expected state is achieved */
			/* If Time out return HAL ERROR */
			i--;
			if( i == 0)
			{
				return HAL_ERROR;
			}
			
		}
		return HAL_OK;
	}
	
}