/**********************************************************************************
  
*@file		GP2Y101.c
*@author	rohirto
*@version	1.0
*@date 		30/11/2020
*@brief 	Source for Sensor Handling of Sensor GP2Y101 Optical Dust Sensor
*					Connected to PA1 ADC1. Requires LED (PA2) to be high to give output
					Conditions: Sensitivity: Typical 0.5V per 0.1mg/m3
					Voltage at No dust: Typical 0.9V. GPIO Init already done
**************************************************************************************************************
**/

#include "main.h"
#include "sensors.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


uint32_t ADC_Store; 

extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

void AQI_LED_ON(void );
void AQI_LED_OFF(void );
void GP2Y101_Delay (uint16_t time);

/* RTOS */
extern xSemaphoreHandle GP2Y101_SEM;
extern SemaphoreHandle_t xMu_ADC1_MB;

/* Sensor related Global Variables */



/**
  * @brief GP2Y101 RTOS Task 
  * @param void Pointer
  * @retval none
	* Gets value from GP2Y101 when AQI_LED is high
  */
void GP2Y101_Task(void *pvParams) 
{
	uint8_t* debug_messagePtr;
	float AQI = 0;
	int indx = 1;
	for(;;)
	{

		/* Get the data */
		AQI = GP2Y101_GetData();
		sprintf((char*)pcDebugBuffer, "%d. AQI = %f ug/m3\n", indx, AQI);
		indx++;
		Debug_Mutex();  /* Sends the data to the Debug Queue */ 
		/* Run Task for every 5 secs */
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}


void AQI_LED_ON()
{
	HAL_GPIO_WritePin(AQI_DIGITAL_LED_GPIO_Port, AQI_DIGITAL_LED_Pin, GPIO_PIN_SET);
}
void AQI_LED_OFF()
{
	HAL_GPIO_WritePin(AQI_DIGITAL_LED_GPIO_Port, AQI_DIGITAL_LED_Pin, GPIO_PIN_RESET);
}



float GP2Y101_GetData( void ) 
{
	float voltageMeasured = 0, calcVoltage = 0, dustDensity = 0;
	/* Take the Mutex */
	xSemaphoreTake(xMu_ADC1_MB, portMAX_DELAY);
	/* Turn On the  LED*/
	AQI_LED_ON();
	GP2Y101_Delay(SAMPLINGTIME);    
	/* Sample value */
	//HAL_ADC_Start_DMA(GP2Y101_ANALOG, &ADC_Store, NO_OF_CHANNELS);
	/* Without DMA */
	
	HAL_ADC_Start(GP2Y101_ANALOG);
	ADC_Store = HAL_ADC_GetValue(GP2Y101_ANALOG);
	HAL_ADC_Stop(GP2Y101_ANALOG);
	
	GP2Y101_Delay(DELTATIME);
	/* Turn Off the LED */
	AQI_LED_OFF(); 
	GP2Y101_Delay(SLEEPTIME);
	
	/* Give back the Mutex */
	xSemaphoreGive( xMu_ADC1_MB );
	/* Assuming ADC to be in 12 Bit Mode */
	/* Can check ADC1 CFGR1 Register */
	voltageMeasured = ADC_Store;
	calcVoltage = voltageMeasured * (5 / 1024.0); 
	dustDensity = 170 * calcVoltage - 0.1;  //in microgram per meter cube
	
	return dustDensity;
}



/**
  * @brief GP2Y101 Initialization and Start
  * @param ADC Handle
  * @retval HAL Status
  */
HAL_StatusTypeDef GP2Y101_Init(ADC_HandleTypeDef* hadc) 
{
	HAL_StatusTypeDef Status;
	/* Enable the Clock */
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	/* Calibrate The ADC */
	Status = HAL_ADCEx_Calibration_Start(hadc);
	if(Status == HAL_OK)
	{
		/* Start The DMA */
		//Status = HAL_ADC_Start_DMA(&hadc, ADC_Array, NO_OF_CHANNELS);
	}
	return Status;
	
}

/**
  * @brief GP2Y101 Delay 
  * @param None
  * @retval None 
	*	GP2Y101 requires microseconds delay, Using Timer 3 as a delay Generator
	* Timer 3: Clock Source 48MHz, Prescaler: 48 Thus 1 tick of 1us 
  */
void GP2Y101_Delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(GP2Y101_TIMER, 0);
	while ((__HAL_TIM_GET_COUNTER(GP2Y101_TIMER))<time);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
    
}