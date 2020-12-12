/**********************************************************************************
  
*@file		BMP280.c
*@author	rohirto
*@version	1.0
*@date 		2/12/2020
*@brief 	BMP280 Sensor library. Connected on I2C1
					Interface with STM32 Addr 0x76 . GPIO init already done.
					Note that BMP280 does not have Humidity Sensor as opposed to BME280
					BMP280 CHIP ID = 0x58
					BME280 CHIP ID = 0x60
**************************************************************************************************************
**/
/* Includes */
#include "main.h"
#include "sensors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "semphr.h"

/* BMP HANDLE*/
BMP280_HandleTypedef bmp280; 
bmp280_params_t params;
float global_pressure, global_temperature_bmp;
int32_t t_fine; //!< temperature with high resolution, stored as an attribute
                  //!< as this is used for temperature compensation reading
                  //!< humidity and pressure
int32_t t_fine_adjust = 0; //!< add to compensate temp readings and in turn
                             //!< to pressure and humidity readings
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;



/**
  * @brief GP2Y101 RTOS Task 
  * @param void Pointer
  * @retval none
	* Gets value from GP2Y101 when AQI_LED is high
  */
void BMP280_Task(void *pvParams)  
{
	int indx = 1;
	uint8_t* debug_messagePtr;
	for(;;)
	{
		/* Mutex to protect printing */
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
		/* Sample BMP280 Data for Temperature and Pressure */
		//if(BMP280_ReadFloat(&bmp280, &global_temperature_bmp, &global_pressure) == HAL_OK)
		if(BMP280_ReadFloat24(&bmp280, &global_temperature_bmp) == HAL_OK) 
		{ 
			/* Give back the Mutex */
			xSemaphoreGive( xI2CMutex ); 
			/* Print Data on Debug */
			sprintf((char*)pcDebugBuffer, "%d. Pressure  = %f \t Temp = %f\n", indx, global_pressure, global_temperature_bmp);
			indx ++;
		}
		else
		{
			/* Print Error */ 
			sprintf((char*)pcDebugBuffer, "BMP Get Data Failed");
		}
		
		Debug_Mutex();  /* Sends the data to the Debug Queue */ 
		/* Run Task for every 5 secs */
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}	

HAL_StatusTypeDef BMP280_Init()
{
	/* Enable the clock */
	//__HAL_I2C_ENABLE(BMP280_SLAVE);
	bmp280_init_default_params(&bmp280.params); 
	bmp280.addr = DEVICE_ADDR; 
	bmp280.i2c = &hi2c1;
	
	/* Check the Chip ID */
	if (BMP_Read_data(&bmp280, BME280_REGISTER_CHIPID, &bmp280.id, 1) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	
	/* Soft Reset the BMP, done mainly to ensure SPI desnt interfere with I2C */
		/* Soft Reset the BMP280 */
	if(BMP280_WriteRegister8(&bmp280, BME280_REGISTER_SOFTRESET, BMP280_RESET_VALUE) == HAL_ERROR) 
	{
		return HAL_ERROR;
	}
	/* Wait for Chip to Wake up */
	HAL_Delay(10);
	// Wait until finished copying over the NVP data.
	while (1) 
	{
		uint8_t status;
		int i = 9999;
		if (BMP_Read_data(&bmp280, BME280_REGISTER_STATUS, &status, 1) == HAL_OK
				&& (status & 1) == 0)
			break;
		else
		{
			i--;
			if( i == 0)
			{
				/* time out */
				return HAL_ERROR;
			}
		}
	}
	HAL_Delay(10);
	
	/* Read Calibration Data */
	if(BMP280_ReadCalibrationDdata(&bmp280) == HAL_ERROR)
	{
		return HAL_ERROR; 
	}
	HAL_Delay(10);
	/* Write configuration */
	uint8_t config = (params.standby << 5) | (params.filter << 2);
	if (BMP280_WriteRegister8(&bmp280, BME280_REGISTER_CONFIG, config) == HAL_ERROR) 
	{
		return HAL_ERROR;
	}
	HAL_Delay(10);
	if (params.mode == BMP280_MODE_FORCED) {
		params.mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}
	uint8_t ctrl = (params.oversampling_temperature << 5)
			| (params.oversampling_pressure << 2) | (params.mode);
	
	if (BMP280_WriteRegister8(&bmp280, BME280_REGISTER_CONTROL, ctrl) == HAL_ERROR) 
	{
		return HAL_ERROR;
	}
	HAL_Delay(100);
	
	return HAL_OK;
}
/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(BMP280_HandleTypedef *dev, int32_t adc_temp,
		int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) dev->dig_T1 << 1)))
			* (int32_t) dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dev->dig_T1)
			* ((adc_temp >> 4) - (int32_t) dev->dig_T1)) >> 12)
			* (int32_t) dev->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return ((*fine_temp * 5 + 128) >> 8);
}
static inline int32_t compensate_temperature24(BMP280_HandleTypedef *dev, int32_t temp24,
		int32_t *fine_temp)
{
	uint32_t var1, var2;
	
	temp24 >>=4;
	var1 = ((((temp24 >> 3) - ((int32_t)dev->dig_P1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
	
  var2 = (((((temp24 >> 4) - ((int32_t)dev->dig_T1)) * ((temp24 >> 4) - ((int32_t)dev->dig_T1))) >>
           12) * ((int32_t)dev->dig_T3)) >> 14;
	
	*fine_temp = var1 + var2 ;
	return ((*fine_temp * 5 + 128) >> 8);

}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(BMP280_HandleTypedef *dev, int32_t adc_press,
		int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t) dev->dig_P5) << 17);
	var2 = var2 + (((int64_t) dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dev->dig_P3) >> 8)
			+ ((var1 * (int64_t) dev->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dev->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dev->dig_P7 << 4);
	return p;
}

HAL_StatusTypeDef BMP280_ReadFixed(BMP280_HandleTypedef *dev, int32_t *temperature, uint32_t *pressure) 
{
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];
	size_t size = 6;

	if(BMP_Read_data(dev, BME280_REGISTER_PRESSUREDATA, data, size) == HAL_ERROR) 
	{
		return HAL_ERROR; 
	}

	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensate_temperature(dev, adc_temp, &fine_temp);
	*pressure = compensate_pressure(dev, adc_pressure, fine_temp);


	return HAL_OK;
}

HAL_StatusTypeDef BMP280_ReadFixed24(BMP280_HandleTypedef *dev, int32_t *temperature)
{
	int32_t var1, var2;
	
	int32_t adc_T; 
	if(BMP_Read_data24(dev, BME280_REGISTER_TEMPDATA, &adc_T) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	if(adc_T == 0x800000)
	{
		return HAL_ERROR;
	}
	adc_T >>= 4;
	var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) *
          ((int32_t)dev->dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >>
           12) *
          ((int32_t)dev->dig_T3)) >>
         14;
	
	t_fine = var1 + var2 + t_fine_adjust;
	
	return HAL_OK;
	
}

HAL_StatusTypeDef BMP280_ReadFloat(BMP280_HandleTypedef *dev, float *temperature, float *pressure)
	{
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	if(BMP280_ReadFixed(dev, &fixed_temperature, &fixed_pressure) == HAL_OK) 
		{
		*temperature = (float) fixed_temperature / 100;
		*pressure = (float) fixed_pressure / 256;
		return HAL_OK;
	}

	return HAL_ERROR;
}
	
HAL_StatusTypeDef BMP280_ReadFloat24(BMP280_HandleTypedef *dev, float *temperature)
	{
	int32_t fixed_temperature;
	//uint32_t fixed_pressure;
	if(BMP280_ReadFixed24(dev, &fixed_temperature) == HAL_OK) 
		{
		*temperature = (t_fine * 5 + 128) >> 8;
		*temperature = *temperature / 100;
		return HAL_OK;
	}

	return HAL_ERROR;
}
HAL_StatusTypeDef BMP280_ReadCalibrationDdata(BMP280_HandleTypedef *dev) {

	if ((BMP_Read_data16(dev, BME280_REGISTER_DIG_T1, &dev->dig_T1) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_T2, (uint16_t *) &dev->dig_T2) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_T3, (uint16_t *) &dev->dig_T3) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P1, &dev->dig_P1) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P2, (uint16_t *) &dev->dig_P2) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P3, (uint16_t *) &dev->dig_P3) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P4, (uint16_t *) &dev->dig_P4) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P5, (uint16_t *) &dev->dig_P5) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P6, (uint16_t *) &dev->dig_P6) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P7, (uint16_t *) &dev->dig_P7) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P8, (uint16_t *) &dev->dig_P8) == HAL_OK)
			&& (BMP_Read_data16(dev, BME280_REGISTER_DIG_P9, (uint16_t *) &dev->dig_P9) == HAL_OK))
	{

		return HAL_OK;
	}

	return HAL_ERROR;
}
void bmp280_init_default_params(bmp280_params_t *params)
{
	params->mode = BMP280_MODE_NORMAL;
	params->filter = BMP280_FILTER_OFF;
	params->oversampling_pressure = BMP280_STANDARD;
	params->oversampling_temperature = BMP280_STANDARD;
	params->oversampling_humidity = BMP280_STANDARD;
	params->standby = BMP280_STANDBY_1000;
}

HAL_StatusTypeDef BMP_Read_data(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t *value,
		uint8_t len) 
{
	uint16_t tx_buff;
	tx_buff = (dev->addr << 1);
	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, value, len, 5000) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
	
}

HAL_StatusTypeDef BMP_Read_data16(BMP280_HandleTypedef *dev, uint8_t addr, uint16_t *value) 
{
	uint16_t tx_buff;
	uint8_t rx_buff[2];
	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, rx_buff, 2, 5000)
			== HAL_OK) 
	{
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return HAL_OK;
	} else
	{
		return HAL_ERROR;
	}

}

HAL_StatusTypeDef BMP280_WriteRegister8(BMP280_HandleTypedef *dev, uint8_t addr, uint8_t value)
{
	uint16_t tx_buff;

	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Write(dev->i2c, tx_buff, addr, 1, &value, 1, 10000) == HAL_ERROR)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
} 


HAL_StatusTypeDef BMP_Read_data24(BMP280_HandleTypedef *dev, uint8_t addr, int32_t *value) 
{
	uint16_t tx_buff;
	uint8_t rx_buff[3];
	tx_buff = (dev->addr << 1);

	if (HAL_I2C_Mem_Read(dev->i2c, tx_buff, addr, 1, rx_buff, 3, 5000)
			== HAL_OK) 
	{
		*value = (uint32_t) ((0x00 << 24)|(rx_buff[2] << 16)|(rx_buff[1] << 8) | rx_buff[0]);
		return HAL_OK;
	} else
	{
		return HAL_ERROR;
	} 

}