/**********************************************************************************
  
*@file		mqtt_client.c
*@author	rohirto
*@version	1.0
*@date 		14/12/2020
*@brief 	MQTT Client Library. MQTT over TCP. Uses underlying WiFiComm layer for TCP Access
**************************************************************************************************************
**/
/* Inculdes */
#include "main.h"
#include "mqtt_client.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"


/* Externs */
extern ESP_handle wifi_module;

uint32_t data_len; 
uint8_t encodedByte;
int X;
int8_t MQTT_Protocol_len, ClientID_len, username_len, pass_len, topic_len;
char MQTT_packetID;


/**
* @brief MQTT Init Task 
* @param void pointer RTOS Task
* @retval None
* MQTT initialization, also used to reconnect 
* 
*/ 
void MQTT_Init_Task(void* pvParams)
{
	
	for(;;)
	{
		if((wifi_module.mqtt_connected == false) && (wifi_module.error_flag!= true) && wifi_module.publish_flag == false)
		{
			
			wifi_module.mqtt_port = MQTT_PORT;
			wifi_module.keep_alive = 1;
			if(!strcmp(wifi_module.mqtt_broker, ""))
			{			
				strcpy(wifi_module.mqtt_broker, MQTT_BROKER);
			}
			if(mqtt_connect(&wifi_module) == HAL_ERROR) 
			{
				wifi_module.error_flag = true;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
		
}

/**
* @brief MQTT Publish Task 
* @param void pointer RTOS Task
* @retval None
* MQTT Publish 
* 
*/ 
void MQTT_Publish_Task(void* pvParams)
{
	
	for(;;)
	{
		/* Wait for Task Notifica*/
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(wifi_module.mqtt_connected == true && wifi_module.publish_flag == true
			&& wifi_module.error_flag!= true && wifi_module.respone_expected == false)
			{
				if(mqtt_publish(&wifi_module) != HAL_OK)
				{
					//Error
					wifi_module.error_flag = true;
				}
			}
	}
}

void MQTT_Ping_Task(void *pvParams)
{
	for(;;)
	{
		if((wifi_module.mqtt_connected == true) && (wifi_module.error_flag!=true))
		{
			/* Send Ping after 30 seconds */
			memset((char*)wifi_module.data_tx_buf,0,TX_MAX_SIZE);
			sprintf((char*)wifi_module.data_tx_buf,"Ping");
			if(tcp_send(&wifi_module) != HAL_OK)  
			{
				//Error 
			}
		}
		vTaskDelay(30000);
	}
}

HAL_StatusTypeDef mqtt_connect(ESP_handle *wfcntxt)
{
	/* Send MQTT packet */
	wfcntxt->mqtt_lvl = MQTT_LVL;
	wfcntxt->mqtt_flags = MQTT_FLAGS;
	wfcntxt->mqtt_qos = MQTT_QOS;
	MQTT_packetID = 0x0001;
	wfcntxt->mqtt_keep_alive = MQTT_KEEPALIVE;
	data_len = 0;
	
	MQTT_Protocol_len = strlen(MQTT_PROTOCOL_NAME);
	ClientID_len = strlen(MQTT_CLIENT_ID);
	username_len = strlen(MQTT_USERNAME);
	pass_len = strlen(MQTT_PASSWORD);
	
	if( wfcntxt->command_code == CIPSTART_START)
	{
		/* Clear len fields */
		wfcntxt->received_len = 0;
		wfcntxt->send_len = 0;
		if(tcp_connect(wfcntxt) != HAL_OK)
		{
			return HAL_ERROR;
		}
		/* Small Delay */
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	if(wfcntxt->command_code == CIPSTART_OK && wfcntxt->tcp_connected == true)
	{
		data_len = (wfcntxt->mqtt_flags & 0xC0)?(MQTT_Protocol_len + 2 + 4 + ClientID_len + 2 + username_len + 2 + pass_len +2)
		:(MQTT_Protocol_len + 2 + 4 + ClientID_len + 2 );
		X = data_len;
		/* Contruct mqtt packet */
		memset((char*)wfcntxt->data_tx_buf,0,TX_MAX_SIZE);
		sprintf((char*)wfcntxt->data_tx_buf,"");
		uint8_t* str_ptr = wfcntxt->data_tx_buf;
		/* Send the Packet */
		memset((char*)wfcntxt->data_tx_buf,0,TX_MAX_SIZE);
		/* Control Packet type */
		*str_ptr++ = MQTT_CONNECT;
		wfcntxt->send_len++;
		/* Remaining Length */
		do
		{
			encodedByte = X % 128;
			X = X /128;
			if(X>0)
			{
				encodedByte |= 128;
			}
			*str_ptr++ = encodedByte;
			wfcntxt->send_len++;
		}while(X>0);
		
		/* Protocol Name len*/
		*str_ptr++ = (MQTT_Protocol_len >> 8);
		wfcntxt->send_len++;
		*str_ptr++ = (MQTT_Protocol_len & 0xFF);
		wfcntxt->send_len++;
		/* Protocol Name */
		X = sprintf((char*)str_ptr,MQTT_PROTOCOL_NAME);
		str_ptr = str_ptr + X;
		wfcntxt->send_len = wfcntxt->send_len + X;
		
		/* MQTT Level */
		*str_ptr++ = wfcntxt->mqtt_lvl;
		wfcntxt->send_len++;
		*str_ptr++ = wfcntxt->mqtt_flags;
		wfcntxt->send_len++;
		
		/* Keep Alive */
		*str_ptr++ = wfcntxt->mqtt_keep_alive >> 8;
		wfcntxt->send_len++;
		*str_ptr++ = wfcntxt->mqtt_keep_alive & 0xFF;
		wfcntxt->send_len++;
		
		/* Client Id Len */
		*str_ptr++ = ClientID_len >> 8;
		wfcntxt->send_len++;
		*str_ptr++ = ClientID_len & 0xFF;
		wfcntxt->send_len++;
		
		/* Client ID */
		X = sprintf((char*)str_ptr, MQTT_CLIENT_ID);
		str_ptr = str_ptr + X;
		wfcntxt->send_len = wfcntxt->send_len + X;
		
		
		if(wfcntxt->mqtt_flags & 0xC0)		/* If username and Password */
		{	
			/* User name Len */
			*str_ptr++ = username_len >> 8;
			wfcntxt->send_len++;
			*str_ptr++ = username_len & 0xFF;
			wfcntxt->send_len++;			
			// Username and Password 
			/* User name */
			X = sprintf((char*)str_ptr, MQTT_USERNAME);
			str_ptr = str_ptr + X;
			wfcntxt->send_len = wfcntxt->send_len + X;
			
			/* Pass Word Len */
			*str_ptr++ = pass_len >> 8;
			wfcntxt->send_len++;
			*str_ptr++ = pass_len & 0xFF;
			wfcntxt->send_len++;
			
			/* Pass word  */
			X = sprintf((char*)str_ptr, MQTT_PASSWORD);
			str_ptr = str_ptr + X;
			wfcntxt->send_len = wfcntxt->send_len + X;
		}
		
		//*str_ptr = 0x1A;
		//wfcntxt->send_len++;
		
		/* Send Packet */
		if(tcp_send(wfcntxt) != HAL_OK)
		{
			//Error
			return HAL_ERROR;
		}
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	if(wfcntxt->command_code == CIPSEND_OK && wfcntxt->tcp_connected == true)
	{
		/* Send Complete */
		wfcntxt->command_code = CIPSEND_RECV;
		wfcntxt->recieve_flag = true;
		return HAL_OK;
	}
	return HAL_OK;
}

HAL_StatusTypeDef mqtt_publish(ESP_handle *wfcntxt)
{
	/* Contruct mqtt packet */
	memset((char*)wfcntxt->data_tx_buf,0,TX_MAX_SIZE);
	//sprintf((char*)wfcntxt->data_tx_buf,"");
	uint8_t* str_ptr = wfcntxt->data_tx_buf;
	topic_len = wfcntxt->topic_len;
	data_len = 0;
	
	if(wfcntxt->mqtt_connected == true && wfcntxt->command_code == IDLE)
	{
		//wfcntxt->received_len = 0;
		wfcntxt->send_len = 0;
		
		*str_ptr++ = MQTT_PUBLISH;
		wfcntxt->send_len++;
		data_len = strlen((char*)wfcntxt->topic) + 2; /* This topic consists topic + data */
		X = data_len;
		do
		{
			encodedByte = X%128;
			X = X/128;
			if( X > 0)
			{
				encodedByte |= 128;
			}
			*str_ptr++ = encodedByte;
			wfcntxt->send_len++;
		}while(X>0);

		/* Topic Len */
		*str_ptr++ = topic_len >> 8;
		wfcntxt->send_len++;
		*str_ptr++ = topic_len & 0xFF;
		wfcntxt->send_len++;
		/* Topic */
		X = sprintf((char*)str_ptr,"%s",wfcntxt->topic);
		str_ptr = str_ptr + X;
		wfcntxt->send_len = wfcntxt->send_len + X;
//		*str_ptr = 0x1A;
//		wfcntxt->send_len++;

		/* Send Packet */
		if(tcp_send(wfcntxt) != HAL_OK)
		{
			//Error
			return HAL_ERROR;
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	if(wfcntxt->command_code == CIPSEND_OK && wfcntxt->mqtt_connected == true)
	{
		/* Send Complete */
		wfcntxt->command_code = IDLE;	/* QOS 0 packet gets no response */
		//wfcntxt->recieve_flag = true;
		wfcntxt->publish_flag = false;
		return HAL_OK;
	}
	return HAL_OK;
}



