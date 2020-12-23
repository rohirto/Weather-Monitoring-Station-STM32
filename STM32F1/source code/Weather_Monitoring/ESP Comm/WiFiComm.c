/**********************************************************************************
  
*@file		WiFiComm.c
*@author	rohirto
*@version	1.0
*@date 		12/12/2020
*@brief 	WiFi Coomunication Library file which will talk with ESP 8266 over UART2
					ESP understands AT Commands 
**************************************************************************************************************
**/

/* Includes */
#include "main.h"
#include "debug.h"
#include "WiFiComm.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"
#include "UartRingBuffer_multi.h"
#include "mqtt_client.h"


SemaphoreHandle_t xUART3Mutex;

ESP_handle wifi_module;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern bool byte_recieved;
extern char prev_c, present_c;
extern ring_buffer rx_buffer1;   

void CommCBTask(void* pvParams)
{
//    uint32_t Event_to_Process;
		uint8_t* str_ptr;
		uint8_t* debug_messagePtr;
    for(;;)
    {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			/* Callback function */
			/* If response expected the go to response processing function */
			if((wifi_module.respone_expected == false) && (wifi_module.error_flag != true) && (wifi_module.recieve_flag == true) 
				&& ((wifi_module.command_code == IDLE) || (wifi_module.command_code == CIPSEND_RECV))) 
			{
				char temp_buf[7], str_len[3];
				memset(wifi_module.data_rx_buf, 0, RX_MAX_SIZE);
				str_ptr = wifi_module.data_rx_buf;
				
				HAL_Delay(500);
				if(IsDataAvailable(device_uart))
				{ 
					if(rx_buffer1.buffer[rx_buffer1.tail] == 'C' && rx_buffer1.buffer[rx_buffer1.tail + 5] == 'D')
					{
						/* Closed */
						if(Copy_upto(CLOSED,(char*)wifi_module.data_rx_buf, device_uart))
						{
							wifi_module.command_code = CONN_CLOSED;
							
						}
						else
						{
							wifi_module.error_flag = true;
						}
					}
					else if(rx_buffer1.buffer[rx_buffer1.tail] == '\r' && rx_buffer1.buffer[rx_buffer1.tail + 6] == ',') /* Recievd Data \r\n+IPD, */
					{
						if(Copy_upto(DATA_RECEIVED, temp_buf, device_uart))
						{
							if(!Copy_upto(":", str_len, device_uart))
							{
								//Error 
								wifi_module.error_flag = true;
							}
						}
						if(str_len[0] == 0x3A)/* ':' */
						{
							str_len[0] = '\0';
						}
						wifi_module.received_len = atoi(str_len);
						int len = wifi_module.received_len;
						
						while(len>=0)
						{
							if(IsDataAvailable(device_uart))
							{
								int data  = Uart_read(device_uart);
								*str_ptr++ = data;
							}
							len--;
						}
					}
						
				}
				if(strcmp((char*)wifi_module.data_rx_buf,""))
				{
					if(esp_callback(&wifi_module) != HAL_OK)
					{
						/* Command Error */
						sprintf((char *)pcDebugBuffer, "DataPrintFail\r\n");
						Debug_Mutex();
					}
				}
				
			}
			if((wifi_module.respone_expected == true) && (wifi_module.error_flag != true)) 
			{
				memset(wifi_module.wifi_rcmd_str, 0, RCMD_MAX_SIZE);
				/* Process Response */
				if(IsDataAvailable(device_uart))
				{
					if(wifi_module.command_code == CIPSEND_START)
					{
						if(!Copy_upto(ENTER_DATA,(char*)wifi_module.wifi_rcmd_str,device_uart))
						{
//							if(IsDataAvailable(device_uart))  
//							Copy_upto(ERROR,(char*)wifi_module.wifi_rcmd_str,device_uart);
							wifi_module.error_flag = true;
						}
					}
					else
					{
						if(!Copy_upto(OK,(char*)wifi_module.wifi_rcmd_str,device_uart))
						{
							if(IsDataAvailable(device_uart))  
							Copy_upto(ERROR,(char*)wifi_module.wifi_rcmd_str,device_uart);
						}
					}
					
				}
				if(strcmp((char*)wifi_module.wifi_rcmd_str,""))
				{
					if(Process_CMD_Response(&wifi_module.wifi_rcmd_str) != HAL_OK)
					{
						/* Command Error */
						sprintf((char *)pcDebugBuffer, "Command Process Fail\r\n");
						Debug_Mutex();
						
					}
				}
			}     
    }
}

void TCP_SendData_Task(void* pvParams)
{
//	uint32_t Event_to_Process;

	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(strcmp((char*)wifi_module.data_tx_buf,""))
		{
			/* Send Tx Buf */
			ESP_Send_Data(wifi_module.data_tx_buf);
			
		}
		//memset((char*)wifi_module.data_tx_buf,0,TX_MAX_SIZE); 
		
		/* After writing data go back to command mode */
		
				
	}
}

void ESP_Error_Handler_Task(void* pvParams)
{
//	uint32_t Event_to_Process;
	uint8_t* debug_messagePtr;
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(wifi_module.error_flag == true)
		{
			/* Flush Uart Buffer */
			Uart_flush(device_uart);
			Uart_Tail_Zero(device_uart);
			if(ESP_Init() == HAL_OK)
			{
				sprintf((char *)pcDebugBuffer, "Resetting ESP");
				Debug_Mutex();
			}
		} 
	}
}
 
HAL_StatusTypeDef tcp_connect(ESP_handle *wfcntxt) 
{
	//wfcntxt->command_code = MQTT_CONNECT_CMD;
	sprintf((char*)wfcntxt->wifi_cmd_str, "%s%s%s%s,%s,%d\r\n",AT,AT_CIPSTART,SET_COMMMAND,TCP,(char*)wfcntxt->mqtt_broker,wfcntxt->mqtt_port); 
	
	/* Send to ESP Module */
	if(ESP_Send_Cmd(wfcntxt->wifi_cmd_str) != HAL_OK) 
	{
		//ERROR
		return HAL_ERROR;
	}
	wfcntxt->command_code = CIPSTART_RESPONSE;
	return HAL_OK;
}

HAL_StatusTypeDef tcp_send(ESP_handle *wfcntxt)
{
	wfcntxt->command_code = CIPSEND_START;
	sprintf((char*)wfcntxt->wifi_cmd_str, "%s%s%s%d%s",AT,AT_CIPSEND,SET_COMMMAND,wfcntxt->send_len,CRLF);
	/* Send to ESP Module */
	if(ESP_Send_Cmd(wfcntxt->wifi_cmd_str) != HAL_OK) 
	{
		//ERROR
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef esp_callback(ESP_handle *wfcntxt) 
{
	/* Echo on Debug */
	//uint8_t* debug_messagePtr;
	uint8_t* str_ptr = wfcntxt->data_rx_buf;
	if(wifi_module.tcp_connected == true)
	{
		switch(wfcntxt->command_code)
		{
			case CIPSEND_RECV:
				/* When some data is sent to MQTT Server and Response is e
					expected from the server */
				
				if(*str_ptr == MQTT_CONNACK) /* CONNECT ACK */
				{
					if(*(++str_ptr + 3) == 0x00)
					{
						/* Connect ACK */
						wfcntxt->mqtt_connected = true;
						wfcntxt->command_code = IDLE;
						return HAL_OK;
					}
				}
				else if (*str_ptr == MQTT_PUBACK)	/* No PUBACK for QoS 0*/
				{
					if(*(++str_ptr + 3) == 0x00)
					{
						wfcntxt->publish_flag = false;
						wfcntxt->command_code = IDLE;
						return HAL_OK;
					}
				}
			
				break;
				
			case CONN_CLOSED:
				wfcntxt->mqtt_connected = false;
				wfcntxt->tcp_connected = false;
				wfcntxt->publish_flag = false;
				wfcntxt->respone_expected = false;
				wfcntxt->send_data_flag = false;
				wfcntxt->command_code = CIPSTART_START;
				return HAL_OK;
				break;
		}
//		strcpy((char*)pcDebugBuffer, (char*)wfcntxt->data_rx_buf);
//		//Debug_Mutex();
//		Uart_sendstring((char*)wfcntxt->data_rx_buf, pc_uart);
	}
	
	
//	/* What event happened */
//	int event = 0;
//	if(Look_for(IP_EVENT, buffer))
//	{
//		event = 1;
//	}
//	switch(event)
//	{
//		case IP_EVENT_CODE:
//			/* Some flag to be set so that IP address is updated automatically */
//		wifi_module.update_cntxt = true;
//			break; 
		
//	}
	return HAL_ERROR;
}

HAL_StatusTypeDef Process_CMD_Response(void *buffer)   
{
	/* Search for OK or Error */
	/* CIP START Process */
	
	switch(wifi_module.command_code)
	{
//		case MQTT_CONNECT_CMD:
//			if(Look_for(AT_CIPSTART, (char*)wifi_module.wifi_rcmd_str))
//			{
//				if(Look_for(OK, (char*)wifi_module.wifi_rcmd_str))
//				{
//					/* Ok Condition */
//					wifi_module.mqtt_connected = true;
//					wifi_module.respone_expected = false;
//					wifi_module.command_code = IDLE;
//					return HAL_OK;
//				}
//				else
//				{
//					wifi_module.error_flag = true;
//					wifi_module.command_code = IDLE;
//					return HAL_ERROR;
//				}
//			}
//			break;
			case CIPSTART_RESPONSE:
			if(Look_for(OK,(char*)wifi_module.wifi_rcmd_str))
			{
				wifi_module.command_code = CIPSTART_OK;
				wifi_module.tcp_connected = true;
				wifi_module.respone_expected = false;
				return HAL_OK;
			}
//			else(Look_for(ERROR,(char*)wifi_module.wifi_rcmd_str))
//			{
//				wifi_module.command_code = IDLE;
//				wifi_module.error_flag =true;
//				return HAL_ERROR;
//			}
			
			break;
			
		case CIPSEND_START:
			if(Look_for(ENTER_DATA, (char*)wifi_module.wifi_rcmd_str))
			{
				/* ESP is ready to recieve data STM32 Will send the Data  */
				wifi_module.send_data_flag = true;
				wifi_module.respone_expected = false;
				wifi_module.command_code = CIPSEND_ENTERDATA;
				return HAL_OK;
			}
			else if(Look_for(ERROR, (char*)wifi_module.wifi_rcmd_str))
			{
				wifi_module.error_flag = true;
				wifi_module.command_code = IDLE;
				return HAL_ERROR;
			}
			break;
		
		case CIPSEND_ENTERDATA:
			if(Look_for(SEND_OK, (char*)wifi_module.wifi_rcmd_str))
			{
				/* Send Complete */
				wifi_module.command_code = CIPSEND_OK; 
				wifi_module.respone_expected = false;
				return HAL_OK;
			}
			break;
			
		case CIPSEND_OK:
			if(Look_for(SEND_OK, (char*)wifi_module.wifi_rcmd_str))
			{
				/* Send Complete */
				//wifi_module.command_code = IDLE;
				wifi_module.respone_expected = false;
				return HAL_OK;
			}
			else
			{
				wifi_module.error_flag = true;
				return HAL_ERROR;
			}
			break;
		case IDLE:
			if(Look_for(SEND_OK, (char*)wifi_module.wifi_rcmd_str))
			{
				/* Send Complete */
				wifi_module.command_code = IDLE;
				wifi_module.respone_expected = false;
				return HAL_OK;
			}
			
			break;
	}
	wifi_module.respone_expected = true;	
	return HAL_ERROR;
}
HAL_StatusTypeDef ESP_Init()
{
	
	/* Check AT */
	memset((char*)wifi_module.wifi_cmd_str, 0, CMD_MAX_SIZE);
	sprintf((char*)wifi_module.wifi_cmd_str, "%s%s%s",AT,AT_RST,CRLF);
	Uart_sendstring((char*)wifi_module.wifi_cmd_str, device_uart); 
	
	HAL_Delay(5000);
	char* str_ptr = (char*)wifi_module.wifi_rcmd_str;
	/* Check response */
	while(IsDataAvailable(device_uart))
	{
		int data = Uart_read(device_uart);
    *str_ptr++ = data;
	} 
	if(!Look_for(READY,(char*) (char*)wifi_module.wifi_rcmd_str))
	{
		memset((char*)wifi_module.wifi_rcmd_str, 0, RCMD_MAX_SIZE);
		return HAL_ERROR;
	}
	memset((char*)wifi_module.wifi_rcmd_str, 0, RCMD_MAX_SIZE);
	/* Initialize the wifi context */
	
	/* Update Context */
	memset((char*)wifi_module.wifi_cmd_str, 0, CMD_MAX_SIZE);
	sprintf((char*)wifi_module.wifi_cmd_str, "%s%s\r\n",AT,AT_CIFSR);
	Uart_sendstring((char*)wifi_module.wifi_cmd_str, device_uart); 

	HAL_Delay(1000);
	/* Check response */
	if(!Get_after(",\"", 13, (char*)wifi_module.ip_addr, device_uart))
	{
		//Error
	}
	if(!Get_after(",\"", 17, (char*)wifi_module.mac_addr, device_uart))
	{
		//Error
	}
	
	/* Flush Uart Buffer */
	Uart_flush(device_uart);
	Uart_Tail_Zero(device_uart); 
	/* Clear Buffers */
	memset((char*)wifi_module.data_rx_buf, 0, RX_MAX_SIZE);  
	memset((char*)wifi_module.data_tx_buf, 0, TX_MAX_SIZE);
	memset((char*)wifi_module.wifi_cmd_str, 0, CMD_MAX_SIZE);
	memset((char*)wifi_module.wifi_rcmd_str, 0, RCMD_MAX_SIZE);
	/* Set flags */
	wifi_module.command_code = CIPSTART_START;
	wifi_module.error_flag = false;
	wifi_module.mqtt_connected = false;
	wifi_module.respone_expected = false;
	wifi_module.tcp_connected = false;
	wifi_module.send_data_flag = false;
	wifi_module.recieve_flag = false;
	wifi_module.received_len = 0;
	wifi_module.send_len = 0;
	
	
	
	return HAL_OK;
}


void ESP_init_context (ESP_handle *wfcntxt) {

    
    //wfcntxt->inittask_done  = 0xEE;
    //wfcntxt->wfc_rssi       = 0;    // Initialize with Zero
    //wfcntxt->cmd_mode       = 1;
    //wfcntxt->mqtt_progress  = 0;
    //wfcntxt->wfc_inact      = WFC_DEFAULT_AP_INACT;
    //wfcntxt->mqtt_port      = WFC_DEFAULT_MQTT_PORT;
    //wfcntxt->tcp_port       = WFC_DEFAULT_TCP_PORT;
    //wfcntxt->keep_alive     = WFC_DEFAULT_KEEP_ALIVE;
    //wfcntxt->data_mode      = LOCAL_MODE;
    
    //wfcntxt->mqtt_connected = 0;
    //wfcntxt->tcpserverstart = 0;
    
	//wfcntxt->wfc_mode    = WFC_STATION;
    
    //wfcntxt->dfu_state   = NO_DFU;
    //wfcntxt->wps_state   = WPS_DISABLE;
    //wfcntxt->power_state = MAX_PERF;
    wfcntxt->data_callback      = NULL;
    wfcntxt->cmdresp_callback      = NULL;
    //memset (&wfcntxt->cred_data, 0, sizeof (WFC_CRED));
        
    // Init the RX COMM layer call back function
    //CommCallback_Init (uart_callback); 
}



//HAL_StatusTypeDef Callback_Init(ESP_handle *wifi, comm_cb data_cb, comm_cb cmdresp_cb) 
//{
//	wifi->data_callback = data_cb;  
//	wifi->cmdresp_callback = cmdresp_cb;
//	return HAL_OK; 
//}
//HAL_StatusTypeDef CommCallback_Init(comm_cb cb)
//{
//	CommCB = cb;
//	return HAL_OK;
//}
//HAL_StatusTypeDef CmdRespCallback_Init(comm_cb cb)
//{
//	CmdRespCB = cb;
//	return HAL_OK;
//}
HAL_StatusTypeDef ESP_Send_Cmd(uint8_t *cmd)
{
	if(cmd != NULL)
	{
		/* Take Mutex */
		xSemaphoreTake(xUART3Mutex, portMAX_DELAY);
		Uart_sendstring((char*)cmd, device_uart);
		/* Give back Mutex */
		xSemaphoreGive( xUART3Mutex );
		wifi_module.respone_expected = true;
		HAL_Delay(1000);
	}
	else
	{
		return HAL_ERROR;
	}
	
	return HAL_OK;
}
HAL_StatusTypeDef ESP_Send_Data(uint8_t *data)
{
	uint8_t *str_ptr = wifi_module.data_tx_buf;
	int len;
	if(wifi_module.send_len == 0)
	{
		return HAL_ERROR;
	}
	else
	{
		len = wifi_module.send_len;
	}
	if(data != NULL)
	{
		/* Take Mutex */
		xSemaphoreTake(xUART3Mutex, portMAX_DELAY);
//		Uart_sendstring((char*)data, device_uart);
		while(len >=0)
		{
			Uart_write(*str_ptr++, device_uart);
			len--;
		}
		/* Give back Mutex */
		xSemaphoreGive( xUART3Mutex );
		wifi_module.send_data_flag = false;
		wifi_module.respone_expected = true;
		HAL_Delay(1000);
	}
	else
	{
		return HAL_ERROR;
	}
	
	return HAL_OK;
}