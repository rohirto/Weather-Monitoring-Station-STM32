/**
  ******************************************************************************
  * @file           : WiFiComm.h
  * @brief          : Header File which contains all necessry defines, Macros
											Prototypes, Handles regarding WiFi Communication.
  ******************************************************************************
  */
	#ifndef WIFICOMM_H
	#define WIFICOMM_H
	
	/* Includes */
	#include "main.h"
	#include <stdio.h>
	#include <stdint.h>
	#include <stdbool.h>
	
	
	/* Defines */
	#define CMD_MAX_SIZE 	100
	#define RCMD_MAX_SIZE	550
	
	#define TX_MAX_SIZE		100
	#define RX_MAX_SIZE		550	
	
	/* Event and Event codes */
	#define IP_EVENT				"GOT IP"
	#define IP_EVENT_CODE			1
	
	/* Command code */
	#define IDLE							0
	#define MQTT_CONNECT_CMD			1
	#define MQTT_PING_CMD					2
	#define CIPSTART_START				3
	#define CIPSTART_RESPONSE			4
	#define CIPSTART_OK						5
	#define CIPSEND_START					6
	#define CIPSEND_RESPONSE			7
	#define CIPSEND_ENTERDATA			8
	#define CIPSEND_OK						9
	#define CIPSEND_RECV					10
	#define CONN_CLOSED						11
	
	#define CRLF					"\r\n"
	#define OK						"OK\r\n"
	#define ERROR					"ERROR\r\n"
	#define READY					"ready"
	#define CLOSED				"CLOSED\r\n"
	#define ENTER_DATA		"\r\nOK\r\n> "	
	#define SEND_OK				"SEND OK\r\n"
	#define DATA_RECEIVED	"\r\n+IPD,"
	
	#define TCP						"\"TCP\""
	#define KEEP_ALIVE		1
	/* Command Type */
	#define TEST_COMMAND	"=?"				/* Queries the Set Commands’ internal parameters and their range of values.*/
	#define QUERY_COMMAND	"?"					/* Returns the current value of parameters. */
	#define SET_COMMMAND	"="					/* Sets the value of user-defined parameters in commands, and runs these commands. */
	#define EXECUTE_COMMAND	""				/* Runs commands with no user-defined parameters. */
	
	
	/* Basic AT Commands */
	#define AT						"AT"
	#define AT_RST				"+RST"		/* Resets the AT Module */
	
	/* WiFi AT Commands */
	#define AT_CWMODE			"+CWMODE"	/* Set or Know the Wi-Fi mode (AP or STA)*/
	#define AT_CWJAP			"+CWJAP"	/* Connect to an AP */
	#define AT_CWQAP			"+CWQAP"	/* Disconnect from AP */
	
	/* TCP IP AT Commands */
	#define AT_CIPSTATUS	"+CIPSTATUS"	/* Obtain the TCP/UDP/SSL connection status and information.*/
	#define AT_CIFSR			"+CIFSR"
	#define AT_CIPSTART		"+CIPSTART"		/* Establish TCP connection, UDP transmission, or SSL connection.*/
	#define AT_CIPSEND 		"+CIPSEND"		/* Send data in the normal or Wi-Fi passthrough modes.*/
	#define AT_CIPCLOSE		"+CIPCLOSE"	/* Close TCP/UDP/SSL connection.*/
	#define AT_CIPSERVER	"+CIPSERVER" 	/* Delete/create a TCP or SSL server. */
	#define AT_CIPMODE		"+CIPMODE"		/* Configure the transmission mode. */
	#define AT_CIPRECONNINTV "+CIPRECONNINTV" /*Set the TCP reconnection interval for the Wi-Fi passthrough mode.*/
	
	/* Callbacks for Event Programming */
	typedef void (*ESP_Callback) (uint16_t, int, void*);
		/* Call back Comm */
	typedef HAL_StatusTypeDef (*comm_cb)(void*);
	
	/* Typedefs */
	
	typedef struct wifi_cred_t {
    char  ssid[32];
    char  passphrase[80];
    int   authmode;
    int   encryptmode;
	}WFC_CRED_DATA;
	
	typedef enum
	{
    OPEN_SECURITY,
    WEP_SECURITY,
    WPA_SECURITY,
    WPA2_SECURITY,
	}security_type;
	
	
	typedef struct CRED_Data
	{
		uint8_t *SSID;
		uint8_t *pass;
		security_type security;
		
	}cred;
	
	typedef struct ESPHandle_TypeDef
	{
		char mac_addr[24];
		uint8_t ip_addr[37]; 
		char mqtt_broker[32];
		uint8_t keep_alive;
		uint8_t cmd_mode; 	/* TCP or MQTT port */
		int tcp_port;
		int mqtt_port;
		int cwmode;					/* AP or STA mode */
		bool             mqtt_connected;
		bool             tcp_connected;
		uint8_t wifi_cmd_str[CMD_MAX_SIZE];
		uint8_t wifi_rcmd_str[RCMD_MAX_SIZE];
		
		uint8_t data_tx_buf[TX_MAX_SIZE];
		uint8_t data_rx_buf[RX_MAX_SIZE];
		int received_len;
		int	send_len;
		
		bool respone_expected;
		bool error_flag;
		bool send_data_flag;
		bool recieve_flag;
		bool publish_flag;
		uint8_t command_code;		/* Which command was said */
		comm_cb data_callback;
		comm_cb cmdresp_callback;
		
		bool update_cntxt;
		
		char topic[50];
		int8_t topic_len;
		int8_t protocol_len; 
		int8_t ClientID_len;
		int8_t username_len;
		int8_t pass_len;
		char mqtt_lvl;
		char mqtt_flags;
		char mqtt_qos;
		uint32_t mqtt_keep_alive;
		
		int publish_indx;
		 
		
	}ESP_handle;
	
	
	

	
	/* Function Prototypes */
	HAL_StatusTypeDef ESP_Init(void);
	void CommCBTask(void* pvParams);	
	HAL_StatusTypeDef Callback_Init(ESP_handle *wifi, comm_cb data_cb, comm_cb cmdresp_cb);
	HAL_StatusTypeDef CmdRespCallback_Init(comm_cb cb);
	void wireless_callback(uint16_t ID, int wfc_event, void* pData);
	HAL_StatusTypeDef CommCallback_Init(comm_cb cb);
	void uart_callback (ESP_handle *wfcntxt, void *buf, uint16_t len);
	HAL_StatusTypeDef esp_callback(ESP_handle *wfcntxt);
	HAL_StatusTypeDef Process_CMD_Response(void *buffer);	
	void TCP_SendData_Task(void* pvParams);
	HAL_StatusTypeDef ESP_Send_Cmd(uint8_t *cmd);
	HAL_StatusTypeDef ESP_Send_Data(uint8_t *data);
	HAL_StatusTypeDef tcp_connect(ESP_handle *wfcntxt);
	HAL_StatusTypeDef tcp_send(ESP_handle *wfcntxt);
	void ESP_Error_Handler_Task(void* pvParams);
	
	#endif