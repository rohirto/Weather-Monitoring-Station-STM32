/**
  ******************************************************************************
  * @file           : mqtt_client.h
  * @brief          : Header file for mqtt_client.c Contains all MQTT protocol 
											related stuff
  ******************************************************************************
  */
	
	#ifndef MQTT_CLIENT_H
	#define MQTT_CLIENT_H
	
	#include "main.h"
	#include "WiFiComm.h"
	
	
	/* Defines */
	
	#define MQTT_PROTOCOL_NAME			"MQIsdp"
	#define MQTT_BROKER							"\"io.adafruit.com\""
	#define MQTT_PORT								1883
	#define MQTT_CLIENT_ID					"STM32"
	#define MQTT_USERNAME						"ADAFRUIT USERNAME"
	#define MQTT_PASSWORD						"ADAFRUIT IO KEY"
	#define MQTT_TEMPERATURE_TOPIC	"rohirto/feeds/temperature"
	#define MQTT_HUMIDITY_TOPIC			"rohirto/feeds/humidity"
	#define MQTT_PRESSURE_TOPIC			"rohirto/feeds/pressure"
	#define MQTT_ALTITUDE_TOPIC			"rohirto/feeds/altitude"
	#define MQTT_AQI_TOPIC					"rohirto/feeds/aqi"
	
	/* Sending Indexes - Used to sequentially Publish */ 
	#define IDLE_INDEX							0
	#define TEMPERATURE_INDEX				1
	#define HUMIDITY_INDEX					2
	#define PRESSURE_INDEX					3
	#define AQI_INDEX								4
	
	
	#define MQTT_LVL								0x03
	#define MQTT_FLAGS							0xC2		//Username and Password
	//#define MQTT_FLAGS							0x02		// No username and Password
	#define MQTT_KEEPALIVE					60
	#define MQTT_QOS								0x00
	#define MQTT_PACKETID						0x0001
		
	
	/* MQTT Control Packet Types  */
	#define MQTT_CONNECT			0x10
	#define MQTT_CONNACK			0x20
	#define MQTT_PUBLISH			0x30
	#define MQTT_PUBACK				0x40
	#define MQTT_PUBREC				0b0101
	#define MQTT_PUBREL				0b0110
	#define MQTT_PUBCOMP			0b0111
	#define MQTT_SUBSCRIBE		0b1000
	#define MQTT_SUBACK				0b1001
	#define MQTT_UNSUBSCRIBE	0b1010
	#define MQTT_UNSUBACK			0b1011
	#define MQTT_PINGREQ			0b1100
	#define MQTT_PINGRESP			0b1101
	#define MQTT_DISCONNECT		0b1110
	
	/* MQTT Flags */
	#define MQTT_CONNECT_FLAG				0b0000		
	#define MQTT_CONNACK_FLAG				0b0000
	#define MQTT_PUBLISH_FLAG				
	#define MQTT_PUBACK_FLAG				0b0000
	#define MQTT_PUBREC_FLAG				0b0000
	#define MQTT_PUBREL_FLAG				0b0010
	#define MQTT_PUBCOMP_FLAG				0b0000
	#define MQTT_SUBSCRIBE_FLAG			0b0010
	#define MQTT_SUBACK_FLAG				0b0000
	#define MQTT_UNSUBSCRIBE_FLAG		0b0010
	#define MQTT_UNSUBACK_FLAG			0b0000
	#define MQTT_PINGREQ_FLAG				0b0000
	#define MQTT_PINGRESP_FLAG			0b0000
	#define MQTT_DISCONNECT_FLAG		0b0000
	
	/* Fixed Header Remaining Length */
	#define MQTT_FIXED_HEADER_REM_L	0x00
	
	/* Variable Header */
	#define MQTT_CONNECT_VARIABLE_HEADER	
	
	
	/* Functions Prototypes */
	void MQTT_Init_Task(void* pvParams);
	void MQTT_Ping_Task(void *pvParams);
	void MQTT_Publish_Task(void* pvParams);
	HAL_StatusTypeDef mqtt_connect(ESP_handle *wfcntxt);
	HAL_StatusTypeDef mqtt_publish(ESP_handle *wfcntxt);
	
	#endif
