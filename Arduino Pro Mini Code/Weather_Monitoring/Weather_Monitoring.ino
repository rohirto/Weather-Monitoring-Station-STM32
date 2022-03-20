/* 
 *  Pins USED:
 *  nRF: SPI: 9,10,11,12,13
 *  DHT11: 1 wire : 7
 *  BME280: I2C : 
 *  Dust Sensor: 6,8
 *  Pond sensor: One Wire: 3
 *  
 */

#include "DHTesp.h"
#include <string.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LowPower.h>
#include <OneWire.h>
#include <DallasTemperature.h>
/***********NRF24 *********************/
#define CE_PIN   9
#define CSN_PIN 10
const byte slaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
char dataToSend[50];
char txNum = '0';

#define SEALEVELPRESSURE_HPA (1013.25)


/****** MQTT Defns *********************/
//unsigned char encodedByte;
//int X;
//unsigned long datalength;
//unsigned short MQTTProtocolNameLength;
//unsigned short MQTTClientIDLength;
//unsigned short MQTTUsernameLength;
//unsigned short MQTTPasswordLength;
//unsigned short topiclength;
////const char MQTTHost[30]="io.adafruit.com";
////const char MQTTPort[5]="1883";
//const char clientID[8]="rohirto";
//const char MQTT_Protocol_Name[7]="MQIsdp";
//const char MQTT_Lvl = 0x03;
//const char MQTT_Flags = 0xC2;
//const unsigned int MQTT_KeepAlive = 60;
//const char MQTT_Username[8] = "rohirto";
//const char MQTT_Password[33] = "aio_FHZk43hBz4VEQEMO0TmBE0vXkwIt";
//const char MQTT_QoS = 0x00;
//const char MQTT_Common_Topic[15] ="rohirto/feeds/";
//const char MQTT_Temperature[12]="temperature";
//const char MQTT_Humidity[9]="humidity";
//const char MQTT_Pressure[9]="pressure";
//const char MQTT_Altitude[9]="altitude";
//const char MQTT_AQI[4]="aqi";
//char topic[30];
char str[40];
char str_temp[10];
//

#define TEMP_INDEX      1
#define HUMD_INDEX      2
#define PRESSURE_INDEX  3
#define ALT_INDEX       4
#define AQI_INDEX       5
#define POND_TEMP_INDEX 6
#define UV_SENSOR_INDEX 7
/**************************************/
DHTesp dht;
//// for DHT11, 
////      VCC: 5V or 3V
////      GND: GND
////      DATA: 3
const byte pinDHT11 = 7;
float temperature_data = 0, pond_temperature_data = 0;
float humidity_data = 0;
Adafruit_BME280 bme;
float pressure_data = 0 , altitude_data = 0;
/** Dust Sensor variables ******************/
int measurePin = A0; //Connect dust sensor to Arduino A0 pin
int ledPower = 8;   //Connect 3 led driver pins of dust sensor to Arduino D2

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
// DS18B20 
// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 3
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);
/********** UV Sensor Def***********************************/
#define UV_SENSOR_PIN   A1
float uvSensorVolt, uvSensorValue;
int uvIndex; 
/************************Timer definations********************************/
unsigned long startMillis;  //Some global vaiable anywhere in program
unsigned long currentMillis;
volatile byte temp_humd_timer = 30;  // In 10 secs multiple //1 min timer 
volatile byte temp_humd_timer_elapsed = false;
volatile byte occupancy_timer_elapsed = false;
volatile byte ten_sec_counter = 0;
volatile byte occupancy_timer = 3;
/****************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  Serial.println(F("Weather Monitoring Project"));
  //Sensors Setup
  Sensors_setup();
  //NRF Setup
  NRF24_setup();
  //Timer start
  startMillis = millis();
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Sleep for 5 mins i = 38
  //Sleep for 30 sec i =4
  // ATmega328P, ATmega168
  int i = 0;
  for(i = 0; i<=38; i++)
  {
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
  }

  DHT_11_loop();
  BMP_280_loop();
  GP2Y101_loop();
  DS18B20_loop(); 
  S12SD_loop();
  i = 0;
  for(i = TEMP_INDEX ; i <= UV_SENSOR_INDEX; i++)
  {
    switch(i)
    {
      case TEMP_INDEX:
      dtostrf(temperature_data, 4, 2, str_temp);
      sprintf(dataToSend,"1~%s~", str_temp);
      send();
      delay(10000);
      break;
      case HUMD_INDEX:
      dtostrf(humidity_data, 4, 2, str_temp);
      sprintf(dataToSend,"2~%s~", str_temp);
      send();
      delay(10000);
      break;
      case PRESSURE_INDEX:
      dtostrf(pressure_data, 4, 2, str_temp);
      sprintf(dataToSend,"3~%s~", str_temp);
      send();
      delay(10000);
      break;
      case ALT_INDEX:
      dtostrf(altitude_data, 4, 2, str_temp);
      sprintf(dataToSend,"4~%s~", str_temp);
      send();
      delay(10000);
      break;
      case AQI_INDEX:
      dtostrf(dustDensity, 4, 2, str_temp);
      sprintf(dataToSend,"5~%s~", str_temp);
      send();
      delay(10000);
      break;
      case POND_TEMP_INDEX:
      dtostrf(pond_temperature_data, 4, 2, str_temp);
      sprintf(dataToSend,"6~%s~", str_temp);
      send();
      delay(10000);
      break;
      case UV_SENSOR_INDEX:
      dtostrf(uvIndex, 4, 2, str_temp);
      sprintf(dataToSend,"9~%s~", str_temp);   //9th value
      send();
      delay(10000);
      break;
      default:
      //Do nothing
      break;
    }
  }
  
  //Update Timers  
  //timer_function();
  //if(temp_humd_timer_elapsed == true)
  //{
    //DHT_11_loop();
    //BMP_280_loop();
    //GP2Y101_loop();

    /*** Data To send **************/
    /* PROTOCOL: 
     *  Start of Packet: '@@'
     *  Data Start: '~' 
     *  End of Packet: '@@'
     */
//    int i = 0;
//    for(i = TEMP_INDEX ; i <= AQI_INDEX; i++)
//    {
//      switch(i)
//      {
//        case TEMP_INDEX:
//        dtostrf(temperature_data, 4, 2, str_temp);
//        sprintf(dataToSend,"1~%s~", str_temp);
//        send();
//        delay(10000);
//        break;
//        case HUMD_INDEX:
//        dtostrf(humidity_data, 4, 2, str_temp);
//        sprintf(dataToSend,"2~%s~", str_temp);
//        send();
//        delay(10000);
//        break;
//        case PRESSURE_INDEX:
//        dtostrf(pressure_data, 4, 2, str_temp);
//        sprintf(dataToSend,"3~%s~", str_temp);
//        send();
//        delay(10000);
//        break;
//        case ALT_INDEX:
//        dtostrf(altitude_data, 4, 2, str_temp);
//        sprintf(dataToSend,"4~%s~", str_temp);
//        send();
//        delay(10000);
//        break;
//        case AQI_INDEX:
//        dtostrf(dustDensity, 4, 2, str_temp);
//        sprintf(dataToSend,"5~%s~", str_temp);
//        send();
//        delay(10000);
//        break;
//      }
//    }
//    //sprintf test
//    dtostrf(temperature_data, 4, 2, str_temp);
//    sprintf(str,"%s:\t%s",MQTT_Temperature,str_temp);
//    Serial.println(str);
//    dtostrf(humidity_data, 4, 2, str_temp);
//    sprintf(str,"%s:\t%s",MQTT_Humidity,str_temp);
//    Serial.println(str);
//    dtostrf(pressure_data, 4, 2, str_temp);
//    sprintf(str,"%s:\t%s",MQTT_Pressure,str_temp);
//    Serial.println(str);
//    dtostrf(altitude_data, 4, 2, str_temp);
//    sprintf(str,"%s:\t%s",MQTT_Altitude,str_temp);
//    Serial.println(str);
//    dtostrf(dustDensity, 4, 2, str_temp);
//    sprintf(str,"%s:\t%s",MQTT_AQI,str_temp);
//    Serial.println(str);
    
    //MQTT Publish Data 
    /* CIPSTART */
//    Serial.println("AT+CIPSTART=\"TCP\",\"io.adafruit.com\",1883");
//    delay(5000);
//    SendConnectPacket();
//    delay(5000);
//    MQTT_Publish(TEMP_INDEX);
//    delay(5000);
//    MQTT_Publish(HUMD_INDEX);
//    delay(5000);
//    MQTT_Publish(PRESSURE_INDEX);
//    delay(5000);
//    MQTT_Publish(ALT_INDEX);
//    delay(5000);
//    MQTT_Publish(AQI_INDEX);
//    delay(5000);
//    Serial.println("AT+CIPCLOSE");
//    delay(5000);
 //   //temp_humd_timer_elapsed = false;
//  }



}

void DHT_11_loop()
{
  humidity_data = dht.getHumidity();
  temperature_data = dht.getTemperature();
  
}
void BMP_280_loop()
{
  pressure_data = bme.readPressure()/100;
  altitude_data = bme.readAltitude(1013.25);
}
void GP2Y101_loop()
{
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024.0);

  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = 170 * calcVoltage - 0.1;  //in microgram per meter cube

}
void DS18B20_loop()
{
  // Send the command to get temperatures
  sensors.requestTemperatures(); 
  pond_temperature_data = sensors.getTempCByIndex(0);
}
void Sensors_setup()
{
  //Dust Sensor 
  pinMode(ledPower,OUTPUT);
  //Digital Pin 2 is input
  //DHT Setup
  dht.setup(pinDHT11, DHTesp::DHT11); // GPIO13

  //BMP280 Pressure and Altitude Setup
  bme.begin(0x76); //Start I2C at 0x76 address

  //DS18B20
   sensors.begin();  // Start up the library
}

void S12SD_loop()
{
  uvSensorValue = analogRead(UV_SENSOR_PIN);
  uvSensorVolt = uvSensorValue/1024*5.0;
  uvIndex = uvSensorVolt/0.1;   // If index <= 2 "LOW", >2 && <=5 "MED", >5 && <=7 "HIGH", >7 && <=10 "Very High", >10 "Extreme"
}
void NRF24_setup()
{
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5); // delay, count
  radio.openWritingPipe(slaveAddress);
}
void timer_function()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if( currentMillis - startMillis >= 10000)
  {
    startMillis = currentMillis;
    ten_sec_counter++;

     if((ten_sec_counter % occupancy_timer) == 0)
    {
      occupancy_timer_elapsed = true;
    }
    if((ten_sec_counter % temp_humd_timer) == 0)  //test whether the period has elapsed
    {
      temp_humd_timer_elapsed = true;
      ten_sec_counter = 0;  //IMPORTANT to save the start time of the current LED state.
    }   
  }
 
}

void send() {

    bool rslt;
    rslt = radio.write( &dataToSend, sizeof(dataToSend) );
        // Always use sizeof() as it gives the size as the number of bytes.
        // For example if dataToSend was an int sizeof() would correctly return 2

    Serial.print("Data Sent ");
    Serial.print(dataToSend);
    if (rslt) {
        Serial.println("  Acknowledge received"); 
    }
    else {
        Serial.println("  Tx failed");
    }
}
//void MQTT_Publish(int index)
//{
//  
//  /* CIP SEND */
//  memset(str,0,40);
//  memset(topic,0,30);
//  topiclength = 0;
//  datalength = 0;
//  encodedByte = 0x00;
//  switch(index)
//  {
//    case TEMP_INDEX:
//      topiclength = sprintf(topic,"%s%s",MQTT_Common_Topic,MQTT_Temperature);
//      dtostrf(temperature_data, 4, 2, str_temp);
//      datalength = sprintf(str,"%s%s",topic,str_temp);
//      break;
//    case HUMD_INDEX:
//      topiclength = sprintf(topic,"%s%s",MQTT_Common_Topic,MQTT_Humidity);
//      dtostrf(humidity_data, 4, 2, str_temp);
//      datalength = sprintf(str,"%s%s",topic,str_temp);
//      break;
//    case PRESSURE_INDEX:
//      topiclength = sprintf(topic,"%s%s",MQTT_Common_Topic,MQTT_Pressure);
//      topiclength = 0x16;
//      dtostrf(pressure_data, 4, 2, str_temp);
//      datalength = sprintf(str,"%s%s",topic,str_temp);
//      break;
//    case ALT_INDEX:
//      topiclength = sprintf(topic,"%s%s",MQTT_Common_Topic,MQTT_Altitude);
//      topiclength = 0x16;
//      dtostrf(altitude_data, 4, 2, str_temp);
//      datalength = sprintf(str,"%s%s",topic,str_temp);
//      break;
//    case AQI_INDEX:
//      topiclength = sprintf(topic,"%s%s",MQTT_Common_Topic,MQTT_AQI);
//      topiclength = 0x11;
//      dtostrf(dustDensity, 4, 2, str_temp);
//      datalength = sprintf(str,"%s%s",topic,str_temp);
//      break;
//    default:
//      break;
//  }
//  
//  X = datalength +4;
//  Serial.print("AT+CIPSEND=");
//  delay(100);
//  Serial.println(X);
//  delay(8000);
//  Serial.write(0x30);
//   X = datalength + 2;
//  do {
//    encodedByte = X % 128;
//    X = X / 128;
//    if (X > 0) {
//      encodedByte |= 128;
//    }
//    Serial.write(encodedByte);
//  }
//  while (X > 0);
//  Serial.write(topiclength >> 8);
//  Serial.write(topiclength & 0xFF);
//  Serial.print(str);
//  Serial.write(0x1A);
//
//  delay(8000);
//  
//  
//}
//void SendConnectPacket()
//{
//  /* CIP SEND */
//  Serial.println("AT+CIPSEND=66");
//  delay(8000);
//  Serial.write(0x10);
//  MQTTProtocolNameLength = strlen(MQTT_Protocol_Name);
//  MQTTClientIDLength = strlen(clientID);
//  MQTTUsernameLength = strlen(MQTT_Username);
//  MQTTPasswordLength = strlen(MQTT_Password);
//  datalength = MQTTProtocolNameLength + 2 + 4 + MQTTClientIDLength + 2 + MQTTUsernameLength + 2 + MQTTPasswordLength + 2;
//  X = datalength;
//  do {
//    encodedByte = X % 128;
//    X = X / 128;
//    if (X > 0) {
//      encodedByte |= 128;
//    }
//    Serial.write(encodedByte);
//  }
//  while (X > 0);
//  Serial.write(MQTTProtocolNameLength >> 8);
//  Serial.write(MQTTProtocolNameLength & 0xFF);
//  Serial.print(MQTT_Protocol_Name);
//  Serial.write(MQTT_Lvl); // LVL
//  Serial.write(MQTT_Flags); // Flags
//  Serial.write(MQTT_KeepAlive >> 8);
//  Serial.write(MQTT_KeepAlive & 0xFF);
//  Serial.write(MQTTClientIDLength >> 8);
//  Serial.write(MQTTClientIDLength & 0xFF);
//  Serial.print(clientID);
//  Serial.write(MQTTUsernameLength >> 8);
//  Serial.write(MQTTUsernameLength & 0xFF);
//  Serial.print(MQTT_Username);
//  Serial.write(MQTTPasswordLength >> 8);
//  Serial.write(MQTTPasswordLength & 0xFF);
//  Serial.print(MQTT_Password);
//  Serial.write(0x1A);
//  
//}
