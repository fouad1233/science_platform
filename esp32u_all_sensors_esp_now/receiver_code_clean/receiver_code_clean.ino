/*
This file is used to get sensor values from esp now and transmit it with serial
*/
// Include Libraries for espnow
#include <esp_now.h>
#include <WiFi.h>

#include "Arduino.h"

#include <ArduinoJson.h>


uint8_t read_sensor_flag; //TIMER FLAG

// Define a data structure
typedef struct{
  float uv;
  float visible;
  float ir;
  float temp_BME;
  float pressure;
  float altitude;
  float humidity;
  uint32_t lum;
  uint16_t ir_TSL, full, visible_TSL, lux;
  float CO_gas_val ;
  int co2_concentration;
  int co2_temp;
  float met_gas_val;
  int o2_concentration;
} struct_message;

// Create a structured object
struct_message myData;



// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}


void json_data_set(void); // JSON FUNCTION
StaticJsonDocument<300> doc;

// TIMER SETUP
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1;
}


void setup()
{
  Serial.begin(115200);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register the send callback
  esp_now_register_recv_cb(OnDataRecv);

  
  //Timer Configurations
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer); //Just Enable
}


void loop()
{
  if (read_sensor_flag ){

    json_data_set_esp_now();
    serializeJson(doc, Serial);
    Serial.println();
    read_sensor_flag = 0;
  }
}


void json_data_set_esp_now(void){
  doc["temp"] = myData.temp_BME; // float
  doc["humidity"] = myData.humidity; // float
  doc["pressure"] = myData.pressure; // float
  doc["altitude"] = myData.altitude; // float

  doc["uv_SI"] = myData.uv; //float
  doc["visible_SI"] = myData.visible; //float
  doc["ir_SI"] = myData.ir; //float

  doc["ir_TSL"] = myData.ir_TSL; // uint16_t 
  //doc["full"] = full; // uint16_t
  doc["visible_TSL"] = myData.visible_TSL; // uint16_t
  doc["lux"] = myData.lux; // uint16_t

  doc["co2_concentration"] = myData.co2_concentration; // int

  doc["CO_gas_val"] = myData.CO_gas_val ; //float

  doc["met_gas_val"] = myData.met_gas_val ;  //float

  doc["o2_concentration"] = myData.o2_concentration; //int
  
}