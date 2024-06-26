/*
This file is used to get sensor values from esp now and transmit it with serial
*/
// Include Libraries for espnow
#include <esp_now.h>
#include <WiFi.h>

#include "Arduino.h"

#include <ArduinoJson.h>

#include <Wire.h>
#include "Arduino.h"


// TIMER SETUP
uint8_t read_sensor_flag; //TIMER FLAG
hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1;
}


// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAC, 0x2D, 0xCC}; //40:91:51:AC:2D:CC


// Define a data structure
typedef struct{
  float temp_BME;
  float pressure;
  float altitude;
  float humidity;

  float met_gas_val;
  
  float weight_average; //hx711 new
} struct_receive_message;

typedef struct{
  int motorState = 0;  
  //AGIRLIK OLCUMU ICIN ROVERDAN GELEN ACI VERISI EKLENECEK
}struct_motor_message;

// Create a structured object
struct_receive_message myData;
struct_motor_message motorData;

// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Callback function uart interrupt
void UART_RX_IRQ() {
  uint16_t size = Serial.available();
  //Serial.printf("Got %d bytes on Serial to read\n", size);
  // Read the incoming data
  String json_str = Serial.readString();
  // Declare a JSON document object
  StaticJsonDocument<200> xiaver;
  // Decode the JSON string into the document object
  DeserializationError err = deserializeJson(xiaver, json_str);
  // Check if decoding was successful
    if (err == DeserializationError::Ok) {
      // Get the values of the fields in the document
      motorData.motorState = xiaver["motorState"];

    } else {
      // Print an error message
      Serial.print("JSON decoding failed: ");
      Serial.println(err.c_str());
    }
  //while(Serial.available())  {
    //Serial.write(Serial.read());
  //}
  //Serial.printf("\nSerial data processed!\n");
}

void json_data_set_esp_now(void); // JSON FUNCTION

StaticJsonDocument<300> doc;




void setup()
{
  Serial.begin(115200);
  //Callback function activate
  Serial.onReceive(UART_RX_IRQ);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //Timer Configurations
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000/2, true);  //Updated 1 second to 500 ms
  timerAlarmEnable(My_timer); //Just Enable
}


void loop()
{
  if (read_sensor_flag ){

    json_data_set_esp_now();
    serializeJson(doc, Serial);
    Serial.println();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motorData, sizeof(motorData));
    //motorData.motorFlag = 0;
    
    read_sensor_flag = 0;
  }
}


void json_data_set_esp_now(void){
  doc["temp"] = myData.temp_BME;                 // float
  doc["humidity"] = myData.humidity;             // float
  doc["pressure"] = myData.pressure;             // float
  doc["altitude"] = myData.altitude;             // float

  doc["met_gas_val"] = myData.met_gas_val;       // float
  doc["weight_average"] = myData.weight_average; // float
}