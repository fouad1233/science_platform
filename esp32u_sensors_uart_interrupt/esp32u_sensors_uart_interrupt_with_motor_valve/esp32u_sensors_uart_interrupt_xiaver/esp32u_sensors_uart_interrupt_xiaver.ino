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

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_TCS34725.h"

// TIMER SETUP
uint8_t read_sensor_flag; //TIMER FLAG
hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1;
}


// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAC, 0x2D, 0xCC}; //40:91:51:AC:2D:CC

//Color Sensor TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
// Define a data structure
typedef struct{
  
  float temp_BME;
  float pressure;
  float altitude;
  float humidity;

  uint32_t lum;
  uint16_t ir_TSL, full, visible_TSL;
  float lux;
  
  float CO_gas_val;

  float met_gas_val;
  int o2_concentration;

  //uint16_t r, g, b, c, colorTemp; //Color sensor

  /*
  float uv;
  float visible;
  float ir;

  int co2_concentration;
  int co2_temp;
  */
  
} struct_receive_message;
typedef struct{
  uint16_t r, g, b, c, colorTemp;
} struct_color_sensor;

typedef struct{
  int motorFlag = 0;
  int tube_to_go = 1;
  int received_states[5] = {1,1,1,1,1};  // 0,1,2,3 indexler->1,2,3,4 RELAY  5 pump(pwm) 
  
}struct_motor_message;

// Create a structured object
struct_receive_message myData;
struct_motor_message motorData;
struct_color_sensor colorSensor;




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
      motorData.motorFlag = xiaver["motorFlag"];
      motorData.tube_to_go = xiaver["tube_to_go"];
      //motorData.received_states =
      //memcpy(motorData.received_states, xiaver["received_status"], 5*sizeof(int));

      for(int i=0;i<=4;i++)
      {
        motorData.received_states[i] = xiaver["received_states"][i];
      }

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
void TCS34725_setup(void);
void read_TCS34725(void);
void print_TCS34725(void);
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
  //sensor begin 
  TCS34725_setup();
}


void loop()
{
  if (read_sensor_flag ){

    json_data_set_esp_now();
    serializeJson(doc, Serial);
    Serial.println();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motorData, sizeof(motorData));
    motorData.motorFlag = 0;
    // Serial.println();
    // if (result == ESP_OK) {
    //   Serial.println("Sending confirmed");
    // }
    // else {
    //   Serial.println("Sending error");
    // }
    read_TCS34725();
    
    read_sensor_flag = 0;
  }
}

void TCS34725_setup(void)
{

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1);
  }

}
void read_TCS34725(void){
  tcs.getRawData(&colorSensor.r, &colorSensor.g, &colorSensor.b, &colorSensor.c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorSensor.colorTemp = tcs.calculateColorTemperature_dn40(colorSensor.r, colorSensor.g, colorSensor.b, colorSensor.c);
  //lux = tcs.calculateLux(r, g, b);

}

void print_TCS34725(void){
  Serial.print("Color Temp: "); Serial.print(colorSensor.colorTemp, DEC); Serial.print(" K - ");
  Serial.print("R: "); Serial.print(colorSensor.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(colorSensor.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(colorSensor.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(colorSensor.c, DEC); Serial.print(" ");
  Serial.println(" ");

}

void json_data_set_esp_now(void){
  doc["temp"] = myData.temp_BME; // float
  doc["humidity"] = myData.humidity; // float
  doc["pressure"] = myData.pressure; // float
  doc["altitude"] = myData.altitude; // float


  doc["ir_TSL"] = myData.ir_TSL; // uint16_t 
  //doc["full"] = full; // uint16_t
  doc["visible_TSL"] = myData.visible_TSL; // uint16_t
  doc["lux"] = myData.lux; // float


  doc["CO_gas_val"] = myData.CO_gas_val; //float

  doc["met_gas_val"] = myData.met_gas_val;  //float

  doc["o2_concentration"] = myData.o2_concentration; //int

  doc["red"] = colorSensor.r;
  doc["green"] = colorSensor.g;
  doc["blue"] = colorSensor.b;
  doc["c"] = colorSensor.c;
  doc["color_temp"] = colorSensor.colorTemp;



  /*
  doc["uv_SI"] = myData.uv; //float 
  doc["visible_SI"] = myData.visible; //float 
  doc["ir_SI"] = myData.ir; //float 
  doc["co2_concentration"] = 0; // int  
*/
}