/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/
// Include Libraries for espnow
#include <esp_now.h>
#include <WiFi.h>

#include "SI114X.h"
#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include "MHZ19.h"

uint8_t read_sensor_flag; //TIMER FLAG
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAC, 0x28, 0x38}; //40:91:51:AC:28:38

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
  float CO_gas_val;
  int co2_concentration;
  int co2_temp;
  float met_gas_val;
  int o2_concentration; //O2 sensor o2 concentration
} struct_send_message;

typedef struct{
  bool motorFlag = false;
  int tube_to_go;
}struct_motor_message;

// Create a structured object
struct_send_message myData;
struct_motor_message motorData;


// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&motorData, incomingData, sizeof(motorData));
}

//UV sensor SI1145
SI114X SI1145 = SI114X(); // initialise sunlight sensor


//BME280
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme; // I2C


//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);


//Co2 Sensor MH-Z19
const int rx_pin = 16; //Serial rx pin no
const int tx_pin = 17; //Serial tx pin no
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);



//CO Sensor MQ7 and CH4 Sensor MQ4
int adc_resolution = 4095;
#define MQ7_input 27    /*Digital pin 5 for sensor input*/
int CO_Aout;
#define MQ4_input 33
int met_Aout;


//O2 Sensor MIX8410
const float VRefer = 3.3; // voltage of adc reference
const int O2_pin = 4;
float MeasuredVout;
float Concentration;
float Concentration_Percentage;


//StepMotor Definings
#define stepPin 14 
#define dirPin 12 
int freq = 624;
const int stepChannel = 0;
const int resolution = 8;
int halfDutyCycle = 128;
int lowDutyCycle = 0;
int angle = 90;
unsigned long previousMillis = 0; 
unsigned long currentMillis;
long interval = (57*200*angle*1000)/(11*360*freq);  // interval at which to stop PWM (miliseconds)



//Functions
void readAllSensors(void);
void stepmotor_setup(void);
void SI1145_setup(void);
void BME280_setup(void);
void TSL2591_setup(void);
void mhz19_setup(void);
void read_SI1145(void);
void read_BME280(void);
void read_TSL2591(void);
void read_mhz19(void);
void read_MQ7(void);
void read_MQ4(void);
void read_MIX8410(void);
float readO2Vout(void);
float readConcentration(void);


// TIMER SETUP
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1;
  //readSensor();
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
  esp_now_register_recv_cb(OnDataRecv);
  
  SI1145_setup();
  BME280_setup();
  TSL2591_setup();
  mhz19_setup();
  delay(3000);
  
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer); //Just Enable

  stepmotor_setup();
}


void loop()
{
  if (read_sensor_flag ){
    readAllSensors();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    Serial.println();
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }

    read_sensor_flag = 0;
  }

  
  if (motorData.motorFlag){
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      ledcWrite(stepChannel, lowDutyCycle);
      motorData.motorFlag = false;
    }
  }
}


void readAllSensors(void) // function to read sensor values and print with Serial library
{
  read_SI1145(); // UV Sensor
  
  read_BME280(); //Pressure, Temprature and Humidity Sensor

  read_TSL2591(); // light sensor

  read_mhz19(); //Co2 Sensor

  read_MQ7(); // MQ7 CO Sensor

  read_MQ4(); // MQ4 METHANE Sensor

  read_MIX8410(); // MIX8410 O2 Sensor
}


void stepmotor_setup(void){
  //Stepmotor setup
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  digitalWrite(dirPin,HIGH); // Setting direction to clockwise
}



///////////SETUP FUNCTIONS///////////
void SI1145_setup(void)
{
  while (!SI1145.Begin())
  {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
}


void BME280_setup(void)
{
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));

  unsigned status;
  
  // default settings
  status = bme.begin(0x76);  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      while (1) delay(10);
  }
}


void TSL2591_setup(void)
{ 
  
  //////begin the sensor
  if (tsl.begin()) 
  {
    
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  //////configure the sensor
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

   /* Display the gain and integration time for reference sake */  
  
  tsl2591Gain_t gain = tsl.getGain();
  }


void mhz19_setup(void)
{
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(true); //AUTO CALIBRATION// IN ORDER TO AUTOCALIBRATE SET setAutoCalibration(true) and calibrateSpan(5000)
  mhz19_uart->calibrateZero(); //TO CALIBRATE REMOVE COMMENT LINE
  mhz19_uart->calibrateSpan(5000);

  //delay(3000); // Issue #14 //
}



void read_SI1145(void)
{
  myData.visible = SI1145.ReadVisible(); // visible radiation
  myData.ir = SI1145.ReadIR(); // IR radiation
  myData.uv = SI1145.ReadUV(); // UV index
}


void read_BME280(void)
{
  myData.temp_BME = bme.readTemperature();
  myData.pressure = bme.readPressure() / 100.0F;
  myData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  myData.humidity = bme.readHumidity();
}


void read_TSL2591(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!

  //uint32_t lum = tsl.getFullLuminosity();
  //uint16_t ir, full;

  myData.lum = tsl.getFullLuminosity();
  myData.ir_TSL = myData.lum >> 16;
  myData.full = myData.lum & 0xFFFF;
  myData.visible_TSL = myData.full - myData.ir_TSL;
  myData.lux = tsl.calculateLux(myData.full, myData.ir_TSL);
}



void read_mhz19(void)
{
  measurement_t m = mhz19_uart->getMeasurement();
  myData.co2_concentration = m.co2_ppm;
  myData.co2_temp = m.temperature;
}



void read_MQ7(void) //CO Sensor
{
  CO_Aout = analogRead(MQ7_input);  /*Analog value read function*/
  myData.CO_gas_val = (9800/adc_resolution)*CO_Aout+200;
}



void read_MQ4(void)
{
  met_Aout = analogRead(MQ4_input);  /*Analog value read function*/
  myData.met_gas_val = (9800/adc_resolution)*met_Aout+200;
}



void read_MIX8410(void)
{
  myData.o2_concentration = readConcentration();
}



//MIX8410 O2 Sensor Helper Func
float readO2Vout(void)
{
    long sum = 0;
    for(int i=0; i<32; i++)
    {
        sum += analogRead(O2_pin);
    }
 
    sum >>= 5;
 
    float MeasuredVout = sum * (VRefer / 1023.0);
    return MeasuredVout;
}


//MIX8410 O2 Sensor Helper Func
float readConcentration(void)
{
    // Vout samples are with reference to 3.3V
    MeasuredVout = readO2Vout();
 
    //float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
    //when its output voltage is 2.0V,
    Concentration = MeasuredVout * 0.21 / 2.0;
    Concentration_Percentage=Concentration*100;
    return Concentration_Percentage;
}
