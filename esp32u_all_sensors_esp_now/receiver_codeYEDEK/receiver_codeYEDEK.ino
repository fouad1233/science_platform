/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/
// Include Libraries for espnow
#include <esp_now.h>
#include <WiFi.h>

#include "SI114X.h"
#include "Arduino.h"

#include <Wire.h>
#include <ArduinoJson.h>

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include "MHZ19.h"

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
 

//UV sensor SI1145 variables
float uv;
float visible;
float ir;
SI114X SI1145 = SI114X(); // initialise sunlight sensor

//BME280 variables and macros
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme; // I2C
float temp_BME;
float pressure;
float altitude;
float humidity;


//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint32_t lum;
uint16_t ir_TSL, full, visible_TSL, lux;


//Co2 Sensor MH-Z19
const int rx_pin = 16; //Serial rx pin no
const int tx_pin = 17; //Serial tx pin no
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);
int co2_concentration;
int co2_temp;


//CO Sensor MQ7 and CH4 Sensor MQ4
int adc_resolution = 4095;
#define MQ7_input 27    /*Digital pin 5 for sensor input*/
float CO_gas_val = 0;
int CO_Aout;

#define MQ4_input 33
float met_gas_val = 0;
int met_Aout;



//O2 Sensor MIX8410
const float VRefer = 3.3;       // voltage of adc reference
const int O2_pin = 4;
int o2_concentration; //O2 sensor o2 concentration

//Functions
void readAllSensors(void);
void SI1145_setup(void);
void BME280_setup(void);
void TSL2591_setup(void);
void mhz19_setup(void);
void read_SI1145(void);
void read_BME280(void);
void advancedRead_TSL2591(void);
void read_mhz19(void);
void read_MQ7(void);
void read_MQ4(void);
void read_MIX8410(void);
float readO2Vout(void);
float readConcentration(void);

void printAllSensors(void);
void print_SI1145(void);
void print_BME280(void);
void print_TSL2591(void);
void print_mhz19(void);
void print_MQ7(void);
void print_MQ4(void);
void print_MIX8410(void);

void json_data_set(void); // JSON FUNCTION
StaticJsonDocument<300> doc;

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
  esp_now_register_recv_cb(OnDataRecv);

  


  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer); //Just Enable
}


void loop()
{
  if (read_sensor_flag ){
    //readAllSensors();
    //printAllSensors();
    
    

    //json_data_set();
    json_data_set_esp_now();
    serializeJson(doc, Serial);
    Serial.println();
    read_sensor_flag = 0;
  }

  //delay(10000);
}


void readAllSensors(void) // function to read sensor values and print with Serial library
{
  read_SI1145(); // UV Sensor
  
  read_BME280(); //Pressure, Temprature and Humidity Sensor

  advancedRead_TSL2591(); // light sensor

  read_mhz19(); //Co2 Sensor

  read_MQ7(); // MQ7 CO Sensor

  read_MQ4(); // MQ4 METHANE Sensor

  read_MIX8410(); // MIX8410 O2 Sensor
}

void printAllSensors(void)
{
  print_SI1145();
  print_BME280();
  print_TSL2591();
  print_mhz19();
  print_MQ7();
  print_MQ4();
  print_MIX8410();
}

void json_data_set(void)
{
  doc["temp"] = round(temp_BME * 100) / 100; // float
  doc["humidity"] = round(humidity * 100) / 100; // float
  doc["pressure"] = round(pressure * 100) / 100; // float
  doc["altitude"] = round(altitude * 100) / 100; // float


  doc["ir_TSL"] = ir_TSL; // uint16_t 
  //doc["full"] = full; // uint16_t
  doc["visible_TSL"] = visible_TSL; // uint16_t
  doc["lux"] = lux; // uint16_t

  doc["co2_concentration"] = co2_concentration; // int

  doc["CO_gas_val"] = round(CO_gas_val * 100) / 100; //float

  doc["met_gas_val"] = round(met_gas_val * 100) / 100;  //float

  doc["o2_concentration"] = o2_concentration; //int
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
void esp_now_data_set(void)
{
  myData.temp_BME = round(temp_BME * 100) / 100; // float
  myData.humidity = round(humidity * 100) / 100; // float
  myData.pressure = round(pressure * 100) / 100; // float
  myData.altitude = round(altitude * 100) / 100; // float


  myData.ir_TSL = ir_TSL; // uint16_t 
  //doc["full"] = full; // uint16_t
  myData.visible_TSL = visible_TSL; // uint16_t
  myData.lux = lux; // uint16_t

  myData.uv = uv;
  myData.visible = visible;
  myData.ir = ir;

  myData.co2_concentration = co2_concentration; // int

  myData.CO_gas_val = round(CO_gas_val * 100) / 100; //float

  myData.met_gas_val = round(met_gas_val * 100) / 100;  //float

  myData.o2_concentration = o2_concentration; //int
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
  visible = SI1145.ReadVisible(); // visible radiation
  ir = SI1145.ReadIR(); // IR radiation
  uv = SI1145.ReadUV(); // UV index

  // Serial.print("\n======== UV SENSOR ========\n");
  // Serial.print("visible radiation: ");
  // Serial.println(visible);
  // Serial.print("IR radiation: ");
  // Serial.println(ir);
  // Serial.print("UV index: ");
  // Serial.println(uv);
}

void print_SI1145(void)
{
  Serial.print("\n======== UV SENSOR ========\n");
  Serial.print("visible radiation: ");
  Serial.println(visible);
  Serial.print("IR radiation: ");
  Serial.println(ir);
  Serial.print("UV index: ");
  Serial.println(uv);
}


void read_BME280(void)
{
  temp_BME = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();


  // Serial.print("\n======== BME280 ========\n");
  // Serial.print("Temperature = ");
  // Serial.print(temp_BME);
  // Serial.println(" Celcius");

  // Serial.print("Pressure = ");
  // Serial.print(pressure);
  // Serial.println(" hPa");

  // Serial.print("Approx. Altitude = ");
  // Serial.print(altitude);
  // Serial.println(" m");

  // Serial.print("Humidity = ");
  // Serial.print(humidity);
  // Serial.println(" %");
}

void print_BME280(void)
{
  Serial.print("\n======== BME280 ========\n");
  Serial.print("Temperature = ");
  Serial.print(temp_BME);
  Serial.println(" Celcius");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
}

void advancedRead_TSL2591(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!

  //uint32_t lum = tsl.getFullLuminosity();
  //uint16_t ir, full;

  lum = tsl.getFullLuminosity();
  ir_TSL = lum >> 16;
  full = lum & 0xFFFF;
  visible_TSL = full - ir_TSL;
  lux = tsl.calculateLux(full, ir_TSL);

  // Serial.print("\n======== TSL2591 ========\n");
  // Serial.print(F("IR: ")); 
  // Serial.print(ir_SI);  
  // Serial.print(F("  \n"));
  // Serial.print(F("Full: ")); 
  // Serial.print(full); 
  // Serial.print(F("  \n"));
  // Serial.print(F("Visible: ")); 
  // Serial.print(visible_SI); 
  // Serial.print(F("  \n"));
  // Serial.print(F("Lux: ")); 
  // Serial.println(lux, 6);
}

void print_TSL2591(void)
{
  Serial.print("\n======== TSL2591 ========\n");
  Serial.print(F("IR: ")); 
  Serial.print(ir_TSL);  
  Serial.print(F("  \n"));
  Serial.print(F("Full: ")); 
  Serial.print(full); 
  Serial.print(F("  \n"));
  Serial.print(F("Visible: ")); 
  Serial.print(visible_TSL); 
  Serial.print(F("  \n"));
  Serial.print(F("Lux: ")); 
  Serial.println(lux, 6);
}

void read_mhz19(void)
{
  measurement_t m = mhz19_uart->getMeasurement();
  co2_concentration = m.co2_ppm;
  co2_temp = m.temperature;

  // Serial.print("\n======== MH-Z19 ========\n");
  // Serial.print("co2: ");
  // Serial.println(co2_concentration);
  // Serial.print("temp: ");
  // Serial.println(co2_temp);
}

void print_mhz19(void)
{
  Serial.print("\n======== MH-Z19 ========\n");
  Serial.print("co2: ");
  Serial.println(co2_concentration);
  Serial.print("temp: ");
  Serial.println(co2_temp);
}


void read_MQ7(void) //CO Sensor
{
  CO_Aout = analogRead(MQ7_input);  /*Analog value read function*/
  CO_gas_val = (9800/adc_resolution)*CO_Aout+200;

  // Serial.print("\n======== MQ-7 CO ========\n");
  // Serial.print("Gas Sensor: ");  
  // Serial.print(CO_Aout);   /*Read value printed*/
  // Serial.print("\nGas Value: ");
  // Serial.print(CO_gas_val);
  // Serial.print("ppm\t");
  // if (CO_Aout > 1200) {    /*if condition with threshold 1800*/
  //   Serial.println("Gas");  
  //   //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  // }
  // else {
  //   Serial.println("No Gas");
  //   //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  // }
}

void print_MQ7(void)
{
  Serial.print("\n======== MQ-7 CO ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(CO_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(CO_gas_val);
  Serial.print("ppm\t");
  if (CO_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
}


void read_MQ4(void)
{
  met_Aout = analogRead(MQ4_input);  /*Analog value read function*/
  met_gas_val = (9800/adc_resolution)*met_Aout+200;

  // Serial.print("\n======== MQ-4 METHANE ========\n");
  // Serial.print("Gas Sensor: ");  
  // Serial.print(met_Aout);   /*Read value printed*/
  // Serial.print("\nGas Value: ");
  // Serial.print(met_gas_val);
  // Serial.print("ppm\t");
  // if (met_Aout > 1200) {    /*if condition with threshold 1800*/
  //   Serial.println("Gas");  
  //   //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  // }
  // else {
  //   Serial.println("No Gas");
  //   //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  // }
}

void print_MQ4(void)
{
  Serial.print("\n======== MQ-4 METHANE ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(met_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(met_gas_val);
  Serial.print("ppm\t");
  if (met_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
}


void read_MIX8410(void)
{
  o2_concentration = readConcentration();
  //float Vout =0;
  // Serial.print("\n======== MIX8410 O2 ========\n");
  // //Serial.print("Vout =");
  // //Vout = readO2Vout();
  // //Serial.print(Vout);
  // Serial.print("O2 Concentration: ");
  // Serial.println(o2_concentration);
}

void print_MIX8410(void)
{
  Serial.print("\n======== MIX8410 O2 ========\n");
  //Serial.print("Vout =");
  //Vout = readO2Vout();
  //Serial.print(Vout);
  Serial.print("O2 Concentration: ");
  Serial.println(o2_concentration);

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
    float MeasuredVout = readO2Vout();
 
    //float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
    //when its output voltage is 2.0V,
    float Concentration = MeasuredVout * 0.21 / 2.0;
    float Concentration_Percentage=Concentration*100;
    return Concentration_Percentage;
}
