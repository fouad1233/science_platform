#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_TCS34725.h"

#define DEBUG 1

#if DEBUG == 1    
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

/*Send data structure to xaiver*/
//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

//Color Sensor TCS34725
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

typedef struct{

  float temp_BME;
  float pressure;
  float altitude;
  float humidity;
  
  uint32_t lum;
  uint16_t ir_TSL, full, visible_TSL, lux;
  
  float CO_gas_val;
  float met_gas_val;
  int o2_concentration; //O2 sensor o2 concentration

  uint16_t r, g, b, c, colorTemp; //Color sensor
  
  /*
  float uv;
  float visible;
  float ir;
  
  int co2_concentration = 0;
  int co2_temp = 0;
  */

}struct_send_message;
// Creating sending and receiving structure objects
struct_send_message myData;
void TSL2591_setup(void);
void TCS34725_setup(void);
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  TSL2591_setup();
  TCS34725_setup();

}

void loop() {
  // put your main code here, to run repeatedly:
  read_TSL2591(); // light sensor
  read_TCS34725(); // Color sensor

  print_TSL2591();
  print_TCS34725();


}
//Removed the while loop to prevent waiting at start if the sensor is not connected
void TSL2591_setup(void)
{ 
  
  //////begin the sensor
  if (tsl.begin()) 
  {
    
  } 
  else 
  {
    debugln(F("No sensor found ... check your wiring?"));
    //while (1);  //??????????????
  }
  //////configure the sensor
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

   /* Display the gain and integration time for reference sake */  
  
  tsl2591Gain_t gain = tsl.getGain();
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


void read_TCS34725(void){
  tcs.getRawData(&myData.r, &myData.g, &myData.b, &myData.c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  myData.colorTemp = tcs.calculateColorTemperature_dn40(myData.r, myData.g, myData.b, myData.c);
  //lux = tcs.calculateLux(r, g, b);

}
void print_TSL2591(void)
{
  Serial.print("\n======== TSL2591 ========\n");
  Serial.print(F("IR: ")); 
  Serial.print(myData.ir_TSL);  
  Serial.print(F("  \n"));
  Serial.print(F("Full: ")); 
  Serial.print(myData.full); 
  Serial.print(F("  \n"));
  Serial.print(F("Visible: ")); 
  Serial.print(myData.visible_TSL); 
  Serial.print(F("  \n"));
  Serial.print(F("Lux: ")); 
  Serial.println(myData.lux, 6);
}

void print_TCS34725(void){
  Serial.print("Color Temp: "); Serial.print(myData.colorTemp, DEC); Serial.print(" K - ");
  Serial.print("R: "); Serial.print(myData.r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(myData.g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(myData.b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(myData.c, DEC); Serial.print(" ");
  Serial.println(" ");

}
