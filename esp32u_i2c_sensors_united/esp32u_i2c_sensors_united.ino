/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/

#include "SI114X.h"
#include "Arduino.h"

#include <Wire.h>

#include "DFRobot_BMP280.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"


//UV sensor variables

float uv;
float visible;
float ir;

//BMP280 variables and macros
float temp;
uint32_t press;
float alti;
#define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********
BMP   bmp(&Wire, BMP::eSdoLow);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// show BMP280 last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch(eStatus) {
    case BMP::eStatusOK:
      Serial.println("everything ok");
      break;
    case BMP::eStatusErr:   
      Serial.println("unknow error");
      break;
    case BMP::eStatusErrDeviceNotDetected:
      Serial.println("device not detected");
      break;
    case BMP::eStatusErrParameter:
      Serial.println("parameter error");
      break;
    default: 
      Serial.println("unknow status");
      break;
  }
}

SI114X SI1145 = SI114X(); // initialise sunlight sensor

void sendSensor() // function to read sensor values and print with Serial library
{
//////// UV Sensor
  visible = SI1145.ReadVisible(); // visible radiation
  ir = SI1145.ReadIR(); // IR radiation
  uv = SI1145.ReadUV(); // UV index

//////// Pressure Sensor
  
  temp = bmp.getTemperature();
  press = bmp.getPressure();
  alti = bmp.calAltitude(SEA_LEVEL_PRESSURE, press);
//////// tsl2591
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;

  Serial.print("\n======== UV SENSOR ========\n");
  Serial.print("visible radiation: ");
  Serial.println(visible);
  Serial.print("IR radiation: ");
  Serial.println(ir);
  Serial.print("UV index: ");
  Serial.println(uv/100);
  
  Serial.print("\n======== BMP280 ========\n");
  Serial.print("temperature (unit Celsius): "); Serial.println(temp);
  Serial.print("pressure (unit pa):         "); Serial.println(press);
  Serial.print("altitude (unit meter):      "); Serial.println(alti);
  Serial.println("========  end print  ========");

  Serial.print("\n======== TSL2591 ========\n");
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("\nFull: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("\nVisible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("\nLux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
  Serial.println("========  end print  ========");
  
}


void SI1145_setup()
{
  Serial.println("Beginning Si1145!");
  while (!SI1145.Begin())
  {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
  Serial.println("Si1145 is ready!");
}


void BMP280_setup()
{
  bmp.reset();
  Serial.println("bmp config test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");

  bmp.setConfigFilter(BMP::eConfigFilter_off);        // set config filter
  bmp.setConfigTStandby(BMP::eConfigTStandby_125);    // set standby time
  bmp.setCtrlMeasSamplingTemp(BMP::eSampling_X8);     // set temperature over sampling
  bmp.setCtrlMeasSamplingPress(BMP::eSampling_X8);    // set pressure over sampling
  bmp.setCtrlMeasMode(BMP::eCtrlMeasModeNormal);     // set control measurement mode to make these settings effective
}
void TSL2591_setup()
{ 
  
  //////begin the sensor
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
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
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
    }
    Serial.print  (F("Timing:       "));
    Serial.print((tsl.getTiming() + 1) * 100, DEC); 
    Serial.println(F(" ms"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
  }
  
  

void advancedRead_TSL2591(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}
void setup()
{
  // Serial.begin(115200);
  // Serial.println("Beginning Si1145!");
  // while (!SI1145.Begin())
  // {
  //   Serial.println("Si1145 is not ready!");
  //   delay(1000);
  // }
  // Serial.println("Si1145 is ready!");
  //delay(1000);

/////////
  // bmp.reset();
  // Serial.println("bmp config test");
  // while(bmp.begin() != BMP::eStatusOK) {
  //   Serial.println("bmp begin faild");
  //   printLastOperateStatus(bmp.lastOperateStatus);
  //   delay(2000);
  // }
  // Serial.println("bmp begin success");

  // bmp.setConfigFilter(BMP::eConfigFilter_off);        // set config filter
  // bmp.setConfigTStandby(BMP::eConfigTStandby_125);    // set standby time
  // bmp.setCtrlMeasSamplingTemp(BMP::eSampling_X8);     // set temperature over sampling
  // bmp.setCtrlMeasSamplingPress(BMP::eSampling_X8);    // set pressure over sampling
  // bmp.setCtrlMeasMode(BMP::eCtrlMeasModeNormal);     // set control measurement mode to make these settings effective
  //delay(100);
  Serial.begin(115200);
  SI1145_setup();
  BMP280_setup();
  TSL2591_setup();
  delay(1000);
//////////
}

void loop()
{
  sendSensor();
  delay(1000);
}
