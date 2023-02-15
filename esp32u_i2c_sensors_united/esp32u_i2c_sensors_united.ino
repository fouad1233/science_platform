/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/

#include "Arduino.h"
#include "SI114X.h"
#include <Wire.h>

#include "DFRobot_BMP280.h"


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

}

void setup()
{
  Serial.begin(115200);
  Serial.println("Beginning Si1145!");
  while (!SI1145.Begin())
  {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
  Serial.println("Si1145 is ready!");
  delay(1000);

/////////
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

  delay(100);
//////////
}

void loop()
{
  sendSensor();
  delay(1000);
}
