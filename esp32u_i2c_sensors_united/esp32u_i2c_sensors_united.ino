/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/

#include "SI114X.h"
#include "Arduino.h"

#include <Wire.h>

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include "MHZ19.h"

//UV sensor SI1145 variables

float uv;
float visible;
float ir;
SI114X SI1145 = SI114X(); // initialise sunlight sensor

//BMP280 variables and macros
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI


//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);


//Co2 Sensor MH-Z19
const int rx_pin = 16; //Serial rx pin no
const int tx_pin = 17; //Serial tx pin no
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);


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


void setup()
{
  Serial.begin(115200);
  SI1145_setup();
  BME280_setup();
  TSL2591_setup();
  mhz19_setup();
  delay(1000);
}


void loop()
{
  
  for(int i=0;i<30;i++)
  {
    Serial.println();
  }

  readSensor();

  delay(2000);
}


void readSensor() // function to read sensor values and print with Serial library
{
  read_SI1145(); // UV Sensor
  
  read_BME280(); //Pressure, Temprature and Humidity Sensor

  advancedRead_TSL2591(); // light sensor

  read_mhz19(); //Co2 Sensor

  read_MQ7(); // MQ7 CO Sensor

  read_MQ4(); // MQ4 METHANE Sensor

  read_MIX8410(); // MIX8410 O2 Sensor
}


///////////SETUP FUNCTIONS///////////
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


void BME280_setup()
{
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));

  unsigned status;
  
  // default settings
  status = bme.begin(0x76);  
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
//        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  
  Serial.println("-- Default Test --");
  Serial.println();
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


void mhz19_setup(void)
{
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(true); //AUTO CALIBRATION// IN ORDER TO AUTOCALIBRATE SET setAutoCalibration(true) and calibrateSpan(5000)
  mhz19_uart->calibrateZero(); //TO CALIBRATE REMOVE COMMENT LINE
  mhz19_uart->calibrateSpan(5000);

  delay(3000); // Issue #14
  Serial.print("MH-Z19 now warming up...  status:");
  Serial.println(mhz19_uart->getStatus());
}



void read_SI1145(void)
{
  visible = SI1145.ReadVisible(); // visible radiation
  ir = SI1145.ReadIR(); // IR radiation
  uv = SI1145.ReadUV(); // UV index

  Serial.print("\n======== UV SENSOR ========\n");
  Serial.print("visible radiation: ");
  Serial.println(visible);
  Serial.print("IR radiation: ");
  Serial.println(ir);
  Serial.print("UV index: ");
  Serial.println(uv);
  //Serial.println("========  end print  ========");
}


void read_BME280(void)
{
  Serial.print("\n======== BME280 ========\n");
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" Celcius");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F );
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  //Serial.println("========  end print  ========");
}

void advancedRead_TSL2591(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print("\n======== TSL2591 ========\n");
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] \n"));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  \n"));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  \n"));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  \n"));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
  //Serial.println("========  end print  ========");
}

void read_mhz19(void)
{
  measurement_t m = mhz19_uart->getMeasurement();

  Serial.print("\n======== MH-Z19 ========\n");
  Serial.print("co2: ");
  Serial.println(m.co2_ppm);

  Serial.print("temp: ");
  Serial.println(m.temperature);
  //Serial.println("========  end print  ========");
}


void read_MQ7(void) //CO Sensor
{
  CO_Aout = analogRead(MQ7_input);  /*Analog value read function*/
  CO_gas_val = (9800/adc_resolution)*CO_Aout+200;
  Serial.print("\n======== MQ-7 CO ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(CO_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(CO_gas_val);
  Serial.print("ppm\t");
  if (CO_gas_val > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
  //Serial.println("========  end print  ========");
}


void read_MQ4(void)
{
  met_Aout = analogRead(MQ4_input);  /*Analog value read function*/
  met_gas_val = (9800/adc_resolution)*met_Aout+200;
  Serial.print("\n======== MQ-4 METHANE ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(met_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(met_gas_val);
  Serial.print("ppm\t");
  if (met_gas_val > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
  //Serial.println("========  end print  ========");
}


void read_MIX8410(void)
{
  float Vout =0;
  Serial.print("\n======== MIX8410 O2 ========\n");
  //Serial.print("Vout =");
  Vout = readO2Vout();
  //Serial.print(Vout);
  Serial.print("O2 Concentration: ");
  Serial.println(readConcentration());
  //Serial.println("========  end print  ========");
}


//MIX8410 O2 Sensor Helper Func
float readO2Vout()
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
float readConcentration()
{
    // Vout samples are with reference to 3.3V
    float MeasuredVout = readO2Vout();
 
    //float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
    //when its output voltage is 2.0V,
    float Concentration = MeasuredVout * 0.21 / 2.0;
    float Concentration_Percentage=Concentration*100;
    return Concentration_Percentage;
}


