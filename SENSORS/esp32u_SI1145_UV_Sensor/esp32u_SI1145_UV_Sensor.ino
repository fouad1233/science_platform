#include "Arduino.h"
#include "SI114X.h" // library to read SI1145
#include <Wire.h> //library for I2C bus
#include "SoftwareSerial.h"

float uv;
float visible;
float ir;

SI114X SI1145 = SI114X(); // initialise sunlight sensor

void sendSensor() // function to read sensor values and send them to Blynk
{
  visible = SI1145.ReadVisible(); // visible radiation
  ir = SI1145.ReadIR(); // IR radiation
  uv = SI1145.ReadUV(); // UV index
  // the UV index is multiplied by 100 so to get the integer index, divided by 100!
  Serial.print("=================================\n");
  Serial.print("visible radiation: ");
  Serial.println(visible);
  Serial.print("IR radiation: ");
  Serial.println(ir);
  Serial.print("UV index: ");
  Serial.println(uv/100);
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
  }

void loop()
{
  sendSensor();
  delay(1000);
}
