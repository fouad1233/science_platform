#include <Arduino.h>
#include <string.h>
#define pumpPin 25
#define pumpChannel 0
int freq = 50;//425
const int resolution = 8;
int pumpDutyCycle = 256*14/100;
int input;

void setup() {
  Serial.begin(115200);
  pinMode(25, OUTPUT);
}

void loop() {
  /*
  if(Serial.available()){
    input = Serial.parseInt();
    if (input){
      freq = input;
      ledcSetup(pumpChannel, freq, resolution);
      ledcWrite(pumpChannel, pumpDutyCycle);
      Serial.print("Freq: ");
      Serial.print(freq);      
      Serial.println();
    }
    */

    // input = Serial.parseInt();
    // if (input){
    //   // freq = input;
    //   pumpDutyCycle = input;
    //   ledcWrite(pumpChannel, pumpDutyCycle);
    //   Serial.print("Duty Cycle: ");
    //   Serial.print(pumpDutyCycle);
    //   Serial.println();
    // }
  digitalWrite(pumpPin, HIGH);
  Serial.println("acık");
  delay(3000);
  digitalWrite(pumpPin, LOW);
  Serial.println("kapalı");
  delay(3000);
  
}