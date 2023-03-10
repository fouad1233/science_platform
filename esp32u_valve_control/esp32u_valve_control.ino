#include <Arduino.h>

#define RELAY1 27
#define RELAY2 14
#define RELAY3 12

long input;
int states[] = {HIGH,HIGH,HIGH};
void printStates(void);

void setup() {
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2,OUTPUT);
  pinMode(RELAY3,OUTPUT);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2 ,HIGH);
  digitalWrite(RELAY3 ,HIGH);
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()){
    input = Serial.parseInt();
    switch (input)
    {
      case 1:
        if(states[input-1]){
          digitalWrite(RELAY1, LOW);
          states[input-1]=LOW; 
          Serial.println("Valve 1 açıldı");
        }
        else{
          digitalWrite(RELAY1,HIGH);
          states[input-1]=HIGH;
          Serial.println("Valve 1 kapandı");
        }
        printStates();
        break;
      
      case 2:
        if(states[input-1]){
          digitalWrite(RELAY2, LOW);
          states[input-1]=LOW;
          Serial.println("Valve 2 açıldı");
        }
        else{
          digitalWrite(RELAY2,HIGH);
          states[input-1]=HIGH;
          Serial.println("Valve 2 kapandı");
        }
        printStates();
        break;

      case 3:
        if(states[input-1]){
          digitalWrite(RELAY3, LOW);
          states[input-1]=LOW;
          Serial.println("Valve 3 açıldı");
        }
        else{
          digitalWrite(RELAY3,HIGH);
          states[input-1]=HIGH;
          Serial.println("Valve 3 kapandı");
        }
        printStates();
        break;
    
    }
    
  } 
}

void printStates(void)
{
  for(int i=0;i<=2;i++){
    if (states[i]){
      Serial.print("Valve ");
      Serial.print(i+1);
      Serial.print(" kapalı, ");
    }
    else{
      Serial.print("Valve ");
      Serial.print(i+1);
      Serial.print(" açık, ");
    }
  }
  Serial.println();
  Serial.println();
}