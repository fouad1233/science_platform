// the number of the LED pin
#define stepPin 14 
#define dirPin 12 
// setting PWM properties
int freq = 624;
const int stepChannel = 0;
const int resolution = 8;
int dutyCycle = 128;

int angle = 90;
unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long currentMillis;
long interval = (57*200*angle*1000)/(11*360*freq);  // interval at which to blink (miliseconds)
int stepState = LOW;


void setup(){
  // configure LED PWM functionalitites
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  //Initialize
  digitalWrite(dirPin,HIGH);
  ledcWrite(stepChannel, dutyCycle);
}

void loop(){
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    ledcWrite(stepChannel, 0);
  }
}


// void map(int angle, int direction, int stepPin, int dirPin, int pulse_time){
//   digitalWrite(dirPin,direction);
//   for(int x = 0; x < (int)((57*200*angle)/(11*360)); x++) {
//     digitalWrite(stepPin,HIGH); 
//     delayMicroseconds(pulse_time); 
//     digitalWrite(stepPin,LOW); 
//     delayMicroseconds(pulse_time); 
//   }
//   delay(500);
// }