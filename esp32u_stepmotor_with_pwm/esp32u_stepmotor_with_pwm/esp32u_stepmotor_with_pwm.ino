// the number of the LED pin
#define stepPin 14 
#define dirPin 12 
// setting PWM properties
int freq = 624;
const int stepChannel = 0;
const int resolution = 8;
const int dutyCycle = 128;

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
  // // increase the LED brightness
  // for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
  //   // changing the LED brightness with PWM
  //   ledcWrite(ledChannel, dutyCycle);
  //   delay(15);
  // }

  // // decrease the LED brightness
  // for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
  //   // changing the LED brightness with PWM
  //   ledcWrite(ledChannel, dutyCycle);   
  //   delay(15);
  //}
}