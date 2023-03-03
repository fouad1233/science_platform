// the number of the LED pin
#define stepPin 14 
#define dirPin 12 

#define buttonPin 16


// setting PWM properties
int freq = 624;
const int stepChannel = 0;
const int resolution = 8;
int halfDutyCycle = 128;
int lowDutyCycle = 0;

int angle = 90;
unsigned long previousMillis = 0; 
unsigned long currentMillis;
long interval = (57*200*angle*1000)/(11*360*freq);  // interval at which to blink (miliseconds)
int stepState = LOW;
bool motorFlag = false;

void EXTIsetup(void);

void setup(){
  EXTIsetup(); //Configure external interrupt of button
  /*PWM Setup*/
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  //Button Setup
  digitalWrite(dirPin,HIGH);
  //ledcWrite(stepChannel, dutyCycle);
}

void loop(){
  currentMillis = millis();
  if (motorFlag){
    if (currentMillis - previousMillis >= interval) {
      ledcWrite(stepChannel, lowDutycycle);
      motorFlag = false;
    }
  }
}

void IRAM_ATTR ext_INT(){  //When the interrupt triggered it will set the Flag true, set the previousMillis to the moment
  motorFlag = true; 
  previousMillis = millis();
  ledcWrite(stepChannel, halfDutycycle);
}

void EXTIsetup(void){  //Button External interrupt configurations
  pinMode(buttonPin, INPUT_PULLUP); 
  attachInterrupt(buttonPin, ext_INT, RISING); //
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