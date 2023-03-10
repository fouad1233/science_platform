// the number of the LED pin
#define stepPin 14 
#define dirPin 12 

#define buttonPin 16
#define proximityPin 2

// setting PWM properties
int freq = 624/2;
const int stepChannel = 0;
const int resolution = 8;
int halfDutyCycle = 128;
int lowDutyCycle = 0;

int angle = 90;
unsigned long previousMillis = 0; 
unsigned long currentMillis;
long interval = (57*200*angle*1000)/(11*360*freq);  // interval at which to stop PWM (miliseconds)
bool motorFlag = false;

unsigned long lastPressed=0;
unsigned long currentPressed=0;


void setup(){
  buttonSetup(); //Configure external interrupt of button
  Serial.begin(115200);
  /*PWM Setup*/
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  digitalWrite(dirPin,LOW);
  //proximitySetup(); //Set the proximity sensor interruption and start the step motor rotation automatically
}

void loop(){
  if (motorFlag){
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      ledcWrite(stepChannel, lowDutyCycle);
      motorFlag = false;
    }
  }
}

void IRAM_ATTR button_INT(){  //When the interrupt triggered it will set the Flag true, set the previousMillis to the moment
  //currentPressed = millis();
  if (!motorFlag){   //currentPressed-lastPressed>=1000
    motorFlag = true; 
    previousMillis = millis();
    ledcWrite(stepChannel, halfDutyCycle);
    Serial.println("ON");
    //lastPressed = currentPressed;
  }
}

void buttonSetup(void){  //Button External interrupt configurations
  pinMode(buttonPin, INPUT_PULLUP); 
  attachInterrupt(buttonPin, button_INT, RISING); 
}


// void IRAM_ATTR proximity_INT(){
//   ledcWrite(stepChannel, lowDutyCycle);
//   motorFlag = false;
//   detachInterrupt(proximityPin, proximity_INT, RISING);
// }

// void proximitySetup(void){
//   pinMode(proximityPin, INPUT_PULLUP);
//   attachInterrupt(proximityPin, proximity_INT, RISING);
//   ledcWrite(stepChannel, haldDutyCycle);
// }

