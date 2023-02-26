/*     Simple Stepper Motor Control Exaple Code
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */

// defines pins numbers
const int stepPin = 14; 
const int dirPin = 12; 

int pulse_time = 800;

void map(int, int, int, int, int);

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void loop() {
  map(30, HIGH, stepPin, dirPin, pulse_time);
}

void map(int angle, int direction, int stepPin, int dirPin, int pulse_time){
  digitalWrite(dirPin,direction);
  for(int x = 0; x < (int)((57*200*angle)/(11*360)); x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(pulse_time); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(pulse_time); 
  }
  delay(500);
}
