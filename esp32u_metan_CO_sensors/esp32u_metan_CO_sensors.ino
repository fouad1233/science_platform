int LED = 32;            /*LED pin defined*/
#define Sensor_input 33    /*Digital pin 5 for sensor input*/
float gas_value = 0;
int adc_resolution = 4095;
void setup() {
  Serial.begin(115200);  /*baud rate for serial communication*/
  pinMode(LED, OUTPUT);  /*LED set as Output*/
}
void loop() {
  
  int sensor_Aout = analogRead(Sensor_input);  /*Analog value read function*/
  gas_value = (9800/adc_resolution)*sensor_Aout+200;
  Serial.print("Gas Sensor: ");  
  Serial.print(sensor_Aout);   /*Read value printed*/
  Serial.print("\t");
  Serial.print("\t"); 
  Serial.print("Gas Value: ");
  Serial.print(gas_value);
  Serial.print("ppm\t");
  if (sensor_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
  delay(1000);                 /*DELAY of 1 sec*/
}
