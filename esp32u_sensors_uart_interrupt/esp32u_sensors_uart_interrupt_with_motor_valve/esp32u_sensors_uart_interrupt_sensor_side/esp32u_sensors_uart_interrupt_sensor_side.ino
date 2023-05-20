#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"


/*
FOR DEBUG PURPOSES USE debug(x) for Serial.print(x) and debugln(x) form Serial.println(x) 
ACTIVATE IT BY SETTING DEBUG TO 1
*/
#define DEBUG 0

#if DEBUG == 1    
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

//Functions
void readAllSensors(void);
void stepmotor_setup(void);
void BME280_setup(void);
void TSL2591_setup(void);
void valve_pump_setup(void);

void read_TSL2591(void);
void read_BME280(void);
void read_MQ7(void);
void read_MQ4(void);
void read_MIX8410(void);
float readO2Vout(void);
float readConcentration(void);

void printAllSensors(void);
void print_BME280(void);
void print_TSL2591(void);
void print_MQ7(void);
void print_MQ4(void);
void print_MIX8410(void);


//TIMER
uint8_t read_sensor_flag; //TIMER FLAG
hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1; //SET "read_sensor_flag" TO 1 ON TIMER INTERRUPT
}


// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAC, 0x28, 0x38}; //40:91:51:AC:28:38


//RELAY PINS
int relay[4] = {12,14,27,25}; 


//Pump
#define pumpPin 4
int pumpState = 0;


//StepMotor
#define stepPin 2
#define dirPin 15 
int motorFreq = 624;
const int stepChannel = 0;
const int motorResolution = 8;
int halfDutyCycle = 128;
int lowDutyCycle = 0;
int angle = 90;
unsigned long previousMillis = 0; 
unsigned long currentMillis;
const long constInterval = (57*200*angle*1000)/(11*360*motorFreq);  // interval when PWM stops(miliseconds)
long interval  = constInterval;
int current_tube = 1; //1 2 3 4 positions
int motorFlag = 0;


class adjacency{
  public:
    //A=1 B=2 C=3 D=4
    /*
    D   <-  C
            ^
    |       |    
    A   ->  B       
    */
    const int adjacencies[4][4] =  {{0,1,2,-1},    //Adjacency matrix ABCD
                                    {-1,0,1,2},
                                    {2,-1,0,1},
                                    {1,2,-1,0}};
    int length;

    long getInterval(int tube_to_go); //USED TO CALCULATE THE TIME FOR MOTOR TO TURN
};

long adjacency::getInterval(int tube_to_go)
{
  length = adjacencies[current_tube-1][tube_to_go-1];
  if (length<0)
  { //HIGH CLOCKWISE
    digitalWrite(dirPin,LOW);    //TERSİ OLABİLİR KONTROL ET
  }
  else if(length>0)
  {
    digitalWrite(dirPin,HIGH);   //TERSİ OLABİLİR KONTROL ET  
  }

  return (long)(abs((float)length)*(float)constInterval*1.5);
}
adjacency angles;



/*Send data structure to xaiver*/
typedef struct{

  float temp_BME;
  float pressure;
  float altitude;
  float humidity;
  
  uint32_t lum;
  uint16_t ir_TSL, full, visible_TSL, lux;
  
  float CO_gas_val;
  float met_gas_val;
  int o2_concentration; //O2 sensor o2 concentration

  uint16_t r, g, b, c, colorTemp; //Color sensor

}struct_send_message;

/*Received data structure from xaiver*/
typedef struct{
  int motorFlag = 0;
  int tube_to_go = 1;
  int received_states[5] = {1,1,1,1,0};  // 0,1,2,3 indexler->1,2,3,4 RELAY  5 pump(pwm)

}struct_motor_message;

// Creating sending and receiving structure objects
struct_send_message myData;
struct_motor_message motorData;


// Peer info
esp_now_peer_info_t peerInfo;
 

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  debug("\r\nLast Packet Send Status:\t");
  debugln(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  
  memcpy(&motorData, incomingData, sizeof(motorData)); //Copy data coming from xaiver


  /*If tube to go is different than the current tube on platform calculate interval time, 
  set direction to turn, start the pwm, set the current tube to tube_to_go and set the motorflag to 1*/
  if (motorData.tube_to_go!=current_tube)
  {
    
    interval = angles.getInterval(motorData.tube_to_go);

    debugln();
    debugln("interval");
    debugln(interval);
    debugln("current_tube");
    debugln(current_tube);
    debugln("Tube to go ");
    debugln(motorData.tube_to_go);

    ledcWrite(stepChannel, halfDutyCycle);
    current_tube = motorData.tube_to_go;
    previousMillis = millis();
    motorFlag = 1;
    
  }

  debugln("pump state: ");
  debugln(pumpState);
  debugln("received pump: ");
  debugln(motorData.received_states[4]);


  /*If pump is running and stop command was made 
  stop the pump and set pumpState to 0*/
  if (pumpState && !motorData.received_states[4])
  {
    digitalWrite(pumpPin ,HIGH);
    pumpState = 0;
  }


  /*If pump is not running and run command was made 
  run the pump and set pumpState to 1*/
  else if(!pumpState && motorData.received_states[4])
  {
    digitalWrite(pumpPin ,LOW);
    pumpState = 1;
  }


  debugln("received states");
  //Iterating over relay pins and received relay states. Set what the received relay states says
  for(int i=0;i<4;i++)
  {
    digitalWrite(relay[i],motorData.received_states[i]);
    debugln(motorData.received_states[i]);
  }

}



//BME280
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme; 

//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);


//CO Sensor MQ7 and CH4 Sensor MQ4
int adc_resolution = 4095;
#define MQ7_input 33    /*Digital pin 5 for sensor input*/
int CO_Aout;
#define MQ4_input 35
int met_Aout;


//O2 Sensor MIX8410
#define O2_pin 32
const float VRefer = 3.3; // voltage of adc reference
float MeasuredVout;
float Concentration;
float Concentration_Percentage;


void setup()
{
  Serial.begin(115200);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    debugln("Error initializing ESP-NOW");
    return; //????????
  }
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    debugln("Failed to add peer");
    return; //????????
  }
  esp_now_register_recv_cb(OnDataRecv);


  //running sensor setups
  BME280_setup();
  TSL2591_setup();
  stepmotor_setup();
  valve_pump_setup();
  delay(3000);


  //1 second timer interrupt
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer); 


}


void loop()
{
  //If the sensor flag is 1 and motor is not turning, it reads al the sensors at once and sends them to xaiver side esp32.
  //After finishing all the tasks set read_sensor_flag to 0
  //Sensors will not be read and sensor data will not be sent to xavier side

  if (read_sensor_flag && !motorFlag){
    readAllSensors();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    debugln();
    if (result == ESP_OK) {
      debugln("Sending confirmed");
    }
    else {
      debugln("Sending error");
    }

    read_sensor_flag = 0; //Set to 0 after reading and sending all sensor data

    printAllSensors();
  }

  /*If motorFlag is set to 1 it immediately starts to check if enough time("interval" variable) has been passed to turn the step motor rotation off
  If enough time was passed it turns pwm signal off and set motorFlag to 0*/
  if (motorFlag){

    currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
      ledcWrite(stepChannel, lowDutyCycle);
      motorFlag = 0; 
    }
  }
}


void readAllSensors(void) // function to read sensor values and print with Serial library
{
  
  read_BME280(); //Pressure, Temprature and Humidity Sensor

  read_TSL2591(); // light sensor

  read_MQ7(); // MQ7 CO Sensor

  read_MQ4(); // MQ4 METHANE Sensor

  read_MIX8410(); // MIX8410 O2 Sensor

}

//Adjusts the valve and pump pins and pump pwm signal
void valve_pump_setup(void){
  
  pinMode(relay[0],OUTPUT);
  pinMode(relay[1],OUTPUT);
  pinMode(relay[2],OUTPUT);
  pinMode(relay[3],OUTPUT);
  pinMode(pumpPin, OUTPUT);

  //Setting relay pins HIGH at start
  digitalWrite(relay[0] ,HIGH); 
  digitalWrite(relay[1] ,HIGH);
  digitalWrite(relay[2] ,HIGH);
  digitalWrite(relay[3] ,HIGH);
  digitalWrite(pumpPin ,HIGH);

}



//Sets step motor pins and pwm settings
void stepmotor_setup(void){

  //Stepmotor setup
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, motorFreq, motorResolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  digitalWrite(dirPin,HIGH); // Setting direction to clockwise
}



                          ///////////SETUP FUNCTIONS///////////
//Removed the while loop to prevent waiting at start if the sensor is not connected
void BME280_setup(void)
{
  while(!Serial);    // time to get serial running
  debugln(F("BME280 test"));
  unsigned status;
  
  // default settings
  status = bme.begin(0x76);  
  if (!status) {
      debugln("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      //while (1) delay(10);  //???????????????
  }
}



//Removed the while loop to prevent waiting at start if the sensor is not connected
void TSL2591_setup(void)
{ 
  
  //////begin the sensor
  if (tsl.begin()) 
  {
    
  } 
  else 
  {
    debugln(F("No sensor found ... check your wiring?"));
    //while (1);  //??????????????
  }
  //////configure the sensor
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

   /* Display the gain and integration time for reference sake */  
  
  tsl2591Gain_t gain = tsl.getGain();
}



///////////////////////////Sensor Readings//////////////////////////////////////
void read_BME280(void)
{
  myData.temp_BME = bme.readTemperature();
  myData.pressure = bme.readPressure() / 100.0F;
  myData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  myData.humidity = bme.readHumidity();
}


void read_TSL2591(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!

  //uint32_t lum = tsl.getFullLuminosity();
  //uint16_t ir, full;

  myData.lum = tsl.getFullLuminosity();
  myData.ir_TSL = myData.lum >> 16;
  myData.full = myData.lum & 0xFFFF;
  myData.visible_TSL = myData.full - myData.ir_TSL;
  myData.lux = tsl.calculateLux(myData.full, myData.ir_TSL);
}


void read_MQ7(void) //CO Sensor
{
  CO_Aout = analogRead(MQ7_input);  /*Analog value read function*/
  myData.CO_gas_val = (9800/adc_resolution)*CO_Aout+200;
}


void read_MQ4(void)
{
  met_Aout = analogRead(MQ4_input);  /*Analog value read function*/
  myData.met_gas_val = (9800/adc_resolution)*met_Aout+200;
}


void read_MIX8410(void)
{
  myData.o2_concentration = readConcentration();
}


//MIX8410 O2 Sensor Helper Func
float readO2Vout(void)
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
float readConcentration(void)
{
    // Vout samples are with reference to 3.3V
    MeasuredVout = readO2Vout();
 
    //float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
    //when its output voltage is 2.0V,
    Concentration = MeasuredVout * 0.21 / 2.0;
    Concentration_Percentage=Concentration*100;
    return Concentration_Percentage;
}


void printAllSensors(void)
{
  print_BME280();
  print_TSL2591();
  print_MQ7();
  print_MQ4();
  print_MIX8410();
}


void print_BME280(void)
{
  Serial.print("\n======== BME280 ========\n");
  Serial.print("Temperature = ");
  Serial.print(myData.temp_BME);
  Serial.println(" Celcius");

  Serial.print("Pressure = ");
  Serial.print(myData.pressure);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(myData.altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(myData.humidity);
  Serial.println(" %");
}

void print_TSL2591(void)
{
  Serial.print("\n======== TSL2591 ========\n");
  Serial.print(F("IR: ")); 
  Serial.print(myData.ir_TSL);  
  Serial.print(F("  \n"));
  Serial.print(F("Full: ")); 
  Serial.print(myData.full); 
  Serial.print(F("  \n"));
  Serial.print(F("Visible: ")); 
  Serial.print(myData.visible_TSL); 
  Serial.print(F("  \n"));
  Serial.print(F("Lux: ")); 
  Serial.println(myData.lux, 6);
}


void print_MQ7(void)
{
  Serial.print("\n======== MQ-7 CO ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(CO_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(myData.CO_gas_val);
  Serial.print("ppm\t");
  if (CO_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
}

void print_MQ4(void)
{
  Serial.print("\n======== MQ-4 METHANE ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(met_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(myData.met_gas_val);
  Serial.print("ppm\t");
  if (met_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
    //digitalWrite (LED, LOW) ; /*LED set HIGH if Gas detected */
  }
  else {
    Serial.println("No Gas");
    //digitalWrite (LED, HIGH) ;  /*LED set LOW if NO Gas detected */
  }
}

void print_MIX8410(void)
{
  Serial.print("\n======== MIX8410 O2 ========\n");
  //Serial.print("Vout =");
  //Vout = readO2Vout();
  //Serial.print(Vout);
  Serial.print("O2 Concentration: ");
  Serial.println(myData.o2_concentration);
}
