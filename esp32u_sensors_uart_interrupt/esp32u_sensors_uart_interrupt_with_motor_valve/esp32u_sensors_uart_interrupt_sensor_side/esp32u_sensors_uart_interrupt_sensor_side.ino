/*
This file is used to read UV sensor(SI1445) and pressure sensor(BMP280) for now. 
Other i2c sensors will be added to the same file.
*/
// Include Libraries for espnow
#include <esp_now.h>
#include <WiFi.h>

#include "SI114X.h"
#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include "MHZ19.h"

//Valve and pump Definings
#define RELAY1 27
#define RELAY2 14
#define RELAY3 12
#define RELAY4 25        //pump
#define pumpPin 4
#define pumpChannel 1
void valve_pump_setup(void);

int pumpFreq = 425;
const int pumpResolution = 8;
int pumpDutyCycle = 256*14/100;
int pwmpumpState = 0;
//int current_states[] = {1,1,1,1};

//StepMotor Definings
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
const long constInterval = (57*200*angle*1000)/(11*360*motorFreq);  // interval at which to stop PWM (miliseconds)
long interval  = constInterval;
int current_tube = 1; //1 2 3 4 positions

class adjacency{
  public:
    //A=1 B=2 C=3 D=4
    /*
    D   <-  C
            ^
    |       |    
    A   ->  B       
    */
    // const int A[4] = {0,1,2,-1};
    // const int B[4] = {-1,0,1,2};
    // const int C[4] = {2,-1,0,1};
    // const int D[4] = {1,2,-1,0};
    const int adjacencies[4][4] =  {{0,1,2,-1},
                                    {-1,0,1,2},
                                    {2,-1,0,1},
                                    {1,2,-1,0}};
    int length;

    long getInterval(int tube_to_go);
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
    digitalWrite(dirPin,HIGH);    //TERSİ OLABİLİR KONTROL ET  
  }

  return abs(length)*constInterval;
}
adjacency angles;


uint8_t read_sensor_flag; //TIMER FLAG
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xAC, 0x28, 0x38}; //40:91:51:AC:28:38

// Define a data structure
typedef struct{
  float uv;
  float visible;
  float ir;
  float temp_BME;
  float pressure;
  float altitude;
  float humidity;
  uint32_t lum;
  uint16_t ir_TSL, full, visible_TSL, lux;
  float CO_gas_val;
  int co2_concentration;
  int co2_temp;
  float met_gas_val;
  int o2_concentration; //O2 sensor o2 concentration
} struct_send_message;

typedef struct{
  int motorFlag = 0;
  int tube_to_go = 1;
  int received_states[5] = {1,1,1,1,0};  // 0,1,2,3 indexler->RELAY 1,2,3,4(pump) 4.index pump(pwm) 

}struct_motor_message;

// Create a structured object
struct_send_message myData;
struct_motor_message motorData;


// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  
  memcpy(&motorData, incomingData, sizeof(motorData));
  if (motorData.motorFlag && motorData.tube_to_go!=current_tube)
  {
    interval = angles.getInterval(motorData.tube_to_go);
    ledcWrite(stepChannel, halfDutyCycle);

  }

  
  digitalWrite(RELAY1,motorData.received_states[0]);
  digitalWrite(RELAY2,motorData.received_states[1]);
  digitalWrite(RELAY3,motorData.received_states[2]);
  digitalWrite(RELAY4,motorData.received_states[3]);

  if (pwmpumpState && !motorData.received_states[4])
  {
    ledcWrite(pumpChannel, 0);
    pwmpumpState = 0;
  }
  else if(!pwmpumpState && motorData.received_states[4])
  {
    ledcWrite(pumpChannel, pumpDutyCycle);
    pwmpumpState = 1;
  }

  Serial.print("motor Flag ");
  Serial.println(motorData.motorFlag);
  Serial.println("Tube to go ");
  Serial.println(motorData.tube_to_go);
  Serial.println("received states");
  for(int i=0;i<=4;i++)
  {
    Serial.println(motorData.received_states[i]);
  }
}

//UV sensor SI1145
SI114X SI1145 = SI114X(); // initialise sunlight sensor


//BME280
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme; // I2C


//Light Sensor TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);


//Co2 Sensor MH-Z19
const int rx_pin = 16; //Serial rx pin no
const int tx_pin = 17; //Serial tx pin no
MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);



//CO Sensor MQ7 and CH4 Sensor MQ4
int adc_resolution = 4095;
#define MQ7_input 33    /*Digital pin 5 for sensor input*/
int CO_Aout;
#define MQ4_input 26
int met_Aout;


//O2 Sensor MIX8410
#define O2_pin 13
const float VRefer = 3.3; // voltage of adc reference
float MeasuredVout;
float Concentration;
float Concentration_Percentage;


//Functions
void readAllSensors(void);
void stepmotor_setup(void);
void SI1145_setup(void);
void BME280_setup(void);
void TSL2591_setup(void);
void mhz19_setup(void);
void read_SI1145(void);
void read_BME280(void);
void read_TSL2591(void);
void read_mhz19(void);
void read_MQ7(void);
void read_MQ4(void);
void read_MIX8410(void);
float readO2Vout(void);
float readConcentration(void);


// TIMER SETUP
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1;
  //readSensor();
}

void setup()
{
  Serial.begin(115200);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  SI1145_setup();
  BME280_setup();
  TSL2591_setup();
  mhz19_setup();
  delay(3000);
  
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer); //Just Enable

  stepmotor_setup();
  valve_pump_setup();
}


void loop()
{
  if (read_sensor_flag ){
    readAllSensors();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    Serial.println();
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    }
    else {
      Serial.println("Sending error");
    }

    read_sensor_flag = 0;
  }

  
  if (motorData.motorFlag){
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("motorFlag");
      ledcWrite(stepChannel, lowDutyCycle);
      motorData.motorFlag = false;
      current_tube = motorData.tube_to_go;
    }
  }
}


void readAllSensors(void) // function to read sensor values and print with Serial library
{
  read_SI1145(); // UV Sensor
  
  read_BME280(); //Pressure, Temprature and Humidity Sensor

  read_TSL2591(); // light sensor

  read_mhz19(); //Co2 Sensor

  read_MQ7(); // MQ7 CO Sensor

  read_MQ4(); // MQ4 METHANE Sensor

  read_MIX8410(); // MIX8410 O2 Sensor
}

void valve_pump_setup(void){
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2,OUTPUT);
  pinMode(RELAY3,OUTPUT);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2 ,HIGH);
  digitalWrite(RELAY3 ,HIGH);

  ledcSetup(pumpChannel, pumpFreq, pumpResolution);
  ledcAttachPin(pumpPin, pumpChannel);
  ledcWrite(pumpChannel, pumpDutyCycle);
}

void stepmotor_setup(void){
  //Stepmotor setup
  pinMode(dirPin,OUTPUT);
  ledcSetup(stepChannel, motorFreq, motorResolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(stepPin, stepChannel);
  digitalWrite(dirPin,HIGH); // Setting direction to clockwise
}



///////////SETUP FUNCTIONS///////////
void SI1145_setup(void)
{
  while (!SI1145.Begin())
  {
    Serial.println("Si1145 is not ready!");
    delay(1000);
  }
}


void BME280_setup(void)
{
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));

  unsigned status;
  
  // default settings
  status = bme.begin(0x76);  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      while (1) delay(10);
  }
}


void TSL2591_setup(void)
{ 
  
  //////begin the sensor
  if (tsl.begin()) 
  {
    
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  //////configure the sensor
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

   /* Display the gain and integration time for reference sake */  
  
  tsl2591Gain_t gain = tsl.getGain();
  }


void mhz19_setup(void)
{
  mhz19_uart->begin(rx_pin, tx_pin);
  mhz19_uart->setAutoCalibration(true); //AUTO CALIBRATION// IN ORDER TO AUTOCALIBRATE SET setAutoCalibration(true) and calibrateSpan(5000)
  mhz19_uart->calibrateZero(); //TO CALIBRATE REMOVE COMMENT LINE
  mhz19_uart->calibrateSpan(5000);

  //delay(3000); // Issue #14 //
}



void read_SI1145(void)
{
  myData.visible = SI1145.ReadVisible(); // visible radiation
  myData.ir = SI1145.ReadIR(); // IR radiation
  myData.uv = SI1145.ReadUV(); // UV index
}


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



void read_mhz19(void)
{
  measurement_t m = mhz19_uart->getMeasurement();
  myData.co2_concentration = m.co2_ppm;
  myData.co2_temp = m.temperature;
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
