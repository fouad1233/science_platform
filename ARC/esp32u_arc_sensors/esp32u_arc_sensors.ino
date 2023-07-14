#include <esp_now.h> // Include Libraries for espnow
#include <WiFi.h>
#include <Wire.h>
#include "Arduino.h"

#include <Stepper.h>

#include "HX711.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

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

void BME280_setup();
void HX711_setup();
void readAllSensors();
void read_BME280();
void read_HX711();
void read_MQ4();
void printAllSensors();
void print_BME280();
void motor_rotate();
void print_HX711();
void print_MQ4();

//TIMER
uint8_t read_sensor_flag; //TIMER FLAG
hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer()
{
  read_sensor_flag = 1; //SET "read_sensor_flag" TO 1 ON TIMER INTERRUPT
}


// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = { 0x40, 0x91, 0x51, 0xAC, 0x28, 0x38 };  //40:91:51:AC:28:38

uint16_t adc_resolution = 4095;

// Define a data structure
typedef struct {
  float temp_BME;
  float humidity;
  float met_gas_val;
  float weight_average; //hx711 new
} struct_send_message;

typedef struct {
  int motorState = 0;
  //AGIRLIK OLCUMU ICIN ROVERDAN GELEN ACI VERISI EKLENECEK

} struct_motor_message;


// Creating sending and receiving structure objects
struct_send_message myData;
struct_motor_message motorData;
int motorCurrentState = 0;
int motorFlag = 0;
int motor_direction = 0;

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  
  memcpy(&motorData, incomingData, sizeof(motorData)); //Copy data coming from xaiver

  motorFlag = motorData.motorState!=motorCurrentState; //If motorState and CurrentState are different -> TRUE, else -> FALSE 
  motorCurrentState = motorData.motorState; 
  
}

// Step motor pinouts and const.
const int stepsPerRevolution = 342;  // 2048/6 = 60 degrees
// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

//HX711 Pinouts
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;
float one_read_weight; //hx711 new

HX711 scale; //scale activated

//BME280
#define SEALEVELPRESSURE_HPA (1024.00)   // sea level pressure
Adafruit_BME280 bme;

//Sensor MQ4
#define MQ4_input 35
int met_Aout;

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(80); // setup for HX711 Weight Sensor for reading
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

    // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return; //????????
  }
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  myStepper.setSpeed(15);

  
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
  HX711_setup();
  delay(3000);

//1 second timer interrupt
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer);

}

void loop() {
  //If the sensor flag is 1 and motor is not turning, it reads al the sensors at once and sends them to xaiver side esp32.
  //After finishing all the tasks set read_sensor_flag to 0
  //Sensors will not be read and sensor data will not be sent to xavier side

  if (read_sensor_flag /*&& !motorFlag*/){
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
    
    read_sensor_flag = 0; //Set to 0 after reading and sending all sensor data

    printAllSensors();
  }


  if (motorFlag)
  {
    motor_rotate();  
    motorFlag = 0;
  }
}

void readAllSensors(void) // function to read sensor values and print with Serial library
{
  read_BME280(); //Pressure, Temprature and Humidity Sensor
  read_MQ4(); // MQ4 METHANE Sensor
  read_HX711(); // HX711 Weight Sensor
}

                         ///////////SETUP FUNCTIONS///////////
//Removed the while loop to prevent waiting at start if the sensor is not connected
void BME280_setup()
{
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));
  unsigned status;
  
  // default settings
  status = bme.begin(0x76);  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      //while (1) delay(10);  //???????????????
  }
}

void HX711_setup()
{
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)
            
  scale.set_scale(490.9288);
  //scale.set_scale(-471.497);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
}

void read_BME280()
{
  myData.temp_BME = bme.readTemperature();
  myData.humidity = bme.readHumidity();
}

void read_HX711()
{
  one_read_weight = scale.get_units();
  myData.weight_average = scale.get_units(10);

  scale.power_down();             // put the ADC in sleep mode
  delay(5000);
  scale.power_up();

}

void read_MQ4()
{
  met_Aout = analogRead(MQ4_input);  /*Analog value read function*/
  myData.met_gas_val = (9800/adc_resolution)*met_Aout+200;
}

void printAllSensors()
{
  print_BME280();
  print_MQ4();
  print_HX711();
}


void print_BME280()
{
  Serial.print("\n======== BME280 ========\n");
  Serial.print("Temperature = ");
  Serial.print(myData.temp_BME);
  Serial.println(" Celcius");

  Serial.print("Humidity = ");
  Serial.print(myData.humidity);
  Serial.println(" %");
}

void print_MQ4()
{
  Serial.print("\n======== MQ-4 METHANE ========\n");
  Serial.print("Gas Sensor: ");  
  Serial.print(met_Aout);   /*Read value printed*/
  Serial.print("\nGas Value: ");
  Serial.print(myData.met_gas_val);
  Serial.print("ppm\t");
  if (met_Aout > 1200) {    /*if condition with threshold 1800*/
    Serial.println("Gas");  
  }
  else {
    Serial.println("No Gas");
  }
}

void print_HX711()
{
  Serial.print("\n======== HX711 Weight Sensor ========\n");
  Serial.print("one reading:\t");
  Serial.print(one_read_weight, 1);
  Serial.print("\t| average:\t");
  Serial.println(myData.weight_average, 5);
}

void motor_rotate()
{
  if (motor_direction)
    myStepper.step(stepsPerRevolution);
  else
    myStepper.step(-stepsPerRevolution);
}