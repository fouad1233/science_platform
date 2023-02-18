#include "MHZ19.h"

const int rx_pin = 16; //Serial rx pin no
const int tx_pin = 17; //Serial tx pin no

//const int pwmpin = 14;  // FOR USE OF PWM

MHZ19 *mhz19_uart = new MHZ19(rx_pin,tx_pin);
//MHZ19 *mhz19_pwm = new MHZ19(pwmpin);  // FOR USE OF PWM 

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  setup
  ----------------------------------------------------------*/
void setup()
{
    Serial.begin(9600); //115200
    mhz19_uart->begin(rx_pin, tx_pin);
    mhz19_uart->setAutoCalibration(true); //AUTO CALIBRATION// IN ORDER TO AUTOCALIBRATE SET setAutoCalibration(true) and calibrateSpan(5000)
    mhz19_uart->calibrateZero(); //TO CALIBRATE REMOVE COMMENT LINE
    mhz19_uart->calibrateSpan(5000);

    delay(3000); // Issue #14
    Serial.print("MH-Z19 now warming up...  status:");
    Serial.println(mhz19_uart->getStatus());
    delay(1000);
}

/*----------------------------------------------------------
    MH-Z19 CO2 sensor  loop
  ----------------------------------------------------------*/
void loop()
{
    measurement_t m = mhz19_uart->getMeasurement();

    Serial.print("co2: ");
    Serial.println(m.co2_ppm);

    Serial.print("temp: ");
    Serial.println(m.temperature);


    //int co2ppm = mhz19_pwm->getPpmPwm();      // IN ORDER TO READ IT WITH PWM REMOVE COMMENT LINES
    //Serial.print("co2: ");
    //Serial.println(co2ppm);
    
    delay(1000);
}