# science_platform
## bmp280
This sensor is the sensor of dfrobot not adafruit so it's not work at the moment, trying to fix errors.

## esp now
- esp32 initiator(transmitter) mac adress is as following -> 40:91:51:AC:28:38
- esp32 responder (receiver) mac adress is as following -> 40:91:51:AC:2D:CC
 
## mh-z19c
- Requirements: 
    + EspSoftwareSerial library from Arduino IDE(7.0.0 version used for this sensor)
    + MHZ19 library (.rar file is given with source code, library can be installed through Arduino IDE>Sketch>Include Library> Add .ZIP library)
    + For further information about this library you can visit https://github.com/crisap94/MHZ19

- Some common errors you may encounter:
    + The CO2 data may stuck at 5000 ppm, in order to fix that you can calibrate the sensor with the functions given in the library and source code. Some explaining was made about that in source file.

- Calibration
    + You can calibrate MH-Z19 with the given instructions in the source file or you may use the inside "MH-Z19C Calibration" file. Even though this file were given, you may download it from https://github.com/WifWaf/MH-Z19. 

    +    HOW TO CALIBRATE:
        + Hardware Method  
        By pulling the zero HD low (0V) for 7 seconds as per the datasheet.
        + Software Method
        You may run the code in "MH-Z19C Calibration", disconnect MH-Z19 from device after sketch ends (20+ minutes) and upload new
        code to avoid recalibration.
        + Auto calibration 
        As it is mentioned inside "MH-Z19C Calibration" folder if this is set to true, the sensor will adjust it's self over a few weeks
        according to the lowest observed CO2 values each day. *You don't need to run this sketch!