#define photodiodePin 34 // Analog input pin for photodiode
float voltage = 0.0; // Variable to store voltage output
float calibration = 0.01; // Calibration factor to convert voltage to wavelength

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Read the voltage from the photodiode
  int reading = analogRead(photodiodePin);
  // Convert the reading to a voltage
  voltage = reading * (3.3 / 4095.0);
  // Calculate the wavelength from the voltage using the calibration factor
  float wavelength = voltage * calibration;
  // Print the wavelength to the serial monitor
  Serial.print("Wavelength: ");
  Serial.print(wavelength);
  Serial.println(" microns");
  Serial.print("analog ");
  Serial.println(reading);
  Serial.print("voltage ");
  Serial.println(voltage);
  delay(1000); // Wait for one second
}