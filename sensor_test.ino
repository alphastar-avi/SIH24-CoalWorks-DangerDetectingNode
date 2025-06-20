#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;

// Define the pins for the sensors
#define SOUND_SENSOR_PIN 34
#define MQ4_SENSOR_PIN 35
#define MQ7_SENSOR_PIN 33


#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Initialize serial for GPS
HardwareSerial gpsSerial(1);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected, check wiring!");
    while (1);
  }

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("No BMP280 detected, check wiring!");
    while (1);
  }

  Serial.println("Setup complete. Collecting data...");
}

void loop() {
  //hold GPS coordinates
  float latitude = 0.0, longitude = 0.0;
  bool gpsAvailable = false;

  // Timeoutfor GPS signal
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { 
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
      if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        gpsAvailable = true;
        break;
      }
    }
    if (gpsAvailable) break;
  }

  
  if (!gpsAvailable) {
    latitude = 13.0827;  //  Chennai
    longitude = 80.2707; //  Chennai
    Serial.println("GPS signal not acquired. Using predefined Chennai coordinates.");
  }

  // MPU6050
  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);

  //accelerometer data
  float accelXg = accelX / 16384.0; 
  float accelYg = accelY / 16384.0;
  float accelZg = accelZ / 16384.0;

  // BMP280
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; 
  float altitude = bmp.readAltitude(1013.25); 

  // Read sound sensor
  int soundValue = analogRead(SOUND_SENSOR_PIN);
  float soundDB = map(soundValue, 0, 4095, 0, 266); 

  
  int mq4Value = analogRead(MQ4_SENSOR_PIN);
  int mq7Value = analogRead(MQ7_SENSOR_PIN);
  float methanePPM = map(mq4Value, 0, 4095, 0, 100); 
  float carbonMonoxidePPM = map(mq7Value, 0, 4095, 0, 200); 


  Serial.println("--------------- Sensor Data ---------------");
  Serial.printf("GPS Location: Lat = %.6f, Lon = %.6f\n", latitude, longitude);
  Serial.printf("Accel X: %.2f g  Accel Y: %.2f g  Accel Z: %.2f g\n", accelXg, accelYg, accelZg);
  Serial.printf("Temperature = %.2f *C\n", temperature);
  Serial.printf("Pressure = %.2f hPa\n", pressure);
  Serial.printf("Altitude = %.2f m\n", altitude);
  Serial.printf("Sound Level (dB): %.2f\n", soundDB);
  Serial.printf("Sound RAW : %d\n", soundValue);
  Serial.printf("Methane (CH4) PPM: %.2f\n", methanePPM);
  Serial.printf("Carbon Monoxide (CO) PPM: %.2f\n", carbonMonoxidePPM);
  Serial.println("------------------------------------------");


  delay(1000); 
}