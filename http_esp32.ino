#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Ultrasonic.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "Unknown Hotspot";
const char* password = "123456789";

// Server URL
const String serverUrl = "http://192.168.156.53:4001/something";

// Sensor pins
#define DHTPIN 4        // DHT11 data pin
#define DHTTYPE DHT11   // DHT 11 
#define SOUND_SENSOR_PIN 34  // Sound sensor A0 pin
#define IR_SENSOR_PIN 25  // IR proximity 
#define TRIG_PIN 5      // SR04 Trigger pin
#define ECHO_PIN 18      // SR04 Echo pin

DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize sensors
  dht.begin();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void loop() {
  // Reading sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  float distance = ultrasonic.read();
  int soundLevel = analogRead(SOUND_SENSOR_PIN);
  bool irProximity = digitalRead(IR_SENSOR_PIN);
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print sensor data to Serial Monitor
  Serial.println("Sensor Readings:");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("Sound Level: ");
  Serial.print(soundLevel);
  Serial.println(" (raw)");

  Serial.print("IR Proximity: ");
  Serial.println(irProximity);

  Serial.print("Accel X: ");
  Serial.println(a.acceleration.x);
  Serial.print("Accel Y: ");
  Serial.println(a.acceleration.y);
  Serial.print("Accel Z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Gyro X: ");
  Serial.println(g.gyro.x);
  Serial.print("Gyro Y: ");
  Serial.println(g.gyro.y);
  Serial.print("Gyro Z: ");
  Serial.println(g.gyro.z);
  Serial.println("----------");

  // Send data to server if connected to Wi-Fi
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    // Prepare JSON data
    StaticJsonDocument<256> jsonDoc;
    jsonDoc["humidity"] = humidity;
    jsonDoc["temperature"] = temperature;
    jsonDoc["distance"] = distance;
    jsonDoc["soundLevel"] = soundLevel;
    jsonDoc["irProximity"] = irProximity;
    jsonDoc["accelX"] = a.acceleration.x;
    jsonDoc["accelY"] = a.acceleration.y;
    jsonDoc["accelZ"] = a.acceleration.z;
    jsonDoc["gyroX"] = g.gyro.x;
    jsonDoc["gyroY"] = g.gyro.y;
    jsonDoc["gyroZ"] = g.gyro.z;
    
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    
    // Send POST request with JSON data
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }
    
    http.end(); // Free resources
  } else {
    Serial.println("Error in WiFi connection");
  }
  
  delay(1000); // Wait 10 seconds before next request
}
