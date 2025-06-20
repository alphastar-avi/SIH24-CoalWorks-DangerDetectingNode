#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>

// Sensor Instances
MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;

// Sensor/Actuator Pins
#define SOUND_SENSOR_PIN 34
#define MQ4_SENSOR_PIN 35
#define MQ7_SENSOR_PIN 33
#define BUTTON_PIN 15
#define RELAY_PIN 26
#define BUZZER_PIN 27
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Mesh Network Config
#define MESH_PREFIX "WiFiMesh"
#define MESH_PASSWORD "12345678"
#define MESH_PORT 5555

Scheduler userScheduler;
painlessMesh mesh;
HardwareSerial gpsSerial(1);

// Timing Variables
unsigned long lastSentTime = 0;
const unsigned long sendInterval = 1000;
bool alertTriggered = false;

void triggerBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

String readSensorsAndCreateJSON(bool alertStatus) {
  float latitude = 13.0827, longitude = 80.2707; // Default to Chennai
  bool gpsAvailable = false;

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

  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);
  float accelXg = accelX / 16384.0;
  float accelYg = accelY / 16384.0;
  float accelZg = accelZ / 16384.0;

  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;
  float altitude = bmp.readAltitude(1013.25);

  int soundValue = analogRead(SOUND_SENSOR_PIN);
  float soundDB = map(soundValue, 0, 4095, 0, 266);
  float methanePPM = map(analogRead(MQ4_SENSOR_PIN), 0, 4095, 0, 100);
  float carbonMonoxidePPM = map(analogRead(MQ7_SENSOR_PIN), 0, 4095, 0, 200);

  StaticJsonDocument<512> doc;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["accelX"] = accelXg;
  doc["accelY"] = accelYg;
  doc["accelZ"] = accelZg;
  doc["temperature"] = temperature;
  doc["pressure"] = pressure;
  doc["altitude"] = altitude;
  doc["soundDB"] = soundDB;
  doc["methanePPM"] = methanePPM;
  doc["carbonMonoxidePPM"] = carbonMonoxidePPM;
  doc["alert"] = alertStatus ? "yes" : "no";

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void checkButton() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(BUTTON_PIN);

  if (currentState == LOW && lastState == HIGH) {
    alertTriggered = true;
    digitalWrite(RELAY_PIN, HIGH);
    triggerBuzzer();
  }
  lastState = currentState;
}

void sendMessage() {
  String jsonData = readSensorsAndCreateJSON(alertTriggered);
  mesh.sendBroadcast(jsonData);
  Serial.println("Data sent: " + jsonData);
  alertTriggered = false;
  digitalWrite(RELAY_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected, check wiring!");
    while (1);
  }

  if (!bmp.begin(0x76)) {
    Serial.println("No BMP280 detected, check wiring!");
    while (1);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive([](uint32_t from, String &msg) {
    Serial.printf("Received from %u: %s\n", from, msg.c_str());
  });

  Serial.println("Setup complete. Collecting data...");
}

void loop() {
  mesh.update();
  checkButton();
  if (millis() - lastSentTime >= sendInterval) {
    sendMessage();
    lastSentTime = millis();
  }
}