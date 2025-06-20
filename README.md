

https://github.com/user-attachments/assets/7a2e3b81-b254-43a4-9c17-493ed9443309


# Danger Detecting Node

A real-time safety system that monitors hazards like gas leaks, mine collapse, and extreme sound, while helping workers communicate, tracking their oxygen levels, detecting falls, checking productivity, and monitoring their location.

## Features

* Mesh network communication using `painlessMesh`
* GPS data using `TinyGPS++`
* Air quality monitoring (Methane and CO via MQ4 and MQ7)
* Sound level detection
* Temperature, pressure, and altitude from BMP280
* Motion detection via MPU6050 accelerometer
* Manual alert trigger with button
* Buzzer and relay-based alert response

## Hardware

* **ESP32**
* **MPU6050** – Accelerometer
* **BMP280** – Barometric pressure and temperature sensor
* **MQ4** – Methane sensor
* **MQ7** – Carbon monoxide sensor
* **NEO-8M** - GPS
* **Sound sensor**
* **Relay & Buzzer**
* **Push button**

## Pin Configuration

| Component         | ESP32 Pin                  |
| ----------------- | -------------------------- |
| **Sound Sensor**  | 34                         |
| **MQ4 (Methane)** | 35                         | 
| **MQ7 (CO)**      | 33                         | 
| **Button**        | 15                         |
| **Relay**         | 26                         | 
| **Buzzer**        | 27                         | 
| **GPS RX**        | 16                         | 
| **GPS TX**        | 17                         | 
| **MPU6050 (I2C)** | GPIO21 (SDA), GPIO22 (SCL) | 
| **BMP280 (I2C)**  | GPIO21 (SDA), GPIO22 (SCL) | 


## Mesh Network Setup

```cpp
#define MESH_PREFIX "WiFiMesh"
#define MESH_PASSWORD "12345678"
#define MESH_PORT 5555
```

## Core Functionality

* **Sensor Reading:** Periodically collects data from all sensors and encodes it into a JSON string.
* **GPS:** Attempts to get a fix within 5 seconds. (For open-pit Coalmines)
* **Alert Trigger:** Pressing the button activates the buzzer, sets the relay, and sends a flagged message.
* **Buzzer:** Provides a 500ms alert signal.
* **JSON Payload Includes:**

  * Latitude, Longitude
  * Acceleration (X, Y, Z)
  * Temperature, Pressure, Altitude
  * Sound Level (dB)
  * Methane and CO levels (ppm)
  * Alert flag

## Mesh Communication

Each node broadcasts its sensor data over the mesh every second:

```cpp
mesh.sendBroadcast(jsonData);
```

Incoming messages from other nodes are logged via serial:

```cpp
mesh.onReceive([](uint32_t from, String &msg) {
  Serial.printf("Received from %u: %s\n", from, msg.c_str());
});
```

## Setup Instructions

1. Wire all components to the ESP32 according to the pin configuration.
2. Flash the provided code to the ESP32 using Arduino IDE
3. Open the serial monitor to view data and debug logs.
4. Deploy multiple nodes to form a self-healing mesh network.

## Mind
* GPS might take a few seconds to get a fix. Place antenna outdoors for better signal...
* Mesh nodes should be within Wi-Fi range of each other for stable communication.

---

