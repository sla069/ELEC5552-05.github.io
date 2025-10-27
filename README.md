# Indoor Surveillance Drone
ELEC5552 Team 05 Indoor Surveillance Drone Project

An **autonomous indoor surveillance drone** built using the **Seeed Studio XIAO ESP32S3 Sense**, designed for **real-time monitoring, data collection, and wireless control**.  

---

## Overview

The Indoor Surveillance Drone is developed as part of a research and design project focused on indoor environmental surveillance without the use of GPS for the UWA Aviation Labs (UWAAL).  
It integrates a range of onboard sensors, live web dashboards, and local data storage to achieve semi-autonomous surveillance and sensing.

Key capabilities include:
- Real-time Wi-Fi access point (AP) control  
- Live sensor data viewing via web interface  
- Data logging in CSV format without a wired connection  
- Directional control of motors via web interface
- Live camera streaming and video recording

---

## Features

- Autonomous and manual operation modes  
- Web-based interface for live data and control  
- Wireless CSV data logging via Wi-Fi  
- Altitude, temperature, and motion sensing
- Wireless camera interface for live streaming and saving of footage to onboard microSD card

---

## Hardware Components

| Component | Description | Notes |
|------------|-------------|-------|
| **Seeed Studio XIAO ESP32S3 Sense** | Main MCU with camera and Wi-Fi | Handles all control, sensing, and communication |
| **Adafruit BMP280** | Pressure & temperature sensor | Used for altitude estimation |
| **Adafruit MPU6050** | 6-DOF IMU sensor | Provides acceleration and gyro data |
| **Li-Po Battery** | Power source | 3.7V rechargeable |
| **Drone Frame & Motors** | Propulsion system | Utilised for movement |
| **MicroSD** | Local memory | For video storage |

---

## Setup and Installation

### Hardware Setup

| Connection | BMP280 | MPU6050 | Notes |
|-------------|---------|---------|-------|
| **SDA**     | GPIO 6  | GPIO 6  | Shared I2C line |
| **SCL**     | GPIO 7  | GPIO 7  | Shared I2C line |
| **VCC**     | 3.3V    | 3.3V    | Power supply |
| **GND**     | GND     | GND     | Common ground |

- Power the **Seeed Studio XIAO ESP32S3** or the motor circuit via a Li-Po battery. *(depending which functionality intended to use as the motor control circuit is powered on a different board)*
- Connect the onboard camera if using the camera module and ensure ESP+camera module are not connected on the PCB connectors.   

---

### Software Setup

1. **Install Prerequisites**
   - Download and install the [Arduino IDE](https://www.arduino.cc/en/software).  
   - Add ESP32 board support:
     - Go to *File → Preferences → Additional Board Manager URLs* and add:  
       ```
       https://dl.espressif.com/dl/package_esp32_index.json
       ```
     - Then open *Tools → Board → Boards Manager*, search **ESP32**, and install it.

2. **Install Required Libraries**
   In Arduino IDE, go to *Sketch → Include Library → Manage Libraries…* and install:
   - `Adafruit BMP280`
   - `Adafruit MPU6050`
   - `Adafruit Unified Sensor`
   - `WiFi`
   - `WebServer`
   - `FS` / `SPIFFS`

3. **Clone the Repository**
   ```bash
   git clone https://github.com/<sla069>/ELEC5502-05.git
   cd ELEC5502-05

*(If you’re using Arduino IDE only, you can skip the terminal step — just click **Code → Download ZIP**, then extract the files manually.)*

### Flashing programs

1. **Select which functionality to upload.**
Inside the repository, there are three independent Arduino sketches (.ino files):
/motor_control/wifi_controls.ino     → Controls drone motors and movement  
/camera_module/ESP32-CAM_MJPEG2SD.ino     → Handles camera functions and Wi-Fi streaming  
/sensor_module/combined_gyro_baro_pcb.ino       → Reads BMP280 + MPU6050 and serves web dashboard
   - Open the desired .ino file in Arduino IDE depending on which subsystem you want to test or run.
   - Each file should be uploaded separately to the ESP dev board as a standalone program.

2. **Board Configuration**
   - Go to Tools → Board and select Seeed XIAO ESP32S3
   - Choose the correct COM port under Tools → Port.
  
3. **Compile and Upload**

4. **Access Web Interfaces**
   - For the motor module:
     - Connect to the wifi network: ESP_DRONE
     - Open your browser and navigate to: http://192.168.4.1/

   - For the sensors:
     - Connect to the wifi network: Drone_sensors
     - Open your browser and navigate to: http://192.168.4.1/
    
   - For the camera module:
     - Connect to the wifi network: Drone Live Footage
     - Open your browser and navigate to: http://192.168.4.1/
