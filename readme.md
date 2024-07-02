
# ESP32 Control System

## Overview

This ESP32 control system automates various tasks based on environmental conditions and time of day. It includes temperature monitoring, water level measurement, light intensity monitoring, relay control, servo motor activation, and time-based actions.

## Components and Pin Connections

### 1. DS18B20 Temperature Sensor
- **Data Pin:** GPIO 14 (ONE_WIRE_BUS)

### 2. Relay Module
- **Relay 1:** GPIO 19 (RELAY1_PIN)
- **Relay 2:** GPIO 18 (RELAY2_PIN)
- **Relay 3:** GPIO 5 (RELAY3_PIN)
- **Relay 4:** GPIO 17 (RELAY4_PIN)

### 3. Ultrasonic Sensor 1
- **Trigger Pin:** GPIO 26 (TRIG_PIN)
- **Echo Pin:** GPIO 25 (ECHO_PIN)

### 4. New Ultrasonic Sensor
- **Trigger Pin:** GPIO 33 (NEW_TRIG_PIN)
- **Echo Pin:** GPIO 32 (NEW_ECHO_PIN)
- **LED Pin:** GPIO 2 (LED_PIN)

### 5. Servo Motor
- **Signal Pin:** GPIO 27 (SERVO_PIN)

### 6. RTC (DS3231)
- **SDA:** GPIO 21
- **SCL:** GPIO 22

### 7. Photodiodes
- **Photodiode 1:** ADC1_CH0 (GPIO 36) - photodiodePin1
- **Photodiode 2:** ADC1_CH3 (GPIO 39) - photodiodePin2

### 8. Test LED (Optional)
- **GPIO 2** - testLedPin

### 9. Extra Button
- **Button Pin:** GPIO 16 (buttonPin)

## Features

### 1. Temperature Monitoring
- The system checks the temperature using a DS18B20 sensor.
- If the temperature is 31°C or higher, or if it's between 6:00 AM and 12:00 PM, specific actions are triggered.

### 2. Water Level Measurement
- Ultrasonic sensors measure water height at regular intervals.
- If the water level is below a certain threshold, actions are taken to refill or maintain the water level.

### 3. Light Intensity Monitoring
- Photodiodes monitor light intensity in the surroundings.
- Based on light conditions, the system adjusts certain parameters or activates/deactivates components.

### 4. Relay Control
- Relays control various devices such as pumps, heaters, or lights.
- Depending on the temperature, water level, and light intensity, relays are activated or deactivated.

### 5. Servo Motor Activation
- A servo motor performs specific actions at predefined intervals, such as opening or closing a valve.

### 6. Time-Based Actions
- Certain actions are triggered based on the time of day, such as turning on lights at night or activating specific devices during specific hours.

## Technical Overview

### Initialization
- Necessary libraries are included and pin connections are defined for different sensors, actuators, and modules.
- Communication interfaces, such as OneWire for the DS18B20 sensor and I2C for the RTC DS3231, are set up.

### Setup Function
- Serial communication is initialized.
- Pin modes are set, and sensors and actuators are initialized.

### Loop Function
- Continuously performs the following tasks:
  - Measures water height using ultrasonic sensors and prints the distance to the serial monitor.
  - Measures temperature using the DS18B20 sensor and prints the temperature.
  - Measures light intensity using photodiodes, calculates output values, and prints them.
  - Controls relays based on conditions, including time of day and sensor readings.
  - Controls the servo motor to perform an action every minute.
  - Implements various control logic based on sensor readings and time conditions.
  - Delays for a certain period between iterations.
