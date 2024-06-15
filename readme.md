# ESP32 Control System

## Pin Connections:

1. **DS18B20 Sensor:**
   - Data Pin: GPIO 4 (ONE_WIRE_BUS)

2. **Relay Module:**
   - Relay 1: GPIO 5 (RELAY1_PIN)
   - Relay 2: GPIO 16 (RELAY2_PIN)
   - Relay 3: GPIO 25 (RELAY3_PIN)
   - Relay 4: GPIO 26 (RELAY4_PIN)

3. **Ultrasonic Sensor 1:**
   - Trigger Pin: GPIO 12 (TRIG_PIN)
   - Echo Pin: GPIO 13 (ECHO_PIN)

4. **New Ultrasonic Sensor:**
   - Trigger Pin: GPIO 14 (NEW_TRIG_PIN)
   - Echo Pin: GPIO 33 (NEW_ECHO_PIN)
   - LED Pin: GPIO 32 (LED_PIN)

5. **Servo Motor:**
   - Signal Pin: GPIO 27 (SERVO_PIN)

6. **RTC (DS3231):**
   - SDA: GPIO 21
   - SCL: GPIO 22

7. **Photodiodes:**
   - Photodiode 1: ADC1_CH7 (GPIO 35) - photodiodePin1
   - Photodiode 2: ADC1_CH6 (GPIO 34) - photodiodePin2

8. **Test LED (Optional):**
   - GPIO 2 - testLedPin

## Overview:

This project involves controlling various components using an ESP32 microcontroller. Here's a brief overview of what happens in the code:

- The code starts by initializing necessary libraries and defining pin connections for different sensors, actuators, and modules.
- It sets up communication interfaces, such as OneWire for DS18B20 sensor, I2C for RTC DS3231, and digital pins for controlling relays, ultrasonic sensors, LED, and servo motor.
- In the `setup()` function, it initializes serial communication, sets pin modes, and initializes sensors and actuators.
- The `loop()` function continuously performs the following tasks:
  - Measures water height using ultrasonic sensors and prints the distance to the serial monitor.
  - Measures temperature using the DS18B20 sensor and prints the temperature.
  - Measures light intensity using photodiodes, calculates output values, and prints them.
  - Controls relays based on certain conditions, including time of day and sensor readings.
  - Controls the servo motor to perform an action every minute.
  - Implements various control logic based on sensor readings and time conditions.
  - Delays for a certain period between iterations.
