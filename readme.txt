Pin Connections Summary:
DS18B20 Temperature Sensor:

Data Pin (DQ) to ESP32 GPIO 4 (ONE_WIRE_BUS)
VCC to 3.3V
GND to GND
Relay Module:

Relay 1 Control Pin to ESP32 GPIO 5 (RELAY1_PIN)
Relay 2 Control Pin to ESP32 GPIO 16 (RELAY2_PIN)
VCC to 5V (or 3.3V, depending on your relay module)
GND to GND
Ultrasonic Sensor:

Trig Pin to ESP32 GPIO 12 (TRIG_PIN)
Echo Pin to ESP32 GPIO 13 (ECHO_PIN)
VCC to 5V
GND to GND
RTC Module (DS3231):

SDA to ESP32 GPIO 21
SCL to ESP32 GPIO 22
VCC to 3.3V
GND to GND
RTC Output 1 to ESP32 GPIO 25 (RTC_PIN1)
RTC Output 2 to ESP32 GPIO 26 (RTC_PIN2)
RTC Output 3 to ESP32 GPIO 27 (RTC_PIN3)