# Components and Pin Connections

### 1. DS18B20 Temperature Sensor
- **Data Pin:** GPIO 14 (ONE_WIRE_BUS)

### 2. Relay Module
- **Relay 1:** GPIO 19 (RELAY1_PIN) - Solenoid valve
- **Relay 2:** GPIO 18 (RELAY2_PIN) - Water Pump
- **Relay 3:** GPIO 5 (RELAY3_PIN) - White Light
- **Relay 4:** GPIO 17 (RELAY4_PIN) - Blue Light

### 3. Ultrasonic Sensor 1 - Check Water Level Distance
- **Trigger Pin:** GPIO 26 (TRIG_PIN)
- **Echo Pin:** GPIO 25 (ECHO_PIN)

### 4. New Ultrasonic Sensor
- **Trigger Pin:** GPIO 33 (NEW_TRIG_PIN)
- **Echo Pin:** GPIO 32 (NEW_ECHO_PIN)
- **LED Pin:** GPIO 2 (LED_PIN)

### 5. Servo Motor - For Feeder
- **Signal Pin:** GPIO 27 (SERVO_PIN)

### 6. RTC (DS3231)
- **SDA:** GPIO 22
- **SCL:** GPIO 21

### 7. Photodiodes
- **Photodiode 1:** ADC1_CH0 (GPIO 36) - photodiodePin1
- **Photodiode 2:** ADC1_CH3 (GPIO 39) - photodiodePin2

### 8. Feeder Food Status Indicator
- **GPIO 2** - testLedPin

### 9. Extra Button
- **Button Pin:** GPIO 16 (buttonPin)
