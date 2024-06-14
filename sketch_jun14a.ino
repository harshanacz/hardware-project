#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// DS18B20 Sensor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Relay Module
#define RELAY1_PIN 5
#define RELAY2_PIN 16

// Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13

// RTC Module
RTC_DS3231 rtc;
const int RTC_PIN1 = 25; // 6:00 PM to 8:00 PM
const int RTC_PIN2 = 26; // 8:00 PM to 6:00 AM
const int RTC_PIN3 = 27; // Every 6 hours

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Initialize relay pins as outputs
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH); // Ensure relay 1 is off initially (active-low)
  digitalWrite(RELAY2_PIN, HIGH); // Ensure relay 2 is off initially (active-low)

  // Initialize the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start up the DS18B20 sensor library
  sensors.begin();

  // Initialize RTC pins as outputs
  pinMode(RTC_PIN1, OUTPUT);
  pinMode(RTC_PIN2, OUTPUT);
  pinMode(RTC_PIN3, OUTPUT);
  digitalWrite(RTC_PIN1, LOW);
  digitalWrite(RTC_PIN2, LOW);
  digitalWrite(RTC_PIN3, LOW);

  // Initialize the RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, set the time!");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Uncomment to set the RTC to the date & time this sketch was compiled
  }
}

void loop() {
  // RTC functionality
  DateTime now = rtc.now();

  // 6:00 PM to 8:00 PM
  if (now.hour() >= 18 && now.hour() < 20) {
    digitalWrite(RTC_PIN1, HIGH);
  } else {
    digitalWrite(RTC_PIN1, LOW);
  }

  // 8:00 PM to 6:00 AM
  if (now.hour() >= 20 || now.hour() < 6) {
    digitalWrite(RTC_PIN2, HIGH);
  } else {
    digitalWrite(RTC_PIN2, LOW);
  }

  // Every 6 hours
  if (now.hour() % 6 == 0 && now.minute() == 0 && now.second() == 0) {
    digitalWrite(RTC_PIN3, HIGH);
    delay(1000); // Keep the pin high for 1 second
  } else {
    digitalWrite(RTC_PIN3, LOW);
  }

  // Original functionality
  // Measure water height using the ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Request temperature readings
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  // Print the temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  static unsigned long lastRelay2Time = 0;
  static bool solenoidActive = false;

  if (solenoidActive) {
    // Solenoid is active, wait until distance is 10 cm or lower
    if (distance <= 10) {
      // Turn off solenoid (relay 2)
      digitalWrite(RELAY2_PIN, LOW); // Turn relay 2 on (active-low)
      delayMicroseconds(1000000);
      digitalWrite(RELAY2_PIN, HIGH);
      solenoidActive = false;
      Serial.println("Relay 2 OFF");
      delay(200); // Add delay to prevent immediate reactivation
    }
  } else {
    // Normal operation
    if (temperatureC > 31.0) {
      if (distance < 40) {
        digitalWrite(RELAY1_PIN, LOW); // Turn relay 1 on (active-low)
        Serial.println("Relay 1 ON");
      } else {
        digitalWrite(RELAY1_PIN, HIGH); // Turn relay 1 off (active-low)
        Serial.println("Relay 1 OFF");

        // Activate solenoid (relay 2) as soon as relay 1 turns off
        digitalWrite(RELAY2_PIN, LOW); // Turn relay 2 on (active-low)
        delayMicroseconds(1000000);
        digitalWrite(RELAY2_PIN, HIGH);
        lastRelay2Time = millis(); // Record the time when relay 2 turned on
        Serial.println("Relay 2 ON");
        solenoidActive = true;
      }
    } else {
      digitalWrite(RELAY1_PIN, HIGH); // Turn relay 1 off (active-low)
      Serial.println("Relay 1 OFF");
    }
  }

  // Check if relay 2 has been active for more than 100 ms (0.1 second)
  if (solenoidActive && millis() - lastRelay2Time > 100) {
    digitalWrite(RELAY2_PIN, HIGH); // Turn relay 2 off (active-low)
    Serial.println("Relay 2 OFF");
  }

  // Wait 1 second before taking another reading
  delay(1000);
}
