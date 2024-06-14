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
}

void loop() {
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
    if (temperatureC > 36.0) {
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
