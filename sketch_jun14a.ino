#include <OneWire.h>
#include <DallasTemperature.h>

// DS18B20 Sensor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Relay Module
#define RELAY_PIN 5

// Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);

  // Initialize the relay pin as an output
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Ensure relay is off initially (active-low)
  
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

  // Check if the water level is too low
  if (distance >= 40) {
    digitalWrite(RELAY_PIN, HIGH); // Turn relay off (active-low)
    Serial.println("Relay OFF due to low water level");
  } else {
    // Request temperature readings
    sensors.requestTemperatures();
  
    // Fetch the temperature in Celsius
    float temperatureC = sensors.getTempCByIndex(0);

    // Print the temperature to the Serial Monitor
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");

    // Control relay based on temperature
    if (temperatureC > 36.0) {
      digitalWrite(RELAY_PIN, LOW); // Turn relay on (active-low)
      Serial.println("Relay ON");
    } else {
      digitalWrite(RELAY_PIN, HIGH); // Turn relay off (active-low)
      Serial.println("Relay OFF");
    }
  }

  // Wait 1 second before taking another reading
  delay(1000);
}
