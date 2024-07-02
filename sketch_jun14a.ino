#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <ESP32Servo.h>  // Use ESP32Servo library

// DS18B20 Sensor
#define ONE_WIRE_BUS 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Relay Module
#define RELAY1_PIN 19
#define RELAY2_PIN 18
#define RELAY3_PIN 5
#define RELAY4_PIN 17

// Ultrasonic Sensor 1
#define TRIG_PIN 26
#define ECHO_PIN 25

// New Ultrasonic Sensor
#define NEW_TRIG_PIN 33
#define NEW_ECHO_PIN 32
#define LED_PIN 2 // LED to be turned on when distance is less than 10 cm

// Servo Motor
#define SERVO_PIN 27
Servo myServo;

// RTC
RTC_DS3231 rtc;
#define SDA_PIN 21
#define SCL_PIN 22

// Photodiodes
const int photodiodePin1 = 36; // Define the pin connected to the first BPW34 photodiode
const int photodiodePin2 = 39; // Define the pin connected to the second BPW34 photodiode
const int testLedPin = 2; // Define an LED pin for testing (optional)

// Extra Button
const int buttonPin = 16; // Define the pin connected to the extra button

// Define calibration points (analog readings and corresponding output values)
const int calibrationPoints[][2] = {
  {50, 0},        // Analog reading 50 corresponds to output 0 (new for very low light conditions)
  {100, 500},     // Analog reading 100 corresponds to output 500
  {500, 2500},    // Analog reading 500 corresponds to output 2500
  {1000, 5000},   // Analog reading 1000 corresponds to output 5000
  {1500, 7500},   // Analog reading 1500 corresponds to output 7500 (good light condition)
  {3000, 10000}   // Analog reading 3000 corresponds to output 10000 (best light condition)
};

unsigned long lastPhotodiodeReadTime = 0; // Variable to store the last photodiode read time
const unsigned long photodiodeReadInterval = 5000; // 5 sec interval (60000 milliseconds)
int outputValue1 = 0; // Global variable to store the mapped output value of the first photodiode
int outputValue2 = 0; // Global variable to store the mapped output value of the second photodiode

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);
  Serial.println("Starting setup...");

  // Initialize relay pins as outputs
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH); // Ensure relay 1 is off initially (active-low)
  digitalWrite(RELAY2_PIN, HIGH); // Ensure relay 2 is off initially (active-low)
  digitalWrite(RELAY3_PIN, HIGH); // Ensure relay 3 is off initially (active-low)
  digitalWrite(RELAY4_PIN, HIGH); // Ensure relay 4 is off initially (active-low)

  // Initialize the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the new ultrasonic sensor pins
  pinMode(NEW_TRIG_PIN, OUTPUT);
  pinMode(NEW_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // Initialize LED pin as output

  // Initialize the servo motor
  myServo.attach(SERVO_PIN);

  // Start up the DS18B20 sensor library
  sensors.begin();

  // Initialize the RTC
  Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with specified SDA and SCL pins
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize the photodiode pins and test LED pin
  pinMode(photodiodePin1, INPUT); // Set the first photodiode pin as input
  pinMode(photodiodePin2, INPUT); // Set the second photodiode pin as input
  pinMode(testLedPin, OUTPUT); // Set the test LED pin as output (optional)

  // Initialize the extra button pin
  pinMode(buttonPin, INPUT); // Set the button pin as input

  Serial.println("Setup completed successfully.");
}

void loop() {
  Serial.println("Loop started...");
  
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

  // Measure distance with the new ultrasonic sensor
  long newDuration, newDistance;
  digitalWrite(NEW_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(NEW_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(NEW_TRIG_PIN, LOW);

  newDuration = pulseIn(NEW_ECHO_PIN, HIGH);
  newDistance = newDuration * 0.034 / 2;

  // Print the new distance to the Serial Monitor with label "Feeder Distance:"
  Serial.print("Feeder Distance: ");
  Serial.print(newDistance);
  Serial.println(" cm");

  // Check if the distance is less than 10 cm and light the LED
  if (newDistance < 10) {
    digitalWrite(LED_PIN, HIGH); // Turn on the LED
    Serial.println("LED ON");
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off the LED
    Serial.println("LED OFF");
  }

  // Request temperature readings
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  // Print the temperature to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  // Get the current time
  DateTime now = rtc.now();
  int currentHour = now.hour();
  int currentMinute = now.minute();
  int currentSecond = now.second();

  // Photodiode reading and output calculation
  if (millis() - lastPhotodiodeReadTime >= photodiodeReadInterval) {
    int sensorValue1 = analogRead(photodiodePin1); // Read the analog value from the first photodiode
    outputValue1 = mapSensorValue(sensorValue1); // Map the sensor value to the desired output range using calibration points

    int sensorValue2 = analogRead(photodiodePin2); // Read the analog value from the second photodiode
    outputValue2 = mapSensorValue(sensorValue2); // Map the sensor value to the desired output range using calibration points

    digitalWrite(testLedPin, HIGH); // Turn on the test LED (optional)
    delay(100); // Delay for stability
    digitalWrite(testLedPin, LOW); // Turn off the test LED (optional)

    // Print the output values to the serial monitor
    Serial.print("Photodiode 1 Output Value: ");
    Serial.println(outputValue1);
    Serial.print("Photodiode 2 Output Value: ");
    Serial.println(outputValue2);

    lastPhotodiodeReadTime = millis(); // Update the last photodiode read time
  }

  // static unsigned long lastRelay2Time = 0;
  static bool solenoidActive = false;

  // Check time range and conditions for operation
  if ((currentHour >= 6 && currentHour < 12 && outputValue1 < 700 && outputValue2 < 1200) || (temperatureC > 31.0 && (currentHour < 6 || currentHour >= 12))) {
    // Solenoid control
    if (solenoidActive) {
      // Solenoid is active, wait until distance is 10 cm or lower
      if (distance <= 10) {
        // Turn off solenoid (relay 2)
    //  digitalWrite(RELAY2_PIN, LOW);  // Turn relay 2 on (active-low)
  // delay(10000);  // Delay for 10 seconds
   digitalWrite(RELAY2_PIN, HIGH);
        solenoidActive = false;
        Serial.println("Relay 2 OFF");
        delay(200); // Add delay to prevent immediate reactivation
      }
    } else {
      // Normal operation
      if (distance < 40) {
        digitalWrite(RELAY1_PIN, LOW); // Turn relay 1 on (active-low)
        Serial.println("Relay 1 ON");
      } else {
        digitalWrite(RELAY1_PIN, HIGH); // Turn relay 1 off (active-low)
        Serial.println("Relay 1 OFF");

        // Activate solenoid (relay 2) as soon as relay 1 turns off
        digitalWrite(RELAY2_PIN, LOW);  // Turn relay 2 on (active-low)
        // digitalWrite(RELAY4_PIN, LOW);
  // delay(10000);  // Delay for 10 seconds
  // digitalWrite(RELAY2_PIN, HIGH);
       // lastRelay2Time = millis(); // Record the time when relay 2 turned on
        Serial.println("Relay 2 ON");
        solenoidActive = true;
      }
    }

    // Check if relay 2 has been active for more than 100 ms (0.1 second)
    // if (solenoidActive && millis() - lastRelay2Time > 100) {
      // digitalWrite(RELAY2_PIN, HIGH); // Turn relay 2 off (active-low)
    //   Serial.println("Relay 2 OFF");
    // }
  } else {
    digitalWrite(RELAY1_PIN, HIGH); // Turn relay 1 off (active-low)
    Serial.println("Relay 1 OFF");
  }

  // Control additional relays based on time (swapped)
  if (currentHour >= 20 && currentHour < 23) {
    digitalWrite(RELAY4_PIN, LOW); // Turn relay 4 on (active-low)
    Serial.println("Relay 4 ON");
  } else {
    digitalWrite(RELAY4_PIN, HIGH); // Turn relay 4 off (active-low)
    Serial.println("Relay 4 OFF");
  }

  if (currentHour >= 23 || currentHour < 6) {
    digitalWrite(RELAY3_PIN, LOW); // Turn relay 3 on (active-low)
    Serial.println("Relay 3 ON");
  } else {
    digitalWrite(RELAY3_PIN, HIGH); // Turn relay 3 off (active-low)
    Serial.println("Relay 3 OFF");
  }

  // Control servo motor for 2 seconds every minute
  static unsigned long lastServoTime = 0;
  if (currentSecond == 0 && millis() - lastServoTime > 60000) { // Check every minute
    myServo.write(90); // Move servo to 90 degrees
    delay(2000); // Wait for 2 seconds
    myServo.write(0); // Move servo back to 0 degrees
    lastServoTime = millis(); // Update last servo activation time
  }

  // Check the button state
  if (digitalRead(buttonPin) == HIGH) {
    // Perform action when the button is pressed
    Serial.println("Button Pressed");
  }

  // Wait 1 second before taking another reading
  delay(1000);
}

// Function to map sensor value to desired output range using calibration points
int mapSensorValue(int sensorValue) {
  for (int i = 0; i < sizeof(calibrationPoints) / sizeof(calibrationPoints[0]); i++) {
    if (sensorValue <= calibrationPoints[i][0]) {
      // Linear interpolation between calibration points
      int outputValue = map(sensorValue, calibrationPoints[i-1][0], calibrationPoints[i][0], calibrationPoints[i-1][1], calibrationPoints[i][1]);
      return outputValue;
    }
  }
  // If sensor value exceeds the maximum calibration point, return maximum output value
  return calibrationPoints[sizeof(calibrationPoints) / sizeof(calibrationPoints[0]) - 1][1];
}
