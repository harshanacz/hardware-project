#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <ESP32Servo.h>
#include <FirebaseESP32.h>

int eventCounter = 0;

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
#define LED_PIN 2  // LED to be turned on when distance is less than 10 cm

// Servo Motor
#define SERVO_PIN 27
Servo myServo;

// RTC
RTC_DS3231 rtc;
#define SDA_PIN 22
#define SCL_PIN 21

// Photodiodes
const int photodiodePin1 = 36;
const int photodiodePin2 = 39;
const int testLedPin = 2;  // LED pin for testing

// Extra Button
const int buttonPin = 16;

// Define calibration points (analog readings and corresponding output values)
const int calibrationPoints[][2] = {
  { 50, 0 },
  { 100, 500 },
  { 500, 2500 },
  { 1000, 5000 },
  { 1500, 7500 },
  { 3000, 10000 }
};

// Firebase project details
#define FIREBASE_HOST "https://hardware-project-17-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "zuNL5wdSfqulEhUTfDhCB2ViEHeIcGAu5NNrfYa4"

#define WIFI_SSID "Harshana Phone"
#define WIFI_PASSWORD "gogo2518"

unsigned long lastPhotodiodeReadTime = 0;
const unsigned long photodiodeReadInterval = 5000;  // 5 sec interval
int outputValue1 = 0;
int outputValue2 = 0;
bool relay1Active = false;

// Firebase data object
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

bool relay1EventRecorded = false; // Flag to track event recording for Relay 1

void setup() {
  Serial.begin(9600);
  Serial.println("Starting setup...");

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(NEW_TRIG_PIN, OUTPUT);
  pinMode(NEW_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  myServo.attach(SERVO_PIN);

  sensors.begin();

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(photodiodePin1, INPUT);
  pinMode(photodiodePin2, INPUT);
  pinMode(testLedPin, OUTPUT);

  pinMode(buttonPin, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi with IP: ");
  Serial.println(WiFi.localIP());

  // Initialize Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

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

  Serial.print("Feeder Distance: ");
  Serial.print(newDistance);
  Serial.println(" cm");

  if (newDistance < 10) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED OFF");
  }

  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  DateTime now = rtc.now();
  Serial.print("Current RTC Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  int currentHour = now.hour();
  int currentMinute = now.minute();
  int currentSecond = now.second();

  if (millis() - lastPhotodiodeReadTime >= photodiodeReadInterval) {
    int sensorValue1 = analogRead(photodiodePin1);
    outputValue1 = mapSensorValue(sensorValue1);

    int sensorValue2 = analogRead(photodiodePin2);
    outputValue2 = mapSensorValue(sensorValue2);

    Serial.print("Photodiode 1 Output Value: ");
    Serial.println(outputValue1);
    Serial.print("Photodiode 2 Output Value: ");
    Serial.println(outputValue2);

    lastPhotodiodeReadTime = millis();
  }

  static bool solenoidActive = false;

  Serial.print("Temperature: ");
  Serial.println(temperatureC);
  Serial.print("Current Hour: ");
  Serial.println(currentHour);

  if ((currentHour >= 6 && currentHour < 15 && outputValue1 < 700 && outputValue2 < 700)) {
    Serial.println("Reason for relay 01 active: Photodiode values less than 400 and current hour between 6am and 3pm.");
    relay1Active = true;
  } else if (temperatureC > 31.0 && (currentHour >= 6 && currentHour < 12)) {
    Serial.println("Reason for relay 01 active: Temperature greater than 31.0 and current hour between 6am and 12pm.");
    relay1Active = true;
  }

  if (relay1Active) {
    if (!relay1EventRecorded) {
      sendEventDataToFirebase("Relay 1 activated");
      relay1EventRecorded = true; // Set flag to true after recording event
    }

    if (solenoidActive) {
      if (distance <= 10) {
        digitalWrite(RELAY2_PIN, HIGH);  // Turn off Relay 2
        solenoidActive = false;
        relay1Active = false;  // Reset the flag once the process is completed
        relay1EventRecorded = false; // Reset event recorded flag
        Serial.println("Relay 2 OFF");
        delay(200);  // Delay for stability
      }
    } else {
      if (distance < 40) {
        digitalWrite(RELAY1_PIN, LOW);
        Serial.println("Relay 1 ON");
      } else {
        digitalWrite(RELAY1_PIN, HIGH);
        Serial.println("Relay 1 OFF");

        digitalWrite(RELAY2_PIN, LOW);
        Serial.println("Relay 2 ON");
        solenoidActive = true;
      }
    }
  } else {
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);  // Ensure Relay 2 is off
    Serial.println("Relay 1 & 2 OFF");
  }

  if (currentHour >= 20 && currentHour < 23) {
    digitalWrite(RELAY4_PIN, LOW);
    Serial.println("Relay 4 ON");
  } else {
    digitalWrite(RELAY4_PIN, HIGH);
    Serial.println("Relay 4 OFF");
  }

  if (currentHour >= 23 || currentHour < 6) {
    digitalWrite(RELAY3_PIN, LOW);
    Serial.println("Relay 3 ON");
  } else {
    digitalWrite(RELAY3_PIN, HIGH);
    Serial.println("Relay 3 OFF");
  }

  static unsigned long lastServoTime = 0;
  if (currentSecond == 0 && millis() - lastServoTime > 60000) {
    myServo.write(90);
    delay(2000);
    myServo.write(0);
    lastServoTime = millis();
  }

  if (digitalRead(buttonPin) == HIGH) {
    Serial.println("Button Pressed");
  }

  delay(1000);
}

int mapSensorValue(int sensorValue) {
  for (int i = 0; i < sizeof(calibrationPoints) / sizeof(calibrationPoints[0]); i++) {
    if (sensorValue <= calibrationPoints[i][0]) {
      int outputValue = map(sensorValue, calibrationPoints[i - 1][0], calibrationPoints[i][0], calibrationPoints[i - 1][1], calibrationPoints[i][1]);
      return outputValue;
    }
  }
  return calibrationPoints[sizeof(calibrationPoints) / sizeof(calibrationPoints[0]) - 1][1];
}

void sendEventDataToFirebase(const char* reason) {
  eventCounter++;
  // Create a JSON object to hold the data
  FirebaseJson json;
  json.set("event", "Relay 1 Activated");
  json.set("reason", reason);
  json.set("count", eventCounter);

  // Set the JSON object in the Realtime Database
  if (Firebase.set(firebaseData, "/events/event" + String(eventCounter), json)) {
    Serial.println("Event data set successfully in Firebase");
  } else {
    Serial.print("Failed to set event data in Firebase: ");
    Serial.println(firebaseData.errorReason());
  }
}
