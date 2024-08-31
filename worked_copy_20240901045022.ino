#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <BlynkSimpleEsp8266.h>

// Define custom I2C pins using GPIO numbers
#define SDA_PIN 4  // GPIO4 corresponds to D2
#define SCL_PIN 5  // GPIO5 corresponds to D1

// Define the pin for the moisture sensor
#define MOISTURE_PIN A0


// Define the pin for controlling the output (numeric value for ESP8266)
#define CONTROL_PIN 0  // GPIO0 corresponds to D3 on NodeMCU

// Gravitational acceleration (approximate value in m/s^2)
#define GRAVITY 9.81

// Define sensor thresholds
const int moistureThreshold = 500;  // Example threshold value for moisture
const float accelerationThreshold = 20; // Example threshold value for acceleration

// Blynk authentication
#define BLYNK_AUTH_TOKEN "QRTtns1r112DYhXU8qWPJftT4oBTmld2" // Your Blynk Auth Token

// Wi-Fi credentials
#define WIFI_SSID "PASU" // Your Wi-Fi SSID
#define WIFI_PASSWORD "1234567890" // Your Wi-Fi Password

// Blynk template ID (for reference only; not used in Blynk.begin())
#define BLYNK_TEMPLATE_ID "TMPL32Sfn-tH2" // Your Blynk Template ID

// Blynk template name (for reference only; not used in Blynk.begin())
#define BLYNK_TEMPLATE_NAME "pasu" // Your Blynk Template Name

// Create an instance of the ADXL345 accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the custom I2C pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the ADXL345 accelerometer
  if (!accel.begin()) {
    Serial.println("Couldn't find the ADXL345 sensor");
    while (1);
  }

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);

  // Set pin mode
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, LOW);

  Serial.println("Accelerometer and moisture sensor initialized successfully.");
}

void loop() {
  sensors_event_t event;

  // Read moisture sensor value
  int moistureValue = analogRead(MOISTURE_PIN);
  Serial.print("Moisture: ");
  Serial.println(moistureValue);

  // Get acceleration data from the ADXL345 sensor
  accel.getEvent(&event);

  // Calculate the magnitude of acceleration
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float magnitude = sqrt(x * x + y * y + z * z);

  // Adjust for gravity
  float adjustedMagnitude = (magnitude - GRAVITY)*100;

  // Print adjusted acceleration magnitude to the Serial Monitor
  Serial.print("Adjusted Acceleration Magnitude: ");
  Serial.print(adjustedMagnitude);
  Serial.println(" m/s^2");

  // Check thresholds and control digital pin
  if (moistureValue> moistureThreshold || adjustedMagnitude > accelerationThreshold) {
    digitalWrite(CONTROL_PIN, HIGH);
  } else {
    digitalWrite(CONTROL_PIN, LOW);
  }

  // Send data to Blynk
  Blynk.virtualWrite(V2, moistureValue); // Send moisture data to virtual pin V2
  Blynk.virtualWrite(V3, adjustedMagnitude); // Send acceleration data to virtual pin V3

  // Process Blynk events
  Blynk.run();

  // Wait for a second before taking another reading
  delay(1000);
}

// Button press handler
BLYNK_WRITE(V1) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    Serial.println("alert");
    digitalWrite(CONTROL_PIN, HIGH);
  } else {
    digitalWrite(CONTROL_PIN, LOW);
  }
}
