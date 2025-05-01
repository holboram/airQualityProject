#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <SdsDustSensor.h>
#include <SPI.h>
#include "arduino_secrets.h"
#include "Ozone2Click.h"

// --- Function Declarations ---
// Functions to handle network and MQTT connections
void connectNB();
void connectMQTT();
// Function to publish sensor data to MQTT broker
void publishMessage(float pm25, float pm10, float o3, float so2, float no2);
// Functions to read sensor values
float readO3();
float getSO2();
float getNO2();
// Function to convert temperature sensor voltage to temperature
float getTemperature(float VtempVoltage);
// Functions to calibrate zero offset for SO2 and NO2 sensors
void calibrateZeroOffset_SO2();
void calibrateZeroOffset_NO2();
// Utility functions for UUID generation and array operations
void generateUUID(char *uuid);
float average(float *arr, int size);
float median(float *arr, int size);
void swap(float &a, float &b);

// --- Pin definitions ---
const int CS_PIN = 7; // Chip select pin for Ozone2Click sensor

// --- O3 Calibration Constants ---
const float RO = 2581.7; // Reference resistance for O3 sensor
const float A = 0.10;    // Calibration constant A for O3 sensor
const float B = -1.1;    // Calibration constant B for O3 sensor

Ozone2Click o3Sensor(CS_PIN); // Instantiate O3 sensor object

// --- SO2 Sensor ---
// Analog pins connected to SO2 sensor gas, reference, and temperature outputs
const int VgasPin_SO2 = A1;
const int VrefPin_SO2 = A2;
const int VtempPin_SO2 = A3;
// Sensitivity and gain constants for SO2 sensor
double sensitivityCode_SO2 = 41.74;
const double TIA_Gain_SO2 = 100.0;
double M_SO2 = sensitivityCode_SO2 * TIA_Gain_SO2 * 0.000001; // Conversion factor
float zeroOffset_SO2 = 0.0; // Zero offset to be calibrated

// --- NO2 Sensor ---
// Analog pins connected to NO2 sensor gas, reference, and temperature outputs
const int VgasPin_NO2 = A4;
const int VrefPin_NO2 = A5;
const int VtempPin_NO2 = A6;
// Sensitivity and gain constants for NO2 sensor
double sensitivityCode_NO2 = 41.74;
const double TIA_Gain_NO2 = 499.0;
double M_NO2 = sensitivityCode_NO2 * TIA_Gain_NO2 * 0.000001; // Conversion factor
float zeroOffset_NO2 = 0.0; // Zero offset to be calibrated

// --- Network Configuration ---
// Credentials and settings for cellular network and MQTT broker
const char pinnumber[] = SECRET_PINNUMBER;
const char broker[] = SECRET_BROKER;
const int mqtt_port = SECRET_PORT;
String deviceId = SECRET_DEVICE_ID;

// Instantiate sensor and network objects
SdsDustSensor sds(Serial1); // Particulate matter sensor connected via Serial1
NB nbAccess;                // Narrowband IoT access object
GPRS gprs;                  // GPRS object for cellular data
NBClient nbClient;          // Client for NB connection
MqttClient mqttClient(nbClient); // MQTT client using NBClient

// Timing intervals for reading sensors and sending data (in milliseconds)
const unsigned long readInterval = 2 * 60 * 1000UL; // Read sensors every 2 minutes
const unsigned long sendInterval = 5 * 60 * 1000UL; // Send data every 5 minutes
unsigned long lastReadMillis = 0; // Timestamp of last sensor read
unsigned long lastSendMillis = 0; // Timestamp of last data send

// Buffers to store sensor readings for filtering
const int maxSamples = 8;
float pm25Buffer[maxSamples], pm10Buffer[maxSamples];
float so2Buffer[maxSamples], no2Buffer[maxSamples];
int sampleIndex = 0;        // Current index in the buffers
int samplesCollected = 0;   // Number of samples collected so far

String cachedTelemetry = ""; // Cache for telemetry data if sending fails

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  //while (!Serial); // Uncomment if waiting for Serial Monitor to open is needed

  // Set MQTT client ID and credentials for authentication
  mqttClient.setId(deviceId);
  mqttClient.setUsernamePassword(SECRET_USERNAME, SECRET_PASSWORD);

  // Initialize particulate matter sensor
  sds.begin();
  sds.setActiveReportingMode(); // Set sensor to active reporting mode
  sds.setQueryReportingMode();  // Enable query reporting mode

  // Initialize SPI bus and O3 sensor
  SPI.begin();
  o3Sensor.begin();

  // Calibrate zero offsets for SO2 and NO2 sensors to remove baseline noise
  calibrateZeroOffset_SO2();
  calibrateZeroOffset_NO2();
}

void loop() {

  // Ensure cellular network and GPRS are connected; attempt connection if not
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) connectNB();

  // Ensure MQTT client is connected; attempt connection if not
  if (!mqttClient.connected()) connectMQTT();

  // Process incoming MQTT packets and maintain connection
  mqttClient.poll();

  unsigned long now = millis();

  // Read sensor values at defined intervals
  if (now - lastReadMillis >= readInterval) {
    lastReadMillis = now;

    // Query particulate matter sensor for PM2.5 and PM10 values
    PmResult pm = sds.queryPm();
    if (pm.isOk()) {
      pm25Buffer[sampleIndex] = pm.pm25; // Store PM2.5 reading
      pm10Buffer[sampleIndex] = pm.pm10; // Store PM10 reading
    } else {
      pm25Buffer[sampleIndex] = 0; // If reading fails, store zero
      pm10Buffer[sampleIndex] = 0;
    }

    // Read SO2 and NO2 concentrations and store in buffers
    so2Buffer[sampleIndex] = getSO2();
    no2Buffer[sampleIndex] = getNO2();

    // Move to next buffer index, wrapping around if needed
    sampleIndex = (sampleIndex + 1) % maxSamples;

    // Keep track of how many samples have been collected (up to maxSamples)
    if (samplesCollected < maxSamples) samplesCollected++;
  }

  // Send averaged sensor data at defined intervals if samples are available
  if (now - lastSendMillis >= sendInterval && samplesCollected > 0) {
    lastSendMillis = now;

    // Calculate median values to reduce noise and outliers
    float pm25 = median(pm25Buffer, samplesCollected);
    float pm10 = median(pm10Buffer, samplesCollected);
    float so2 = median(so2Buffer, samplesCollected);
    float no2 = median(no2Buffer, samplesCollected);

    // Read O3 sensor value (not buffered)
    float o3 = readO3();

    // Publish sensor data via MQTT
    publishMessage(pm25, pm10, o3, so2, no2);
  }

  // If there is cached telemetry data from a previous failed send, retry sending it
  if (cachedTelemetry.length() > 0 && mqttClient.connected()) {
    mqttClient.beginMessage("/aqs/sensors");
    mqttClient.print(cachedTelemetry);
    mqttClient.endMessage();
    Serial.println("📤 Retried cached message.");
    cachedTelemetry = ""; // Clear cache after successful send
  }
}

// Reads O3 sensor multiple times, calculates average ppm, and applies calibration
float readO3() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    long raw = o3Sensor.readRaw();
    if (raw == -1) continue; // Skip invalid readings
    float voltage = o3Sensor.rawToVoltage(raw);          // Convert raw ADC to voltage
    float RS = o3Sensor.voltageToResistance(voltage);   // Convert voltage to sensor resistance
    float ratio = RS / RO;                               // Calculate resistance ratio
    float ppm = A * pow(ratio, B);                       // Apply calibration formula to get ppm
    sum += ppm;
    delay(50); // Small delay between readings
  }
  // Average the readings and apply sensor-specific scaling factor
  return (sum / 10.0) * 1961.0;
}

// Reads SO2 sensor voltage, compensates for temperature, and calculates concentration
float getSO2() {
  float Vgas = analogRead(VgasPin_SO2) * (3.3 / 1023.0); // Sensor gas voltage
  float Vref = analogRead(VrefPin_SO2) * (3.3 / 1023.0); // Reference voltage
  float temp = getTemperature(analogRead(VtempPin_SO2) * (3.3 / 1023.0)); // Temperature in °C

  // Temperature compensation factor for sensor sensitivity
  float span = (temp < 20.0) ? 1.0 + (-0.0033) * (temp - 20.0) : 1.0 + 0.0026 * (temp - 20.0);

  // Adjust sensor voltage by subtracting reference and zero offset
  float adjV = (Vgas - Vref) - zeroOffset_SO2;

  // Calculate SO2 concentration using sensitivity and compensation
  float so2 = adjV / (M_SO2 * span);

  // Return zero if concentration is negative (sensor noise)
  return (so2 < 0) ? 0 : so2;
}

// Reads NO2 sensor voltage, compensates for temperature, and calculates concentration
float getNO2() {
  float Vgas = analogRead(VgasPin_NO2) * (3.3 / 1023.0); // Sensor gas voltage
  float Vref = analogRead(VrefPin_NO2) * (3.3 / 1023.0); // Reference voltage
  float temp = getTemperature(analogRead(VtempPin_NO2) * (3.3 / 1023.0)); // Temperature in °C

  // Temperature compensation factor for sensor sensitivity
  float span = 1.0 + 0.003 * (temp - 20.0);

  // Adjust sensor voltage by subtracting reference and zero offset
  float adjV = (Vgas - Vref) - zeroOffset_NO2;

  // Calculate NO2 concentration using sensitivity and compensation
  float no2 = adjV / (M_NO2 * span);

  // Return zero if concentration is negative (sensor noise)
  return (no2 < 0) ? 0 : no2;
}

// Converts temperature sensor voltage to temperature in degrees Celsius
float getTemperature(float Vtemp) {
  return (Vtemp * 1000) / 10.0; // Convert voltage to mV and then to °C
}

// Calculates average of values in an array
float average(float *arr, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

// Calculates median of values in an array using bubble sort for simplicity
float median(float *arr, int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        swap(arr[j], arr[j + 1]);
      }
    }
  }
  return (size % 2 == 0) ? (arr[size/2 - 1] + arr[size/2]) / 2 : arr[size/2];
}

// Swaps two float values (used in sorting)
void swap(float &a, float &b) {
  float temp = a;
  a = b;
  b = temp;
}

// Publishes sensor data as JSON message to MQTT broker
void publishMessage(float pm25, float pm10, float o3, float so2, float no2) {
  char uuid[37];
  generateUUID(uuid); // Generate unique ID for the message

  DynamicJsonDocument doc(512);
  doc["id"] = uuid;        // Unique message ID
  doc["deviceId"] = deviceId; // Device identifier
  doc["PM2.5"] = pm25;     // PM2.5 concentration
  doc["PM10"] = pm10;      // PM10 concentration
  doc["O3"] = o3;          // Ozone concentration
  doc["SO2"] = so2;        // Sulfur dioxide concentration
  doc["NO2"] = no2;        // Nitrogen dioxide concentration

  String telemetry;
  serializeJson(doc, telemetry); // Convert JSON to string

  Serial.println("📤 JSON to send: " + telemetry);

  // Attempt to send message to MQTT topic
  if (mqttClient.beginMessage("/aqs/sensors")) {
    mqttClient.print(telemetry);
    mqttClient.endMessage();
    Serial.println("✅ Message sent.");
    cachedTelemetry = ""; // Clear cache on successful send
  } else {
    Serial.println("❌ Failed to send. Caching message.");
    cachedTelemetry = telemetry; // Cache message to retry later
  }
}

// Connects to cellular network and attaches GPRS for data transmission
void connectNB() {
  Serial.println("Connecting to cellular network...");
  while ((nbAccess.begin(pinnumber) != NB_READY) || (gprs.attachGPRS() != GPRS_READY)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("✅ Connected to NB!");
}

// Connects to MQTT broker using configured credentials
void connectMQTT() {
  Serial.println("Connecting to MQTT...");
  while (!mqttClient.connect(broker, mqtt_port)) {
    Serial.print("MQTT fail: ");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println("✅ Connected to MQTT broker!");
}

// Calibrates zero offset for SO2 sensor by averaging baseline readings
void calibrateZeroOffset_SO2() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += (analogRead(VgasPin_SO2) - analogRead(VrefPin_SO2)) * (3.3 / 1023.0);
    delay(50);
  }
  zeroOffset_SO2 = sum / 100.0;
}

// Calibrates zero offset for NO2 sensor by averaging baseline readings
void calibrateZeroOffset_NO2() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += (analogRead(VgasPin_NO2) - analogRead(VrefPin_NO2)) * (3.3 / 1023.0);
    delay(50);
  }
  zeroOffset_NO2 = sum / 100.0;
}

// Generates a random UUID string for message identification
void generateUUID(char *uuid) {
  uint16_t r[8];
  for (int i = 0; i < 8; i++) r[i] = random(0, 0xFFFF);
  sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x",
          r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
}
