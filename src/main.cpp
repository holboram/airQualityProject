#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <SdsDustSensor.h>
#include <SPI.h>
#include "arduino_secrets.h"

// Pin definitions
const int CS_PIN = 7;
const int AN_PIN = A0;

// O3 Sensor Constants
const float RL = 10.0;
const float RO = 3.0;
const float A = 0.99;
const float B = -0.38;

// SO2 Sensor Constants
const int VgasPin_SO2 = A1;
const int VrefPin_SO2 = A2;
const int VtempPin_SO2 = A3;
double sensitivityCode_SO2 = 41.74;
const double TIA_Gain_SO2 = 100.0;
double M_SO2 = sensitivityCode_SO2 * TIA_Gain_SO2 * 0.000001;
float zeroOffset_SO2 = 0.0;

// NO2 Sensor Constants
const int VgasPin_NO2 = A4;
const int VrefPin_NO2 = A5;
const int VtempPin_NO2 = A6;
double sensitivityCode_NO2 = 41.74;
const double TIA_Gain_NO2 = 499.0;
double M_NO2 = sensitivityCode_NO2 * TIA_Gain_NO2 * 0.000001;
float zeroOffset_NO2 = 0.0;

// Network Configuration
const char pinnumber[] = SECRET_PINNUMBER;
const char broker[] = SECRET_BROKER;
const int mqtt_port = SECRET_PORT;
String deviceId = SECRET_DEVICE_ID;

SdsDustSensor sds(Serial1);
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqttClient(nbClient);

// Configurable send interval
const unsigned long sendInterval = 60000; // 1 minute
unsigned long lastMillis = 0;

// Cached JSON buffer
String cachedTelemetry = "";

// Function prototypes
void connectNB();
void connectMQTT();
void publishMessage(float pm25, float pm10, float o3, float so2, float no2);
float readO3Average();
float getSO2Average();
float getNO2Average();
float getTemperature(float VtempVoltage);
void calibrateZeroOffset_SO2();
void calibrateZeroOffset_NO2();
void generateUUID(char *uuid);

void setup() {
  Serial.begin(9600);
  if (Serial) {
    while (!Serial);
  }

  mqttClient.setId(deviceId);
  mqttClient.setUsernamePassword(SECRET_USERNAME, SECRET_PASSWORD);
  sds.begin();
  sds.setActiveReportingMode();
  sds.setQueryReportingMode();

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  calibrateZeroOffset_SO2();
  calibrateZeroOffset_NO2();
}

void loop() {
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
    connectNB();
  }

  if (!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.poll();

  if (millis() - lastMillis > sendInterval) {
    lastMillis = millis();

    PmResult pm = sds.queryPm();
    float o3 = readO3Average();
    float so2 = getSO2Average();
    float no2 = getNO2Average();

    if (pm.isOk() && !isnan(o3) && !isnan(so2) && !isnan(no2)) {
      publishMessage(pm.pm25, pm.pm10, o3, so2, no2);
    } else {
      Serial.println("⚠️ Invalid sensor readings. Skipping publish.");
    }
  }

  if (cachedTelemetry.length() > 0 && mqttClient.connected()) {
    mqttClient.beginMessage("/aqs/sensors");
    mqttClient.print(cachedTelemetry);
    mqttClient.endMessage();
    Serial.println("📤 Retried cached message.");
    cachedTelemetry = "";
  }
}

// --- Sensor Reading & Averaging ---
float readO3Average() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(CS_PIN, LOW);
    int sensorValue = analogRead(AN_PIN);
    digitalWrite(CS_PIN, HIGH);
    float voltage = sensorValue * (5.0 / 1023.0);
    float RS = (5.0 - voltage) / voltage * RL;
    float ratio = RS / RO;
    float o3 = A * pow(ratio, B);
    sum += o3;
    delay(50);
  }
  return (sum / 10.0) * 1961.0;  // Convert ppm to µg/m³
}

float getSO2Average() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    float Vgas = analogRead(VgasPin_SO2) * (3.3 / 1023.0);
    float Vref = analogRead(VrefPin_SO2) * (3.3 / 1023.0);
    float temp = getTemperature(analogRead(VtempPin_SO2) * (3.3 / 1023.0));
    float span = (temp < 20.0) ? 1.0 + (-0.0033) * (temp - 20.0) : 1.0 + 0.0026 * (temp - 20.0);
    float adjV = (Vgas - Vref) - zeroOffset_SO2;
    float so2 = adjV / (M_SO2 * span);
    sum += (so2 < 0) ? 0 : so2;
    delay(50);
  }
  return sum / 10.0;
}

float getNO2Average() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    float Vgas = analogRead(VgasPin_NO2) * (3.3 / 1023.0);
    float Vref = analogRead(VrefPin_NO2) * (3.3 / 1023.0);
    float temp = getTemperature(analogRead(VtempPin_NO2) * (3.3 / 1023.0));
    float span = 1.0 + 0.003 * (temp - 20.0);
    float adjV = (Vgas - Vref) - zeroOffset_NO2;
    float no2 = adjV / (M_NO2 * span);
    sum += (no2 < 0) ? 0 : no2;
    delay(50);
  }
  return sum / 10.0;
}

float getTemperature(float Vtemp) {
  return (Vtemp * 1000) / 10.0;
}

// --- MQTT Publishing ---
void publishMessage(float pm25, float pm10, float o3, float so2, float no2) {
  char uuid[37];
  generateUUID(uuid);

  DynamicJsonDocument doc(512);
  doc["id"] = uuid;
  doc["deviceId"] = deviceId;
  doc["PM2.5"] = pm25;
  doc["PM10"] = pm10;
  doc["O3"] = o3;
  doc["SO2"] = so2;
  doc["NO2"] = no2;

  String telemetry;
  serializeJson(doc, telemetry);
  Serial.println("📤 JSON to send: " + telemetry);

  if (mqttClient.beginMessage("/aqs/sensors")) {
    mqttClient.print(telemetry);
    mqttClient.endMessage();
    Serial.println("✅ Message sent.");
    cachedTelemetry = "";
  } else {
    Serial.println("❌ Failed to send. Caching message.");
    cachedTelemetry = telemetry;
  }
}

// --- Network + MQTT ---
void connectNB() {
  Serial.println("Connecting to cellular network...");
  while ((nbAccess.begin(pinnumber) != NB_READY) || (gprs.attachGPRS() != GPRS_READY)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("✅ Connected to NB!");
}

void connectMQTT() {
  Serial.println("Connecting to MQTT...");
  while (!mqttClient.connect(broker, mqtt_port)) {
    Serial.print("MQTT fail: ");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println("✅ Connected to MQTT broker!");
}

// --- Calibration ---
void calibrateZeroOffset_SO2() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += (analogRead(VgasPin_SO2) - analogRead(VrefPin_SO2)) * (3.3 / 1023.0);
    delay(50);
  }
  zeroOffset_SO2 = sum / 100.0;
}

void calibrateZeroOffset_NO2() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += (analogRead(VgasPin_NO2) - analogRead(VrefPin_NO2)) * (3.3 / 1023.0);
    delay(50);
  }
  zeroOffset_NO2 = sum / 100.0;
}

// --- UUID Generator ---
void generateUUID(char *uuid) {
  uint16_t r[8];
  for (int i = 0; i < 8; i++) r[i] = random(0, 0xFFFF);
  sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x",
          r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
}

