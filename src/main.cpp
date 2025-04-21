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
void connectNB();
void connectMQTT();
void publishMessage(float pm25, float pm10, float o3, float so2, float no2);
float readO3();
float getSO2();
float getNO2();
float getTemperature(float VtempVoltage);
void calibrateZeroOffset_SO2();
void calibrateZeroOffset_NO2();
void generateUUID(char *uuid);
float average(float *arr, int size);
float median(float *arr, int size);
void swap(float &a, float &b);

// --- Pin definitions ---
const int CS_PIN = 7;

// --- O3 Calibration Constants ---
const float RO = 2581.7;
const float A = 0.10;
const float B = -1.1;

Ozone2Click o3Sensor(CS_PIN);

// --- SO2 Sensor ---
const int VgasPin_SO2 = A1;
const int VrefPin_SO2 = A2;
const int VtempPin_SO2 = A3;
double sensitivityCode_SO2 = 41.74;
const double TIA_Gain_SO2 = 100.0;
double M_SO2 = sensitivityCode_SO2 * TIA_Gain_SO2 * 0.000001;
float zeroOffset_SO2 = 0.0;

// --- NO2 Sensor ---
const int VgasPin_NO2 = A4;
const int VrefPin_NO2 = A5;
const int VtempPin_NO2 = A6;
double sensitivityCode_NO2 = 41.74;
const double TIA_Gain_NO2 = 499.0;
double M_NO2 = sensitivityCode_NO2 * TIA_Gain_NO2 * 0.000001;
float zeroOffset_NO2 = 0.0;

// --- Network Configuration ---
const char pinnumber[] = SECRET_PINNUMBER;
const char broker[] = SECRET_BROKER;
const int mqtt_port = SECRET_PORT;
String deviceId = SECRET_DEVICE_ID;

SdsDustSensor sds(Serial1);
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqttClient(nbClient);

const unsigned long readInterval = 2 * 60 * 1000UL;
const unsigned long sendInterval = 5 * 60 * 1000UL;// Time between sending data to the server
unsigned long lastReadMillis = 0;
unsigned long lastSendMillis = 0;

const int maxSamples = 8;
float pm25Buffer[maxSamples], pm10Buffer[maxSamples];
float so2Buffer[maxSamples], no2Buffer[maxSamples];
int sampleIndex = 0;
int samplesCollected = 0;

String cachedTelemetry = "";

void setup() {
  Serial.begin(9600);
  while (!Serial);
  mqttClient.setId(deviceId);
  mqttClient.setUsernamePassword(SECRET_USERNAME, SECRET_PASSWORD);
  sds.begin();
  sds.setActiveReportingMode();
  sds.setQueryReportingMode();
  SPI.begin();
  o3Sensor.begin();
  calibrateZeroOffset_SO2();
  calibrateZeroOffset_NO2();
}

void loop() {

  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) connectNB();
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.poll();

  unsigned long now = millis();

  if (now - lastReadMillis >= readInterval) {
    lastReadMillis = now;
    PmResult pm = sds.queryPm();
    if (pm.isOk()) {
      pm25Buffer[sampleIndex] = pm.pm25;
      pm10Buffer[sampleIndex] = pm.pm10;
    } else {
      pm25Buffer[sampleIndex] = 0;
      pm10Buffer[sampleIndex] = 0;
    }
    so2Buffer[sampleIndex] = getSO2();
    no2Buffer[sampleIndex] = getNO2();
    sampleIndex = (sampleIndex + 1) % maxSamples;
    if (samplesCollected < maxSamples) samplesCollected++;
  }

  if (now - lastSendMillis >= sendInterval && samplesCollected > 0) {
    lastSendMillis = now;
    float pm25 = median(pm25Buffer, samplesCollected);
    float pm10 = median(pm10Buffer, samplesCollected);
    float so2 = median(so2Buffer, samplesCollected);
    float no2 = median(no2Buffer, samplesCollected);
    float o3 = readO3();
    publishMessage(pm25, pm10, o3, so2, no2);
  }

  if (cachedTelemetry.length() > 0 && mqttClient.connected()) {
    mqttClient.beginMessage("/aqs/sensors");
    mqttClient.print(cachedTelemetry);
    mqttClient.endMessage();
    Serial.println("📤 Retried cached message.");
    cachedTelemetry = "";
  }
}

float readO3() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    long raw = o3Sensor.readRaw();
    if (raw == -1) continue;
    float voltage = o3Sensor.rawToVoltage(raw);
    float RS = o3Sensor.voltageToResistance(voltage);
    float ratio = RS / RO;
    float ppm = A * pow(ratio, B);
    sum += ppm;
    delay(50);
  }
  return (sum / 10.0) * 1961.0;
}

float getSO2() {
  float Vgas = analogRead(VgasPin_SO2) * (3.3 / 1023.0);
  float Vref = analogRead(VrefPin_SO2) * (3.3 / 1023.0);
  float temp = getTemperature(analogRead(VtempPin_SO2) * (3.3 / 1023.0));
  float span = (temp < 20.0) ? 1.0 + (-0.0033) * (temp - 20.0) : 1.0 + 0.0026 * (temp - 20.0);
  float adjV = (Vgas - Vref) - zeroOffset_SO2;
  float so2 = adjV / (M_SO2 * span);
  return (so2 < 0) ? 0 : so2;
}

float getNO2() {
  float Vgas = analogRead(VgasPin_NO2) * (3.3 / 1023.0);
  float Vref = analogRead(VrefPin_NO2) * (3.3 / 1023.0);
  float temp = getTemperature(analogRead(VtempPin_NO2) * (3.3 / 1023.0));
  float span = 1.0 + 0.003 * (temp - 20.0);
  float adjV = (Vgas - Vref) - zeroOffset_NO2;
  float no2 = adjV / (M_NO2 * span);
  return (no2 < 0) ? 0 : no2;
}

float getTemperature(float Vtemp) {
  return (Vtemp * 1000) / 10.0;
}

float average(float *arr, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  return sum / size;
}

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

void swap(float &a, float &b) {
  float temp = a;
  a = b;
  b = temp;
}

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

void generateUUID(char *uuid) {
  uint16_t r[8];
  for (int i = 0; i < 8; i++) r[i] = random(0, 0xFFFF);
  sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x",
          r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
}



