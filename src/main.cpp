#include <Arduino.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <ArduinoJson.h>
#include <TimeLib.h>  // Time library for time conversion
#include <SdsDustSensor.h>
#include <SPI.h>
#include "arduino_secrets.h"

// Pin definitions
const int CS_PIN = 7; // Chip Select
const int AN_PIN = A0; // Analog pin

// Constants for O₃ Sensor (Ozone2click)
const float RL = 10.0;  // Load resistance in kOhms
const float RO = 3.0;   // Sensor resistance in clean air
const float A = 0.99;   // MQ131 constant
const float B = -0.38;  // MQ131 constant
const int NUM_READINGS = 10;  // Averaging readings

// Constants for SO₂ Sensor
const int VgasPin = A1;
const int VrefPin = A2;
const int VtempPin = A3;
const float molecularWeightSO2 = 64.07;
const float molarVolume = 24.45;
double sensitivityCode = 41.74;  // Example sensitivity code from sensor label
double TIA_Gain = 100.0; 
double M = sensitivityCode * TIA_Gain * 0.000001;
const float baselineTemp = 20.0; 
float zeroOffset = 0.0;

// Sensitive data in arduino_secrets.h
const char pinnumber[] = SECRET_PINNUMBER;
const char broker[] = SECRET_BROKER;
String deviceId = SECRET_DEVICE_ID;

// Initialize the dust sensor
SdsDustSensor sds(Serial1);

// Cellular & MQTT Objects
NB nbAccess;
GPRS gprs;
NBClient nbClient;
BearSSLClient sslClient(nbClient);
MqttClient mqttClient(sslClient);

unsigned long lastMillis = 0;  // Time tracking

// Function declarations
unsigned long getTime();
void connectNB();
void calibrateZeroOffset();
void connectMQTT();
void publishMessage(float pm25, float pm10, float o3, float so2);
void onMessageReceived(int messageSize);
float readO3();
float getSO2();
float getTemperature(float VtempVoltage);

// Generate a **UUID** for each message
void generateUUID(char *uuid) {
    uint16_t r1 = random(0, 0xFFFF);
    uint16_t r2 = random(0, 0xFFFF);
    uint16_t r3 = random(0, 0xFFFF);
    uint16_t r4 = random(0, 0xFFFF);
    uint16_t r5 = random(0, 0xFFFF);
    uint16_t r6 = random(0, 0xFFFF);
    uint16_t r7 = random(0, 0xFFFF);
    uint16_t r8 = random(0, 0xFFFF);

    sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x",
            r1, r2, r3, r4, r5, r6, r7, r8);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Initializing...");

    if (!ECCX08.begin()) {
        Serial.println("No ECCX08 present!");
        while (1);
    }

    ECCX08SelfSignedCert.beginReconstruction(0, 8);
    ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
    ECCX08SelfSignedCert.endReconstruction();

    ArduinoBearSSL.onGetTime(getTime);
    sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

    mqttClient.setId(deviceId);
    String username = String(broker) + "/" + deviceId + "/api-version=2018-06-30";
    mqttClient.setUsernamePassword(username, "");
    mqttClient.onMessage(onMessageReceived);

    // Initialize SDS011 Dust Sensor
    sds.begin();
    sds.setActiveReportingMode();
    sds.setQueryReportingMode();

    // Initialize SPI
    SPI.begin();
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    calibrateZeroOffset();
}

void loop() {
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
        connectNB();
    }

    if (!mqttClient.connected()) {
        connectMQTT();
    }

    mqttClient.poll();  

    if (millis() - lastMillis > 60000) {
        lastMillis = millis();

        Serial.println("Reading sensor data...");

        PmResult pm = sds.queryPm();
        float o3 = readO3();
        float so2 = getSO2();

        Serial.print("PM2.5: "); Serial.println(pm.pm25);
        Serial.print("PM10: "); Serial.println(pm.pm10);
        Serial.print("O3: "); Serial.println(o3);
        Serial.print("SO2: "); Serial.println(so2);

        if (pm.isOk() && !isnan(o3) && !isnan(so2)) {
            Serial.println("Publishing data...");
            publishMessage(pm.pm25, pm.pm10, o3, so2);
        } else {
            Serial.println("Invalid sensor readings, skipping publish...");
        }
    }
}

unsigned long getTime() {
    return nbAccess.getTime();
}

void connectNB() {
    Serial.println("Attempting to connect to cellular network...");
    while ((nbAccess.begin(pinnumber) != NB_READY) || (gprs.attachGPRS() != GPRS_READY)) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected to cellular network!");
}

void connectMQTT() {
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(broker);

    while (!mqttClient.connect(broker, 8883)) {
        Serial.print("MQTT Connection failed: ");
        Serial.println(mqttClient.connectError());
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }

    Serial.println("Connected to MQTT broker!");
    mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#");
}

void publishMessage(float pm25, float pm10, float o3, float so2) {
    char uuid[37];
    generateUUID(uuid);  // Generate a unique ID

    DynamicJsonDocument doc(1024);
    doc["id"] = uuid;
    doc["partitionKey"] = "Timestamp";
    doc["deviceId"] = deviceId;
    doc["timestamp"] = millis();

    JsonObject data = doc.createNestedObject("data");
    data["pm2.5"] = pm25;
    data["pm10"] = pm10;
    data["O3"] = o3;
    data["SO2"] = so2;

    String telemetry;
    serializeJson(doc, telemetry);
    Serial.println("Publishing JSON: " + telemetry);

    mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
    mqttClient.print(telemetry);
    mqttClient.endMessage();
}

void onMessageReceived(int messageSize) {
    Serial.print("Received message: ");
    while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
    }
    Serial.println();
}

float readO3() {
    digitalWrite(CS_PIN, LOW);
    int sensorValue = analogRead(AN_PIN);
    digitalWrite(CS_PIN, HIGH);

    float voltage = sensorValue * (5.0 / 1023.0);
    float RS = (5.0 - voltage) / voltage * RL;
    float ratio = RS / RO;
    return A * pow(ratio, B);  // O₃ value is in ppm
}

float getSO2() {
    float VgasVoltage = analogRead(VgasPin) * (3.3 / 1023.0);
    float VrefVoltage = analogRead(VrefPin) * (3.3 / 1023.0);
    float VtempVoltage = analogRead(VtempPin) * (3.3 / 1023.0);
    float temperature = getTemperature(VtempVoltage);

    float spanAdjust = (temperature < 20.0) ? 1.0 + (-0.0033) * (temperature - 20.0) : 1.0 + 0.0026 * (temperature - 20.0);
    float adjustedVoltage = (VgasVoltage - VrefVoltage) - zeroOffset;
    float so2 = (adjustedVoltage) / (M * spanAdjust);
    return so2 < 0 ? 0 : so2;
}

float getTemperature(float VtempVoltage) {
    return (VtempVoltage * 1000) / 10.0;
}

void calibrateZeroOffset() {
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        int VgasRaw = analogRead(VgasPin);
        int VrefRaw = analogRead(VrefPin);
        sum += (VgasRaw - VrefRaw) * (3.3 / 1023.0);
        delay(100);
    }
    zeroOffset = sum / 100.0;
    Serial.print("Calibrated zeroOffset: "); Serial.println(zeroOffset, 6);
}

