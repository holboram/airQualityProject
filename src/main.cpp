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
const int NUM_READINGS = 10;

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
const char broker[] = SECRET_BROKER;  // Socat MQTT forwarder
const int mqtt_port = SECRET_PORT;
String deviceId = SECRET_DEVICE_ID;

SdsDustSensor sds(Serial1);
NB nbAccess;
GPRS gprs;
NBClient nbClient;
MqttClient mqttClient(nbClient);
unsigned long lastMillis = 0;

// Function prototypes
void connectNB();
void connectMQTT();
void publishMessage(float pm25, float pm10, float o3, float so2, float no2);
void onMessageReceived(int messageSize);
float readO3();
float getSO2();
float getNO2();
float getTemperature(float VtempVoltage);
void calibrateZeroOffset_SO2();
void calibrateZeroOffset_NO2();
void generateUUID(char *uuid);

void setup() {
    Serial.begin(9600);
    while (!Serial);

    mqttClient.setId(deviceId);
    mqttClient.setUsernamePassword(SECRET_USERNAME, SECRET_PASSWORD);
    mqttClient.onMessage(onMessageReceived);

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

    if (millis() - lastMillis > 60000) {
        lastMillis = millis();

        Serial.println("Reading sensor data...");
        PmResult pm = sds.queryPm();
        float o3 = readO3();
        float so2 = getSO2();
        float no2 = getNO2();

        Serial.print("PM2.5: "); Serial.println(pm.pm25, 2);
        Serial.print("PM10: "); Serial.println(pm.pm10, 2);
        Serial.print("O3: "); Serial.println(o3, 2);
        Serial.print("SO2: "); Serial.println(so2, 2);
        Serial.print("NO2: "); Serial.println(no2, 2);

        if (pm.isOk() && !isnan(o3) && !isnan(so2) && !isnan(no2)) {
            Serial.println("Publishing data...");
            publishMessage(pm.pm25, pm.pm10, o3, so2, no2);
        } else {
            Serial.println("Invalid readings, skip publish");
        }
    }
}

float readO3() {
    digitalWrite(CS_PIN, LOW);
    int sensorValue = analogRead(AN_PIN);
    digitalWrite(CS_PIN, HIGH);

    float voltage = sensorValue * (5.0 / 1023.0);
    float RS = (5.0 - voltage) / voltage * RL;
    float ratio = RS / RO;
    return A * pow(ratio, B);
}

float getSO2() {
    float VgasVoltage = analogRead(VgasPin_SO2) * (3.3 / 1023.0);
    float VrefVoltage = analogRead(VrefPin_SO2) * (3.3 / 1023.0);
    float temperature = getTemperature(analogRead(VtempPin_SO2) * (3.3 / 1023.0));

    float spanAdjust = (temperature < 20.0) ? 1.0 + (-0.0033) * (temperature - 20.0)
                                            : 1.0 + 0.0026 * (temperature - 20.0);
    float adjustedVoltage = (VgasVoltage - VrefVoltage) - zeroOffset_SO2;
    float so2 = adjustedVoltage / (M_SO2 * spanAdjust);
    return so2 < 0 ? 0 : so2;
}

float getNO2() {
    float VgasVoltage = analogRead(VgasPin_NO2) * (3.3 / 1023.0);
    float VrefVoltage = analogRead(VrefPin_NO2) * (3.3 / 1023.0);
    float temperature = getTemperature(analogRead(VtempPin_NO2) * (3.3 / 1023.0));

    float spanAdjust = 1.0 + 0.003 * (temperature - 20.0);
    float adjustedVoltage = (VgasVoltage - VrefVoltage) - zeroOffset_NO2;
    float no2 = adjustedVoltage / (M_NO2 * spanAdjust);
    return no2 < 0 ? 0 : no2;
}

float getTemperature(float VtempVoltage) {
    return (VtempVoltage * 1000) / 10.0;
}

void calibrateZeroOffset_SO2() {
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += (analogRead(VgasPin_SO2) - analogRead(VrefPin_SO2)) * (3.3 / 1023.0);
        delay(100);
    }
    zeroOffset_SO2 = sum / 100.0;
    Serial.print("SO2 Zero Offset: "); Serial.println(zeroOffset_SO2, 6);
}

void calibrateZeroOffset_NO2() {
    float sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += (analogRead(VgasPin_NO2) - analogRead(VrefPin_NO2)) * (3.3 / 1023.0);
        delay(100);
    }
    zeroOffset_NO2 = sum / 100.0;
    Serial.print("NO2 Zero Offset: "); Serial.println(zeroOffset_NO2, 6);
}

void connectNB() {
    Serial.println("Attempting to connect to cellular network...");
    while ((nbAccess.begin(pinnumber) != NB_READY) ||
           (gprs.attachGPRS() != GPRS_READY)) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nConnected to cellular network!");
}

void connectMQTT() {
    Serial.print("Connecting to MQTT broker: ");
    Serial.println(broker);

    while (!mqttClient.connect(broker, mqtt_port)) {
        Serial.print("MQTT connection failed: ");
        Serial.println(mqttClient.connectError());
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }

    Serial.println("✅ Connected to MQTT broker!");
    mqttClient.subscribe("/aqs/sensors");
}

void onMessageReceived(int messageSize) {
    Serial.print("📩 Message received: ");
    while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
    }
    Serial.println();
}

void generateUUID(char *uuid) {
    uint16_t r[8];
    for (int i = 0; i < 8; i++) r[i] = random(0, 0xFFFF);
    sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x",
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
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
    Serial.println("📤 Publishing JSON: " + telemetry);

    mqttClient.beginMessage("/aqs/sensors");
    mqttClient.print(telemetry);
    mqttClient.endMessage();
}