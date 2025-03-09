#include <Arduino.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <SdsDustSensor.h>
#include <SPI.h>
#include "arduino_secrets.h"

// Pin definitions
const int CS_PIN = 7;          // Chip Select for O3 sensor
const int AN_PIN = A0;         // Analog pin for O3

// O3 Sensor Constants
const float RL = 10.0;         // Load resistance in kOhms
const float RO = 3.0;          // Sensor resistance in clean air
const float A = 0.99;          // MQ131 constants
const float B = -0.38;
const int NUM_READINGS = 10;

// SO2 Sensor Constants
const int VgasPin_SO2 = A1;
const int VrefPin_SO2 = A2;
const int VtempPin_SO2 = A3;
double sensitivityCode_SO2 = 41.74;  // Update with actual code
const double TIA_Gain_SO2 = 100.0;
double M_SO2 = sensitivityCode_SO2 * TIA_Gain_SO2 * 0.000001;
float zeroOffset_SO2 = 0.0;

// NO2 Sensor Constants
const int VgasPin_NO2 = A4;
const int VrefPin_NO2 = A5;
const int VtempPin_NO2 = A6;
double sensitivityCode_NO2 = 41.74;  // Update with actual code
const double TIA_Gain_NO2 = 499.0;    // From datasheet
double M_NO2 = sensitivityCode_NO2 * TIA_Gain_NO2 * 0.000001;
float zeroOffset_NO2 = 0.0;

// Common Constants
const float baselineTemp = 20.0;
const float molecularWeightSO2 = 64.07;
const float molarVolume = 24.45;

// Network configuration
const char pinnumber[] = SECRET_PINNUMBER;
const char broker[] = SECRET_BROKER;
String deviceId = SECRET_DEVICE_ID;

SdsDustSensor sds(Serial1);
NB nbAccess;
GPRS gprs;
NBClient nbClient;
BearSSLClient sslClient(nbClient);
MqttClient mqttClient(sslClient);
unsigned long lastMillis = 0;

// Function prototypes
unsigned long getTime();
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

        PmResult pm = sds.queryPm();
        float o3 = readO3();
        float so2 = getSO2();
        float no2 = getNO2();

        Serial.print("PM2.5: "); Serial.println(pm.pm25);
        Serial.print("PM10: "); Serial.println(pm.pm10);
        Serial.print("O3: "); Serial.println(o3);
        Serial.print("SO2: "); Serial.println(so2);
        Serial.print("NO2: "); Serial.println(no2);

        if (pm.isOk() && !isnan(o3) && !isnan(so2) && !isnan(no2)) {
            publishMessage(pm.pm25, pm.pm10, o3, so2, no2);
        } else {
            Serial.println("Invalid readings, skip publish");
        }
    }
}

// Sensor Functions -----------------------------------------------------------
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

    float spanAdjust = 1.0 + 0.003 * (temperature - baselineTemp);
    float adjustedVoltage = (VgasVoltage - VrefVoltage) - zeroOffset_NO2;
    float no2 = adjustedVoltage / (M_NO2 * spanAdjust);
    return no2 < 0 ? 0 : no2;
}

float getTemperature(float VtempVoltage) {
    return (VtempVoltage * 1000) / 10.0;
}

// Calibration Functions ------------------------------------------------------
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

// Network Functions ----------------------------------------------------------
void generateUUID(char *uuid) {
    uint16_t r[8];
    for(int i=0; i<8; i++) r[i] = random(0, 0xFFFF);
    sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x", 
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
}

void publishMessage(float pm25, float pm10, float o3, float so2, float no2) {
    char uuid[37];
    generateUUID(uuid);

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
    data["NO2"] = no2;

    String telemetry;
    serializeJson(doc, telemetry);
    
    mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
    mqttClient.print(telemetry);
    mqttClient.endMessage();
}

unsigned long getTime() {
    return nbAccess.getTime();
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

    while (!mqttClient.connect(broker, 8883)) {
        Serial.print("MQTT connection failed: ");
        Serial.println(mqttClient.connectError());
        Serial.println("Retrying in 5 seconds...");
        delay(5000);
    }

    Serial.println("Connected to MQTT broker!");
    mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#");
}

void onMessageReceived(int messageSize) {
    Serial.print("Received message: ");
    while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
    }
    Serial.println();
}