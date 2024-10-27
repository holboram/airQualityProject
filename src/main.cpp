#include <Arduino.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <utility/ECCX08SelfSignedCert.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
//#include <Arduino_MKRIoTCarrier.h>
#include <ArduinoJson.h>
#include <TimeLib.h> // Include the Time library for time conversion
#include <SdsDustSensor.h>
#include <SPI.h>

#include "arduino_secrets.h"

// Pin definitions
const int CS_PIN = 7; // Chip Select
const int AN_PIN = A0; // Analog pin

// Constants for conversion
const float RL = 10.0; // Load resistance in kOhms
const float RO = 3.0; // Sensor resistance in clean air
const float A = 0.99; // Constant for MQ131
const float B = -0.38; // Constant for MQ131

// Number of readings for averaging
const int NUM_READINGS = 10;

// Constant for SO2 sensor 
const int gasPin = A1; // SO2 sensor output pin

// Function declarations
unsigned long getTime();
void connectNB();
void connectMQTT();
void publishMessage(float pm25, float pm10, float o3microgramPerM3, float so2);
void onMessageReceived(int messageSize);
float readO3Sensor();

float calculateO3PPM(float RS);

float calculateO3MicrogramPerM3(float ppm);

float getAverageRS();

float getSO2();

// Sensitive data in arduino_secrets.h
const char pinnumber[]   = SECRET_PINNUMBER;
const char broker[]      = SECRET_BROKER;
String     deviceId      = SECRET_DEVICE_ID;

// Initialize the dust sensor
SdsDustSensor sds(Serial1);

NB nbAccess;
GPRS gprs;
NBClient      nbClient;            // Used for the TCP socket connection
BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);

unsigned long lastMillis = 0;

//IoT Carrier
//MKRIoTCarrier carrier;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!ECCX08.begin()) {
    Serial.println("No ECCX08 present!");
    while (1);
  }

  // Reconstruct the self signed cert
  ECCX08SelfSignedCert.beginReconstruction(0, 8);
  ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
  ECCX08SelfSignedCert.endReconstruction();

  // Set a callback to get the current time used to validate the servers certificate
  ArduinoBearSSL.onGetTime(getTime);

  // Set the ECCX08 slot to use for the private key and the accompanying public certificate for it
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  // Set the client id used for MQTT as the device id
  mqttClient.setId(deviceId);

  // Set the username to "<broker>/<device id>/api-version=2018-06-30" and empty password
  String username;

  username += broker;
  username += "/";
  username += deviceId;
  username += "/api-version=2018-06-30";

  mqttClient.setUsernamePassword(username, "");

  // Set the message callback, this function is called when the MQTTClient receives a message
  mqttClient.onMessage(onMessageReceived);

  // Initialize the dust sensor
  sds.begin();
  sds.setActiveReportingMode();
  sds.setQueryReportingMode();

  // Initialize SPI
  SPI.begin();

  // Set pin modes
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
}

void loop() {
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
    connectNB();
  }

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    connectMQTT();
  }

  // poll for new MQTT messages and send keep alives
  mqttClient.poll();

  // publish a message roughly every 10 sec.
  if (millis() - lastMillis > 60000) {
    lastMillis = millis();

    // Read the dust sensor data
    PmResult pm = sds.queryPm();
    //Read the O3 sensor data
    float averageO3Readings = getAverageRS();
     // Calculate PPM
    float ppm = calculateO3PPM(averageO3Readings);
    // Calculate microgram/m³
    float o3microgramPerM3 = calculateO3MicrogramPerM3(ppm);

    // Read the SO2 sensor data
    float so2 = getSO2();

    if (pm.isOk() && o3microgramPerM3 && so2) {
      float pm25 = pm.pm25;
      float pm10 = pm.pm10;
      publishMessage(pm25, pm10, o3microgramPerM3, so2);
      
    }
    
  }
}

unsigned long getTime() {
  // get the current time from the cellular module
  return nbAccess.getTime();
}

void connectNB() {
  Serial.println("Attempting to connect to the cellular network");

  while ((nbAccess.begin(pinnumber) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY)) {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }

  Serial.println("You're connected to the cellular network");
  Serial.println();
}

void connectMQTT() {
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(broker);
  Serial.println(" ");

  while (!mqttClient.connect(broker, 8883)) {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("devices/" + deviceId + "/messages/devicebound/#");
}

void publishMessage(float pm25, float pm10, float o3, float so2) {

  static int messageId = 0; // Declare and initialize the 'meesageId' variable
  DynamicJsonDocument doc(1024);
  doc["id"] = messageId++;
  doc["partitionKey"] = "Timestamp";
  doc["deviceId"] = deviceId;
  doc["pm:2.5"] = pm25;
  doc["pm:10"] = pm10;
  doc["O3"] = o3;
  doc["SO2"] = so2;

  String telemetry;
  serializeJson(doc, telemetry);
  Serial.println(telemetry.c_str());

  // send message, the Print interface can be used to set the message contents
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(telemetry.c_str());
  mqttClient.print(millis());
  mqttClient.endMessage();
}

void onMessageReceived(int messageSize) {

  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();

  Serial.println();

}

float readO3Sensor() {

  // Select the sensor
  digitalWrite(CS_PIN, LOW);
  
  // Read analog value
  int sensorValue = analogRead(AN_PIN);
  
  // Deselect the sensor
  digitalWrite(CS_PIN, HIGH);
  
  // Convert to voltage
  float voltage = sensorValue * (5.0 / 1023.0);
  
  // Calculate resistance of the sensor
  float RS = (5.0 - voltage) / voltage * RL;

  return RS;
}

float calculateO3PPM(float RS) {
  // Calculate ratio
  float ratio = RS / RO;
  
  // Calculate PPM using the equation
  float ppm = A * pow(ratio, B);
  
  return ppm;
}

float calculateO3MicrogramPerM3(float ppm) {
  // Convert PPM to microgram/m³
  float microgramPerM3 = ppm * 214.4; // Conversion factor for O3
  
  return microgramPerM3;
}

float getAverageRS() {
    float totalRS = 0;
    
    // Take multiple readings and average them
    for (int i = 0; i < NUM_READINGS; i++) {
      totalRS += readO3Sensor();
      delay(100); // Small delay between readings
    }
    
    return totalRS / NUM_READINGS;
  }

float getSO2() {
    const float sensorSensitivity = 0.002; // Sensitivity in volts per ppm (2.0 mV/ppm) from datasheet
    const float V_REF = 3.3;   // Reference voltage for MKR NB 1500
    const float ppmToMicrogramPerM3 = 2859.07; // Corrected conversion factor from ppm to µg/m^3 for SO2
    const float calibrationFactor = 0.001; // Adjust this factor based on calibration

    // Read the analog voltage from the SO2 sensor on A1
    int sensorValue = analogRead(gasPin);
    float voltage = sensorValue * (V_REF / 1023.0);  // Calculate the voltage

    // Calculate SO2 concentration in ppm using the confirmed sensitivity from the datasheet
    float so2_concentration_ppm = (voltage / sensorSensitivity) * calibrationFactor;

    // Convert ppm concentration to micrograms per cubic meter (µg/m^3)
    float so2_concentration_ug_m3 = so2_concentration_ppm * ppmToMicrogramPerM3;

    // Debugging: Print intermediate values for verification
    Serial.print("Sensor Value: ");
    Serial.println(sensorValue);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    Serial.print("SO2 Concentration (ppm): ");
    Serial.println(so2_concentration_ppm);
    Serial.print("SO2 Concentration (µg/m³): ");
    Serial.println(so2_concentration_ug_m3);

    return so2_concentration_ug_m3;
  }