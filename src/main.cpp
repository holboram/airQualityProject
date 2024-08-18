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

#include "arduino_secrets.h"

// Function declarations
unsigned long getTime();
void connectNB();
void connectMQTT();
void publishMessage(float pm25, float pm10);
void onMessageReceived(int messageSize);
float readSensor() {};
float calculatePPM(float RS) {};
float calculateMicrogramPerM3(float ppm) {};

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

  // publish a message roughly every 60 seconds.
  if (millis() - lastMillis > 60000) {
    lastMillis = millis();

    // Read the dust sensor data
    PmResult pm = sds.queryPm();
    if (pm.isOk()) {
      float pm25 = pm.pm25;
      float pm10 = pm.pm10;
      publishMessage(pm25, pm10);
      
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

void publishMessage(float pm25, float pm10) {
  // Get the current time in UTC
  unsigned long utcTime = getTime();

  // Convert UTC time to local time (assuming local time is UTC+1)
  unsigned long localTime = utcTime + 3600; // Adjust for local time zone
  
  // Convert local time to human-readable format
  setTime(localTime);
  char timeString[25];
  snprintf(timeString, sizeof(timeString), "%04d-%02d-%02dT%02d:%02d:%02dZ", year(), month(), day(), hour(), minute(), second());

  //static int messageId = 0; // Declare and initialize the 'meesageId' variable
  DynamicJsonDocument doc(1024);
  //doc["id"] = messageId++;
  //doc["partitionKey"] = "Timestamp";
  //doc["deviceId"] = deviceId;
  doc["pm:2.5"] = pm25;
  doc["pm:10"] = pm10;
  doc["localTime"] = localTime;

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