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
const int VgasPin = A1;   // Pin connected to Vgas for gas concentration
const int VrefPin = A2;   // Pin connected to Vref for reference
const int VtempPin = A3;  // Pin connected to Vtemp for temperature
const float molecularWeightSO2 = 64.07; // Molecular weight of SO2 in g/mol
const float molarVolume = 24.45; // molar volume of an ideal gas at standard temp and press (STP) in liters.
double sensitivityCode = 41.74; // Example sensitivity code from the sensor label
double TIA_Gain = 100.0; // TIA gain for Sulfur Dioxide
double M = sensitivityCode*TIA_Gain*0.000001; // Sensitivity in V/ppm
const float VgasZeroVoltage = 1.75; // Zero voltage for SO2 sensor

// Sensor sensitivity at 20 °C (room temperature) in V/ppm
const float baselineSensitivity = 0.003; // Sensitivity in V/ppm
const float baselineTemp = 20.0; // Baseline temperature for calibration

// Temperature coefficients for sensitivity adjustments
const float spanCoeffLow = -0.0033;  // -0.33% per °C (below 20°C)
const float spanCoeffHigh = 0.0026;  // +0.26% per °C (above 20°C)
const float offsetCoeffLow = 0.056;  // ppm/°C (0 to 25°C)
const float offsetCoeffHigh = 0.46;   // ppm/°C (25 to 40°C)

// Global variable to hold the zero offset for gas voltage
float zeroOffset = 0.0;

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
float getTemperature(float VtempVoltage);

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
  if (millis() - lastMillis > 600) {
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
    int Vgas = analogRead(VgasPin);      // Read gas concentration
    int Vref = analogRead(VrefPin);      // Read reference voltage
    int Vtemp = analogRead(VtempPin);    // Read temperature
  
    // Convert the analog readings to voltages (assuming 3.3V and 10-bit ADC)
    double VrefVoltage = Vref * (3.3 / 1023.00);
    double VtempVoltage = Vtemp * (3.3 / 1023.00);
    double VgasVoltage = Vgas/1023.00*3.3;
    float VgasPPM = 1/M*(VgasVoltage-VgasZeroVoltage); // Convert to ppm
    
  
    // Calculate temperature in °C
    float temperature = getTemperature(VtempVoltage);
    
    // Determine sensitivity adjustment based on temperature
    float tempAdjustedSensitivity = baselineSensitivity;
    if (temperature < baselineTemp) {
      tempAdjustedSensitivity *= (1.0 + (spanCoeffLow * (temperature - baselineTemp)));
    } else {
      tempAdjustedSensitivity *= (1.0 + (spanCoeffHigh * (temperature - baselineTemp)));
    }
  
    // Adjust zero shift based on temperature
    float zeroShift = (temperature < 25.0) 
                        ? (offsetCoeffLow * (temperature - baselineTemp))
                        : (offsetCoeffHigh * (temperature - baselineTemp));
  
    // Calculate adjusted gas voltage, subtracting zero offset
    float adjustedGasVoltage = (VgasVoltage - VrefVoltage) - zeroShift - zeroOffset;
  
    // Calculate SO2 concentration in ppm
    float SO2_concentration = adjustedGasVoltage / tempAdjustedSensitivity;

    // Calculate molecular concentration from ppm
    float molecularConcentration = VgasPPM * (molecularWeightSO2 / molarVolume);
    float molecularConcentration2 = VgasPPM * 214.4; 
  
    // Print the results
    Serial.print("Temperature (°C): "); Serial.println(temperature);
    Serial.print("Temperature-Adjusted Sensitivity (V/ppm): "); Serial.println(tempAdjustedSensitivity);
    Serial.print("Zero Shift (ppm): "); Serial.println(zeroShift);
    Serial.print("SO2 Concentration (ppm): "); Serial.println(SO2_concentration);
    Serial.print("Vref voltage: "); Serial.println(VrefVoltage, 6);
    Serial.print("Vgas voltage: "); Serial.println(VgasVoltage, 6);
    Serial.print("Vgas value: "); Serial.println(Vgas, 6);
    Serial.print("Vgas PPM: "); Serial.println(VgasPPM, 6);
    Serial.print("Vgas microgram: "); Serial.println(molecularConcentration, 6);
    Serial.print("Vgas microgram2: "); Serial.println(molecularConcentration2, 6);
    
    return SO2_concentration;
  }

// Helper function to convert Vtemp voltage to °C (adjust based on calibration)
float getTemperature(float VtempVoltage) {
    float tempCoefficient = 100.0;  // Example conversion factor (mV/°C); adjust as per sensor calibration
    return (VtempVoltage * 1000) / tempCoefficient; // Convert to °C
}