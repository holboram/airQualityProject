#ifndef OZONE2CLICK_H
#define OZONE2CLICK_H

#include <Arduino.h>
#include <SPI.h>

class Ozone2Click {
public:
    // Constructor
    Ozone2Click(uint8_t csPin);

    // Initializes SPI and sets CS pin
    void begin();

    // Reads raw 24-bit ADC value from MCP3551 (returns -1 if not ready)
    long readRaw();

    // Converts raw ADC reading to voltage
    float rawToVoltage(long raw);

    // Converts voltage to sensor resistance RS (kΩ)
    float voltageToResistance(float voltage);

private:
    uint8_t _csPin;

    // Helper to read 3 bytes from the MCP3551 via SPI
    long readADCInternal();
};

#endif