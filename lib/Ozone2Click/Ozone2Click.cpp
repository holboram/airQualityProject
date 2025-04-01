#include "Ozone2Click.h"

#define VREF 5.0          // MCP3551 reference voltage (5V)
#define MAX_ADC_VALUE 8388607.0  // 2^23 - 1 for 24-bit signed

Ozone2Click::Ozone2Click(uint8_t csPin) {
    _csPin = csPin;
}

void Ozone2Click::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
}

long Ozone2Click::readRaw() {
    long value = readADCInternal();
    return value;
}

float Ozone2Click::rawToVoltage(long raw) {
    return (raw * VREF) / MAX_ADC_VALUE;
}

float Ozone2Click::voltageToResistance(float voltage) {
    const float RL = 10.0;  // Load resistor (kΩ)
    if (voltage == 0.0) return -1.0; // avoid division by zero
    return (VREF - voltage) / voltage * RL;
}

long Ozone2Click::readADCInternal() {
    byte msb, midb, lsb;
    long value;
    int attempts = 20;

    while (attempts-- > 0) {
        digitalWrite(_csPin, LOW);
        delayMicroseconds(2);

        msb = SPI.transfer(0x00);
        midb = SPI.transfer(0x00);
        lsb = SPI.transfer(0x00);

        digitalWrite(_csPin, HIGH);

        // Check if MSB's bit 7 is 0 (data ready)
        if ((msb & 0x80) == 0) {
            value = ((long)(msb & 0x7F) << 16) | ((long)midb << 8) | lsb;

            // Sign extension for 24-bit to 32-bit
            if (msb & 0x40) {
                value |= 0xFF800000; // Set upper bits if negative
            }

            return value;
        }

        delay(10);  // Retry after delay
    }

    // Timeout or invalid reading
    Serial.println("⚠️ MCP3551 not ready after multiple attempts");
    return -1;
}