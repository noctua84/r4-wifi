//
// Created by Markus Möller on 22.05.2025.
//

#ifndef GENERAL_CFG_H
#define GENERAL_CFG_H

// Serial communication settings
constexpr int SERIAL_BAUD_RATE = 9600;
constexpr int SERIAL_TIMEOUT = 1000; // in ms

// SD Card settings
constexpr int SD_CS_PIN = 10; // Chip select pin for SD card

// Sensor settings
constexpr int DHT11_PIN = 2; // Pin for DHT11 sensor


#endif //GENERAL_CFG_H
