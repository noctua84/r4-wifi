//
// Created by Markus Möller on 24.05.2025.
//

#ifndef WLAN_H
#define WLAN_H

#include <WiFiS3.h>
#include <Arduino.h>

void printEncryptionType(int thisType);
void listNetworks();
void printMacAddress(const byte mac[]);
void scanNetworks();
void checkWifi();

#endif //WLAN_H
