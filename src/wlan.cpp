//
// Created by Markus Möller on 25.05.2025.
//
#include "../include/wlan.h"


void printEncryptionType(const int thisType) {
    switch (thisType) {
        case ENC_TYPE_NONE:
            Serial.println("Open");
            break;
        case ENC_TYPE_WEP:
            Serial.println("WEP");
            break;
        case ENC_TYPE_WPA:
            Serial.println("WPA");
            break;
        case ENC_TYPE_WPA2:
            Serial.println("WPA2");
            break;
        case ENC_TYPE_WPA2_ENTERPRISE:
            Serial.println("WPA2_ENTERPRISE");
            break;
        case ENC_TYPE_WPA3:
            Serial.println("WPA3");
            break;
        default:
            Serial.println("Unknown encryption type");
    }
}

void listNetworks() {
    const int8_t numSsid = WiFi.scanNetworks();

    if (numSsid == -1) {
        Serial.println("No networks found");
        return;
    }

    Serial.print("Found networks: ");
    Serial.print(numSsid);
    Serial.println();

    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
        Serial.print(thisNet);
        Serial.print(": ");
        Serial.print(WiFi.SSID(thisNet));
        Serial.print(" Signal: ");
        Serial.print(WiFi.RSSI(thisNet));
        Serial.print(" dBm ");
        Serial.print("Encryption: ");
        printEncryptionType(WiFi.encryptionType(thisNet));
    }
}

void printMacAddress(const byte mac[]) {
    for (int i = 0; i < 6; i++) {
        if (i > 0) {
            Serial.print(":");
        }
        if (mac[i] < 16) {
            Serial.print("0");
        }
        Serial.print(mac[i], HEX);
    }
    Serial.println();
}

void scanNetworks() {
    byte mac[6];
    Serial.println("Scanning available networks...");
    listNetworks();
    WiFi.macAddress(mac);
    Serial.println();
    Serial.print("MAC address: ");
    printMacAddress(mac);
    delay(10000);
}

void checkWifi() {
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("No WiFi module found");
    } else {
        Serial.println("WiFi module found");
    }

    if (const String fv = CWifi::firmwareVersion(); fv < WIFI_FIRMWARE_LATEST_VERSION) {
        Serial.println("Please upgrade the firmware");
    } else {
        Serial.println("Firmware version is up to date");
    }
}