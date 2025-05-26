//
// Created by Markus Möller on 26.05.2025.
//

#include "mpu6050monitor.h"
#include "mpu6050.h"

MPU6050Monitor::MPU6050Monitor(MPU6050& sensor) : mpu(sensor) {}

/**
 * @brief Prints the sensor data to the Serial console.
 *
 * Formats and outputs the accelerometer, gyroscope, and temperature data
 *
 * @param data
 * @param compact
 */
void MPU6050Monitor::printSensorData(const MPU6050::SensorData &data, const bool compact) {
    if (compact) {
        Serial.print(data.accelX, 2); Serial.print(",");
        Serial.print(data.accelY, 2); Serial.print(",");
        Serial.print(data.accelZ, 2); Serial.print(",");
        Serial.print(data.gyroX, 2); Serial.print(",");
        Serial.print(data.gyroY, 2); Serial.print(",");
        Serial.print(data.gyroZ, 2); Serial.print(",");
        Serial.println(data.temperature, 2);
        return;
    }

    Serial.print("Accel: (");
    Serial.print(data.accelX, 2);
    Serial.print(", ");
    Serial.print(data.accelY, 2);
    Serial.print(", ");
    Serial.print(data.accelZ, 2);
    Serial.print(") g, Gyro: (");
    Serial.print(data.gyroX, 2);
    Serial.print(", ");
    Serial.print(data.gyroY, 2);
    Serial.print(", ");
    Serial.print(data.gyroZ, 2);
    Serial.println(") deg/s, Temp: " + String(data.temperature, 2) + " °C");
}

/**
 * @brief Monitors vibration data from the MPU6050 sensor.
 *
 * Collects vibration data over a specified number of samples and prints it to the Serial console.
 *
 * @param samples Number of samples to collect (default is 100).
 * @param delay_ms Delay between samples in milliseconds (default is 50).
 */
void MPU6050Monitor::monitorVibration(const uint16_t samples = 100, const uint16_t delay_ms = 50, float) const {
    Serial.println("Starting vibration monitoring...");
    Serial.println("Time,Magnitude");

    for (uint16_t i = 0; i < samples; i++) {
        const unsigned long time = millis();
        const float vib = mpu.getVibrationMagnitude();
        Serial.print(time);
        Serial.print(",");
        Serial.println(vib, 4);
        delay(delay_ms);
    }
}
