//
// Created by Markus Möller on 26.05.2025.
//

#ifndef MPU6050MONITOR_H
#define MPU6050MONITOR_H

#include "mpu6050.h"

class MPU6050Monitor {
public:
    // Constructor takes a reference to an MPU6050 sensor object
    explicit MPU6050Monitor(MPU6050& sensor);

    // Enum for output formats
    enum class OutputFormat {
        DETAILED,
        CSV,
        JSON
    };

    // Enum for monitoring modes
    enum class MonitorMode {
        VIBRATION,
        MOTION,
        TERRAIN,
        ALL
    };

    // Prints the sensor data to the Serial console
    static void printSensorData(const MPU6050::SensorData &data, bool compact = false);

    // Monitors vibration data from the MPU6050 sensor
    void monitorVibration(uint16_t samples, uint16_t delay_ms, float vib_mag) const;
private:
    MPU6050& mpu; // Reference to the MPU6050 sensor object
    bool monitoring = false; // Flag to indicate if monitoring is active
};

#endif //MPU6050MONITOR_H
