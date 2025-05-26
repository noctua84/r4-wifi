//
// Created by Markus Möller on 24.05.2025.
//

#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <Arduino.h>

// MPU6050 I2C address
constexpr int MPU6050_ADDR = 0x68;

// MPU6050 register addresses
constexpr int MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr int MPU6050_GYRO_XOUT_H = 0x43;
constexpr int MPU6050_PWR_MGMT_1 = 0x6B;
constexpr int MPU6050_CONFIG = 0x1A;
constexpr int MPU6050_GYRO_CONFIG = 0x1B;
constexpr int MPU6050_ACCEL_CONFIG = 0x1C;
constexpr int MPU6050_SMPRT_DIV = 0x19;
constexpr int MPU6050_WHO_AM_I = 0x75;

class MPU6050 {
public:
    struct SensorData {
        float accelX, accelY, accelZ; // in g (9.81 m/s²)
        float gyroX, gyroY, gyroZ;     // in degrees/second
        float temperature;             // in degrees Celsius
    };

    struct MotionData {
        float velocityX, velocityY, velocityZ; // in m/s
        float speed = 0.0; // Speed in m/s
        unsigned long lastUpdate = 0;
    };

    MotionData motion;

    bool begin();
    SensorData readData() const;
    void setGyroRange(uint8_t range); // 0: ±250, 1: ±500, 2: ±1000, 3: ±2000 degrees/second
    void setAccelRange(uint8_t range); // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g

    // Vibration analysis
    static void setSampleRate(uint8_t rate); // Set sample rate divider (0-255)
    static void setLowPassFilter(uint8_t filter); // Set low-pass filter (0-6)

    // Vibration monitoring
    float getVibrationMagnitude() const; // Calculate the magnitude of vibration from accelerometer data
    float getVibrationRMS() const; // Calculate the RMS value of vibration (Root Mean Square)

    // Terrain and Vehicle monitoring
    bool calibrateOffsets(); // Calibrate the sensor offsets (vehicle is stationary)
    uint8_t detectTerrainType(uint8_t samples = 20) const; // Detect terrain type based on accelerometer data
    bool isMoving(float threshold = 0.1) const; // Check if the vehicle is moving based on accelerometer data

    // Sample buffering for analysis
    void beginSampling(uint16_t sampleCount = 50);
    bool isSamplingComplete();
    float getMaxVibration() const;
    float getAvgVibration() const;

    // Motion data
    void beginSpeedTracking();
    void updateVelocity();
    float getSpeed() const;
    void resetVelocity();
    void calibrateAtReset();

private:
    float accelScale = 16384.0; // Scale factor for accelerometer (±2g)
    float gyroScale = 131.0;    // Scale factor for gyroscope (±250 degrees/second)
    unsigned long prevTime = 0;
    float gravityX = 0.0f, gravityY = 0.0f, gravityZ = 0.0f; // Gravity vector components
    bool velocityTracking = false;

    // Sample buffer for vibration analysis
    static constexpr uint16_t MAX_SAMPLES = 50;
    float vibrationBuffer[MAX_SAMPLES] = {};
    uint16_t sampleIndex = 0;
    uint16_t totalSamples = 0;
    bool sampling = false;

    // Helper methods
    static void writeRegister(uint8_t reg, uint8_t value);
    static uint8_t readRegister(uint8_t reg);
    static void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count);
    static float calculateVectorMagnitude(float x, float y, float z);
};

#endif //MPU6050_H
