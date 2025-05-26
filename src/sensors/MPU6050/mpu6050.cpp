//
// Created by Markus Möller on 25.05.2025.
//

#include "mpu6050.h"

// MPU6050 class implementation
/**
 * @brief Initializes the MPU6050 sensor.
 *
 * Sets up the I2C communication, wakes up the sensor from sleep mode,
 * and configures default settings for the accelerometer and gyroscope.
 *
 * @return
 */
bool MPU6050::begin() {
    Wire.begin();

    // Wake up the MPU6050 (clear sleep mode)
    writeRegister(MPU6050_PWR_MGMT_1, 0x00);

    // Check if the device is responsive
    if (const uint8_t whoAmI = readRegister(MPU6050_WHO_AM_I); whoAmI != MPU6050_ADDR) {
        return false;
    }

    // Configure default settings
    writeRegister(MPU6050_CONFIG, 0x03);      // Set digital low pass filter
    writeRegister(MPU6050_GYRO_CONFIG, 0x00); // Set gyro to ±250 degrees/s range
    writeRegister(MPU6050_ACCEL_CONFIG, 0x00); // Set accel to ±2g range
    writeRegister(MPU6050_SMPRT_DIV, 0x04);   // Set sample rate to 200Hz

    prevTime = millis();
    return true;
}

/**
 * @brief Reads sensor data from the MPU6050.
 *
 * Reads raw accelerometer, gyroscope, and temperature data,
 * converts them to physical values, and returns them in a SensorData structure.
 *
 * @return SensorData containing acceleration, gyroscope, and temperature values.
 */
MPU6050::SensorData MPU6050::readData() const {
    SensorData data{};
    uint8_t buffer[14];
    readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);

    // Convert raw data to physical values
    const auto rawAccelX = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    const auto rawAccelY = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    const auto rawAccelZ = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

    const auto rawTemp = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);

    const auto rawGyroX = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    const auto rawGyroY = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    const auto rawGyroZ = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);

    // Convert to g's and degrees/second
    data.accelX = static_cast<float>(rawAccelX) / accelScale;
    data.accelY = static_cast<float>(rawAccelY) / accelScale;
    data.accelZ = static_cast<float>(rawAccelZ) / accelScale;

    data.temperature = (static_cast<float>(rawTemp) / 340.0f) + 36.53f; // From datasheet formula

    data.gyroX = static_cast<float>(rawGyroX) / gyroScale;
    data.gyroY = static_cast<float>(rawGyroY) / gyroScale;
    data.gyroZ = static_cast<float>(rawGyroZ) / gyroScale;

    return data;
}

/**
 * @brief Sets the gyroscope range.
 *
 * Configures the gyroscope sensitivity range.
 *
 * @param range Gyroscope range (0: ±250, 1: ±500, 2: ±1000, 3: ±2000 degrees/second).
 */
void MPU6050::setGyroRange(uint8_t range) {
    if (range > 3) range = 3;
    const uint8_t value = range << 3;
    writeRegister(MPU6050_GYRO_CONFIG, value);

    // Update the scale factor
    switch (range) {
        case 0: gyroScale = 131.0; break;   // ±250 degrees/s
        case 1: gyroScale = 65.5; break;    // ±500 degrees/s
        case 2: gyroScale = 32.8; break;    // ±1000 degrees/s
        case 3: gyroScale = 16.4; break;    // ±2000 degrees/s
        default: ;
    }
}

/**
 * @brief Sets the accelerometer range.
 *
 * Configures the accelerometer sensitivity range.
 *
 * @param range Accelerometer range (0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g).
 */
void MPU6050::setAccelRange(uint8_t range) {
    if (range > 3) range = 3;
    const uint8_t value = range << 3;
    writeRegister(MPU6050_ACCEL_CONFIG, value);

    // Update the scale factor
    switch (range) {
        case 0: accelScale = 16384.0; break;  // ±2g
        case 1: accelScale = 8192.0; break;   // ±4g
        case 2: accelScale = 4096.0; break;   // ±8g
        case 3: accelScale = 2048.0; break;   // ±16g
        default: ;
    }
}

/**
 * @brief Sets the sample rate for the MPU6050.
 *
 * Configures the sample rate divider to control how often the sensor data is sampled.
 *
 * @param rate Sample rate divider (0-255).
 */
void MPU6050::setSampleRate(uint8_t rate) {
    writeRegister(MPU6050_SMPRT_DIV, rate);
}

/**
 * @brief Sets the low-pass filter for the MPU6050.
 *
 * Configures the digital low-pass filter to reduce noise in the sensor data.
 *
 * @param filter Low-pass filter setting (0-6).
 */
void MPU6050::setLowPassFilter(uint8_t filter) {
    if (filter > 6) filter = 6;
    writeRegister(MPU6050_CONFIG, filter);
}

/**
 * @brief Calculates the magnitude of vibration from accelerometer data.
 *
 * Reads the accelerometer data, removes the gravity component, and calculates the vector magnitude.
 *
 * @return The magnitude of vibration in g's.
 */
float MPU6050::getVibrationMagnitude() const {
    const SensorData data = readData();

    // Remove gravity component from acceleration
    const float accelX = data.accelX - gravityX;
    const float accelY = data.accelY - gravityY;
    const float accelZ = data.accelZ - gravityZ;

    return calculateVectorMagnitude(accelX, accelY, accelZ);
}

/**
 * @brief Calculates the RMS value of vibration.
 *
 * Samples the vibration magnitude multiple times to compute the RMS value.
 *
 * @return The RMS value of vibration in g's.
 */
float MPU6050::getVibrationRMS() const {
    // Calculate RMS over several samples
    constexpr int numSamples = 10;
    float sumSquares = 0.0;

    for (int i = 0; i < numSamples; i++) {
        const float mag = getVibrationMagnitude();
        sumSquares += mag * mag;
        delay(5); // Small delay between readings
    }

    return sqrt(sumSquares / numSamples);
}

/**
 * @brief Calibrates the sensor offsets.
 *
 * Averages several readings when the vehicle is stationary to set the gravity vector components.
 *
 * @return True if calibration was successful.
 */
bool MPU6050::calibrateOffsets() {
    // Calibrate by averaging several readings when stationary
    constexpr int numSamples = 100;
    float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

    for (int i = 0; i < numSamples; i++) {
        const SensorData data = readData();
        sumX += data.accelX;
        sumY += data.accelY;
        sumZ += data.accelZ;
        delay(5);
    }

    // Set gravity components
    gravityX = sumX / numSamples;
    gravityY = sumY / numSamples;
    gravityZ = sumZ / numSamples;

    return true;
}

/**
 * @brief Detects the terrain type based on vibration levels.
 *
 * Analyzes the average vibration magnitude over a number of samples to classify terrain.
 *
 * @param samples Number of samples to average (default is 20).
 * @return Terrain type (0: smooth, 1: mild roughness, 2: moderate roughness, 3: severe roughness).
 */
uint8_t MPU6050::detectTerrainType(const uint8_t samples) const {
    float vibrationLevel = 0.0;

    // Collect samples and calculate average vibration
    for (int i = 0; i < samples; i++) {
        vibrationLevel += getVibrationMagnitude();
        delay(10);
    }
    vibrationLevel /= static_cast<float>(samples);

    // Simple terrain classification based on vibration levels
    if (vibrationLevel < 0.1) {
        return 0; // Smooth surface
    }
    if (vibrationLevel < 0.3) {
        return 1; // Mild roughness
    }
    if (vibrationLevel < 0.6) {
        return 2; // Moderate roughness
    }
    return 3; // Severe roughness
}

/**
 * @brief Checks if the vehicle is moving based on vibration magnitude.
 *
 * Compares the current vibration magnitude against a threshold to determine movement.
 *
 * @param threshold Threshold value for movement detection (default is 0.1 g).
 * @return True if the vehicle is moving, false otherwise.
 */
bool MPU6050::isMoving(const float threshold) const {
    const float vibration = getVibrationMagnitude();
    return vibration > threshold;
}

/**
 * @brief Begins sampling for vibration analysis.
 *
 * Initializes the sampling process, clearing the buffer and setting the sample count.
 *
 * @param sampleCount Number of samples to collect (default is 50, max is 100).
 */
void MPU6050::beginSampling(uint16_t sampleCount) {
    if (sampleCount > MAX_SAMPLES) {
        sampleCount = MAX_SAMPLES;
    }

    sampling = true;
    sampleIndex = 0;
    totalSamples = sampleCount;

    // Clear the buffer
    for (float & i : vibrationBuffer) {
        i = 0.0;
    }
}

/**
 * @brief Checks if the sampling process is complete.
 *
 * Continues sampling until the specified number of samples is reached.
 *
 * @return True if sampling is complete, false otherwise.
 */
bool MPU6050::isSamplingComplete() {
    if (!sampling) {
        return true;
    }

    if (sampleIndex < totalSamples) {
        vibrationBuffer[sampleIndex++] = getVibrationMagnitude();
        return false;
    }

    sampling = false;
    return true;
}

/**
 * @brief Gets the maximum vibration recorded during sampling.
 *
 * Scans through the collected samples to find the maximum vibration value.
 *
 * @return The maximum vibration value in g's.
 */
float MPU6050::getMaxVibration() const {
    float maxVib = 0.0;
    for (uint16_t i = 0; i < totalSamples; i++) {
        if (vibrationBuffer[i] > maxVib) {
            maxVib = vibrationBuffer[i];
        }
    }
    return maxVib;
}

/**
 * @brief Gets the average vibration recorded during sampling.
 *
 * Calculates the average of all collected samples.
 *
 * @return The average vibration value in g's.
 */
float MPU6050::getAvgVibration() const {
    float sum = 0.0;
    for (uint16_t i = 0; i < totalSamples; i++) {
        sum += vibrationBuffer[i];
    }
    return sum / static_cast<float>(totalSamples);
}

/**
 * @brief Begins tracking speed using accelerometer data.
 *
 * Initializes the motion data structure and calibrates the offsets to remove gravity.
 *
 */
void MPU6050::beginSpeedTracking() {
    velocityTracking = true;
    motion.velocityX = 0.0;
    motion.velocityY = 0.0;
    motion.velocityZ = 0.0;
    motion.speed = 0.0;
    motion.lastUpdate = millis();

    // Calibrate to get gravity vector
    calibrateOffsets();
}

/**
 * @brief Updates the velocity based on accelerometer data.
 *
 * Integrates acceleration data over time to calculate velocity,
 * removing the gravity component and applying a deadband filter.
 *
 */
void MPU6050::updateVelocity() {
    if (!velocityTracking) {
        return;
    }

    const SensorData data = readData();
    const unsigned long currentTime = millis();

    if (const float deltaTime = static_cast<float>((currentTime - motion.lastUpdate)) / 1000.0f; deltaTime > 0) {
        // Remove gravity components
        float accelX = data.accelX - gravityX;
        float accelY = data.accelY - gravityY;
        float accelZ = data.accelZ - gravityZ;

        // Apply deadband filter to reduce noise and drift
        constexpr float deadband = 0.02; // g
        if (fabs(accelX) < deadband) accelX = 0;
        if (fabs(accelY) < deadband) accelY = 0;
        if (fabs(accelZ) < deadband) accelZ = 0;

        // Convert acceleration from g to m/s²
        accelX *= 9.81;
        accelY *= 9.81;
        accelZ *= 9.81;

        // Integrate acceleration to get velocity
        motion.velocityX += accelX * deltaTime;
        motion.velocityY += accelY * deltaTime;
        motion.velocityZ += accelZ * deltaTime;

        // Calculate overall speed
        motion.speed = calculateVectorMagnitude(motion.velocityX, motion.velocityY, motion.velocityZ);

        // Update timestamp
        motion.lastUpdate = currentTime;
    }
}

/**
 * @brief Gets the current speed of the vehicle.
 *
 * Returns the calculated speed based on the integrated velocity data.
 *
 * @return The speed in m/s.
 */
float MPU6050::getSpeed() const {
    return motion.speed;
}

/**
 * @brief Resets the velocity tracking data.
 *
 * Clears the velocity components and speed, and resets the last update timestamp.
 *
 */
void MPU6050::resetVelocity() {
    motion.velocityX = 0.0;
    motion.velocityY = 0.0;
    motion.velocityZ = 0.0;
    motion.speed = 0.0;
    motion.lastUpdate = millis();
}

/**
 * @brief Calibrates the sensor offsets at reset.
 *
 * Resets the velocity tracking and recalibrates the offsets to remove gravity.
 * This should be called when the vehicle is stationary at startup.
 *
 */
void MPU6050::calibrateAtReset() {
    resetVelocity();
    calibrateOffsets();
}

// Private helper methods
/** @brief Writes a value to a specific register on the MPU6050.
 *
 * @param reg The register address to write to.
 * @param value The value to write to the register.
 */
void MPU6050::writeRegister(const uint8_t reg, const uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

/**
 * @brief Reads a single byte from a specific register on the MPU6050.
 *
 * @param reg The register address to read from (0x00-0xFF).
 * @return The value read from the register.
 */
uint8_t MPU6050::readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1, true);
    return Wire.read();
}

/**
 * @brief Reads multiple bytes from a specific register on the MPU6050.
 *
 * @param reg The register address to read from (0x00-0xFF).
 * @param buffer Pointer to the buffer where the read data will be stored.
 * @param count Number of bytes to read (typically 1-32, limited by I2C buffer).
 */
void MPU6050::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, count, true);

    for (uint8_t i = 0; i < count && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}

/**
 * @brief Calculates the magnitude of a 3D vector.
 *
 * Computes the Euclidean norm (magnitude) of a vector given its x, y, and z components.
 *
 * @param x The x component of the vector.
 * @param y The y component of the vector.
 * @param z The z component of the vector.
 * @return The magnitude of the vector.
 */
float MPU6050::calculateVectorMagnitude(const float x, const float y, const float z) {
    return sqrt(x*x + y*y + z*z);
}


