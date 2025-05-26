# r4-wifi

## Project Overview

The r4-wifi project is an embedded system application built for Arduino-compatible hardware that integrates WiFi connectivity with sensor data collection and analysis, particularly focusing on MPU6050 accelerometer/gyroscope sensor integration.

## Features

- **WiFi Connectivity**: Configure and connect to WiFi networks
- **MPU6050 Sensor Integration**: 
  - Raw data reading (accelerometer, gyroscope, temperature)
  - Motion and vibration analysis
  - Terrain type detection
  - Speed tracking and estimation
  - Vibration sampling and analysis
- **Configurable Settings**: General configuration and WiFi credentials management

## Project Structure

```
platformio.ini           # PlatformIO configuration file
README.md               # This readme file
cfg/
  general_cfg.h         # General configuration header
  wifi.h                # WiFi configuration (credentials)
  wifi.h.example        # Example WiFi configuration template
include/
  wlan.h                # Wireless LAN interface header
lib/
  # External libraries
src/
  main.cpp              # Main application entry point
  wlan.cpp              # Wireless LAN implementation
  sensors/
    MPU6050/
      mpu6050.h         # MPU6050 sensor class definition
      mpu6050.cpp       # MPU6050 sensor implementation
      mpu6050monitor.h  # MPU6050 monitoring interface
      mpu6050monitor.cpp # MPU6050 monitoring implementation
test/
  test_mpu6050.cpp      # Test suite for MPU6050 functionality
```

## Getting Started

### Prerequisites

- PlatformIO IDE or CLI
- Compatible hardware with WiFi and I2C capabilities
- MPU6050 sensor connected to I2C bus

### Installation

1. Clone the repository
   ```
   git clone https://github.com/yourusername/r4-wifi.git
   ```

2. Configure your WiFi credentials
   ```
   cp cfg/wifi.h.example cfg/wifi.h
   ```
   Then edit `cfg/wifi.h` with your WiFi network credentials.

3. Open the project in PlatformIO IDE or build using PlatformIO CLI
   ```
   pio run
   ```

4. Upload to your device
   ```
   pio run --target upload
   ```

## MPU6050 Sensor Usage

The project includes a comprehensive MPU6050 sensor interface that provides the following features:

### Basic Sensor Operations

```cpp
MPU6050 mpu;

void setup() {
  if (mpu.begin()) {
    Serial.println("MPU6050 initialized successfully");
    // Optional: Configure sensor settings
    mpu.setGyroRange(0);    // 0: ±250°/s, 1: ±500°/s, 2: ±1000°/s, 3: ±2000°/s
    mpu.setAccelRange(0);   // 0: ±2g, 1: ±4g, 2: ±8g, 3: ±16g
    mpu.setSampleRate(0);   // Set sample rate divider
    mpu.setLowPassFilter(3); // Set digital low-pass filter
    
    // Calibrate to remove gravity component
    mpu.calibrateOffsets();
  }
}

void loop() {
  // Read raw sensor data
  MPU6050::SensorData data = mpu.readData();
  
  Serial.print("Accel X: "); Serial.print(data.accelX);
  Serial.print(" Y: "); Serial.print(data.accelY);
  Serial.print(" Z: "); Serial.println(data.accelZ);
  
  Serial.print("Gyro X: "); Serial.print(data.gyroX);
  Serial.print(" Y: "); Serial.print(data.gyroY);
  Serial.print(" Z: "); Serial.println(data.gyroZ);
  
  Serial.print("Temp: "); Serial.println(data.temperature);
  
  delay(100);
}
```

### Vibration Analysis

```cpp
// Get instantaneous vibration magnitude (after gravity compensation)
float vibration = mpu.getVibrationMagnitude();

// Get root mean square of vibration over multiple samples
float vibrationRMS = mpu.getVibrationRMS();

// Detect terrain type based on vibration patterns
uint8_t terrain = mpu.detectTerrainType(10); // Sample 10 readings
switch(terrain) {
  case 0: Serial.println("Smooth surface"); break;
  case 1: Serial.println("Mild roughness"); break;
  case 2: Serial.println("Moderate roughness"); break;
  case 3: Serial.println("Severe roughness"); break;
}

// Check if device is moving (vibration above threshold)
if (mpu.isMoving(0.1)) {
  Serial.println("Device is moving");
}
```

### Sampling and Analysis

```cpp
// Start collecting vibration samples
mpu.beginSampling(100); // Collect 100 samples

// Later, check if sampling is complete and analyze results
if (mpu.isSamplingComplete()) {
  float maxVib = mpu.getMaxVibration();
  float avgVib = mpu.getAvgVibration();
  
  Serial.print("Max vibration: "); Serial.println(maxVib);
  Serial.print("Avg vibration: "); Serial.println(avgVib);
}
```

### Speed Tracking

```cpp
// Initialize speed tracking
mpu.beginSpeedTracking();

// In your loop
void loop() {
  // Update velocity calculations
  mpu.updateVelocity();
  
  // Get current speed
  float speed = mpu.getSpeed();
  Serial.print("Current speed: "); Serial.println(speed);
  
  delay(50);
  
  // Reset velocity when device stops moving
  if (!mpu.isMoving(0.05)) {
    mpu.resetVelocity();
  }
}
```

## License

[Insert your license information here]

## Contributors

[Insert contributor information here]

## Acknowledgments

- The MPU6050 community for documentation and insights
- Arduino and PlatformIO communities for development tools and support

