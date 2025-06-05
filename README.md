# 🛩️ RT-Sense  
## Modular Deterministic Sensor Library Development for Quadcopter Applications
![sensors](https://github.com/user-attachments/assets/dc076f71-9ab0-417a-b31e-d3f7636b1ae6)
---

High-performance and modular C drivers for the following sensors, tailored for real-time quadcopter applications:
- **BMP388** – Digital Barometric Pressure and Temperature Sensor  
- **BMI160** – 6-axis Accelerometer and Gyroscope  
- **HMC5883L** – 3-axis Digital Compass (Magnetometer)

Each driver is written in portable, dependency-free C language and is designed for easy integration with STM32 and other ARM Cortex-M MCUs using HAL or bare metal I2C/SPI interfaces.
The driver libraries for BMP388 and BMI160 are the official libraries of Bosch and ported to the STM32 platform.

## 📌 Features

### BMP388 Driver (Digital Pressure Sensor)
- IIR Filter Coefficient [x3]
- Oversampling Coefficient for Pressure Data [x8]
- Oversampling Coefficient for Temperature Data [x1]
- RMS Noise [11cm]
- ODR [50Hz]
- Drdy interrupt for deterministic application

### BMI160 Driver (6-DOF IMU Sensor)
- Accelerometer Range [+-16g]
- Accelerometer Bandwith [AVG4]
- Accelerometer ODR [1600Hz]
- Gyroscope Range [+-2000dps]
- Gyroscope Bandwith [Normal_Mode]
- Gyroscope ODR [3200Hz]
- Drdy interrupt for deterministic application

### HMC5883L Driver (3-DOF Magnetometer Sensor)
- Oversampling Coefficient [x8]
- Gain Setting [x5]
- ODR [160Hz]
- Drdy interrupt for deterministic application
---
## 🚧 Upcoming Sensor Libraries
The following sensor drivers are currently under development and will be added to this repository soon:
- **INA219A** – High-side current and power monitor. Will support voltage, current, and power calculations over I2C.
- **BNO055** – 9-DOF absolute orientation sensor with built-in sensor fusion (accelerometer, gyroscope, and magnetometer with quaternion output).
- **BMP180** – Digital pressure sensor. Lightweight driver for basic altitude estimation and weather-related applications.
- **HC-SR04** – Ultrasonic distance sensor. Planned with millisecond-accurate timing using interrupt-driven or timer-based echo pulse measurement.

