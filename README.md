# Quadcopter Flight Controller - Arduino (C++)

This repository contains the source code for a **C++-based flight controller** designed for a custom-built quadcopter.  
The system provides **stable and responsive flight control** using sensor fusion, PID control, and multiple flight modes.  
It interfaces with an IMU, barometric pressure sensor, optical flow sensor, and PPM-based remote control receiver.  
Additional functionality includes **Bluetooth telemetry** and **on-the-fly PID tuning**.

---

## 📂 Files Overview

- **voltage_measurement.ino**  
  Implements battery voltage monitoring for real-time power management and failsafe triggers.

- **kalmanfilter.ino**  
  Provides sensor fusion using a **Kalman Filter** to estimate quadcopter orientation (roll, pitch, yaw) from IMU data.

- **PPM.ino**  
  Decodes signals from a **PPM remote control receiver**, mapping pilot stick inputs to control commands (throttle, yaw, pitch, roll).

- **PIDtest.ino**  
  Implements and tests the **dual-loop PID control system** (rate + attitude), stabilizing quadcopter flight and responding to disturbances.

---

## 🚀 Features

- **Sensor Fusion**: Combines IMU and barometer data using a Kalman Filter for accurate orientation and altitude estimation.  
- **PID Control**: Dual-loop PID ensures stability and smooth response in pitch, roll, and yaw axes.  
- **PPM Receiver Integration**: Pilot commands are mapped to control setpoints.  
- **Voltage Monitoring**: Real-time battery voltage feedback for safe operation.  
- **Bluetooth Telemetry**: Wireless debugging, telemetry streaming, and on-the-fly PID tuning.  
- **Flight Modes**: Stabilize, Acro/Rate, Altitude Hold, Position Hold (with optical flow), and Failsafe.

---

## 🛠️ Hardware Requirements

- **Arduino-compatible board** (e.g., Arduino Mega / STM32 for advanced features)  
- **IMU (Accelerometer + Gyroscope + Magnetometer)**  
- **Barometric pressure sensor** (e.g., BMP280)  
- **Optical flow sensor** (for position hold, optional)  
- **PPM remote control receiver**  
- **ESCs and Brushless DC motors**  
- **Li-Po Battery** with voltage divider for monitoring  
- **Bluetooth module (HC-05/HC-06)** for wireless telemetry  

---

## ⚡ Getting Started

1. Clone this repository or download the `.ino` files.  
2. Open the `.ino` files in **Arduino IDE**.  
3. Install required sensor libraries (IMU, BMP280, etc.).  
4. Upload to your Arduino-compatible flight controller board.  
5. Connect sensors, receiver, and motors as per circuit design.  
6. Perform **PID tuning** (manual or via Bluetooth).  
7. Test in a controlled environment before actual flights.

---

