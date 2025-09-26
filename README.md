## Self-Balancing Robot with BLE Control

### Table of Contents
1. [Overview](#overview)  
2. [Architecture](#architecture)  
3. [Key Code & Implementation](#key-code--implementation)  
   - [Angle Estimation](#angle-estimation)  
   - [PID Control](#pid-control)  
   - [Motor Control](#motor-control)  
   - [Bluetooth Low Energy (BLE)](#bluetooth-low-energy-ble)  
   - [Displays & User Interface](#displays--user-interface)  
   - [Power Monitoring & Safety](#power-monitoring--safety)  
4. [Verification & Results](#verification--results)

---

## Overview

This project implements a two-wheeled **self-balancing robot** built on the **Arduino Nano BLE Sense** platform.  
The robot uses the onboard **BMI270 IMU** to continuously measure its tilt angle, applies a **complementary filter** to fuse accelerometer and gyroscope data, and computes a **PID control effort** each loop to stabilize the platform by driving two DC motors.  

The robot supports **BLE remote control** through a custom Flutter app (FORWARD, BACKWARD, LEFT, RIGHT, RESET INTEGRAL) and provides real-time telemetry through a **TFT display** (status graphics) and **16×2 LCD dashboard** (angle, RPM, power, speed, uptime). A **Hall-effect current sensor** monitors power usage, and motors are disabled if the tilt exceeds ±15° for safety.

---

## Architecture

- **Sensor Subsystem:** Reads gyroscope and accelerometer data from BMI270; angle is estimated using a complementary filter.  
- **Control Subsystem:** Runs a PID loop with tuned gains (Kp = 18, Ki = 100, Kd = 1.3) and anti-windup clamping.  
- **Actuation Subsystem:** Dual DRV8833 drivers receive PWM commands for bi-directional motor control.  
- **Bluetooth Subsystem:** BLE service (`BLE-DEVICE`) receives plain-text commands and updates motion flags in real time.  
- **UI Subsystem:**  
  - **TFT:** Displays connection state, battery level, and a smile/frown face based on balance state.  
  - **LCD:** Cycles through angle, power, speed, uptime, and RPM using a debounced button.  
- **Power Monitoring:** TMCS1108 current sensor measures real-time current draw and calculates power consumption.  

This modular design allowed independent development and testing of each subsystem before integration.

---

## Key Code & Implementation

### Angle Estimation

Complementary filter fuses gyro integration and accelerometer absolute angle for a fast, low-drift tilt estimate used in PID:  

```C  
filteredAngle = 0.98 * (filteredAngle + gyroX * deltaT) + 0.02 * accelAngle;  
```

---

### PID Control
```C
float computePID(float error, float deltaT) {
  integral        += error * deltaT;
  integral         = constrain(integral, -30.0, 30.0);
  float derivative = (error - prevError) / deltaT;
  float pidOutput  = Kp * error + Ki * integral + Kd * derivative;
  pidOutput        = constrain(pidOutput, -255.0, 255.0);
  prevError        = error;
  return pidOutput;
}
```
The PID controller computes control effort from tilt error, clamps integral term to prevent windup, and limits output to ±255 (PWM range). Gains were experimentally tuned to achieve fast recovery and minimal overshoot.

---

### Motor Control

Motor direction is set from the sign of the PID output, with PWM magnitude from its absolute value. Turning is performed by scaling one motor’s speed:  

```  
analogWrite(AIN2, motorSpeed * (1 + 0.35 * right));  
```

This enables smooth differential steering without wheel encoders.

---

### Bluetooth Low Energy (BLE)

BLE characteristic accepts commands like **FORWARD**, **BACKWARD**, **LEFT**, **RIGHT**, and **RESET INTEGRAL**.  
Each command updates motion flags and adjusts `desiredAngle`, allowing controlled forward/backward motion or turns while maintaining balance.

---

### Displays & User Interface

- **TFT Display:** Shows a smile when within ±10° of upright, frown otherwise. Includes battery and BLE connection indicators.  
- **LCD Dashboard:** Displays one metric per page (angle, power, speed, uptime, RPM). Button input is debounced to ensure smooth cycling without blocking control.

---

### Power Monitoring & Safety

Current is read via ADC from the TMCS1108, converted to amps, and multiplied by estimated motor voltage to compute power. A low-battery flag toggles the TFT battery icon when repeated balance failures occur.  
Motors automatically cut off when `|angle| > 15°`, preventing uncontrolled movement.

---

## Verification & Results

- **Balance Recovery:** Robot maintains upright position and recovers from disturbances up to ±15°.  
- **Mobility:** Smooth forward/backward travel and controlled turning with no loss of balance.  
- **BLE Responsiveness:** Commands processed with < 50 ms latency.  
- **Telemetry Accuracy:** Angle within ±0.5° of reference; RPM within 5 % of estimated value.  
- **Safety:** Motor cutoff reliably triggered beyond tilt threshold; battery warnings displayed correctly.  

All functional requirements were satisfied, and the system operated reliably during extended runtime tests.

---
