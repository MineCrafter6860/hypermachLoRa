# TEKNOFEST HYİ LoRa Communication System (E22-900T30D)

This repository contains the **Transmitter** and **Receiver** firmware for a high-altitude telemetry system designed for the **TEKNOFEST HYİ Protocol**. The system utilizes the **Raspberry Pi Pico (RP2040)** and **Ebyte E22-900T30D LoRa** modules to transmit 78-byte binary telemetry packets.

---

## 🛰️ Hardware Overview

### **Transmitter (Flight Computer)**
*   **MCU:** Raspberry Pi Pico
*   **LoRa:** Ebyte E22-900T30D (30dBm / 1W)
*   **Sensors:** 
    *   **BME280:** Altitude (pad-referenced), Temperature, Pressure, and Humidity.
    *   **MPU6050:** 3-axis Accelerometer (g) and Gyroscope (°/s).
*   **Protocol:** TEKNOFEST HYİ 78-byte binary packet.

### **Receiver (Ground Station)**
*   **MCU:** Raspberry Pi Pico
*   **LoRa:** Ebyte E22-900T30D
*   **Output:** Re-broadcasts validated HYİ packets via UART2 (GP8/GP9) at 19200 baud for ground station software.

---

## 🔧 Pin Configuration

| Component | Pico Pin (TX) | Pico Pin (RX) | Description |
| :--- | :--- | :--- | :--- |
| **LoRa TX/RX** | GP0 / GP1 | GP0 / GP1 | Serial1 Communication |
| **LoRa M0** | GP2 | GP2 | Mode Selection Pin 0 |
| **LoRa M1** | GP3 | GP3 | Mode Selection Pin 1 |
| **LoRa AUX** | GP15 | GP15 | Activity / Status Monitoring |
| **I2C SDA/SCL** | GP4 / GP5 | N/A | Sensor Bus (TX only) |
| **HYİ Out** | N/A | GP8 (TX) | Ground Station Serial Out |
| **Onboard LED** | GP25 | GP25 | Mirrors AUX status |

---

## 📊 Protocol Details (78-Byte Packet)

The system follows a strict binary structure for compatibility with competition standards:

1.  **Header:** `0xFF 0xFF 0x54 0x52`
2.  **Metadata:** Team ID and Packet Counter
3.  **Telemetry Data:** IEEE 754 Float32 (Little-Endian)
    *   Relative Altitude
    *   GPS Data (Latitude, Longitude, Altitude)
    *   MPU6050 Gyro (X, Y, Z)
    *   MPU6050 Accel (X, Y, Z)
    *   Tilt Angle (0–180°)
4.  **Checksum:** Sum of bytes 4 through 74 modulo 256
5.  **Footer:** `0x0D 0x0A`

---

## 🚀 Getting Started

### 1. Software Requirements
*   **Arduino IDE** with the **arduino-pico** (Earle Philhower) core.
*   **Libraries:**
    *   `Adafruit BME280`
    *   `Adafruit Unified Sensor`
    *   `Wire` & `Math` (Built-in)

### 2. Configuration
You can easily modify the LoRa parameters in the `LoRaSettings` struct in both files:
*   **Channel:** Default is **18**.
*   **Address:** **0x0001** for Transmitter, **0x0002** for Receiver.
*   **Air Data Rate:** Set to **2.4kbps** for maximum range.
*   **Transmission Mode:** **Fixed Mode** is enabled for targeted point-to-point communication.

### 3. Deployment
1.  Upload the **Transmitter** code to the flight computer Pico.
2.  Upload the **Receiver** code to the ground station Pico.
3.  Open the Serial Monitor at **115200 baud** to view real-time telemetry and debug info.
