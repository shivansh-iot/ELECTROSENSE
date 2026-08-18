# ELECTROSENSE ⚡

**Real-time capacitor health monitoring system using Edge AI on ESP32**

ELECTROSENSE is an embedded machine learning system that detects capacitor faults (healthy / faulty / no-capacitor) in real-time using current-sensing hardware and an on-device ML model — with a live web dashboard, OLED display, and buzzer alerts.

---

## 🎯 Overview

Capacitor degradation is a common cause of circuit failure, but manual testing is slow and often reactive. ELECTROSENSE automates this by continuously monitoring a capacitor's electrical behavior (voltage, current, power, and rate of change) and classifying its health using a machine learning model trained and deployed directly on the ESP32 — no cloud dependency, fully edge-based inference.

## ✨ Key Features

- **Real-time classification** of capacitor state: `healthy`, `faulty`, `no_capacitor`
- **95.2% model accuracy** across all three classes
- **On-device ML inference** using Edge Impulse — no internet required for classification
- **Live web dashboard** with real-time charts (voltage, current, power) via WebSocket
- **OLED display** for instant local status readout
- **Buzzer alert system** for immediate fault notification
- **FreeRTOS multi-tasking** — sensor reading, ML inference, dashboard updates, and alerts all run concurrently without blocking each other
- **Automated discharge circuit** for consistent, repeatable measurements between tests

## 🏗️ System Architecture

```
Capacitor → INA219 (current/voltage sensing) → ESP32
                                                    │
                        ┌───────────────────────────┼───────────────────────────┐
                        │                            │                            │
                  Discharge Circuit          Edge Impulse ML Model         FreeRTOS Tasks
                    (BC547 transistor)          (on-device inference)      (parallel execution)
                                                        │
                        ┌───────────────────────────────┼───────────────────────────┐
                        │                                │                            │
                  WebSocket Dashboard              OLED Display                  Buzzer Alert
                  (Chart.js, live data)          (local status)              (fault notification)
```

## 🛠️ Tech Stack

| Layer | Technology |
|---|---|
| Microcontroller | ESP32 |
| Sensing | INA219 (current/voltage sensor) |
| RTOS | FreeRTOS (multi-task architecture) |
| ML Pipeline | Edge Impulse |
| Dashboard | WebSocket + Chart.js |
| Display | SSD1306 OLED (128x64, I2C) |
| Firmware Language | C/C++ (Arduino framework) |
| Data Tooling | Python (serial logging, dataset cleaning) |

## 📊 Model Performance

- **Accuracy:** 95.2% across 3 classes (healthy, faulty, no_capacitor)
- **Sampling rate:** ~30Hz (33ms intervals)
- **Inference window:** 700ms
- **Features used:** Voltage, Current, Power, ΔVoltage (4 features/sample)

## ⚙️ How It Works

1. **Discharge phase** — Capacitor is discharged to a known baseline (`DISCHARGE_TARGET_V = 0.02V`) via a BC547 transistor circuit for consistent test conditions.
2. **Sensing phase** — INA219 continuously measures voltage, current, and power as the capacitor charges. A voltage-rise threshold (`RISE_THRESHOLD = 0.10V`) triggers the capture window.
3. **Inference phase** — Captured data is buffered and passed to the on-device Edge Impulse model, which classifies the capacitor's state.
4. **Output phase** — Result is simultaneously pushed to the live dashboard (WebSocket), shown on the OLED, and triggers the buzzer if a fault is detected.

All phases run as independent FreeRTOS tasks, so sensing, inference, and output never block one another.

## 📁 Project Structure

```
ELECTROSENSE/
├── firmware/           # ESP32 C/C++ source code
├── ml_model/           # Edge Impulse exported model files
├── dashboard/          # WebSocket + Chart.js web dashboard
├── data_tools/         # Python scripts for serial logging & dataset prep
├── docs/               # Circuit diagrams, images
└── README.md
```

## 🚀 Getting Started

```bash
# Clone the repository
git clone https://github.com/<your-username>/ELECTROSENSE.git

# Flash firmware to ESP32 (via Arduino IDE / PlatformIO)
# Open firmware/ and upload to your ESP32 board

# Run the dashboard
# Connect to the ESP32's WebSocket server and open dashboard/index.html
```

### Hardware Required
- ESP32 Dev Board
- INA219 Current/Voltage Sensor
- SSD1306 OLED Display (128x64, I2C)
- BC547 NPN Transistor (discharge circuit)
- Active Buzzer (5V)
- Test capacitors (various values/conditions)

## 🎥 Demo

[Watch the demo video](YOUR_YOUTUBE_LINK_HERE)

## 🔮 Future Improvements

- Expand dataset for more capacitor types/values
- Add data logging to persistent storage (SD card / cloud sync)
- Explore additional fault categories (e.g., leakage current classification)

## 👤 Author

**Shivansh-x**
Final-year B.Tech EEE | Embedded Systems & Edge AI enthusiast
[GitHub](https://github.com/<your-username>) 

---

*Built as an end-to-end exploration of embedded ML — from raw sensor data to a working real-time fault detection system.*
