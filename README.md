# SmartNav: An Autonomous Arduino Rover

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

An autonomous, line-following robotic vehicle with integrated path boundary detection, built on the Arduino UNO platform.

---

## Overview

This repository contains the firmware for the **SmartNav Rover**, a project designed to demonstrate a practical application of embedded systems for autonomous navigation. The system operates by measuring reflected infrared light to differentiate between a black track and a high-contrast white surface, showcasing a robust and efficient method for path-following.

---

## Core Functionality

- **IR Reflection-Based Navigation:** The rover's entire logic is driven by two downward-facing IR sensors. The system is based on a simple principle:
    - **Black Surfaces Absorb IR Light** (low reflection).
    - **White Surfaces Reflect IR Light** (high reflection).
  The rover is programmed to continuously seek the low-reflection path of the black line.

- **Path Boundary and Obstacle Detection:** The same IR sensors that guide the rover also act as its boundary detectors. If a sensor leaves the black line and detects the white surface, it triggers a corrective steering maneuver. If both sensors detect white, it indicates the end of the path or an obstacle, causing the rover to stop.

- **Dual-Mode Visual Status Indicator:** A custom two-LED system offers clear, real-time feedback on the rover's operational state:
  - **Green LED (Active Mode):** A solid green light indicates the vehicle is successfully tracking the black line.
  - **Yellow LED (Standby/Corrective Mode):** A blinking yellow light signifies that the vehicle is powered but stationary, either upon startup or after losing the path.

- **Low-Friction Pivoting:** A ball caster serves as the third wheel, ensuring stability while allowing for smooth, efficient turns with minimal friction.

---

## Hardware Specifications

| Component         | Item                                     | Function                               |
| ----------------- | ---------------------------------------- | -------------------------------------- |
| **Microcontroller** | Arduino UNO R3                           | Central processing unit for all logic. |
| **Motor Driver** | L298N Motor Driver Module                | Manages power and direction for motors.  |
| **Chassis** | 2WD Robotic Car Chassis & DC Geared Motors | Provides the structural frame and drive. |
| **Sensing Array** | 2 x IR Infrared Sensors                  | Measures reflected IR light to find the path. |
| **Status Display** | 1 x Green LED, 1 x Yellow LED            | Provides visual operational feedback.  |
| **Stability** | 1 x Metal Ball Caster Wheel              | Enables smooth, stable pivoting.       |
| **Power Source** | 7.4V Li-ion Battery Pack or 9V Battery   | Powers the entire system.              |

---

## Navigation and Control Algorithm

The firmware employs a differential steering algorithm based entirely on the binary state of the two IR sensors. The rover's primary directive is to keep both sensors over the black line.

1.  **On the Path (All Good):** As long as both sensors detect the black line (low IR reflection), the rover moves straight ahead.

2.  **Steering Correction (Drifting):**
    -   If the **Right Sensor** moves onto the white surface (detects high reflection), it signifies the rover has drifted too far left. The firmware corrects this by slowing or stopping the left motor, causing the rover to pivot right and re-center itself on the line.
    -   If the **Left Sensor** moves onto the white surface, the opposite occurs. The right motor is slowed or stopped, causing a left pivot to regain the path.

3.  **Path Lost / Halt Condition:**
    -   If **both sensors** detect the white surface simultaneously, the firmware interprets this as the end of the path. The rover immediately halts all motor activity to prevent it from driving off the designated course.

---

## Setup and Operation

#### 1. Hardware Assembly
Connect all components (sensors, motor driver, LEDs) to the Arduino UNO according to the pin definitions specified in the header of the `.ino` firmware file.

#### 2. Firmware Deployment
1.  Open the `.ino` sketch file in the Arduino IDE.
2.  Navigate to `Tools > Board` and select `"Arduino Uno"`.
3.  Navigate to `Tools > Port` and select the appropriate COM port for your device.
4.  Click the **Upload** button to flash the firmware.

#### 3. System Test
1.  Power the system using the connected battery pack.
2.  **Crucially, place the rover on a high-contrast surface**, such as a track made of black electrical tape on a white sheet of paper. This ensures optimal sensor performance.
3.  The rover will begin tracking the line. You can test the halt condition by lifting the rover off the line or letting it reach the end of the track.
4.  For debugging, open the **Serial Monitor** at a baud rate of `9600` to view real-time sensor data and status messages.

---

## Gallery

| Front View                                                                                                                              | Back View                                                                                                                             |
| :-------------------------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------: |
| ![Front View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3635.PNG) | ![Back View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3637.PNG) |
