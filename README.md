SmartNav: An Autonomous Arduino Rover

An autonomous, line-following robotic vehicle with integrated obstacle avoidance, built on the Arduino UNO platform.

Overview

This repository contains the firmware for the SmartNav Rover, a project designed to demonstrate a practical application of embedded systems for autonomous navigation. The system integrates real-time sensor data to perform path tracking and collision avoidance, showcasing fundamental skills in microcontroller programming, control logic, and robotics engineering.

Core Functionality

Line-Following Navigation: Employs two independent IR (infrared) sensors on the undercarriage to detect and track a black line on a contrasting surface. The differential drive is modulated based on sensor feedback to maintain the vehicle's course.

Ultrasonic Obstacle Avoidance: A forward-facing HC-SR04 ultrasonic sensor provides robust collision avoidance. When an obstacle is detected within a predefined safety threshold, the system overrides navigation to perform a corrective maneuver.

Dual-Mode Visual Status Indicator: A custom two-LED system offers clear, real-time feedback on the rover's operational state:

Green LED (Active Mode): A solid green light indicates the vehicle is powered on and executing its line-following logic.

Yellow LED (Standby/Neutral Mode): A blinking yellow light signifies that the vehicle is powered but stationary, either upon startup or during an obstacle avoidance maneuver.

Low-Friction Pivoting: A ball caster serves as the third wheel, ensuring stability while allowing for smooth, efficient turns with minimal friction.

Hardware Specifications

Component

Item

Function

Microcontroller

Arduino UNO R3

Central processing unit for all logic.

Motor Driver

L2B98N Motor Driver Module

Manages power and direction for motors.

Chassis

2WD Robotic Car Chassis & DC Geared Motors

Provides the structural frame and drive.

Path Sensing

2 x IR Infrared Line Follower Sensors

Detects the line for path navigation.

Obstacle Sensing

1 x HC-SR04 Ultrasonic Sensor

Measures distance to avoid collisions.

Status Display

1 x Green LED, 1 x Yellow LED

Provides visual operational feedback.

Stability

1 x Metal Ball Caster Wheel

Enables smooth, stable pivoting.

Power Source

7.4V Li-ion Battery Pack or 9V Battery

Powers the entire system.

Control Logic

The firmware operates on a priority-driven control loop where safety is paramount.

Obstacle Detection Priority: In each cycle of the main loop(), the first action is to ping the ultrasonic sensor.

State-Based Action:

If an obstacle is detected, the system enters an avoidance state. The rover halts, reverses, executes a turn to clear the object, and then returns control to the main loop. The yellow LED blinks to indicate this temporary state.

If the path is clear, the system enters a navigation state. It processes data from the dual IR sensors to calculate path deviation and makes real-time adjustments to the wheel motors to steer the vehicle correctly. The green LED remains solid.

Setup and Operation

1. Hardware Assembly

Connect all components (sensors, motor driver, LEDs) to the Arduino UNO according to the pin definitions specified in the header of the .ino firmware file.

2. Firmware Deployment

Open the .ino sketch file in the Arduino IDE.

Navigate to Tools > Board and select "Arduino Uno".

Navigate to Tools > Port and select the appropriate COM port for your device.

Click the Upload button to flash the firmware.

3. System Test

Power the system using the connected battery pack.

Place the rover on a surface with a clearly marked black line.

Test the obstacle avoidance by placing an object in the rover's path.

For debugging, open the Serial Monitor at a baud rate of 9600 to view real-time sensor data and status messages.

### 📸 Gallery

Here are some pictures of the final build.

| Front View | Back View |
| :---: | :---: |
| ![Front View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3635.PNG) | ![Back View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3637.PNG) |
