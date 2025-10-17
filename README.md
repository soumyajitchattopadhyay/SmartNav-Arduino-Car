🚗 SmartNav Arduino Car

An autonomous line-following and obstacle-avoiding robot, custom-built with an Arduino UNO.

Suggestion: You can replace the link above with a GIF of your car in action!

About This Project

This repository contains the firmware and documentation for the SmartNav Arduino Car, an autonomous vehicle built from the ground up. This project demonstrates a practical application of embedded systems for real-time path tracking and collision avoidance, showcasing skills in microcontroller programming, sensor integration, and robotics.

✨ Key Features

Path Navigation: Utilizes two independent IR sensors underneath the chassis to accurately track and follow a black line on a contrasting surface.

Obstacle Avoidance: Equipped with a forward-facing HC-SR04 ultrasonic sensor to detect objects in its path and execute an avoidance maneuver.

Visual Status Indicator: A unique dual-LED system provides real-time feedback on the car's operational state:

🟢 Green LED: Solid light indicates the car is actively moving and following the path.

🟡 Yellow LED: Blinks intermittently when the car is stopped (in neutral) but still powered on.

Stable & Smooth Maneuverability: A ball caster wheel at the front provides a stable third point of contact, allowing for smooth, low-friction turns and pivots.

🛠️ Hardware Components

Component

Item

Purpose

Microcontroller

Arduino UNO R3

The "brain" of the robot

Motor Driver

L298N Motor Driver Module

Controls the speed and direction of motors

Chassis

2WD Robotic Car Chassis with DC Geared Motors

The body and drivetrain of the robot

Path Sensing

2 x IR Infrared Line Follower Sensors

To detect the black line for navigation

Obstacle Sensing

1 x HC-SR04 Ultrasonic Sensor

To detect objects in front of the car

Status Display

1 x Green LED, 1 x Yellow LED

Visual feedback on the car's state

Stability

1 x Metal Ball Caster Wheel

Provides a smooth pivot point

Power

7.4V Li-ion Battery Pack or 9V Battery

Powers the entire system

🧠 How It Works

The robot's logic operates on a simple but effective priority system: safety first.

Check for Obstacles: The loop() function continuously pings the ultrasonic sensor to measure the distance to any object in front. This is the highest priority action.

Avoid or Follow:

If an obstacle is detected within a predefined threshold (e.g., 15 cm), the car immediately stops, reverses, turns to navigate around the object, and then attempts to resume its task. The yellow "neutral" LED blinks during this maneuver.

If the path is clear, the robot reads the values from the two bottom-facing IR sensors. By comparing the readings, it determines its position relative to the black line and adjusts the speed of the left and right wheels to steer itself correctly. The green "moving" LED stays lit.

🚀 Getting Started

1. Assemble the Hardware

Connect all sensors, the L298N motor driver, and the LEDs to the Arduino UNO as defined by the pin constants in the .ino sketch file.

2. Upload the Firmware

Open the firmware sketch in the Arduino IDE.

Go to Tools > Board and select "Arduino Uno".

Go to Tools > Port and select the correct COM port for your device.

Click the Upload button.

3. Power On and Test

Power the robot with your battery pack.

Place the car on a surface with a black line path.

Watch it go! You can test the obstacle avoidance by placing your hand in front of the ultrasonic sensor.

To see debug messages, open the Serial Monitor at a baud rate of 9600.

### 📸 Gallery

Here are some pictures of the final build.

| Front View | Back View |
| :---: | :---: |
| ![Front View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3635.PNG) | ![Back View of the SmartNav Car](https://raw.githubusercontent.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/main/images/IMG_3637.PNG) |

[Front View](https://github.com/soumyajitchattopadhyay/SmartNav-Arduino-Car/blob/main/images/IMG_3635.PNG)

Top-Down View



