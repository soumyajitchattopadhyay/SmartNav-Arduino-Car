Arduino Autobot Workshop Project

This repository contains the firmware for an autonomous robotic cart built on the Arduino UNO platform. The project demonstrates a hands-on implementation of an embedded system capable of real-time path tracking and obstacle avoidance, showcasing skills in robotics, microcontroller programming, and control systems.

Features

Autonomous Navigation: The robot can navigate a predefined path by following a black line on a white surface.

Obstacle Avoidance: Utilizes ultrasonic and IR proximity sensors to detect and maneuver around obstacles in its path.

Sensor Fusion: Implements a priority-based control system where obstacle avoidance overrides line-following behavior to ensure collision-free operation.

PID Control: A Proportional-Integral-Derivative (PID) control algorithm is used for smooth and adaptive line tracking, dynamically adjusting motor speeds to correct any deviation from the path.

Embedded Systems Design: The entire system is controlled by an Arduino UNO microcontroller programmed in C++.

Hardware Components

Chassis: 2WD Robotic Cart Chassis

Microcontroller: Arduino UNO R3

Motor Driver: L298N Motor Driver Module

Motors: 2 x DC Geared Motors

Obstacle Detection:

1 x HC-SR04 Ultrasonic Sensor

1 x IR Proximity/Obstacle Sensor

Line Tracking: 1 x TCRT5000 3-Channel IR Line Follower Sensor Array

Power Supply: 7.4V Li-ion Battery Pack or 9V Battery

Software & Libraries

IDE: Arduino IDE

Language: C++

Libraries: No external libraries are required. The code uses the standard Arduino core functions.

Code Structure

The main Arduino sketch (autobot_controller.ino) is organized into several key sections:

Pin Definitions & Constants:

Defines the Arduino pins connected to motors and sensors.

Sets constants for PID gains, motor speeds, and obstacle detection thresholds.

setup():

Initializes serial communication for debugging.

Configures all pins (motors, sensors) as either INPUT or OUTPUT.

Ensures the robot is stationary on startup.

loop():

This is the main control loop that runs continuously.

It first checks for obstacles using the ultrasonic and IR proximity sensors. This check is given the highest priority.

If an obstacle is detected, it calls the obstacleAvoidance() function.

If no obstacle is present, it calls the lineFollowing() function to continue tracking the path.

Core Logic Functions:

lineFollowing(): Reads data from the 3-channel IR sensor array. It calculates an error value based on the robot's position relative to the line. This error is fed into the PID algorithm to calculate a correction value, which then adjusts the left and right motor speeds to steer the robot back onto the line.

obstacleAvoidance(): A simple but effective function that stops the robot, reverses for a short duration, turns right to avoid the obstacle, and then stops again, allowing the main loop to resume control.

getUltrasonicDistance(): A helper function that triggers the HC-SR04 sensor and returns the measured distance in centimeters.

Motor Control Functions:

A set of low-level functions (moveForward, moveBackward, turnLeft, turnRight, stopMotors) provide direct control over the L298N motor driver, making the main logic cleaner and more readable.

How to Use

Assemble the Hardware: Connect all the sensors, motor driver, and motors to the Arduino UNO as defined in the autobot_controller.ino file.

Upload the Code: Open the autobot_controller.ino sketch in the Arduino IDE.

Select Board and Port: Go to Tools > Board and select "Arduino Uno". Then, go to Tools > Port and select the correct port for your device.

Compile and Upload: Click the upload button to flash the firmware to the Arduino.

Power On: Power the robot using the battery pack.

Test: Place the robot on a surface with a black line and observe its line-following and obstacle-avoidance behaviors. You can open the Serial Monitor (Tools > Serial Monitor) at a baud rate of 9600 to see debug messages.

Conclusion

This project serves as a practical demonstration of fundamental robotics concepts, including sensor integration, control algorithms, and autonomous decision-making in an embedded system.