/**
 * @file autobot_controller.ino
 * @author Soumyajit Chattopadhyay
 * @brief Firmware for an autonomous robotic cart using Arduino UNO.
 * @details This code enables a robotic cart to perform line following and obstacle avoidance.
 * It uses ultrasonic and IR proximity sensors for obstacle detection and an IR array for line tracking.
 * A PID control algorithm is implemented for smooth line following. The robot prioritizes obstacle
 * avoidance over line following.
 *
 * @version 1.0
 * @date 2022-06-29
 */

// --- Pin Definitions ---

// Motor A (Right Motor)
const int ENA = 5; // Enable A (PWM Speed Control)
const int IN1 = 4; // Input 1
const int IN2 = 3; // Input 2

// Motor B (Left Motor)
const int ENB = 6; // Enable B (PWM Speed Control)
const int IN3 = 8; // Input 3
const int IN4 = 7; // Input 4

// Ultrasonic Sensor (HC-SR04)
const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

// IR Proximity Sensor (Obstacle Detection)
const int PROXIMITY_SENSOR_PIN = 2;

// IR Line Follower Sensor Array (Using 3 sensors for this example)
const int LINE_LEFT_PIN = A0;
const int LINE_CENTER_PIN = A1;
const int LINE_RIGHT_PIN = A2;

// --- Constants & Control Variables ---

// PID Controller constants for line following
double Kp = 10;   // Proportional gain
double Ki = 0.5;  // Integral gain
double Kd = 2;    // Derivative gain

// PID control variables
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;

// Motor speed settings
const int BASE_SPEED = 130; // Base speed for the motors (0-255)
const int MAX_SPEED = 200;
const int MIN_SPEED = 100;

// Obstacle detection thresholds
const int OBSTACLE_DISTANCE_CM = 20; // in centimeters

// --- Function Prototypes ---
void moveForward(int leftSpeed, int rightSpeed);
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
long getUltrasonicDistance();
void lineFollowing();
void obstacleAvoidance();

/**
 * @brief Initializes all pins, serial communication, and sensors.
 */
void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Autobot Initializing...");

  // Set motor control pins as OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set IR proximity sensor pin as INPUT
  pinMode(PROXIMITY_SENSOR_PIN, INPUT_PULLUP); // Use internal pull-up

  // Set line follower sensor pins as INPUT
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_CENTER_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);

  stopMotors(); // Ensure motors are stopped at startup
  Serial.println("Autobot Ready.");
}

/**
 * @brief Main loop to run the robot's logic.
 * It continuously checks for obstacles and follows the line.
 */
void loop() {
  // Sensor Fusion: Obstacle avoidance has higher priority than line following.
  long distance = getUltrasonicDistance();
  int proximityState = digitalRead(PROXIMITY_SENSOR_PIN);

  if (distance < OBSTACLE_DISTANCE_CM || proximityState == LOW) {
    obstacleAvoidance();
  } else {
    lineFollowing();
  }
  delay(10); // Small delay for stability
}

/**
 * @brief Controls the robot to follow a black line on a white surface.
 * Implements a PID algorithm to adjust motor speeds for smooth turning.
 */
void lineFollowing() {
  // Read line sensor values (assuming LOW means on the line)
  bool onLeft = digitalRead(LINE_LEFT_PIN) == LOW;
  bool onCenter = digitalRead(LINE_CENTER_PIN) == LOW;
  bool onRight = digitalRead(LINE_RIGHT_PIN) == LOW;

  // --- PID Error Calculation ---
  // A simple way to calculate error for a 3-sensor array
  if (onCenter && !onLeft && !onRight) {
    error = 0; // Perfectly on the line
  } else if (onLeft && !onRight) {
    error = -1; // Veering to the right
  } else if (onRight && !onLeft) {
    error = 1; // Veering to the left
  } else if (onLeft && onCenter && !onRight) {
    error = -0.5;
  } else if (onRight && onCenter && !onLeft) {
    error = 0.5;
  }
  // If all sensors are off the line, we might use lastError to decide direction.
  else if (!onLeft && !onCenter && !onRight) {
     // If we lose the line, we can stop or use last error to guess direction
     if(lastError < 0) error = -2;
     else error = 2;
  }


  // --- PID Calculation ---
  integral = integral + error;
  derivative = error - lastError;
  lastError = error;

  // The PID correction value
  double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Calculate motor speeds based on correction
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  // Constrain speeds to be within the valid range
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  moveForward(leftSpeed, rightSpeed);
}


/**
 * @brief Handles obstacle avoidance logic.
 * Stops and turns when an obstacle is detected.
 */
void obstacleAvoidance() {
  Serial.println("Obstacle Detected!");
  stopMotors();
  delay(300);

  // Simple avoidance: back up a little, then turn right.
  moveBackward();
  delay(500);
  turnRight();
  delay(700);
  stopMotors();
  delay(300);
}


/**
 * @brief Measures distance using the HC-SR04 ultrasonic sensor.
 * @return The distance in centimeters.
 */
long getUltrasonicDistance() {
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin for the duration of the sound wave travel
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (speed of sound is ~343 m/s or 29.1 us/cm)
  return duration / 2 / 29.1;
}

// --- Basic Motor Control Functions ---

/**
 * @brief Moves the robot forward with specified speeds for each motor.
 * @param leftSpeed Speed for the left motor (0-255).
 * @param rightSpeed Speed for the right motor (0-255).
 */
void moveForward(int leftSpeed, int rightSpeed) {
  // Left Motor Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, leftSpeed);

  // Right Motor Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, rightSpeed);
}

/**
 * @brief Moves the robot backward at base speed.
 */
void moveBackward() {
  // Left Motor Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, BASE_SPEED);

  // Right Motor Backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, BASE_SPEED);
}

/**
 * @brief Turns the robot left on the spot.
 */
void turnLeft() {
  // Left Motor Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, BASE_SPEED);

  // Right Motor Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, BASE_SPEED);
}

/**
 * @brief Turns the robot right on the spot.
 */
void turnRight() {
  // Left Motor Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, BASE_SPEED);

  // Right Motor Backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, BASE_SPEED);
}

/**
 * @brief Stops both motors.
 */
void stopMotors() {
  // Stop Left Motor
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  // Stop Right Motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}
