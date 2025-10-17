/**
 * @file    SmartNav_Firmware.ino
 * @author  Soumyajit Chattopadhyay (Code adapted and corrected)
 * @brief   Firmware for the SmartNav autonomous rover.
 * @details This code is specifically tailored for a 2WD Arduino robot with:
 * - 2 downward-facing IR sensors for line following.
 * - 1 forward-facing HC-SR04 ultrasonic sensor for obstacle avoidance.
 * - 1 green LED and 1 yellow LED for status indication.
 * The robot prioritizes obstacle avoidance over line following.
 *
 * @version 2.0 
 * @date    2022-06-29
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

// IR Line Follower Sensors (2-Sensor Setup)
const int LINE_LEFT_PIN = A0;
const int LINE_RIGHT_PIN = A2; // Using A2 as per the original code's right pin

// Status LEDs
const int GREEN_LED_PIN = 9;  
const int YELLOW_LED_PIN = 10; 

// --- Constants ---
const int BASE_SPEED = 150; // Base speed for the motors (0-255)
const int TURN_SPEED = 180; // Speed when making sharp turns
const int OBSTACLE_DISTANCE_CM = 20; // Stop if an obstacle is within this distance

// --- Function Prototypes ---
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
long getUltrasonicDistance();
void followLine();
void avoidObstacle();
void setStatusLEDs(bool isMoving);


void setup() {
  Serial.begin(9600);
  Serial.println("SmartNav Rover Initializing...");

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

  // Set IR line follower sensor pins as INPUT
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);
  
  // Set LED pins as OUTPUT
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);

  stopMotors(); // Ensure motors are stopped at startup
  setStatusLEDs(false); // Set LEDs to "stopped" state
  Serial.println("SmartNav Rover Ready.");
}


void loop() {
  // Priority 1: Check for obstacles.
  if (getUltrasonicDistance() < OBSTACLE_DISTANCE_CM) {
    avoidObstacle();
  } 
  // Priority 2: Follow the line.
  else {
    followLine();
  }
  delay(10); // Small delay for loop stability
}

/**
 * @brief Logic for following a line with TWO IR sensors.
 * LOW (0) = Black Line Detected
 * HIGH (1) = White Surface Detected
 */
void followLine() {
  bool leftSeesWhite = digitalRead(LINE_LEFT_PIN) == HIGH;
  bool rightSeesWhite = digitalRead(LINE_RIGHT_PIN) == HIGH;

  setStatusLEDs(true); // Green ON, Yellow OFF

  // Case 1: Both sensors see black. Go straight.
  if (!leftSeesWhite && !rightSeesWhite) {
    moveForward();
  } 
  // Case 2: Left sensor sees white, right sees black. Turn left.
  else if (leftSeesWhite && !rightSeesWhite) {
    turnLeft();
  }
  // Case 3: Right sensor sees white, left sees black. Turn right.
  else if (!leftSeesWhite && rightSeesWhite) {
    turnRight();
  }
  // Case 4: Both sensors see white. Path lost. Stop.
  else {
    stopMotors();
    setStatusLEDs(false); // Switch to "stopped" state
  }
}

/**
 * @brief Handles the obstacle avoidance maneuver.
 */
void avoidObstacle() {
  Serial.println("Obstacle Detected!");
  
  // Stop and signal "neutral" state
  stopMotors();
  setStatusLEDs(false);
  delay(300);

  // Simple avoidance: back up, then turn right.
  moveBackward();
  delay(500);
  turnRight();
  delay(700);
  
  // Stop again before handing back to the main loop.
  stopMotors();
  delay(300);
}


/**
 * @brief Manages the state of the two status LEDs.
 * @param isMoving True if the robot is actively moving, false if stopped.
 */
void setStatusLEDs(bool isMoving) {
  if (isMoving) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW);
    // Simple blink for the yellow LED when stopped
    digitalWrite(YELLOW_LED_PIN, (millis() / 500) % 2); 
  }
}

// --- Sensor & Motor Control Functions ---

long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration / 2 / 29.1; // Convert duration to cm
}

void moveForward() {
  // Left Motor Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, BASE_SPEED);
  // Right Motor Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, BASE_SPEED);
}

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

void turnLeft() {
  // Left Motor Backward (to pivot)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, TURN_SPEED);
  // Right Motor Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, TURN_SPEED);
}

void turnRight() {
  // Left Motor Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, TURN_SPEED);
  // Right Motor Backward (to pivot)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, TURN_SPEED);
}

void stopMotors() {
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}

