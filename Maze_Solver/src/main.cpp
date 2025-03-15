// Pins for ultrasonic sensors
#define trigPinFront 3     // Front ultrasonic sensor Trig pin
#define echoPinFront 8     // Front ultrasonic sensor Echo pin
#define trigPinLeft A2   // Left ultrasonic sensor Trig pin
#define echoPinLeft A3   // Left ultrasonic sensor Echo pin
#define trigPinRight A0  // Right ultrasonic sensor Trig pin
#define echoPinRight A1  // Right ultrasonic sensor Echo pin

// Pin definitions for motors
#define leftMotorPWM 9     // PWM pin for speed control (left motor)
#define rightMotorPWM 10   // PWM pin for speed control (right motor)
#define leftMotorIN1 6     // IN1 pin for left motor (direction control)
#define leftMotorIN2 7     // IN2 pin for left motor (direction control)
#define rightMotorIN3 4    // IN3 pin for right motor (direction control)
#define rightMotorIN4 5    // IN4 pin for right motor (direction control)

// Pin for IR sensor
#define irSensorPin A5     // IR sensor pin for white surface detection

// Threshold distance to consider a wall
#define WALL_DISTANCE 15

// PID control constants
#define Kp 0.6
#define Ki 0.0
#define Kd 0.09

float integral = 0;
float previous_error = 0;
const int irthreshold = 500; // Threshold value for white surface detection


void setup() {
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorIN3, OUTPUT);
  pinMode(rightMotorIN4, OUTPUT);

  pinMode(irSensorPin, INPUT);

  Serial.begin(9600);
}
// Function to calculate distance using the ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  float duration, distance;

  // Clear the trigPin by setting it LOW for 2 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse on the trigPin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the pulse duration in microseconds
  duration = pulseIn(echoPin, HIGH, 10000);

  // Calculate the distance (duration * speed of sound (0.034 cm/us) / 2)
  distance = duration * 0.034 / 2;
  // Serial.println(duration);Serial.println("-------DURATIONnnnnn---------");
  if (duration == 0)
    distance = 100;

  return distance;  // Return distance in cm
}
//----------------------------------SMOOTH DISTANCE-------------------------------------------
float getSmoothedDistance(int trigPin, int echoPin) {
    int numSamples = 5;
    float totalDistance = 0;

    for (int i = 0; i < numSamples; i++) {
        totalDistance += getDistance(trigPin, echoPin);
        delay(10); // Small delay between readings
    }
    // Serial.print(totalDistance / numSamples);Serial.print("------------SMOOTH DISTANCE--------------");
    return totalDistance / numSamples;
}
void moveForward(int speedleft, int speedright) {
  // Set both motors to move forward
  analogWrite(leftMotorPWM, speedleft);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotorIN1, LOW);  // Logic for forward motion
  digitalWrite(leftMotorIN2, HIGH);
  digitalWrite(rightMotorIN3, LOW);
  digitalWrite(rightMotorIN4, HIGH);
}
//------------------------------WALL FOLLOW-----------------------------------------
void wall_follow(long left_distance, long right_distance) {
  right_distance = right_distance + 0.5;
  if (left_distance > 20) left_distance = 20;
  if (right_distance > 20) right_distance = 20;
  float error = left_distance - right_distance;
  if (left_distance > 15 && right_distance < 15) error = 2 * (5 - right_distance);
  if (left_distance < 15 && right_distance > 15) error = 2 * (left_distance - 5);

  static float pre_error = error;
  float d = error - pre_error;
  pre_error = error;
  int base_speed = 150;
  float pd = error * 0.4 + d * 0.05;
  moveForward(base_speed - pd , base_speed + pd );
  // Serial.println(pd);
}
void stopMotors() {
  // Set both motors to stop
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}
//---------------------------------STABILIZER---------------------
void stabilizeWithPID() {
  // Stabilize the robot using PID logic before starting the next step
  while (true) {
    long leftDistance = getDistance(trigPinLeft, echoPinLeft);
    long rightDistance = getDistance(trigPinRight, echoPinRight);
    long frontDistance = getDistance(trigPinFront, echoPinFront);

    // Ensure front distance is safe
    if (frontDistance < 10) {
      stopMotors();
      // Serial.println("Obstacle detected during stabilization!");
      return;
    }

    wall_follow(leftDistance, rightDistance);

    // Break condition: When the PID error is small enough, consider the robot stabilized
    if (abs(leftDistance - rightDistance) < 1) { // Adjust threshold as needed
      stopMotors();
      delay(500); // Optional stabilization delay
      break;
    }

    delay(10); // Prevent excessive loop speed
  }
}
//-------------------------------------MOVE 1 TILE-----------------------------------------
// Motor control functions
void moveOneTile() {
  unsigned long stepDuration = 1000;  // Duration for one tile movement in milliseconds
  unsigned long startTime = millis();

  // Ensure distances are stabilized before starting
  // stabilizeWithPID();

  // Move forward for one step while applying PID corrections
  while (millis() - startTime < stepDuration) {
    // Read distances from ultrasonic sensors
    long leftDistance = getDistance(trigPinLeft, echoPinLeft);
    long rightDistance = getDistance(trigPinRight, echoPinRight);

    // Apply PID corrections while moving forward
    wall_follow(leftDistance, rightDistance);

    // Optional: Check for obstacles during movement
    long frontDistance = getDistance(trigPinFront, echoPinFront);
    // Serial.print("frontDistance: ");
    // Serial.println(frontDistance);
    if (frontDistance < 10) { // Stop if an obstacle is detected
      stopMotors();
      return; // Exit function if obstacle is detected
    }

    delay(10); // Delay to prevent excessive loop speed
  }

  // Stop the motors after moving one step
  stopMotors();
  delay(500); // Short delay for stability

  // Ensure distances are stabilized again before the next step
  // stabilizeWithPID();
}
//-------------------------------RIGHT LEFT ACCURATE TURN----------------------------------------
void turnRightUntilAccurateWithBalance() {
  while (true) {
    // Serial.println("WORKING");
    float leftDistance = getDistance(trigPinLeft, echoPinLeft);

    // Check if the front distance is close to 18 cm (allowing a small tolerance)
    if (leftDistance > 15) {
      stopMotors();
      delay(200);

      // Add an additional delay for balance after reaching 18 cm
      analogWrite(leftMotorPWM, 160);   // Continue turning right
      analogWrite(rightMotorPWM, 160);

      digitalWrite(leftMotorIN1, LOW);
      digitalWrite(leftMotorIN2, HIGH);

      digitalWrite(rightMotorIN3, HIGH);
      digitalWrite(rightMotorIN4, LOW);

      delay(200);  // Balance delay (adjust this value as needed)
      stopMotors();
      // Serial.println("STOPPED");
      break;
    }

    // Continue turning right
    analogWrite(leftMotorPWM, 160);   // Left motor moves forward
    analogWrite(rightMotorPWM, 160);  // Right motor moves backward

    digitalWrite(leftMotorIN1, LOW);
    digitalWrite(leftMotorIN2, HIGH);

    digitalWrite(rightMotorIN3, HIGH);
    digitalWrite(rightMotorIN4, LOW);

    delay(50); // Add a short delay for stability and to allow accurate distance readings
  }
}

void turnLeftUntilAccurateWithBalance() {
  while (true) {
    float rightDistance = getDistance(trigPinRight, echoPinRight);
    // Serial.println(rightDistance);

    // Check if the front distance is close to 18 cm (allowing a small tolerance)
    if (rightDistance > 16) {
      stopMotors();
      delay(200);

      // Add an additional delay for balance after reaching 18 cm
      analogWrite(leftMotorPWM, 160);   // Continue turning left
      analogWrite(rightMotorPWM, 160);

      digitalWrite(leftMotorIN1, HIGH);
      digitalWrite(leftMotorIN2, LOW);

      digitalWrite(rightMotorIN3, LOW);
      digitalWrite(rightMotorIN4, HIGH);

      delay(200);  // Balance delay (adjust this value as needed)
      stopMotors();
      // Serial.println("stopped");
      break;
    }

    // Continue turning left
    analogWrite(leftMotorPWM, 160);   // Left motor moves backward
    analogWrite(rightMotorPWM, 160);  // Right motor moves forward

    digitalWrite(leftMotorIN1, HIGH);
    digitalWrite(leftMotorIN2, LOW);

    digitalWrite(rightMotorIN3, LOW);
    digitalWrite(rightMotorIN4, HIGH);

    delay(50); // Add a short delay for stability and to allow accurate distance readings
  }
}
void turn180AccurateWithBalance() {
  while (true) {
    float frontDistance = getDistance(trigPinFront, echoPinFront);
    // Serial.println(frontDistance);

    // Check if the front distance is close to 18 cm (allowing a small tolerance)
    if (frontDistance > 30) {
      stopMotors();
      delay(200);

      // Add an additional delay for balance after reaching 18 cm
      analogWrite(leftMotorPWM, 160);   // Continue turning left
      analogWrite(rightMotorPWM, 160);

      digitalWrite(leftMotorIN1, HIGH);
      digitalWrite(leftMotorIN2, LOW);

      digitalWrite(rightMotorIN3, LOW);
      digitalWrite(rightMotorIN4, HIGH);

      delay(120);  // Balance delay (adjust this value as needed)
      stopMotors();
      // Serial.println("stopped");
      break;
    }

    // Continue turning left
    analogWrite(leftMotorPWM, 160);   // Left motor moves backward
    analogWrite(rightMotorPWM, 160);  // Right motor moves forward

    digitalWrite(leftMotorIN1, HIGH);
    digitalWrite(leftMotorIN2, LOW);

    digitalWrite(rightMotorIN3, LOW);
    digitalWrite(rightMotorIN4, HIGH);

    delay(50); // Add a short delay for stability and to allow accurate distance readings
  }
}


//-------------------------------------MAIN--------------------------------------
void loop() {

  int irValue = analogRead(irSensorPin); // Read the IR sensor value
  // Serial.print("IrvALUE:--");Serial.print(irValue);
  if (irValue < irthreshold) {
    Serial.println("White paper detected! Stopping motors.");
    stopMotors(); // Stop the micromouse when white paper is detected
    while (true); // Halt the program indefinitely
  }

  delay(10);  // Minimize false detection due to echo
  float frontDistance = getDistance(trigPinFront, echoPinFront);
  delay(10);
  float leftDistance = getDistance(trigPinLeft, echoPinLeft);
  delay(10);
  float rightDistance = getDistance(trigPinRight, echoPinRight);

  // Print the distances to the Serial Monitor for debugging
  Serial.print("L: ");
  Serial.print(leftDistance);
  Serial.print("\tF: ");
  Serial.print(frontDistance);
  Serial.print("\tR: ");
  Serial.println(rightDistance);

    if (rightDistance > 15) {
        turnRightUntilAccurateWithBalance();
        delay(100);
        moveOneTile();
      } else if (leftDistance > 15) {
        turnLeftUntilAccurateWithBalance();
        delay(100);
        moveOneTile();
      } else if (frontDistance > 10) {
      // wall_follow(leftDistance, rightDistance);
      moveOneTile();  // Adjust alignment
    } else {
        // No gaps, perform a 180-degree turn
        turn180AccurateWithBalance();
        delay(100);
        moveOneTile();
      }
  
  delay(100);  // Short delay between actions

}
