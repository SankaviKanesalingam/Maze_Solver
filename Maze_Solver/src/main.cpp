#include <Arduino.h>
// Pin definitions for motors
#define leftMotorPWM 9     // PWM pin for speed control (left motor)
#define rightMotorPWM 10   // PWM pin for speed control (right motor)
#define leftMotorIN1 6     // IN1 pin for left motor (direction control)
#define leftMotorIN2 7     // IN2 pin for left motor (direction control)
#define rightMotorIN3 4    // IN3 pin for right motor (direction control) # in3,4 have been changed to get correct direction
#define rightMotorIN4 5    // IN4 pin for right motor (direction control)

// Pin definitions for ultrasonic sensors
#define trigPinFront 3     // Front ultrasonic sensor Trig pin
#define echoPinFront 8     // Front ultrasonic sensor Echo pin
#define trigPinLeft 12     // Left ultrasonic sensor Trig pin
#define echoPinLeft 11     // Left ultrasonic sensor Echo pin
#define trigPinRight 13    // Right ultrasonic sensor Trig pin
#define echoPinRight 2     // Right ultrasonic sensor Echo pin

// Function to calculate distance using the ultrasonic sensor
long getDistance(int trigPin, int echoPin) {

  long duration, distance;

  // Clear the trigPin by setting it LOW for 2 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse on the trigPin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the pulseIn on the echoPin
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance
  distance = (duration * 0.034) / 2;

  return distance; // Return distance in cm
}

// // Motor control functions
// void moveForward(int speed) {
//   // Set both motors to move forward
//   analogWrite(leftMotorPWM, speed);
//   analogWrite(rightMotorPWM, speed);

//   // Logic for forward motion
//   digitalWrite(leftMotorIN1, LOW);
//   digitalWrite(leftMotorIN2, HIGH);
//   digitalWrite(rightMotorIN3, LOW);
//   digitalWrite(rightMotorIN4, HIGH);
// }
// Motor control functions
void moveForward(int speedleft, int speedright) {
  // Set both motors to move forward
  analogWrite(leftMotorPWM, speedleft);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotorIN1, LOW);  // Logic for forward motion
  digitalWrite(leftMotorIN2, HIGH);
  digitalWrite(rightMotorIN3, LOW);
  digitalWrite(rightMotorIN4, HIGH);
}


void stopMotors() {
  // Set both motors to stop
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}

void turnRight() {
  // Turn right by adjusting the motor speeds for a 90-degree turn
  analogWrite(leftMotorPWM, 100); // Left motor moves forward at normal speed
  analogWrite(rightMotorPWM, 100); // Right motor moves backward at normal speed

  // Left motor moves forward
  digitalWrite(leftMotorIN1, LOW);
  digitalWrite(leftMotorIN2, HIGH);

  // Right motor moves backward
  digitalWrite(rightMotorIN3, HIGH);
  digitalWrite(rightMotorIN4, LOW);
  
  delay(170); // Adjust delay for a 90-degree turn (experiment with this value)

  // After turning, stop the right motor
  stopMotors();
  Serial.println("stopinggggggggggggd...");
}

void sturnRight() {
  // Turn right by adjusting the motor speeds for a 90-degree turn
  analogWrite(leftMotorPWM, 100); // Left motor moves forward at normal speed
  analogWrite(rightMotorPWM, 100); // Right motor moves backward at normal speed

  // Left motor moves forward
  digitalWrite(leftMotorIN1, LOW);
  digitalWrite(leftMotorIN2, HIGH);

  // Right motor moves backward
  digitalWrite(rightMotorIN3, HIGH);
  digitalWrite(rightMotorIN4, LOW);
   Serial.println("-----------------------------------------sRRRRRRRR");
  delay(55); // Adjust delay for a 90-degree turn (experiment with this value)

  // After turning, stop the right motor
  stopMotors();
  Serial.println("stopinggggggggggggd...");
}


void turnLeft() {
  // Turn right by adjusting the motor speeds for a 90-degree turn
  analogWrite(leftMotorPWM, 100); // Left motor moves forward at normal speed
  analogWrite(rightMotorPWM, 100); // Right motor moves backward at normal speed

  // Left motor moves forward
  digitalWrite(leftMotorIN2, LOW);
  digitalWrite(leftMotorIN1, HIGH);

  // Right motor moves backward
  digitalWrite(rightMotorIN4, HIGH);
  digitalWrite(rightMotorIN3, LOW);
  Serial.println("---------------------------------------------11111111");
  delay(170); // Adjust delay for a 90-degree turn (experiment with this value)

  // After turning, stop the right motor
  stopMotors();
  Serial.println("stopinggggggggggggd...");
}

void sturnLeft() {
  // Turn right by adjusting the motor speeds for a 90-degree turn
  analogWrite(leftMotorPWM, 100); // Left motor moves forward at normal speed
  analogWrite(rightMotorPWM, 100); // Right motor moves backward at normal speed

  // Left motor moves forward
  digitalWrite(leftMotorIN2, LOW);
  digitalWrite(leftMotorIN1, HIGH);

  // Right motor moves backward
  digitalWrite(rightMotorIN4, HIGH);
  digitalWrite(rightMotorIN3, LOW);
  Serial.println("----------------------------------------sLLLLLLLLLLLLLL");
  delay(55); // Adjust delay for a 90-degree turn (experiment with this value)

  // After turning, stop the right motor
  stopMotors();
  Serial.println("stopinggggggggggggd...");
}

void setup() {
  // Initialize motor control pins as outputs
  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorIN3, OUTPUT);
  pinMode(rightMotorIN4, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  // Begin serial communication for debugging and printing sensor values
  Serial.begin(9600);
  Serial.println("Ultrasonic Sensor Readings:");
}
//---------------------------------------Wall follow------------------------------------------------------

void wall_follow(long left_distance, long right_distance) {
  if (left_distance > 20) left_distance = 20;
  if (right_distance > 20) right_distance = 20;
  float error = left_distance - right_distance;
  if (left_distance > 15 && right_distance < 15) error =  2*(5 - right_distance);
  if (left_distance < 15 && right_distance > 15) error =  2*(left_distance - 5);

  static float pre_error = error;
  float d = error - pre_error;
  pre_error = error;

  float pd = error * 0.5 + d * 0.05;
  moveForward(56 - pd, 56 + pd);
  Serial.println(pd);
}

//---------------------------------------------------------------------------------------

void loop() {
  // Get distance readings from all three ultrasonic sensors
  float frontDistance = getDistance(trigPinFront, echoPinFront);
  float leftDistance = getDistance(trigPinLeft, echoPinLeft);
  float rightDistance = getDistance(trigPinRight, echoPinRight);

  // Print the distances to the Serial Monitor for debugging
  Serial.print("F: ");
  Serial.print(frontDistance);
  Serial.print(" cm, L: ");
  Serial.print(leftDistance);
  Serial.print(" cm, R: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Move forward if there is no obstacle in front
  if ( 10< leftDistance - rightDistance &&  leftDistance - rightDistance< 60) {
    Serial.println("slightly turn leftttttt-------------.");
    // moveForward(60); // Move forward at slow speed
    sturnLeft();
    wall_follow(leftDistance,rightDistance);
    delay(1000);
  }

  else if ( 8<  rightDistance - leftDistance &&  rightDistance - leftDistance< 60) {
    Serial.println("slightly turn rightttttt-------------.");
    // moveForward(60); // Move forward at slow speed
    sturnRight();
    wall_follow(leftDistance,rightDistance);
    delay(1000);
  }




  else if (frontDistance > 15 && frontDistance < 100 ) {
    Serial.println("Moving forward...");
    // moveForward(60); // Move forward at slow speed
    wall_follow(leftDistance,rightDistance);
  }
  
   else if (frontDistance < 15 &&  leftDistance > 15 &&  leftDistance < 100) {
    // Stop and turn right if an obstacle is detected in front
    stopMotors();
    delay(1000);
    Serial.println("Turning left...");
    turnLeft();
    delay(1000);
    // moveForward(60); // Optional: Continue moving forward after turning
    // wall_follow(leftDistance,rightDistance)
    
  }
  
  else if (frontDistance < 15 &&  rightDistance > 15 &&  rightDistance < 100){
    // Stop and turn right if an obstacle is detected in front
    stopMotors();
    delay(1000);
    Serial.println("Turning right...");
    turnRight();
    delay(1000);
    // moveForward(60); // Optional: Continue moving forward after turning
  }
  else if (frontDistance >100 &&  rightDistance <10  &&  leftDistance >15 ){
    // Stop and turn right if an obstacle is detected in front
    stopMotors();
    delay(1000);
    Serial.println("Turning left...");
    turnLeft();
    delay(1000);
    // moveForward(60); // Optional: Continue moving forward after turning
  }
  else if (frontDistance >100 &&  leftDistance <10  &&  rightDistance >15 ){
    // Stop and turn right if an obstacle is detected in front
    stopMotors();
    delay(1000);
    Serial.println("Turning right...");
    turnRight();
    delay(1000);
    // moveForward(60); // Optional: Continue moving forward after turning
  }
  else if (frontDistance <15 &&  rightDistance < 15 ){
    // Stop and turn right if an obstacle is detected in front
    stopMotors();
    delay(1000);
    Serial.println("Turning left...");
    turnLeft();
    delay(1000);
    // moveForward(60); // Optional: Continue moving forward after turning
  }
  delay(500); // Short delay to allow sensor readings to update
}
