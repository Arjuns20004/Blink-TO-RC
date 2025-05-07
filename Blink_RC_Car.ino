#include <Servo.h>
#include <NewPing.h>

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12
#define EYE_SENSOR_PIN 2  // Eye blink sensor pin

#define MAX_REGULAR_MOTOR_SPEED 75
#define MAX_TURN_SPEED 60
#define DISTANCE_TO_CHECK 30

// Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;

// Variables for eye blink detection
unsigned long blinkStartTime = 0;
int blinkCount = 0;
bool motorStarted = false; // To track motor state

void setup() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(EYE_SENSOR_PIN, INPUT); // Eye blink sensor setup

  myServo.attach(SERVO_PIN);
  myServo.write(90); // Set servo to the center
  rotateMotor(0, 0); // Start with motors off
}

void loop() {
  detectEyeBlinks(); // Continuously check for blink patterns

  // If the motor is running, perform obstacle detection and avoidance
  if (motorStarted) {
    int distance = mySensor.ping_cm();

    if (distance > 0 && distance < DISTANCE_TO_CHECK) {
      // Obstacle detected
      rotateMotor(0, 0); // Stop motors
      delay(500);

      // Check both sides to find a clear path
      myServo.write(0); // Rotate servo to the right
      delay(500);
      int distanceRight = mySensor.ping_cm();

      myServo.write(180); // Rotate servo to the left
      delay(500);
      int distanceLeft = mySensor.ping_cm();

      myServo.write(90); // Center the servo
      delay(500);

      // Determine which direction to turn
      if (distanceLeft > distanceRight) {
        // Turn left
        rotateMotor(-MAX_TURN_SPEED, MAX_TURN_SPEED);
        delay(500);
      } else {
        // Turn right
        rotateMotor(MAX_TURN_SPEED, -MAX_TURN_SPEED);
        delay(500);
      }
      rotateMotor(0, 0); // Stop briefly after turning
      delay(500);
    } else {
      // No obstacle, move forward
      rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
    }
  }
}

void detectEyeBlinks() {
  int eyeSignal = digitalRead(EYE_SENSOR_PIN); // Read eye sensor signal

  if (eyeSignal == LOW) { // LOW indicates a blink
    if (blinkStartTime == 0) {
      blinkStartTime = millis(); // Start counting time
    }
    blinkCount++;
    delay(200); // Debounce delay for eye blinks
  }

  // Check if blink detection time window (e.g., 2 seconds) has ended
  if (blinkStartTime > 0 && millis() - blinkStartTime > 2000) {
    if (blinkCount == 2 && !motorStarted) {
      // Two blinks: Start the motor and move forward
      motorStarted = true;
      rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
    } else if (blinkCount == 3 && motorStarted) {
      // Three blinks: Turn left
      rotateMotor(-MAX_TURN_SPEED, MAX_TURN_SPEED);
      delay(1000); // Turn for 1 second
      rotateMotor(0, 0); // Stop after turning
    } else if (blinkCount == 4 && motorStarted) {
      // Four blinks: Turn right
      rotateMotor(MAX_TURN_SPEED, -MAX_TURN_SPEED);
      delay(1000); // Turn for 1 second
      rotateMotor(0, 0); // Stop after turning
    } else if (blinkCount == 5 && motorStarted) {
      // Five blinks: Stop the motor
      motorStarted = false;
      rotateMotor(0, 0); // Stop the motor
    }

    // Reset blink variables for the next detection cycle
    blinkCount = 0;
    blinkStartTime = 0;
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
