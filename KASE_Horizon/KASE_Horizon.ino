/*
The following program has been created by Kartik Jassal and Arav Patel. The program 
*/








//Initialize the libraries 
#include <Servo.h>
#include <ServoEasing.h>
#include <AccelStepper.h>


// Pins for DC motor control
#define motor1_IN1  2  
#define motor1_IN2  3
#define motor2_IN1  4
#define motor2_IN2  5
#define motor3_IN1  6
#define motor3_IN2  7 
#define motor4_IN1  8
#define motor4_IN2  9
#define motor5_IN1  10  
#define motor5_IN2  11
#define motor6_IN1  12
#define motor6_IN2  13


// Define servo objects for corner wheel steering
ServoEasing frontLeftServo;
ServoEasing frontRightServo;
ServoEasing rearLeftServo;
ServoEasing rearRightServo;

//Define servo object for camera tilt
ServoEasing servoCamTilt;


// Define stepper object for camera pan
AccelStepper camPanStepper(1, 46, 45);

// Analog input pins for joysticks
int forwardBackwardJoystickPin = A0;
int rightLeftJoystickPin = A1;

//Same joystick is being used for the pan and tilt function of the camera
int camTiltJoystickPin = A2; // y axis of the same joystick
int camPanJoystickPin = A3; //x axis of the same joystick


// Initial servo angles for straight steering
int servoStraightAngle = 0;

//Initial servo position
int frontLeftServoAngle = 90;
int frontRightServoAngle = 90;
int rearLeftServoAngle = 90;
int rearRightServoAngle = 90;

//Initial rover speed and turning radius
int speed = 0;
int radius = 0;

//Max limit of stepper speed
camPanStepper.setMaxSpeed(1000);

//initial camera tilt and pan values
int camTilt = 90;
int camPan = 0;



float speed1, speed2, speed3 = 0;
float speed1PWM, speed2PWM, speed3PWM = 0;
float thetaInnerFront, thetaInnerBack, thetaOuterFront, thetaOuterBack = 0;

// Wheelbase and track width dimensions for Ackermann steering
float wheelbase = .0;  // Example value in arbitrary units
float trackWidth = 8.0;  // Example value in arbitrary units

void setup() {
  // Attach servo objects to pins
  frontLeftServo.attach(22);
  frontRightServo.attach(23);
  rearLeftServo.attach(24);
  rearRightServo.attach(25);
  servoCamTilt.attach(26);

  // Initialize servos to straight steering angles
  frontLeftServo.write(90);
  frontRightServo.write(90);
  rearLeftServo.write(90);
  rearRightServo.write(90);
  servoCamTilt.write(90);


  frontLeftServo.setSpeed(550);
  frontRightServo.setSpeed(550);
  rearLeftServo.setSpeed(550);
  rearRightServo.setSpeed(550);
  servoCamTilt.setSpeed(200);


  // Set motor pins as output
  digitalWrite(motor1_IN1, LOW);   // PWM value
  digitalWrite(motor1_IN2, LOW); // Forward
  // Motor Wheel 2 - Left Middle
  digitalWrite(motor2_IN1, LOW);
  digitalWrite(motor2_IN2, LOW);
  // Motor Wheel 3 - Left Back
  digitalWrite(motor3_IN1, LOW);
  digitalWrite(motor3_IN2, LOW);
  // right side motors move in opposite direction
  // Motor Wheel 4 - Right Front
  digitalWrite(motor4_IN1, LOW);
  digitalWrite(motor4_IN2, LOW);
  // Motor Wheel 5 - Right Middle
  digitalWrite(motor5_IN1, LOW);
  digitalWrite(motor5_IN2, LOW);
  // Motor Wheel 6 - Right Back
  digitalWrite(motor6_IN1, LOW);
  digitalWrite(motor6_IN2, LOW);

  // Set joystick pins as inputs
  pinMode(forwardBackwardJoystickPin, INPUT);
  pinMode(rightLeftJoystickPin, INPUT);
  pinMode(camTiltJoystickPin, INPUT);
  pinMode(camPanJoystickPin, INPUT);
}

void loop() {
  // Read joystick values
  int forwardBackwardValue = analogRead(forwardBackwardJoystickPin);
  int rightLeftValue = analogRead(rightLeftJoystickPin);
  int tiltValue = analogRead(camTiltJoystickPin);
  int panValue = analogRead(camPanJoystickPin);

// Adjust camera tilt
  if (tiltValue < 508 ) {
    if (camTilt >= 35) {
      camTilt--;
      delay(20);
    }
  }
  if (tiltValue > 508 ) {
    if (camTilt <= 165) {
      camTilt++;
      delay(20);
    }
  }
  servoCamTilt.startEaseTo(camTilt);

// Adjust camera pan
  if (panValue > 508) {
    camPan = map(panValue, 508, 1023, 400, 0);
  }
  else if (panValue < 508 ) {
    camPan = map(panValue, 508, 0, 0, -400);
  }
  else {
    camPan = 0;
  }
  camPanStepper.setSpeed(camPan);    // Camera pan
  camPanStepper.run();

  // Convert joystick values to desired steering angle and speed
  float steeringAngle = map(rightLeftValue, 0, 1023, -30, 30);  // -30 to +30 degrees
  int speed = map(forwardBackwardValue, 0, 1023, -255, 255);

  // Calculate individual wheel speeds for Ackermann steering
  float innerWheelAngle = atan(wheelbase / (trackWidth + 2 * steeringAngle)) * 180.0 / PI;
  float outerWheelAngle = atan(wheelbase / (trackWidth - 2 * steeringAngle)) * 180.0 / PI;

  int frontLeftSpeed = speed;
  int frontRightSpeed = speed;
  int rearLeftSpeed = speed;
  int rearRightSpeed = speed;

  if (steeringAngle > 0) {
    frontLeftSpeed *= cos(innerWheelAngle * PI / 180.0);
    rearRightSpeed *= cos(innerWheelAngle * PI / 180.0);
    frontRightSpeed *= cos(outerWheelAngle * PI / 180.0);
    rearLeftSpeed *= cos(outerWheelAngle * PI / 180.0);
  } else {
    frontLeftSpeed *= cos(outerWheelAngle * PI / 180.0);
    rearRightSpeed *= cos(outerWheelAngle * PI / 180.0);
    frontRightSpeed *= cos(innerWheelAngle * PI / 180.0);
    rearLeftSpeed *= cos(innerWheelAngle * PI / 180.0);
  }

  // Set motor speeds and steering angles
  setMotorSpeeds(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed, speed, speed);
  setServoAngles(servoStraightAngle + steeringAngle, servoStraightAngle - steeringAngle, servoStraightAngle + steeringAngle, servoStraightAngle - steeringAngle);

  // Add a short delay for smoother control
  delay(50);
}



// Helpful functions

// Set servo angles
void setServoAngles(int frontLeft, int frontRight, int rearLeft, int rearRight) {
  frontLeftServo.write(frontLeft);
  frontRightServo.write(frontRight);
  rearLeftServo.write(rearLeft);
  rearRightServo.write(rearRight);
}

// Set motor speeds and direction
void setMotorSpeeds(int motor1Speed, int motor2Speed, int motor3Speed, int motor4Speed, int motor5Speed, int motor6Speed) {
  analogWrite(motor1_IN1, abs(motor1Speed));
  analogWrite(motor2_IN1, abs(motor2Speed));
  analogWrite(motor3_IN1, abs(motor3Speed));
  analogWrite(motor4_IN1, abs(motor4Speed));
  analogWrite(motor5_IN1, abs(motor5Speed));
  analogWrite(motor6_IN1, abs(motor6Speed));

  // Set motor direction
  digitalWrite(motor1_IN2, motor1Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor2_IN2, motor2Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor3_IN2, motor3Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor4_IN2, motor4Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor5_IN2, motor5Speed >= 0 ? HIGH : LOW);
  digitalWrite(motor6_IN2, motor6Speed >= 0 ? HIGH : LOW);
}
