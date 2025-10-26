// Arduino Uno - Receiver code for RC Crane control
// Receives data from Arduino Nano via Serial and controls motors via L293D chips

#include <Servo.h>

// Pin definitions for L293D motor drivers
// L293D #1 - Motors A (Tower) and B (Boom)
#define MOTOR_A_FWD 2    // Tower rotation forward
#define MOTOR_A_REV A4    // Tower rotation reverse
#define MOTOR_B_UP 4     // Boom up
#define MOTOR_B_DOWN A3   // Boom down

// L293D #2 - Motors V (Boom length) and G (Winch)
#define MOTOR_V_EXTEND 6   // Boom extend
#define MOTOR_V_RETRACT 7  // Boom retract
#define MOTOR_G_UP 8       // Winch up
#define MOTOR_G_DOWN A0    // Winch down

// L293D #3 - Motor E (Platform movement) - PWM control
#define MOTOR_E_PWM 5      // PWM speed control
#define MOTOR_E_IN1 A1     // Direction control 1
#define MOTOR_E_IN2 A2     // Direction control 2

// Servo C (Axis rotation)
#define SERVO_C_PIN 3

// Constants
#define DEADZONE_MIN 487
#define DEADZONE_MAX 537
#define PWM_MAX 255
#define SERVO_MIN 0
#define SERVO_MAX 180

// Data packet structure (same as Nano)
struct DataPacket {
  int joyX;     // Joystick X axis (0-1023) - Platform motor
  int joyY;     // Joystick Y axis (0-1023) - Servo control
  byte buttons; // 8 buttons packed into one byte
};

Servo servoC;
DataPacket receivedData;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize motor control pins
  pinMode(MOTOR_A_FWD, OUTPUT);
  pinMode(MOTOR_A_REV, OUTPUT);
  pinMode(MOTOR_B_UP, OUTPUT);
  pinMode(MOTOR_B_DOWN, OUTPUT);
  pinMode(MOTOR_V_EXTEND, OUTPUT);
  pinMode(MOTOR_V_RETRACT, OUTPUT);
  pinMode(MOTOR_G_UP, OUTPUT);
  pinMode(MOTOR_G_DOWN, OUTPUT);
  
  // Platform motor pins
  pinMode(MOTOR_E_PWM, OUTPUT);
  pinMode(MOTOR_E_IN1, OUTPUT);
  pinMode(MOTOR_E_IN2, OUTPUT);
  
  // Initialize servo
  servoC.attach(SERVO_C_PIN);
  servoC.write(90); // Center position
  
  // Turn off all motors initially
  stopAllMotors();
  
  // Reserve string buffer
  inputString.reserve(50);
}

void loop() {
  // Check for incoming serial data
  if (stringComplete) {
    parseReceivedData();
    controlMotors();
    inputString = "";
    stringComplete = false;
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Parse received data string format: "X:value,Y:value,B:value"
void parseReceivedData() {
  int xIndex = inputString.indexOf("X:");
  int yIndex = inputString.indexOf(",Y:");
  int bIndex = inputString.indexOf(",B:");
  
  if (xIndex != -1 && yIndex != -1 && bIndex != -1) {
    receivedData.joyX = inputString.substring(xIndex + 2, yIndex).toInt();
    receivedData.joyY = inputString.substring(yIndex + 3, bIndex).toInt();
    receivedData.buttons = inputString.substring(bIndex + 3).toInt();
  }
}

// Main motor control function
void controlMotors() {
  // Control discrete motors (A, B, V, G) based on button states
  controlDiscreteMotors();
  
  // Control platform motor E with PWM
  controlPlatformMotor();
  
  // Control servo C
  controlServo();
}

// Control motors A, B, V, G based on button pairs
void controlDiscreteMotors() {
  // Motor A - Tower rotation (buttons 0 and 1)
  if (receivedData.buttons & (1 << 0)) {
    // Button 1 pressed - rotate forward
    digitalWrite(MOTOR_A_FWD, HIGH);
    digitalWrite(MOTOR_A_REV, LOW);
  } else if (receivedData.buttons & (1 << 1)) {
    // Button 2 pressed - rotate reverse
    digitalWrite(MOTOR_A_FWD, LOW);
    digitalWrite(MOTOR_A_REV, HIGH);
  } else {
    // No button pressed - stop
    digitalWrite(MOTOR_A_FWD, LOW);
    digitalWrite(MOTOR_A_REV, LOW);
  }
  
  // Motor B - Boom up/down (buttons 2 and 3)
  if (receivedData.buttons & (1 << 2)) {
    // Button 3 pressed - boom up
    digitalWrite(MOTOR_B_UP, HIGH);
    digitalWrite(MOTOR_B_DOWN, LOW);
  } else if (receivedData.buttons & (1 << 3)) {
    // Button 4 pressed - boom down
    digitalWrite(MOTOR_B_UP, LOW);
    digitalWrite(MOTOR_B_DOWN, HIGH);
  } else {
    // No button pressed - stop
    digitalWrite(MOTOR_B_UP, LOW);
    digitalWrite(MOTOR_B_DOWN, LOW);
  }
  
  // Motor V - Boom length (buttons 4 and 5)
  if (receivedData.buttons & (1 << 4)) {
    // Button 5 pressed - extend boom
    digitalWrite(MOTOR_V_EXTEND, HIGH);
    digitalWrite(MOTOR_V_RETRACT, LOW);
  } else if (receivedData.buttons & (1 << 5)) {
    // Button 6 pressed - retract boom
    digitalWrite(MOTOR_V_EXTEND, LOW);
    digitalWrite(MOTOR_V_RETRACT, HIGH);
  } else {
    // No button pressed - stop
    digitalWrite(MOTOR_V_EXTEND, LOW);
    digitalWrite(MOTOR_V_RETRACT, LOW);
  }
  
  // Motor G - Winch (buttons 6 and 7)
  if (receivedData.buttons & (1 << 6)) {
    // Button 7 pressed - winch up
    digitalWrite(MOTOR_G_UP, HIGH);
    digitalWrite(MOTOR_G_DOWN, LOW);
  } else if (receivedData.buttons & (1 << 7)) {
    // Button 8 pressed - winch down
    digitalWrite(MOTOR_G_UP, LOW);
    digitalWrite(MOTOR_G_DOWN, HIGH);
  } else {
    // No button pressed - stop
    digitalWrite(MOTOR_G_UP, LOW);
    digitalWrite(MOTOR_G_DOWN, LOW);
  }
}

// Control platform motor E with PWM and direction
void controlPlatformMotor() {
  int joyY = receivedData.joyY;
  
  if (joyY >= DEADZONE_MIN && joyY <= DEADZONE_MAX) {
    // Dead zone - stop motor
    analogWrite(MOTOR_E_PWM, 0);
    digitalWrite(MOTOR_E_IN1, LOW);
    digitalWrite(MOTOR_E_IN2, LOW);
  } 
  else if (joyY > DEADZONE_MAX) {
    // Forward direction (537-1023)
    int speed = map(joyY, DEADZONE_MAX, 1023, 0, PWM_MAX);
    speed = constrain(speed, 0, PWM_MAX);
    
    digitalWrite(MOTOR_E_IN1, HIGH);
    digitalWrite(MOTOR_E_IN2, LOW);
    analogWrite(MOTOR_E_PWM, speed);
  }
  else if (joyY < DEADZONE_MIN) {
    // Reverse direction (0-487)
    int speed = map(joyY, 0, DEADZONE_MIN, PWM_MAX, 0);
    speed = constrain(speed, 0, PWM_MAX);
    
    digitalWrite(MOTOR_E_IN1, LOW);
    digitalWrite(MOTOR_E_IN2, HIGH);
    analogWrite(MOTOR_E_PWM, speed);
  }
}

// Control servo C based on joystick X
void controlServo() {
  int joyX = receivedData.joyX;
  
  if (joyX >= DEADZONE_MIN && joyX <= DEADZONE_MAX) {
    // Dead zone - center position
    servoC.write(90);
  } else {
    // Map joystick value to servo angle
    int servoAngle;
    if (joyX > DEADZONE_MAX) {
      // Map 537-1023 to 90-180
      servoAngle = map(joyX, DEADZONE_MAX, 1023, 90, 180);
    } else {
      // Map 0-487 to 0-90
      servoAngle = map(joyX, 0, DEADZONE_MIN, 0, 90);
    }
    
    servoAngle = constrain(servoAngle, 0, 180);
    servoC.write(servoAngle);
  }
}


void stopAllMotors() {
  digitalWrite(MOTOR_A_FWD, LOW);
  digitalWrite(MOTOR_A_REV, LOW);
  digitalWrite(MOTOR_B_UP, LOW);
  digitalWrite(MOTOR_B_DOWN, LOW);
  digitalWrite(MOTOR_V_EXTEND, LOW);
  digitalWrite(MOTOR_V_RETRACT, LOW);
  digitalWrite(MOTOR_G_UP, LOW);
  digitalWrite(MOTOR_G_DOWN, LOW);
  
  analogWrite(MOTOR_E_PWM, 0);
  digitalWrite(MOTOR_E_IN1, LOW);
  digitalWrite(MOTOR_E_IN2, LOW);
}