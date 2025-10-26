// Arduino Uno - Odbiornik do sterowania dźwigiem RC
// Odbiera dane od Arduino Nano przez Serial i steruje silnikami przez układy L293D

#include <Servo.h>

// Definicje pinów dla sterowników silników L293D
// L293D #1 - Silniki A (Wieża) i B (Wysięgnik)
#define MOTOR_A_FWD 2    // Obrót wieży do przodu
#define MOTOR_A_REV A4    // Obrót wieży do tyłu
#define MOTOR_B_UP 4     // Wysięgnik do góry
#define MOTOR_B_DOWN A3   // Wysięgnik w dół

// L293D #2 - Silniki V (Długość wysięgnika) i G (Wciągarka)
#define MOTOR_V_EXTEND 6   // Wysunięcie wysięgnika
#define MOTOR_V_RETRACT 7  // Wsunięcie wysięgnika
#define MOTOR_G_UP 8       // Wciągarka do góry
#define MOTOR_G_DOWN A0    // Wciągarka w dół

// L293D #3 - Silnik E (Ruch platformy) - sterowanie PWM
#define MOTOR_E_PWM 5      // PWM sterowanie prędkością
#define MOTOR_E_IN1 A1     // Sterowanie kierunkiem 1
#define MOTOR_E_IN2 A2     // Sterowanie kierunkiem 2

// Servo C (Obrót osi)
#define SERVO_C_PIN 3

// Stałe
#define DEADZONE_MIN 482
#define DEADZONE_MAX 542
#define PWM_MAX 255
#define SERVO_MIN 0
#define SERVO_MAX 180

// Struktura pakietu danych (taka sama jak w Nano)
struct DataPacket {
  int joyX;     // Oś X joysticka (0-1023) - Sterowanie serwem C
  int joyY;     // Oś Y joysticka (0-1023) - Sterowanie silnikiem platformy E
  byte buttons; // 8 przycisków spakowanych w jeden bajt
};

Servo servoC;
DataPacket receivedData;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Inicjalizacja pinów sterowania silnikami
  pinMode(MOTOR_A_FWD, OUTPUT);
  pinMode(MOTOR_A_REV, OUTPUT);
  pinMode(MOTOR_B_UP, OUTPUT);
  pinMode(MOTOR_B_DOWN, OUTPUT);
  pinMode(MOTOR_V_EXTEND, OUTPUT);
  pinMode(MOTOR_V_RETRACT, OUTPUT);
  pinMode(MOTOR_G_UP, OUTPUT);
  pinMode(MOTOR_G_DOWN, OUTPUT);
  
  // Piny silnika platformy
  pinMode(MOTOR_E_PWM, OUTPUT);
  pinMode(MOTOR_E_IN1, OUTPUT);
  pinMode(MOTOR_E_IN2, OUTPUT);
  
  // Inicjalizacja serwa
  servoC.attach(SERVO_C_PIN);
  servoC.write(90); // Pozycja środkowa
  
  // Wyłączenie wszystkich silników na początku
  stopAllMotors();
  
  // Rezerwacja bufora dla stringa
  inputString.reserve(50);
}

void loop() {
  // Sprawdzenie czy są dane z portu szeregowego
  if (stringComplete) {
    parseReceivedData();
    controlMotors();
    inputString = "";
    stringComplete = false;
  }
}

// Obsługa zdarzenia serial
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

// Parsowanie odebranych danych w formacie: "X:wartość,Y:wartość,B:wartość"
void parseReceivedData() {
  int xIndex = inputString.indexOf("X:");
  int yIndex = inputString.indexOf(",Y:");
  int bIndex = inputString.indexOf(",B:");
  
  if (xIndex != -1 && yIndex != -1 && bIndex != -1) {
    receivedData.joyX = inputString.substring(xIndex + 2, yIndex).toInt();
    receivedData.joyY = inputString.substring(yIndex + 3, bIndex).toInt();
    String bits = inputString.substring(bIndex + 3);
    receivedData.buttons = strtol(bits.c_str(), NULL, 2);
  }
}

// Główna funkcja sterowania silnikami
void controlMotors() {
  // Sterowanie silnikami dyskretnymi (A, B, V, G) na podstawie stanów przycisków
  controlDiscreteMotors();
  
  // Sterowanie silnikiem platformy E z PWM
  controlPlatformMotor();
  
  // Sterowanie serwem C
  controlServo();
}

// Sterowanie silnikami A, B, V, G na podstawie par przycisków
void controlDiscreteMotors() {
  // Silnik A - Obrót wieży (przyciski 0 i 1)
  if (receivedData.buttons & (1 << 0)) {
    // Przycisk 1 wciśnięty - obrót do przodu
    digitalWrite(MOTOR_A_FWD, HIGH);
    digitalWrite(MOTOR_A_REV, LOW);
  } else if (receivedData.buttons & (1 << 1)) {
    // Przycisk 2 wciśnięty - obrót do tyłu
    digitalWrite(MOTOR_A_FWD, LOW);
    digitalWrite(MOTOR_A_REV, HIGH);
  } else {
    // Żaden przycisk nie wciśnięty - stop
    digitalWrite(MOTOR_A_FWD, LOW);
    digitalWrite(MOTOR_A_REV, LOW);
  }
  
  // Silnik B - Wysięgnik góra/dół (przyciski 2 i 3)
  if (receivedData.buttons & (1 << 2)) {
    // Przycisk 3 wciśnięty - wysięgnik do góry
    digitalWrite(MOTOR_B_UP, HIGH);
    digitalWrite(MOTOR_B_DOWN, LOW);
  } else if (receivedData.buttons & (1 << 3)) {
    // Przycisk 4 wciśnięty - wysięgnik w dół
    digitalWrite(MOTOR_B_UP, LOW);
    digitalWrite(MOTOR_B_DOWN, HIGH);
  } else {
    // Żaden przycisk nie wciśnięty - stop
    digitalWrite(MOTOR_B_UP, LOW);
    digitalWrite(MOTOR_B_DOWN, LOW);
  }
  
  // Silnik V - Długość wysięgnika (przyciski 4 i 5)
  if (receivedData.buttons & (1 << 4)) {
    // Przycisk 5 wciśnięty - wysunięcie wysięgnika
    digitalWrite(MOTOR_V_EXTEND, HIGH);
    digitalWrite(MOTOR_V_RETRACT, LOW);
  } else if (receivedData.buttons & (1 << 5)) {
    // Przycisk 6 wciśnięty - wsunięcie wysięgnika
    digitalWrite(MOTOR_V_EXTEND, LOW);
    digitalWrite(MOTOR_V_RETRACT, HIGH);
  } else {
    // Żaden przycisk nie wciśnięty - stop
    digitalWrite(MOTOR_V_EXTEND, LOW);
    digitalWrite(MOTOR_V_RETRACT, LOW);
  }
  
  // Silnik G - Wciągarka (przyciski 6 i 7)
  if (receivedData.buttons & (1 << 6)) {
    // Przycisk 7 wciśnięty - wciągarka do góry
    digitalWrite(MOTOR_G_UP, HIGH);
    digitalWrite(MOTOR_G_DOWN, LOW);
  } else if (receivedData.buttons & (1 << 7)) {
    // Przycisk 8 wciśnięty - wciągarka w dół
    digitalWrite(MOTOR_G_UP, LOW);
    digitalWrite(MOTOR_G_DOWN, HIGH);
  } else {
    // Żaden przycisk nie wciśnięty - stop
    digitalWrite(MOTOR_G_UP, LOW);
    digitalWrite(MOTOR_G_DOWN, LOW);
  }
}

// Sterowanie silnikiem platformy E z PWM i kierunkiem
void controlPlatformMotor() {
  int joyY = receivedData.joyY;
  
  if (joyY >= DEADZONE_MIN && joyY <= DEADZONE_MAX) {
    // Strefa martwa - zatrzymanie silnika
    analogWrite(MOTOR_E_PWM, 0);
    digitalWrite(MOTOR_E_IN1, LOW);
    digitalWrite(MOTOR_E_IN2, LOW);
  } 
  else if (joyY > DEADZONE_MAX) {
    // Kierunek do przodu (542-1023)
    int speed = map(joyY, DEADZONE_MAX, 1023, 0, PWM_MAX);
    speed = constrain(speed, 0, PWM_MAX);
    
    digitalWrite(MOTOR_E_IN1, HIGH);
    digitalWrite(MOTOR_E_IN2, LOW);
    analogWrite(MOTOR_E_PWM, speed);
  }
  else if (joyY < DEADZONE_MIN) {
    // Kierunek do tyłu (0-482)
    int speed = map(joyY, 0, DEADZONE_MIN, PWM_MAX, 0);
    speed = constrain(speed, 0, PWM_MAX);
    
    digitalWrite(MOTOR_E_IN1, LOW);
    digitalWrite(MOTOR_E_IN2, HIGH);
    analogWrite(MOTOR_E_PWM, speed);
  }
}

// Sterowanie serwem C na podstawie osi X joysticka
void controlServo() {
  int joyX = receivedData.joyX;
  
  if (joyX >= DEADZONE_MIN && joyX <= DEADZONE_MAX) {
    // Strefa martwa - pozycja środkowa
    servoC.write(90);
  } else {
    // Mapowanie wartości joysticka na kąt serwa
    int servoAngle;
    if (joyX > DEADZONE_MAX) {
      // Mapowanie 542-1023 na 90-180
      servoAngle = map(joyX, DEADZONE_MAX, 1023, 90, 180);

    } else {
      // Mapowanie 0-482 na 0-90
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