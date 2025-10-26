// Arduino Nano - Nadajnik do sterowania dźwigiem RC
// Wysyła dane z joysticka i przycisków przez RX/TX

// Definicje pinów
#define JOY_X A1
#define JOY_Y A0
#define BTN_1 2
#define BTN_2 3
#define BTN_3 4
#define BTN_4 5
#define BTN_5 6
#define BTN_6 7
#define BTN_7 8
#define BTN_8 A2

// Struktura pakietu danych
struct DataPacket {
  int joyX;     // Oś X joysticka (0-1023) - sterowanie serwem C
  int joyY;     // Oś Y joysticka (0-1023) - sterowanie silnikiem platformy E
  byte buttons; // 8 przycisków spakowanych w jeden bajt
};

DataPacket data;

void setup() {
  Serial.begin(9600); // Komunikacja Serial (TX/RX)
  
  // Konfiguracja pinów przycisków z wewnętrznym pull-up
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_5, INPUT_PULLUP);
  pinMode(BTN_6, INPUT_PULLUP);
  pinMode(BTN_7, INPUT_PULLUP);
  pinMode(BTN_8, INPUT_PULLUP);
  
  delay(1000); // Oczekiwanie na inicjalizację
}

void loop() {
  // Odczyt wartości joysticka
  data.joyX = analogRead(JOY_X);
  data.joyY = analogRead(JOY_Y);
  
  // Odczyt przycisków i pakowanie do bajtu
  
  data.buttons = 0;

  // Silnik A - obrót wieży (bity 0 i 1)
  bool btn1 = !digitalRead(BTN_1);
  bool btn2 = !digitalRead(BTN_2);
  if (btn1 && !btn2) data.buttons |= (1 << 0);
  else if (btn2 && !btn1) data.buttons |= (1 << 1);

  // Silnik B - wysięgnik góra/dół (bity 2 i 3)
  bool btn3 = !digitalRead(BTN_3);
  bool btn4 = !digitalRead(BTN_4);
  if (btn3 && !btn4) data.buttons |= (1 << 2);
  else if (btn4 && !btn3) data.buttons |= (1 << 3);

  // Silnik V - długość wysięgnika (bity 4 i 5)
  bool btn5 = !digitalRead(BTN_5);
  bool btn6 = !digitalRead(BTN_6);
  if (btn5 && !btn6) data.buttons |= (1 << 4);
  else if (btn6 && !btn5) data.buttons |= (1 << 5);

  // Silnik G - wciągarka (bity 6 i 7)
  bool btn7 = !digitalRead(BTN_7);
  bool btn8 = !digitalRead(BTN_8);
  if (btn7 && !btn8) data.buttons |= (1 << 6);
  else if (btn8 && !btn7) data.buttons |= (1 << 7);
  
  // Wysyłanie pakietu danych przez Serial
  // Format: "X:wartość,Y:wartość,B:wartość\n"
  Serial.print("X:");
  Serial.print(data.joyX);
  Serial.print(",Y:");
  Serial.print(data.joyY);
  Serial.print(",B:");
  Serial.print(data.buttons, BIN);
  Serial.println(); // Znacznik końca pakietu
  
  delay(250); // Wysyłanie danych co 250ms
}