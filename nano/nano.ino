
// Sends joystick and button data via RX/TX

// Pin definitions
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

// Data packet structure
struct DataPacket {
  int joyX;     // Joystick X axis (0-1023)
  int joyY;     // Joystick Y axis (0-1023)
  byte buttons; // 8 buttons packed into one byte
};

DataPacket data;

void setup() {
  Serial.begin(9600); // HC-05 communication (default baud rate)
  
  // Setup button pins with internal pull-up
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(BTN_5, INPUT_PULLUP);
  pinMode(BTN_6, INPUT_PULLUP);
  pinMode(BTN_7, INPUT_PULLUP);
  pinMode(BTN_8, INPUT_PULLUP);
  
  delay(1000); // Wait for HC-05 to initialize
}

void loop() {
  // Read joystick values
  data.joyX = analogRead(JOY_X);
  data.joyY = analogRead(JOY_Y);
  
  // Read buttons and pack into byte
  data.buttons = 0;
  if (!digitalRead(BTN_1)) data.buttons |= (1 << 0);
  if (!digitalRead(BTN_2)) data.buttons |= (1 << 1);
  if (!digitalRead(BTN_3)) data.buttons |= (1 << 2);
  if (!digitalRead(BTN_4)) data.buttons |= (1 << 3);
  if (!digitalRead(BTN_5)) data.buttons |= (1 << 4);
  if (!digitalRead(BTN_6)) data.buttons |= (1 << 5);
  if (!digitalRead(BTN_7)) data.buttons |= (1 << 6);
  if (!digitalRead(BTN_8)) data.buttons |= (1 << 7);
  
  // Send data packet via HC-05
  // Format: "X:value,Y:value,B:value\n"
  Serial.print("X:");
  Serial.print(data.joyX);
  Serial.print(",Y:");
  Serial.print(data.joyY);
  Serial.print(",B:");
  Serial.print(data.buttons, BIN);
  Serial.println(); // End of packet marker
  
  delay(250);// Send data every 250ms
}