/*
 --- Self-Balancing Robot with BLE Control
 --- Code Author: Joey Negm
 --- Project Partners: Sydney Doung, Bruno Brett
 --- Features:
 --- PID balancing using Arduino BMI270 IMU
 --- BLE remote control (forward, backward, left, right)
 --- Real-time dual display (TFT + LCD)
 --- Power monitoring via Hall-effect current sensor
 --- Safety cutoffs for tilt angle
 */

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <LiquidCrystal.h>
#include <Adafruit_ILI9341.h>

// -------------------- Constants --------------------
#define BUFFER_SIZE 20

// Motor pins
#define BIN1 5 // Motor B direction pin 1
#define BIN2 4 // Motor B direction pin 2
#define AIN1 3 // Motor A direction pin 1
#define AIN2 2 // Motor A direction pin 2

// TFT pins
#define TFT_CS 10  // Chip select pin
#define TFT_DC 9   // Data/command pin
#define TFT_RST 8  // Reset pin (optional, can connect to Arduino reset)

// LCD pins
const int RS = A5, EN = A4, LCD_D4 = A3, LCD_D5 = A2, LCD_D6 = A1, LCD_D7 = A0;

// PID Constants (tune these based on your robot's behavior) // 10.5V 1.444A
const float Kp = 18; // Proportional gain
const float Ki = 100; // Integral gain
const float Kd = 1.3; // Derivative gain

// Complementary filter weights
const float gyroWeight = 0.98;
const float accelWeight = 0.02;

// Button pin and debounce
const int buttonPin = 6;
const unsigned long debounceDelay = 50;

// -------------------- Objects --------------------

// Bluetooth® Service
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);
  
LiquidCrystal LCD(RS,EN,LCD_D4,LCD_D5,LCD_D6,LCD_D7);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// -------------------- Global Variables --------------------

// LCD Dashboard variables
int screenMode = 0; 
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// IMU readings
float ax, ay, az;
float accelAngle = 0;
float filteredAngle = 0;
float deltaT = 0;
unsigned long previousTime = 0;

// PID control variables
float integral = 0;
float prevError = 0;
float pidOutput = 0;
float desiredAngle = 0.4;

// Battery & power
float current = 0;
float voltage = 0;
float powerConsumed = 0;

// Speed / RPM
float speed = 0;
int RPM = 0;

// Dashboard variables
bool batteryLowFlag = false;
bool lastFaceHappy = true;

// Motors / BLE control flags
float forward = 0;
float backward = 0;
float left = 0;
float right = 0;

// --- Setup ---
/**
 * Description: Arduino setup function. Initializes IMU, motors, displays, BLE, and button.
 * Parameters: None
 * Return: void
 */

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize LCD
  LCD.begin(16, 2);
  LCD.clear();

  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize TFT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_YELLOW);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);

  // Write Team A1 on TFT
  tft.fillScreen(ILI9341_YELLOW);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(80, 20);
  tft.setTextSize(2);
  tft.setRotation(4);
  tft.print("TEAM A1");
  tft.setRotation(3);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);

  // Draw face and symbols on TFT
  tft.drawCircle(160, 120, 100, ILI9341_BLACK);
  tft.drawCircle(160, 120, 101, ILI9341_BLACK);
  tft.drawCircle(160, 120, 102, ILI9341_BLACK);
  tft.fillCircle(130, 90, 10, ILI9341_BLACK);
  tft.fillCircle(190, 90, 10, ILI9341_BLACK);
  drawMouth(true);
  drawBatterySymbol(100);


  // Initialize BLE
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("BLE-DEVICE");
  BLE.setDeviceName("BLE-DEVICE");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("Waiting for data");
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");

  // Initialize IMU Angle
  previousTime = millis();
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    accelAngle = atan2(az, ay) * 180.0 / PI - 90;
    filteredAngle = accelAngle;
  }
}


// --- Main Loop ---
/**
 * Description: Arduino main loop. Handles BLE, IMU, PID control, display updates, and motor driving.
 * Parameters: None
 * Return: void
 */
void loop() {

  int motorSpeed = 0;

  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    bool btSymbolDrawn = false;
    while (central.connected()) {

      if (!btSymbolDrawn) {
        drawBluetoothSymbol();
        btSymbolDrawn = true;
    }

      // --- Time update ---
      unsigned long currentTime = millis();
      deltaT = (currentTime - previousTime) / 1000.0;
      previousTime = currentTime;

      // --- IMU readings ---
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        accelAngle = atan2(az, ay) * 180.0 / PI - 90;
      }

      float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        filteredAngle = gyroWeight * (filteredAngle + gyroX * deltaT) + accelWeight * accelAngle;
      }

      // // Adjust desired angle/command weights based on robot behavior
      desiredAngle = 1.9 + 0.6 * forward - 0.16 * backward;

      // --- Battery current & symbol ---
      current = readBatteryCurrent();
      if (abs(filteredAngle) > 5 && motorSpeed < 50) {
        batteryLowFlag = true;
      } else if (abs(filteredAngle) < 2 && motorSpeed > 100) {
        batteryLowFlag = false;
      }
      if (batteryLowFlag) {
        drawBatterySymbol(10);
      } else {
        drawBatterySymbol(100);
      }

      // --- PID control ---
      float error = desiredAngle - filteredAngle;
      pidOutput = computePID(error, deltaT);

      // --- Motor control ---/ Adjust motor speeds to be equal when going straight
      if (abs(filteredAngle) <= 15) {
        motorSpeed = constrain(abs(pidOutput), 0, 255);
        if (pidOutput > 0) {
          analogWrite(AIN1, motorSpeed);
          digitalWrite(AIN2, LOW);
          analogWrite(BIN1, motorSpeed * (1));
          digitalWrite(BIN2, LOW);
        } else {
          analogWrite(AIN2, motorSpeed * (1 + 0.35 * right));
          digitalWrite(AIN1, LOW);
          analogWrite(BIN2, motorSpeed * (1 + 0.41 * left));
          digitalWrite(BIN1, LOW);
        }

        voltage = (map(motorSpeed, 0, 255, 3, 1091)) / 100.0;
        powerConsumed = current * voltage;
        RPM = map(motorSpeed, 30, 255, 48, 474);
        speed = (RPM / 60.0) * (PI * 8.8/100);
      } 
      
      else {
        motorSpeed = constrain(abs(pidOutput), 0, 255);

        analogWrite(AIN1, 0);
        analogWrite(AIN2, 0);
        analogWrite(BIN1, 0);
        analogWrite(BIN2, 0);

        voltage = map(motorSpeed, 0, 255, 3, 1091)/100.0;
        powerConsumed = current * voltage;
        RPM = map(motorSpeed, 30, 255, 48, 474);
        speed = (RPM / 60.0) * (PI * 8.8);
      }

      // --- Display updates ---
      LCD.clear();
      switch(screenMode) {
        case 0: LCD.print("Angle: "); LCD.print(filteredAngle); break;
        case 1: LCD.print("Power: "); LCD.print(powerConsumed); break;
        case 2: LCD.print("Speed: "); LCD.print(speed); LCD.print("m/s"); break;
        case 3: LCD.print("Time: "); LCD.print(currentTime / 1000); LCD.print("s"); break;
        case 4: LCD.print("RPM: "); LCD.print(RPM); break;
        case 5: LCD.print("DA: "); LCD.print(desiredAngle); break;
      }

      // --- Face expression update ---
      bool happy = (abs(filteredAngle) <= 10);
      if (happy != lastFaceHappy) {
          clearMouth();
          drawMouth(happy);
          lastFaceHappy = happy;
      }

      // --- BLE Characteristic Handling ---
      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        const unsigned char* receivedData = customCharacteristic.value();
        char receivedString[length + 1];
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0';
        Serial.println(receivedString);

        // --- Update motor flags ---
        if (strcmp(receivedString, "FORWARD") == 0){
          forward = 1;
          backward = 0;
          left = 0;
          right = 0;         
        }

        else if (strcmp(receivedString, "BACKWARD") == 0){
          forward = 0;
          backward = 1;
          left = 0;
          right = 0;
        }

        else if (strcmp(receivedString, "RESET INTEGRAL") == 0) {
          backward = 0;
          forward = 0;
          integral = 0;
          left = 0;
          right = 0;
        }

        else if (strcmp(receivedString, "LEFT") == 0) {
          backward = 0;
          forward = 1;
          left = 1;
          right = 0;
        }

        else if (strcmp(receivedString, "RIGHT") == 0) {
          backward = 0;
          forward = 1;
          left = 0;
          right = 1;
        }

        customCharacteristic.writeValue("Data received");
      }

      // --- Button Debounce and LCD Cycling ---
      int reading = digitalRead(buttonPin);
      if (reading != lastButtonState) {
        lastDebounceTime = millis();
      }

      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW && currentButtonState == HIGH) {
          screenMode = (screenMode + 1) % 6;
      }
        currentButtonState = reading;
      }
      lastButtonState = reading;
    }

    clearBluetoothSymbol();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected from central.");
  }
}

// -------------------- PID --------------------
/**
 * Description: Computes the PID controller output based on error and elapsed time.
 * Parameters:
 *   - error (float): Current control error (desired angle - measured angle)
 *   - deltaT (float): Elapsed time since last update (seconds)
 * Return: (float) PID output, constrained to [-255, 255]
 */
float computePID(float error, float deltaT) {
  integral += error * deltaT;
  integral = constrain(integral, -30.0, 30.0);
  float derivative = (error - prevError) / deltaT;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  pidOutput = constrain(pidOutput, -255.0, 255.0);
  prevError = error;
  return pidOutput;
}

// -------------------- Mouth Drawing --------------------
/**
 * Description: Draws a mouth on the TFT display, either happy (smile) or sad (frown).
 * Parameters:
 *   - happy (bool): true for smile, false for frown
 * Return: void
 */
void drawMouth(bool happy) {
  for (int x = -40; x <= 40; x++) {
    int y = happy ? (20 - (x * x) / 80) : ((x * x) / 80 - 20);

    for (int thickness = -2; thickness <= 2; thickness++) 
      tft.drawPixel(160 + x, 180 + y + thickness, ILI9341_BLACK);
  }
}

/**
 * Description: Clears the previously drawn mouth (smile or frown) by overwriting with background color.
 * Parameters: None (uses global lastFaceHappy)
 * Return: void
 */
void clearMouth() {
  for (int x = -40; x <= 40; x++) {
    int y = (lastFaceHappy ? (20 - (x * x) / 80) : ((x * x) / 80 - 20));

    for (int thickness = -2; thickness <= 2; thickness++)
      tft.drawPixel(160 + x, 180 + y + thickness, ILI9341_YELLOW);
  }
}

// -------------------- Bluetooth Symbol --------------------
/**
 * Description: Draws a Bluetooth symbol on the TFT in the top-right corner.
 * Parameters: None
 * Return: void
 */
void drawBluetoothSymbol() {
    int x = 290, y = 10;
    uint16_t color = ILI9341_BLACK;

    tft.fillRect(x - 5, y, 20, 30, ILI9341_YELLOW);

    tft.fillRect(x - 1, y, 1, 26, color);

    tft.drawLine(x, y, x + 8, y + 6, color);
    tft.drawLine(x, y + 12, x + 8, y + 6, color);

    tft.drawLine(x, y + 12, x + 8, y + 18, color);
    tft.drawLine(x, y + 25, x + 8, y + 18, color);

    tft.drawLine(x, y + 12, x - 8, y + 6, color);

    tft.drawLine(x, y + 12, x - 8, y + 18, color);

}

/**
 * Description: Clears the Bluetooth symbol from the TFT by overwriting with background color.
 * Parameters: None
 * Return: void
 */
void clearBluetoothSymbol() {
  int x = 290, y = 10; 

  tft.fillRect(x - 5, y, 20, 30, ILI9341_YELLOW);

  tft.fillRect(x - 1, y, 1, 26, ILI9341_YELLOW);

  tft.drawLine(x, y, x + 8, y + 6, ILI9341_YELLOW);
  tft.drawLine(x, y + 12, x + 8, y + 6, ILI9341_YELLOW);

  tft.drawLine(x, y + 12, x + 8, y + 18, ILI9341_YELLOW);
  tft.drawLine(x, y + 25, x + 8, y + 18, ILI9341_YELLOW);

  tft.drawLine(x, y + 12, x - 8, y + 6, ILI9341_YELLOW);

  tft.drawLine(x, y + 12, x - 8, y + 18, ILI9341_YELLOW);
}

// --- Battery Symbol ---
/**
 * Description: Draws a battery icon on the TFT display with a fill level.
 * Parameters:
 *   - level (int): Battery percentage [0–100]
 * Return: void
 */
void drawBatterySymbol(int level) {
    int x = 10, y = 10; 
    int width = 25, height = 12; 
    uint16_t color = ILI9341_BLACK;

    tft.fillRect(x - 2, y - 2, width + 6, height + 4, ILI9341_YELLOW);

    tft.drawRect(x, y, width, height, color);

    tft.fillRect(x + width, y + 3, 3, height - 6, color);

    uint16_t fillColor = (level < 20) ? ILI9341_RED : ILI9341_GREEN;

    int fillWidth = (width - 4) * level / 100;

    if (fillWidth > 0) 
      tft.fillRect(x + 2, y + 2, fillWidth, height - 4, fillColor);
}

// -------------------- Battery Current --------------------
/**
 * Description: Reads battery current from the TMCS1108 Hall-effect sensor.
 * Parameters: None
 * Return: (float) Battery current in Amperes
 */
float readBatteryCurrent() {
  int sensorValue = analogRead(A1);
  float voltage = sensorValue * (3.3 / 1023.0);
  return (voltage - 2.5) / 0.2;
}