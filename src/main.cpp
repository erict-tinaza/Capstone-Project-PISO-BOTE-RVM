#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <HX711.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

//Constant Variables
const int POINTS_BLOCK = 4;
const int MAX_DISTANCE = 20;
const int ITERATIONS = 5;
const float MAX_WEIGHT = 500.00;

//Constant Var Pin Definitions
const int upButton = 40;
const int downButton = 41;
const int selectButton = 42;
//Servo
const int servoPin1 = 36;
const int servoPin2 = 37;
//Sensors

// Capacitive sensor configuration for analog reading
const int CAPACITIVE_SENSOR_PIN = A0;  // Analog pin for capacitive sensor
const int DETECTION_THRESHOLD = 800;    // Threshold for bottle detection (adjust if needed)
const int NO_BOTTLE_THRESHOLD = 500;    // Threshold for confirming bottle removal
const int HYSTERESIS = 50;             // Prevent flickering
const int SAMPLE_COUNT = 5;            // Number of readings to average
const int DEBOUNCE_MS = 50;            // Minimum time between readings
const int inductiveSensorPin = 15;

// Global state for the capacitive sensor

struct CapacitiveSensorState {
    bool isDetecting;
    unsigned long lastReadTime;
    int lastStableValue;
    
    // Constructor
    CapacitiveSensorState() {
        isDetecting = false;
        lastReadTime = 0;
        lastStableValue = 0;
    }
};
CapacitiveSensorState capacitiveSensor;
//Ultasonic Sensor 
const int TRIGGER_PIN = 22;
const int ECHO_PIN = 23;
//RGB LED
const int PIN_RED = 9;
const int PIN_GREEN = 10;
const int PIN_BLUE = 11;
//RFID
const int SS_PIN = 53;
const int RST_PIN = 2;    
//SIM Module
const int SIM_RX = 24;
const int SIM_TX = 25;
//Load Cell
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 27;
//LDR & LED
const int LDR_PIN = A1;
const int LED_INLET_PIN = 12;
//Coin Hopper
const int coinHopperSensor_PIN = 28;
const int relayPin = 45;

//Nokia 5110 LCD
const int PIN_RST = 3;   // RST (with 10kΩ resistor)
const int PIN_CE = 4;    // CE (with 1kΩ resistor)
const int PIN_DC = 5;    // DC (with 10kΩ resistor)
const int PIN_DIN = 6;   // DIN (with 10kΩ resistor)
const int PIN_CLK = 7;   // CLK (with 10kΩ resistor)
const int PIN_BL = 13;   // Backlight with 330Ω resistor

//For setting Nokia Display
const int MIN_CONTRAST = 0;
const int MAX_CONTRAST = 127;
const int CONTRAST_STEP = 5;
const int MIN_BRIGHTNESS = 0;
const int MAX_BRIGHTNESS = 255;
const int BRIGHTNESS_STEP = 10;

// Custom icons for Nokia display
static const unsigned char PROGMEM BOTTLE_ICON[] = {
    0b00011000,
    0b00111100,
    0b00111100,
    0b00111100,
    0b00111100,
    0b01111110,
    0b01111110,
    0b00111100
};

static const unsigned char PROGMEM COIN_ICON[] = {
    0b00111100,
    0b01111110,
    0b11100111,
    0b11000011,
    0b11000011,
    0b11100111,
    0b01111110,
    0b00111100
};

static const unsigned char PROGMEM CARD_ICON[] = {
    0b11111111,
    0b10000001,
    0b10111101,
    0b10100101,
    0b10111101,
    0b10000001,
    0b11111111,
    0b00000000
};

static const unsigned char PROGMEM ERROR_ICON[] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10011001,
    0b10011001,
    0b10100101,
    0b01000010,
    0b00111100
};
static const unsigned char PROGMEM SETTINGS_ICON[] = {
    0b00011000,
    0b00111100,
    0b11111111,
    0b11100111,
    0b11100111,
    0b11111111,
    0b00111100,
    0b00011000
};

//Global Variables
int coinCount = 0;
bool dispensingActive = false;
bool lastSensorState = HIGH;
bool isObjectInside = false;
int totalPoints = 0;
int pointsToRedeem = 0;
bool maintenanceMode = false;
String maintainerNum = "+639219133878";

String previousLcdLine1;
String previousLcdLine2;
String previousNokiaMessage;
const bool DEBUG_SENSORS = true;  // Set to true to enable sensor debugging
const int SENSOR_STABILIZE_TIME = 500;  // Time to wait for sensor readings to stabilize

//Object instantations
Adafruit_PCD8544 nokia(PIN_CLK, PIN_DIN, PIN_DC, PIN_CE, PIN_RST);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2;
MFRC522 mfrc522(SS_PIN, RST_PIN);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial sim800lv2(SIM_RX, SIM_TX);
HX711 scale;
MFRC522::MIFARE_Key key;



//Function Prototypes
void depositAction();
void redeemAction();
bool readCapacitiveSensorData();
int readInductiveSensorData();
void rotateServo(Servo &servo, int angle);
void openCloseBinLid(int lidNum, bool toOpen);
void waitToRemoveObject();
void waitForObjectPresence();
bool verifyObject();
void storePointsAction();
void insertAnotherBottleAction();
void redeemPointsAction();
void setUpRFID();
bool detectCard();
bool writePoints(int points);
int readPoints();
void sendSMS(String message);
bool isBinFull();
void handleMaintenanceMode();
bool isWeightAcceptable();
int readLDRSensorData();
void controlLedInlet(bool isOn);
bool isObjectClear();
void updateMenuDisplay();
void navigateMenu(int direction);
void selectMenuItem();
void pulseColor(int redValue, int greenValue, int blueValue);
void ledStatusCode(int errorCode);
void delayWithMsg(unsigned long duration, String message1, String message2, int statusCode);

MFRC522::StatusCode authenticateBlock(int blockNumber);
void calibrateLoadCell();
void dispenseCoin(int count);

void displayNokiaStatus(const String& message, const unsigned char* icon = nullptr);
void updateMenuDisplay();
void displayMainMenu();
void settingsAction();
void adjustContrastAction();
void adjustBrightnessAction();
void backToMainAction();
void debugSensorReadings();
// Menu structure
struct MenuItem {
    const char *name;
    void (*action)();
};

struct Menu {
    MenuItem *items;
    int itemCount;
    int currentItem;
    const char *title;
};
// Menu definitions
MenuItem mainMenuItems[] = {
    {"Deposit", depositAction},
    {"Redeem", redeemAction},
    {"Settings", settingsAction}
};

MenuItem settingsMenuItems[] = {
    {"Contrast", adjustContrastAction},
    {"Brightness", adjustBrightnessAction},
    {"Back", backToMainAction}
};

MenuItem postDepositMenuItems[] = {
    {"Insert", insertAnotherBottleAction},
    {"Redeem", redeemPointsAction},
    {"Store", storePointsAction}
};

// Menu structures - KEEP THESE
Menu mainMenu = {mainMenuItems, 3, 0, "Main Menu"};
Menu settingsMenu = {settingsMenuItems, 3, 0, "Settings"};
Menu postDepositMenu = {postDepositMenuItems, 3, 0, "Options"};
Menu *currentMenu = &mainMenu;
struct DisplayState {
    bool nokiaDisplayActive;
    bool lcdDisplayActive;
    unsigned long lastUpdateTime;
    int contrast;
    byte backlight;
} displayState;

//Function Implementation
void settingsAction() {
    currentMenu = &settingsMenu;
    updateMenuDisplay();
}

void backToMainAction() {
    currentMenu = &mainMenu;
    updateMenuDisplay();
}

//Functions for Settings
void adjustContrastAction() {
   int currentContrast = displayState.contrast;
    if (currentContrast < MIN_CONTRAST) currentContrast = MIN_CONTRAST;
    if (currentContrast > MAX_CONTRAST) currentContrast = MAX_CONTRAST;
    
    bool adjusting = true;
    unsigned long lastButtonPress = 0;
    
    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(10, 1);
    nokia.print("Contrast");;;
    nokia.drawLine(0, 12, 84, 12, BLACK);
    
    while (adjusting) {
        if (millis() - lastButtonPress > 100) {
            if (digitalRead(upButton) == LOW && currentContrast < MAX_CONTRAST) {
                currentContrast += CONTRAST_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(downButton) == LOW && currentContrast > MIN_CONTRAST) {
                currentContrast -= CONTRAST_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(selectButton) == LOW) {
                adjusting = false;
                lastButtonPress = millis();
            }
            
            // Update contrast
            nokia.setContrast(currentContrast);
            displayState.contrast = currentContrast;
            
            // Update display
            nokia.clearDisplay();
            nokia.drawRect(0, 0, 84, 10, BLACK);
            nokia.setCursor(10, 1);
            nokia.print("Contrast");
            nokia.drawLine(0, 12, 84, 12, BLACK);
            
            // Draw contrast bar
            int barWidth = map(currentContrast, MIN_CONTRAST, MAX_CONTRAST, 0, 64);
            nokia.drawRect(10, 25, 64, 8, BLACK);
            nokia.fillRect(10, 25, barWidth, 8, BLACK);
            
            nokia.setCursor(10, 40);
            nokia.print("Value: ");
            nokia.print(currentContrast);
            
            nokia.display();
            
            // Update LCD display
            lcd.clear();
            lcd.print("Adjust Contrast");
            lcd.setCursor(0, 1);
            lcd.print("Value: ");
            lcd.print(currentContrast);
        }
        delay(50);
    }
    
    // Save contrast value to EEPROM here if needed
    currentMenu = &settingsMenu;
    updateMenuDisplay();
}
void adjustBrightnessAction() {
    int currentBrightness = displayState.backlight;
    if (currentBrightness < MIN_BRIGHTNESS) currentBrightness = MIN_BRIGHTNESS;
    if (currentBrightness > MAX_BRIGHTNESS) currentBrightness = MAX_BRIGHTNESS;
    
    bool adjusting = true;
    unsigned long lastButtonPress = 0;
    
    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(8, 1);
    nokia.print("Brightness");
    nokia.drawLine(0, 12, 84, 12, BLACK);
    
    while (adjusting) {
        if (millis() - lastButtonPress > 100) {
            if (digitalRead(upButton) == LOW && currentBrightness < MAX_BRIGHTNESS) {
                currentBrightness += BRIGHTNESS_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(downButton) == LOW && currentBrightness > MIN_BRIGHTNESS) {
                currentBrightness -= BRIGHTNESS_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(selectButton) == LOW) {
                adjusting = false;
                lastButtonPress = millis();
            }
            
            // Update brightness
            analogWrite(PIN_BL, currentBrightness);
            displayState.backlight = currentBrightness;
            
            // Update display
            nokia.clearDisplay();
            nokia.drawRect(0, 0, 84, 10, BLACK);
            nokia.setCursor(8, 1);
            nokia.print("Brightness");
            nokia.drawLine(0, 12, 84, 12, BLACK);
            
            // Draw brightness bar
            int barWidth = map(currentBrightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS, 0, 64);
            nokia.drawRect(10, 25, 64, 8, BLACK);
            nokia.fillRect(10, 25, barWidth, 8, BLACK);
            
            nokia.setCursor(10, 40);
            nokia.print("Value: ");
            nokia.print(currentBrightness);
            
            nokia.display();
            
            // Update LCD display
            lcd.clear();
            lcd.print("Adjust Brightness");
            lcd.setCursor(0, 1);
            lcd.print("Value: ");
            lcd.print(currentBrightness);
        }
        delay(50);
    }
    
    // Save brightness value to EEPROM here if needed
    currentMenu = &settingsMenu;
    updateMenuDisplay();
}
void displayMainMenu() {
    currentMenu = &mainMenu;
    currentMenu->currentItem = 0;
    
    // Show static welcome message on LCD
    lcd.clear();
    lcd.print("Welcome to");
    lcd.setCursor(0, 1);
    lcd.print("PISO-BOTE");
    
    // Nokia menu display
    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(14, 1);
    nokia.print("Main Menu");
    nokia.drawLine(0, 12, 84, 12, BLACK);
    
    // Draw menu items with icons
    nokia.setCursor(2, 15);
    nokia.print("> Deposit");
    nokia.drawBitmap(70, 13, BOTTLE_ICON, 8, 8, BLACK);
    
    nokia.setCursor(2, 25);
    nokia.print("  Redeem");
    nokia.drawBitmap(70, 23, COIN_ICON, 8, 8, BLACK);
    
    nokia.setCursor(2, 35);
    nokia.print("  Settings");
    nokia.drawBitmap(70, 33, SETTINGS_ICON, 8, 8, BLACK);
    
    nokia.display();
}
bool setupNokiaDisplay() {
    nokia.begin();
    nokia.setContrast(50);
    nokia.setRotation(2);
    nokia.clearDisplay();
    nokia.setTextSize(1);
    nokia.setTextColor(BLACK);
    analogWrite(PIN_BL, HIGH);  // Use analogWrite instead of digitalWrite
    
    // Initialize display state
    displayState.nokiaDisplayActive = true;
    displayState.lcdDisplayActive = true;
    displayState.contrast = 50;
    displayState.backlight = 255;  // Full brightness initially
    displayState.lastUpdateTime = 0;
    
    // Display startup screen
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(14, 1);
    nokia.println("PISO-BOTE");
    nokia.drawLine(0, 12, 84, 12, BLACK);
    nokia.display();
    
    return true;
}
// Replace setPower with display enabling/disabling
void setDisplayPower(bool nokiaOn, bool lcdOn) {
    if (nokiaOn != displayState.nokiaDisplayActive) {
        if (nokiaOn) {
            nokia.begin();
            nokia.setRotation(2);  // Maintain rotation
            nokia.setContrast(displayState.contrast);  // Maintain contrast
            analogWrite(PIN_BL, displayState.backlight);  // Use stored backlight value
        } else {
            nokia.clearDisplay();
            nokia.display();
            analogWrite(PIN_BL, LOW);
        }
        displayState.nokiaDisplayActive = nokiaOn;
    }
    
    if (lcdOn != displayState.lcdDisplayActive) {
        if (lcdOn) {
            lcd.backlight();
        } else {
            lcd.noBacklight();
        }
        displayState.lcdDisplayActive = lcdOn;
    }
}
void displayNokiaStatus(const String& message, const unsigned char* icon) {
    nokia.clearDisplay();
    
    if (icon != nullptr) {
        nokia.drawBitmap(38, 8, icon, 8, 8, BLACK);
        nokia.drawLine(0, 20, 84, 20, BLACK);
        nokia.setCursor(0, 25);
    } else {
        nokia.setCursor(0, 15);
    }
    
    nokia.print(message);
    nokia.display();
}
// Modified updateDualDisplayStatus function
void updateDualDisplayStatus(const String& message1, const String& message2, const unsigned char* icon = nullptr) {
    const int LCD_WIDTH = 16;
    String lcd1 = message1.length() > LCD_WIDTH ? message1.substring(0, LCD_WIDTH) : message1;
    String lcd2 = message2.length() > LCD_WIDTH ? message2.substring(0, LCD_WIDTH) : message2;
    
    // Update LCD only if content has changed
    if (lcd1 != previousLcdLine1 || lcd2 != previousLcdLine2) {
        lcd.clear();
        lcd.print(lcd1);
        if (lcd2.length() > 0) {
            lcd.setCursor(0, 1);
            lcd.print(lcd2);
        }
        
        previousLcdLine1 = lcd1;
        previousLcdLine2 = lcd2;
    }
    
    // Nokia display handling with word wrapping
    const int NOKIA_LINE_WIDTH = 14; // Characters that fit on Nokia display
    String nokiaMessage = message1 + (message2.length() > 0 ? "\n" + message2 : "");
    
    if (nokiaMessage != previousNokiaMessage) {
        nokia.clearDisplay();
        
        if (icon != nullptr) {
            nokia.drawBitmap(38, 8, icon, 8, 8, BLACK);
            nokia.drawLine(0, 20, 84, 20, BLACK);
            nokia.setCursor(0, 25);
        } else {
            nokia.setCursor(0, 15);
        }
        
        // Word wrapping implementation
        int currentY = icon ? 25 : 15;
        int startPos = 0;
        
        // Handle message1
        while (startPos < message1.length()) {
            int endPos = min(startPos + NOKIA_LINE_WIDTH, (int)message1.length());
            if (endPos < message1.length()) {
                int lastSpace = message1.lastIndexOf(' ', endPos);
                if (lastSpace > startPos) {
                    endPos = lastSpace;
                }
            }
            nokia.setCursor(0, currentY);
            nokia.print(message1.substring(startPos, endPos));
            startPos = endPos + 1;
            currentY += 8;
        }
        
        // Handle message2 if present
        if (message2.length() > 0) {
            currentY += 2; // Add spacing between messages
            startPos = 0;
            while (startPos < message2.length()) {
                int endPos = min(startPos + NOKIA_LINE_WIDTH, (int)message2.length());
                if (endPos < message2.length()) {
                    int lastSpace = message2.lastIndexOf(' ', endPos);
                    if (lastSpace > startPos) {
                        endPos = lastSpace;
                    }
                }
                nokia.setCursor(0, currentY);
                nokia.print(message2.substring(startPos, endPos));
                startPos = endPos + 1;
                currentY += 8;
            }
        }
        
        nokia.display();
        previousNokiaMessage = nokiaMessage;
    }
}
void resetSystem() {
    totalPoints = 0;
    pointsToRedeem = 0;
    maintenanceMode = false;
    isObjectInside = false;
    coinCount = 0;
    dispensingActive = false;
    
    // Reset displays
    nokia.clearDisplay();
    nokia.display();
    lcd.clear();
    
    // Reset menu state
    currentMenu = &mainMenu;
    currentMenu->currentItem = 0;
    
    // Return to normal LED status
    ledStatusCode(200);
}

// Add this to the setup function after existing initializations
void additionalSetup() {
    // Reset system state
    resetSystem();
    
    // Display welcome message
    delayWithMsg(2000, "Welcome to", "PISO-BOTE", 200);
    
    // Show main menu
    displayMainMenu();
}
void handleError(const String& message1, const String& message2) {
    ledStatusCode(404);
    delayWithMsg(2000, message1, message2, 404);
    ledStatusCode(200);
}
void updateMenuDisplay() {
    // Show static welcome message on LCD
    lcd.clear();
    lcd.print("Welcome to");
    lcd.setCursor(0, 1);
    lcd.print("PISO-BOTE");
    
    String title = currentMenu->title;
    
    // Nokia-specific menu layout
    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(14, 1);
    nokia.print(title);
    nokia.drawLine(0, 12, 84, 12, BLACK);
    
    if (currentMenu == &mainMenu) {
        // Main menu with icons
        nokia.setCursor(2, 15);
        nokia.print(currentMenu->currentItem == 0 ? ">" : " ");
        nokia.print("Deposit");
        nokia.drawBitmap(70, 13, BOTTLE_ICON, 8, 8, BLACK);

        nokia.setCursor(2, 25);
        nokia.print(currentMenu->currentItem == 1 ? ">" : " ");
        nokia.print("Redeem");
        nokia.drawBitmap(70, 23, COIN_ICON, 8, 8, BLACK);

        nokia.setCursor(2, 35);
        nokia.print(currentMenu->currentItem == 2 ? ">" : " ");
        nokia.print("Settings");
        nokia.drawBitmap(70, 33, SETTINGS_ICON, 8, 8, BLACK);
    } else if (currentMenu == &settingsMenu) {
        // Settings menu with icons
        for (size_t i = 0; i < currentMenu->itemCount; i++) {
            nokia.setCursor(2, 15 + (i * 10));
            nokia.print(currentMenu->currentItem == i ? ">" : " ");
            nokia.print(currentMenu->items[i].name);
            
            // Add icons for settings menu items
            if (i == 0) { // Contrast
                nokia.drawBitmap(70, 13, SETTINGS_ICON, 8, 8, BLACK);
            } else if (i == 1) { // Brightness
                nokia.drawBitmap(70, 23, SETTINGS_ICON, 8, 8, BLACK);
            }
        }
    } else {
        // Post deposit menu
        for (size_t i = 0; i < currentMenu->itemCount; i++) {
            nokia.setCursor(2, 15 + (i * 10));
            nokia.print(currentMenu->currentItem == i ? ">" : " ");
            nokia.print(currentMenu->items[i].name);
        }
    }
    
    nokia.display();
}

void updateProgressDisplay(const String& message1, const String& message2, int progress) {
    String progressBar = "[";
    for (int i = 0; i < 10; i++) {
        progressBar += (i < progress / 10) ? "=" : " ";
    }
    progressBar += "]";
    
    updateDualDisplayStatus(message1, message2 + " " + progressBar);
}
void pulseColor(int redValue, int greenValue, int blueValue) {
    for (int brightness = 0; brightness <= 255; brightness += 5) {
        analogWrite(PIN_RED, (redValue * brightness) / 255);
        analogWrite(PIN_GREEN, (greenValue * brightness) / 255);
        analogWrite(PIN_BLUE, (blueValue * brightness) / 255);
        delay(10);
    }
    for (int brightness = 255; brightness >= 0; brightness -= 5) {
        analogWrite(PIN_RED, (redValue * brightness) / 255);
        analogWrite(PIN_GREEN, (greenValue * brightness) / 255);
        analogWrite(PIN_BLUE, (blueValue * brightness) / 255);
        delay(10);
    }
}

void ledStatusCode(int errorCode) {
    switch (errorCode) {
        case 200: // Green Indicator = Ok
            analogWrite(PIN_RED, 0);
            analogWrite(PIN_GREEN, 255);
            analogWrite(PIN_BLUE, 0);
            break;
   //     case 201:
        case 102: // Orange indicator = Processing
            pulseColor(255, 60, 5);
            break;
        case 404: // Red indicator = error
            pulseColor(255, 0, 0);
            break;
    }
}

void delayWithMsg(unsigned long duration, String message1, String message2, int statusCode) {
    unsigned long startTime = millis();
    
    // Select appropriate icon based on status code
    const unsigned char* icon = nullptr;
    switch (statusCode) {
        case 200: icon = BOTTLE_ICON; break;
        case 404: icon = ERROR_ICON; break;
        case 102: icon = COIN_ICON; break;
    }
    
    updateDualDisplayStatus(message1, message2, icon);
    
    while (millis() - startTime < duration) {
        ledStatusCode(statusCode);
    }
}
int getCapacitiveSensorValue() {
    long sum = 0;
    
    // Take multiple samples to reduce noise
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += analogRead(CAPACITIVE_SENSOR_PIN);
        delay(2);  // Short delay between readings for stability
    }
    
    int averageValue = sum / SAMPLE_COUNT;
    
    if (DEBUG_SENSORS) {
        Serial.print("Raw Capacitive Value: ");
        Serial.println(averageValue);
    }
    
    return averageValue;
}

// Modified sensor reading functions with debugging
// Check if bottle is detected with debouncing and hysteresis
bool readCapacitiveSensorData() {
    // Debounce check
    if (millis() - capacitiveSensor.lastReadTime < DEBOUNCE_MS) {
        return capacitiveSensor.isDetecting;
    }

    capacitiveSensor.lastReadTime = millis();
    int currentValue = getCapacitiveSensorValue();
    
    // Update detection state with hysteresis to prevent flickering
    if (currentValue >= DETECTION_THRESHOLD) {
        capacitiveSensor.isDetecting = true;
        capacitiveSensor.lastStableValue = currentValue;
    } 
    else if (currentValue < NO_BOTTLE_THRESHOLD) {
        capacitiveSensor.isDetecting = false;
        capacitiveSensor.lastStableValue = currentValue;
    }
    
    if (DEBUG_SENSORS) {
        Serial.print("Capacitive Value: ");
        Serial.print(currentValue);
        Serial.print(" | State: ");
        Serial.println(capacitiveSensor.isDetecting ? "BOTTLE DETECTED" : "NO BOTTLE");
    }

    return capacitiveSensor.isDetecting;
}
int readInductiveSensorData() {
    delay(50);  // Short delay for sensor stabilization
    int reading = digitalRead(inductiveSensorPin);
    if (DEBUG_SENSORS) {
        Serial.print("Inductive Reading: ");
        Serial.println(reading);
    }
    return reading;
}
bool isBottleFullyRemoved() {
    int currentValue = getCapacitiveSensorValue();
    return currentValue < NO_BOTTLE_THRESHOLD;
}

void testCapacitiveSensor() {
    Serial.println("\n=== Testing Analog Capacitive Sensor ===");
    Serial.println("Place and remove a plastic bottle multiple times");
    Serial.println("Detection Threshold: " + String(DETECTION_THRESHOLD));
    Serial.println("No Bottle Threshold: " + String(NO_BOTTLE_THRESHOLD));
    Serial.println("Testing for 30 seconds...");
    
    unsigned long startTime = millis();
    int sampleCount = 0;
    int maxReading = 0;
    int minReading = 1023;
    
    while (millis() - startTime < 30000) {
        int rawValue = getCapacitiveSensorValue();
        bool isDetecting = readCapacitiveSensorData();
        
        // Update min/max values
        maxReading = max(maxReading, rawValue);
        minReading = min(minReading, rawValue);
        
        Serial.println("\nReading #" + String(++sampleCount));
        Serial.println("Raw Value: " + String(rawValue));
        Serial.println("Status: " + String(isDetecting ? "BOTTLE DETECTED" : "NO BOTTLE"));
        
        delay(1000);  // Update every second
    }
    
    Serial.println("\n=== Test Results ===");
    Serial.println("Samples Taken: " + String(sampleCount));
    Serial.println("Minimum Reading: " + String(minReading));
    Serial.println("Maximum Reading: " + String(maxReading));
    Serial.println("Current Thresholds:");
    Serial.println("- Detection: " + String(DETECTION_THRESHOLD));
    Serial.println("- No Bottle: " + String(NO_BOTTLE_THRESHOLD));
}
void setupCapacitiveSensor() {
    pinMode(CAPACITIVE_SENSOR_PIN, INPUT);
    capacitiveSensor = CapacitiveSensorState(); // Initialize using constructor
    
    if (DEBUG_SENSORS) {
        testCapacitiveSensor();
    }
}
// Add this to your setup() function
void sensorSetup() {
   // pinMode(capacitiveSensorPin, INPUT);
    pinMode(inductiveSensorPin, INPUT);
    
    // Serial.println("Starting sensor test...");
    // testSensors();
    // Serial.println("Sensor test complete");
}

void rotateServo(Servo &servo, int angle) {
    servo.write(angle);
}


void openCloseBinLid(int lidNum, bool toOpen) {
    int angle = toOpen ? 90 : 0;
    Servo &servo = (lidNum == 1) ? servo1 : servo2;
    rotateServo(servo, angle);
}
void waitToRemoveObject() {
    lcd.clear();
    lcd.print("Remove Bottle!");
    openCloseBinLid(1, true);
    
    unsigned long startTime = millis();
    while (!isBottleFullyRemoved()) {
        ledStatusCode(404);
        
        // Timeout after 10 seconds
        if (millis() - startTime > 10000) {
            lcd.clear();
            lcd.print("Timeout!");
            break;
        }
        delay(100);
    }
    
    isObjectInside = false;
    delay(2000);
    openCloseBinLid(1, false);
    ledStatusCode(200);
}
bool verifyObject() {
    if (!isObjectInside) {
        return false;
    }

    // First notify user and close lid
    delayWithMsg(3000, "Lid is closing...", "Remove hand!!!", 404);
    openCloseBinLid(1, false);
    delay(1000);  // Give time for lid to close and readings to stabilize

    unsigned long startTime = millis();
    bool verificationComplete = false;
    bool verificationResult = false;

    // Main verification loop
    while (!verificationComplete && (millis() - startTime < 3000)) {  // 5 second timeout
        ledStatusCode(102);
        lcd.clear();
        lcd.print("Verifying....");
        
        // Get sensor readings
        bool capacitiveReading = readCapacitiveSensorData();
        int inductiveReading = readInductiveSensorData();
        
        // Debug output
        if (DEBUG_SENSORS) {
            Serial.println("Verification in progress:");
            Serial.println("Capacitive: " + String(capacitiveReading));
            Serial.println("Inductive: " + String(inductiveReading));
        }

        // Check if both sensors detect correctly
        if (getCapacitiveSensorValue() >= DETECTION_THRESHOLD) {
            lcd.clear();
            lcd.print("Verified!");
            ledStatusCode(200);
            
            // Open second lid to drop bottle
            openCloseBinLid(2, true);
            delay(3000);
            openCloseBinLid(2, false);
            
            verificationResult = true;
            verificationComplete = true;
        } 
        // If we've waited at least 2 seconds and verification failed
        else if (millis() - startTime >= 2000) {
            lcd.clear();
            lcd.print("Invalid object");
            ledStatusCode(404);
            
            // Handle invalid object
            waitToRemoveObject();
            delayWithMsg(3000, "Lid closing", "remove hand", 404);
            openCloseBinLid(1, false);
            
            verificationResult = false;
            verificationComplete = true;
        }
        
        delay(100);  // Small delay to prevent tight loop
    }

    // Handle timeout case
    if (!verificationComplete) {
        lcd.clear();
        lcd.print("Verification");
        lcd.setCursor(0, 1);
        lcd.print("timeout!");
        ledStatusCode(404);
        waitToRemoveObject();
        return false;
    }

    return verificationResult;
}
void navigateMenu(int direction) {
    currentMenu->currentItem = (currentMenu->currentItem + direction + currentMenu->itemCount) % currentMenu->itemCount;
    updateMenuDisplay();
    delay(200);
    while (digitalRead(direction > 0 ? downButton : upButton) == LOW);
}

void selectMenuItem() {
    currentMenu->items[currentMenu->currentItem].action();
    updateMenuDisplay();
    delay(200);
    while (digitalRead(selectButton) == LOW);
}
// Example of how to use in waitForObjectPresence
void waitForObjectPresence() {
    unsigned long startTime = millis();
    bool objectDetected = false;
    
    while (!objectDetected) {
        ledStatusCode(102);
        
        if (readCapacitiveSensorData()) {
            // Additional confirmation check to avoid false positives
            delay(100);  // Short delay for stability
            if (readCapacitiveSensorData()) {  // Double-check the reading
                objectDetected = true;
                break;
            }
        }
        
        if (millis() - startTime >= 3000) {
            ledStatusCode(404);
            delayWithMsg(2000, "No bottle", "detected!", 404);
            delayWithMsg(2000, "Lid closing", "remove hand", 404);
            openCloseBinLid(1, false);
            ledStatusCode(200);
            isObjectInside = false;
            return;
        }
    }
    
    Serial.println("Bottle detected!");
    isObjectInside = true;
}

void depositAction() {
    if (maintenanceMode) {
        displayNokiaStatus("System Full", ERROR_ICON);
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }
    
    displayNokiaStatus("Ready for Bottle", BOTTLE_ICON);
    lcd.clear();
    lcd.print("Opening bin....");
    openCloseBinLid(1, true);
    
    displayNokiaStatus("Insert Bottle", BOTTLE_ICON);
    lcd.clear();
    lcd.print("Insert Bottle!");
    
    waitForObjectPresence();
    if (verifyObject()) {
        totalPoints++;
        displayNokiaStatus("Success!", BOTTLE_ICON);
        lcd.clear();
        lcd.print("Deposit success!");
        lcd.setCursor(0, 1);
        lcd.print("Points: ");
        lcd.print(totalPoints);
        delay(2000);
        currentMenu = &postDepositMenu;
        updateMenuDisplay();
    }
}
void insertAnotherBottleAction() {
    depositAction();
}

//RFID Functions
void redeemAction() {
    if (maintenanceMode) {
        displayNokiaStatus("System Full", ERROR_ICON);
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }
    
    displayNokiaStatus("Present Card", CARD_ICON);
    lcd.clear();
    lcd.print("Present RFID card");

    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        if (detectCard()) {
            int currentPoints = readPoints();
            if (currentPoints >= 0) {
                totalPoints = currentPoints;
                pointsToRedeem = 0;
                redeemPointsAction();
                mfrc522.PICC_HaltA();
                mfrc522.PCD_StopCrypto1();
                return;
            }
        }
        delay(100);
    }
    displayNokiaStatus("No Card Found", ERROR_ICON);
    delayWithMsg(2000, "No card detected", "", 404);
}

void redeemPointsAction() {
    lcd.clear();
    lcd.print("Redeem Points");
    lcd.setCursor(0, 1);
    lcd.print("Points: ");
    lcd.print(pointsToRedeem);

    unsigned long lastButtonPress = 0;
    int holdTime = 0;
    const int maxRedeemable = min(totalPoints, 20);
    delay(1000);
    while (true) {
        lcd.setCursor(8, 1);
        lcd.print("    ");
        lcd.setCursor(8, 1);
        lcd.print(pointsToRedeem);

        if (digitalRead(upButton) == LOW || digitalRead(downButton) == LOW) {
            if (millis() - lastButtonPress > 200) {
                int direction = (digitalRead(upButton) == LOW) ? 1 : -1;
                pointsToRedeem += direction * (1 + (holdTime / 1000));
                pointsToRedeem = constrain(pointsToRedeem, 0, maxRedeemable);
                lastButtonPress = millis();
                holdTime += 200;
            }
        } else {
            holdTime = 0;
        }

        if (digitalRead(selectButton) == LOW) {
            if (millis() - lastButtonPress > 2000) {
                delayWithMsg(2000, "Redemption", "Cancelled", 404);
                pointsToRedeem = 0;
                currentMenu = &mainMenu;
                updateMenuDisplay();
                break;
            } else {
                totalPoints -= pointsToRedeem;
                if (writePoints(totalPoints)) {
                    delayWithMsg(2000, "Redeeming: " + String(pointsToRedeem), "Please wait...", 200);
                    dispenseCoin(pointsToRedeem);  // Dispense coins equal to pointsToRedeem
                    delayWithMsg(2000, "Redeemed: " + String(pointsToRedeem), "Remaining: " + String(totalPoints), 200);
                } else {
                    delayWithMsg(2000, "Failed to update", "card. Try again.", 404);
                }
                currentMenu = &mainMenu;
                updateMenuDisplay();
                break;
            }
        }

        delay(50);
    }
}

void setUpRFID() {
    SPI.begin();
    mfrc522.PCD_Init();
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }
    Serial.println("RFID System ready. Present a card.");
}

bool detectCard() {
    return mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();
}

int readPoints() {
    byte buffer[18];
    byte size = sizeof(buffer);

    MFRC522::StatusCode status = authenticateBlock(POINTS_BLOCK);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Authentication Failed.");
        return -1;
    }

    status = mfrc522.MIFARE_Read(POINTS_BLOCK, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Reading failed.");
        return -1;
    }
    int points = (buffer[0] << 8) | buffer[1]; // Combine two bytes into an int
    return points;
}

bool writePoints(int points) {
    byte buffer[16];
    buffer[0] = (points >> 8) & 0xFF; // High byte
    buffer[1] = points & 0xFF;        // Low byte

    MFRC522::StatusCode status = authenticateBlock(POINTS_BLOCK);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Authentication failed");
        return false;
    }

    status = mfrc522.MIFARE_Write(POINTS_BLOCK, buffer, 16);
    if (status != MFRC522::STATUS_OK) {
        Serial.println("Writing failed");
        return false;
    }
    Serial.println("Points updated successfully");
    return true;
}

MFRC522::StatusCode authenticateBlock(int blockNumber) {
    return mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                    blockNumber, &key, &(mfrc522.uid));
}
void storePointsAction() {
    lcd.clear();
    lcd.print("Present RFID Card");
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        if (detectCard()) {
            if (writePoints(totalPoints)) {
                delayWithMsg(2000, "Points stored!", "Total: " + String(totalPoints), 200);
            } else {
                delayWithMsg(2000, "Failed to store", "points. Try again.", 404);
            }
            mfrc522.PICC_HaltA();
            mfrc522.PCD_StopCrypto1();
            currentMenu = &mainMenu;
            updateMenuDisplay();
            totalPoints = 0;
            return;
        }
        delay(100);
    }
    delayWithMsg(2000, "No card detected", "", 404);
    currentMenu = &mainMenu;
    updateMenuDisplay();
}


void setup() {
   if (!setupNokiaDisplay()) {
        // Handle Nokia display initialization failure
        Serial.println(F("Nokia display initialization failed. System halted."));
        while (1) { delay(1000); } // Safety halt
    }
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    pinMode(upButton, INPUT_PULLUP);
    pinMode(downButton, INPUT_PULLUP);
    pinMode(selectButton, INPUT_PULLUP);
    pinMode(CAPACITIVE_SENSOR_PIN, INPUT);
    pinMode(inductiveSensorPin, INPUT);
    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(LED_INLET_PIN, OUTPUT);
    setupCapacitiveSensor(); 

    ledStatusCode(200);

    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    openCloseBinLid(1, false);
    openCloseBinLid(2, false);

    updateMenuDisplay();
    Serial.begin(9600);
    setUpRFID();
    sim800lv2.begin(9600);
    delay(1000);
    
    sendSMS("PISO-BOTE system initialized");

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(-349.994118);
    scale.tare();
     pinMode(coinHopperSensor_PIN, INPUT_PULLUP);  // Use internal pull-up resistor
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW); 
     pinMode(PIN_BL, OUTPUT);
    analogWrite(PIN_BL, 255); 

    Serial.println("Testing servos...");
    
    Serial.println("Testing servo 1");
    openCloseBinLid(1, true);
    delay(1000);
    openCloseBinLid(1, false);
    
    Serial.println("Testing servo 2");
    openCloseBinLid(2, true);
    delay(1000);
    openCloseBinLid(2, false);
    
    Serial.println("Servo test complete");
    Serial.print("Capactive: ");
    Serial.println(readCapacitiveSensorData());

    
}

void loop() {
    if (maintenanceMode) {
        handleMaintenanceMode();
        return;
    }
    
    if (isBinFull()) {
        maintenanceMode = true;
        displayNokiaStatus("System Full", ERROR_ICON);
        return;
    }
    
    if (digitalRead(downButton) == LOW) {
        navigateMenu(1);
    }
    if (digitalRead(upButton) == LOW) {
        navigateMenu(-1);
    }
    if (digitalRead(selectButton) == LOW) {
        selectMenuItem();
    }
      Serial.println(readCapacitiveSensorData());
    delay(100);
}


void sendSMS(String message) {
    sim800lv2.println("AT+CMGF=1");
    delay(1000);
    sim800lv2.println("AT+CMGS=\"" + maintainerNum + "\"");
    delay(1000);
    sim800lv2.println(message);
    delay(100);
    sim800lv2.println((char)26);
    delay(1000);
    Serial.println("Owner has been notified!");
}

bool isBinFull() {
    unsigned int duration = sonar.ping_median(ITERATIONS);
    float distance = (duration / 2.0) * 0.0343;
    if (distance > 0 && distance <= 5) {
        Serial.println("Bin full! Entering maintenance mode...");
        sendSMS("Alert: The PISO-BOTE is full! Please empty it.");
        ledStatusCode(404); 
        return true;
    }
    return false;
}

void handleMaintenanceMode() {
    lcd.clear();
    lcd.print("PISO-BOTE is full");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");

    while (isBinFull()) {
        ledStatusCode(404);
        delay(5000);
    }
    maintenanceMode = false; 
    lcd.clear();
    lcd.print("PISO-BOTE ready");
    ledStatusCode(200); 
    delay(2000);
}

bool isWeightAcceptable() {
    float weight = scale.get_units(10);
    return true; //weight <= MAX_WEIGHT;
     //TODO: change to tjje commented code
}

void calibrateLoadCell() {
    lcd.clear();
    lcd.print("Calibrating...");
    lcd.setCursor(0, 1);
    lcd.print("Remove all weight");
    delay(5000);
    
    scale.tare();
    
    lcd.clear();
    lcd.print("Place known");
    lcd.setCursor(0, 1);
    lcd.print("weight (e.g. 100g)");
    delay(5000);
    
    float knownWeight = 100.0;  // Adjust this to your actual calibration weight
    float reading = scale.get_units(10);
    float calibrationFactor = reading / knownWeight;
    
    scale.set_scale(calibrationFactor);
    
    lcd.clear();
    lcd.print("Calibration done");
    delay(2000);
}

int readLDRSensorData() {
    return analogRead(LDR_PIN);
}

void controlLedInlet(bool isOn) {
    digitalWrite(LED_INLET_PIN, isOn ? HIGH : LOW);
}

bool isObjectClear() {
    if (isObjectInside) {
        delay(100);
        int lightValue = readLDRSensorData();
        delay(100);
        controlLedInlet(false);
        return lightValue > 200;
    }
    return true;
    //TODO: change to false
}
void dispenseCoin(int count) {
    coinCount = 0;
    dispensingActive = true;
    digitalWrite(relayPin, HIGH);  
    
    lcd.clear();
    lcd.print("Dispensing coins");
    lcd.setCursor(0, 1);
    lcd.print("Count: 0");

    unsigned long startTime = millis();  

    while (dispensingActive) {
     
        if (millis() - startTime > 60000) {
            lcd.clear();
            lcd.print("Dispensing error");
            lcd.setCursor(0, 1);
            lcd.print("Please contact staff");
            delay(3000);
            break;  
        }

        bool currentSensorState = digitalRead(coinHopperSensor_PIN);

        
        if (currentSensorState == LOW && lastSensorState == HIGH) {
            coinCount++;
            lcd.setCursor(7, 1);
            lcd.print(coinCount);
            Serial.print("Coin dispensed! Count: ");
            Serial.println(coinCount);

            if (coinCount >= count) {
                digitalWrite(relayPin, LOW);  
                dispensingActive = false;
            }
        }

        lastSensorState = currentSensorState;
        delay(10);  
    }

    digitalWrite(relayPin, LOW);  

    if (coinCount == count) {
        lcd.clear();
        lcd.print("Dispensing done");
    } else {
        lcd.clear();
        lcd.print("Incomplete dispense");
        lcd.setCursor(0, 1);
        lcd.print("Coins: " + String(coinCount) + "/" + String(count));
    }
    delay(2000);
}

void debugSensorReadings() {
    if (DEBUG_SENSORS) {
        Serial.println("\n=== Sensor Readings ===");
        
        // Capacitive sensor
        int capValue = getCapacitiveSensorValue();
        bool isDetecting = readCapacitiveSensorData();
        Serial.print("Capacitive Raw Value: ");
        Serial.print(capValue);
        Serial.print(" | Detecting: ");
        Serial.println(isDetecting ? "YES" : "NO");
        
        // Inductive sensor
        int indValue = readInductiveSensorData();
        Serial.print("Inductive Value: ");
        Serial.println(indValue == HIGH ? "HIGH" : "LOW");
        
        Serial.println("====================\n");
    }
}