#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <HX711.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Constant Variables
//const int POINTS_BLOCK = 4;
const int MAX_DISTANCE = 20;
const int ITERATIONS = 5;
const float MAX_WEIGHT = 500.00;

// Constant Var Pin Definitions
const int upButton = 40;
const int downButton = 41;
const int selectButton = 42;
// Servo
const int servoPin1 = 36;
const int servoPin2 = 37;
// Sensors

// Capacitive sensor configuration for analog reading
const int CAPACITIVE_SENSOR_PIN = A0; // Analog pin for capacitive sensor
const int DETECTION_THRESHOLD = 650;  // Threshold for bottle detection (adjust if needed)
const int NO_BOTTLE_THRESHOLD = 500;  // Threshold for confirming bottle removal
const int HYSTERESIS = 50;            // Prevent flickering
const int SAMPLE_COUNT = 5;           // Number of readings to average
const int DEBOUNCE_MS = 50;           // Minimum time between readings
const int inductiveSensorPin = 15;

// Global state for the capacitive sensor

struct CapacitiveSensorState
{
    bool isDetecting;
    unsigned long lastReadTime;
    int lastStableValue;

    // Constructor
    CapacitiveSensorState()
    {
        isDetecting = false;
        lastReadTime = 0;
        lastStableValue = 0;
    }
};
CapacitiveSensorState capacitiveSensor;
// Ultasonic Sensor
const int TRIGGER_PIN = 22;
const int ECHO_PIN = 23;
// RGB LED
const int PIN_RED = 32;
const int PIN_GREEN = 33;
const int PIN_BLUE = 34;
// RFID
const int SS_PIN = 53;
const int RST_PIN = 49;
// SIM Module
const int SIM_RX = 10;
const int SIM_TX = 11;
// Load Cell
const int LOADCELL_DOUT_PIN = 44;
const int LOADCELL_SCK_PIN = 45;
const float CALIBRATION_FACTOR = -350.82; // Fixed calibration factor
const float MIN_ACCEPTABLE_WEIGHT = 10.0; // Minimum weight in grams
const float MAX_ACCEPTABLE_WEIGHT = 65.0; // Maximum weight in grams
const int READINGS_COUNT = 5;             // Number of readings to average
const float STABILITY_THRESHOLD = 1.0;    // Maximum variation between readings
const int READING_DELAY = 100;
// LDR & LED
const int LDR_PIN = A1;
const int LED_INLET_PIN = 12;
// Coin Hopper
const int coinHopperSensor_PIN = 28;
const int relayPin = 30;
const int COIN_DISPENSE_TIMEOUT = 60000; // 60 second timeout
const int SENSOR_DEBOUNCE_DELAY = 10;    // 10ms debounce delay

// Nokia 5110 LCD
const int PIN_RST = 3; // RST (with 10kΩ resistor)    YELLOW
const int PIN_CE = 4;  // CE (with 1kΩ resistor)      ORANGE
const int PIN_DC = 5;  // DC (with 10kΩ resistor)     GREEN
const int PIN_DIN = 6; // DIN (with 10kΩ resistor)    BLUE
const int PIN_CLK = 7; // CLK (with 10kΩ resistor)    PURPLE
const int PIN_BL = 13; // Backlight with 330Ω resistor   WHITE

// For setting Nokia Display
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
    0b00111100};

static const unsigned char PROGMEM COIN_ICON[] = {
    0b00111100,
    0b01111110,
    0b11100111,
    0b11000011,
    0b11000011,
    0b11100111,
    0b01111110,
    0b00111100};

static const unsigned char PROGMEM CARD_ICON[] = {
    0b11111111,
    0b10000001,
    0b10111101,
    0b10100101,
    0b10111101,
    0b10000001,
    0b11111111,
    0b00000000};

static const unsigned char PROGMEM ERROR_ICON[] = {
    0b00111100,
    0b01000010,
    0b10100101,
    0b10011001,
    0b10011001,
    0b10100101,
    0b01000010,
    0b00111100};
static const unsigned char PROGMEM SETTINGS_ICON[] = {
    0b00011000,
    0b00111100,
    0b11111111,
    0b11100111,
    0b11100111,
    0b11111111,
    0b00111100,
    0b00011000};

// Global Variables
volatile int coinCount = 0;
volatile bool dispensingActive = false;
enum DispenserState
{
    IDLE,
    DISPENSING,
    ERROR,
    COMPLETE
};
volatile DispenserState dispenserState = IDLE;

bool lastSensorState = HIGH;
bool isObjectInside = false;
int totalPoints = 0;
int pointsToRedeem = 0;
bool maintenanceMode = false;
String maintainerNum = "+639932960906";
const int MAX_RFID_INIT_ATTEMPTS = 3;
const int RFID_RESET_DELAY = 50;
const byte POINTS_BLOCK = 1;      // Data block for storing points
const byte SECTOR_NUM = 1;        // Using sector 1 (blocks 4-7)
const byte TRAILER_BLOCK = 7;     // Sector 1 trailer block
const int MAX_POINTS = 999;       // Maximum allowed points

String previousLcdLine1;
String previousLcdLine2;
String previousNokiaMessage;
const bool DEBUG_SENSORS = true;       // Set to true to enable sensor debugging
const int SENSOR_STABILIZE_TIME = 500; // Time to wait for sensor readings to stabilize

// Object instantations
Adafruit_PCD8544 nokia(PIN_CLK, PIN_DIN, PIN_DC, PIN_CE, PIN_RST);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2;
MFRC522 mfrc522(SS_PIN, RST_PIN);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial sim800lv2(10, 11);
HX711 scale;
MFRC522::MIFARE_Key key;

// Function Prototypes
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

void displayNokiaStatus(const String &message, const unsigned char *icon = nullptr);
void updateMenuDisplay();
void displayMainMenu();
void settingsAction();
void adjustContrastAction();
void adjustBrightnessAction();
void backToMainAction();
void debugSensorReadings();
void testRFIDCommunication();
void postDepositRedeemAction();
// Menu structure
struct MenuItem
{
    const char *name;
    void (*action)();
};

struct Menu
{
    MenuItem *items;
    int itemCount;
    int currentItem;
    const char *title;
};
// Menu definitions
MenuItem mainMenuItems[] = {
    {"Deposit", depositAction},
    {"Redeem", redeemAction},
    {"Settings", settingsAction}};

MenuItem settingsMenuItems[] = {
    {"Contrast", adjustContrastAction},
    {"Brightness", adjustBrightnessAction},
    {"Back", backToMainAction}};

MenuItem postDepositMenuItems[] = {
    {"Insert", insertAnotherBottleAction},
    {"Redeem", postDepositRedeemAction},  // Use the new direct redemption function
    {"Store", storePointsAction}
};

// Menu structures - KEEP THESE
Menu mainMenu = {mainMenuItems, 3, 0, "Main Menu"};
Menu settingsMenu = {settingsMenuItems, 3, 0, "Settings"};
Menu postDepositMenu = {postDepositMenuItems, 3, 0, "Options"};
Menu *currentMenu = &mainMenu;
struct DisplayState
{
    bool nokiaDisplayActive;
    bool lcdDisplayActive;
    unsigned long lastUpdateTime;
    int contrast;
    byte backlight;
} displayState;

// Function Implementation
void postDepositRedeemAction()
{
    // Check if there are any points to redeem
    if (totalPoints <= 0) {
        delayWithMsg(2000, "No points to", "redeem!", 404);
        return;
    }

    lcd.clear();
    lcd.print("Redeem Points");
    lcd.setCursor(0, 1);
    lcd.print("Points: ");
    lcd.print(totalPoints);

    // Simple confirmation delay with option to cancel
    delayWithMsg(2000, "Redeeming " + String(totalPoints), "Hold SELECT to cancel", 102);
    
    // Check if user wants to cancel (holding select button)
    unsigned long startTime = millis();
    while (digitalRead(selectButton) == LOW) {
        if (millis() - startTime > 2000) {  // 2-second hold to cancel
            delayWithMsg(2000, "Redemption", "Cancelled", 404);
            currentMenu = &mainMenu;
            updateMenuDisplay();
            return;
        }
        delay(50);
    }

    // Proceed with redemption
    int pointsToDispense = totalPoints;
    totalPoints = 0;  // Clear the points since we're dispensing all

    delayWithMsg(2000, "Redeeming: " + String(pointsToDispense), "Please wait...", 200);
    dispenseCoin(pointsToDispense);
    delayWithMsg(2000, "Redeemed: " + String(pointsToDispense), "Thank you!", 200);
    
    currentMenu = &mainMenu;
    updateMenuDisplay();
}
// Add this helper function to initialize a new card
bool initializeCard() {
    Serial.println(F("Initializing new card..."));
    
    // Authenticate
    MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        POINTS_BLOCK,
        &key,
        &(mfrc522.uid)
    );
    
    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Authentication failed for initialization"));
        return false;
    }
    
    // Initialize with 0 points
    byte buffer[16] = {0};
    status = mfrc522.MIFARE_Write(POINTS_BLOCK, buffer, 16);
    
    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Failed to initialize card"));
        return false;
    }
    
    Serial.println(F("Card initialized successfully"));
    return true;
}
void setupCoinHopper()
{
    // Configure pins with proper pullup/pulldown
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW); // Ensure relay starts OFF

    pinMode(coinHopperSensor_PIN, INPUT_PULLUP);

    // Add initial delay for system stabilization
    delay(100);
}
// Add atomic operations for sensor state changes
bool updateSensorState(bool newState)
{
    noInterrupts(); // Disable interrupts temporarily
    bool oldState = capacitiveSensor.isDetecting;
    capacitiveSensor.isDetecting = newState;
    interrupts(); // Re-enable interrupts
    return oldState;
}
bool initializeRFID()
{
    // Power cycle the RFID module by toggling RST pin
    digitalWrite(RST_PIN, LOW);
    delay(RFID_RESET_DELAY);
    digitalWrite(RST_PIN, HIGH);
    delay(RFID_RESET_DELAY);

    int attempts = 0;
    bool initialized = false;

    while (!initialized && attempts < MAX_RFID_INIT_ATTEMPTS)
    {
        SPI.begin();
        delay(50); // Give SPI time to stabilize

        mfrc522.PCD_Init();
        delay(100); // Give the module time to complete initialization

        // Test if initialization was successful by reading version
        byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);

        if (version == 0x91 || version == 0x92)
        { // Known good versions
            initialized = true;

            // Set antenna gain to maximum
            mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

            // Initialize the key
            for (byte i = 0; i < 6; i++)
            {
                key.keyByte[i] = 0xFF;
            }

            Serial.println(F("RFID initialized successfully"));
            Serial.print(F("Version: 0x"));
            Serial.println(version, HEX);
        }
        else
        {
            attempts++;
            Serial.print(F("RFID init attempt "));
            Serial.print(attempts);
            Serial.println(F(" failed"));
            delay(500); // Wait before retry
        }
    }

    return initialized;
}
// Add this function to verify RFID is working
void testRFIDCommunication()
{
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);

    // Check if the antenna is working by trying to detect a card
    mfrc522.PCD_AntennaOn();
    delay(100); // Give the antenna time to power up

    bool antennaOk = mfrc522.PCD_GetAntennaGain() > 0;
    if (!antennaOk)
    {
        Serial.println(F("RFID antenna error!"));
        return;
    }

    Serial.println(F("RFID system ready"));
}

void settingsAction()
{
    currentMenu = &settingsMenu;
    updateMenuDisplay();
}

void backToMainAction()
{
    currentMenu = &mainMenu;
    updateMenuDisplay();
}

// Functions for Settings
void adjustContrastAction()
{
    int currentContrast = displayState.contrast;
    if (currentContrast < MIN_CONTRAST)
        currentContrast = MIN_CONTRAST;
    if (currentContrast > MAX_CONTRAST)
        currentContrast = MAX_CONTRAST;

    bool adjusting = true;
    unsigned long lastButtonPress = 0;

    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(10, 1);
    nokia.print("Contrast");
    ;
    ;
    nokia.drawLine(0, 12, 84, 12, BLACK);

    while (adjusting)
    {
        if (millis() - lastButtonPress > 100)
        {
            if (digitalRead(upButton) == LOW && currentContrast < MAX_CONTRAST)
            {
                currentContrast += CONTRAST_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(downButton) == LOW && currentContrast > MIN_CONTRAST)
            {
                currentContrast -= CONTRAST_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(selectButton) == LOW)
            {
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
void adjustBrightnessAction()
{
    int currentBrightness = displayState.backlight;
    if (currentBrightness < MIN_BRIGHTNESS)
        currentBrightness = MIN_BRIGHTNESS;
    if (currentBrightness > MAX_BRIGHTNESS)
        currentBrightness = MAX_BRIGHTNESS;

    bool adjusting = true;
    unsigned long lastButtonPress = 0;

    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(8, 1);
    nokia.print("Brightness");
    nokia.drawLine(0, 12, 84, 12, BLACK);

    while (adjusting)
    {
        if (millis() - lastButtonPress > 100)
        {
            if (digitalRead(upButton) == LOW && currentBrightness < MAX_BRIGHTNESS)
            {
                currentBrightness += BRIGHTNESS_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(downButton) == LOW && currentBrightness > MIN_BRIGHTNESS)
            {
                currentBrightness -= BRIGHTNESS_STEP;
                lastButtonPress = millis();
            }
            if (digitalRead(selectButton) == LOW)
            {
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
void displayMainMenu()
{
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
bool setupNokiaDisplay()
{
    nokia.begin();
    nokia.setContrast(50);
    nokia.setRotation(2);
    nokia.clearDisplay();
    nokia.setTextSize(1);
    nokia.setTextColor(BLACK);
    analogWrite(PIN_BL, HIGH); // Use analogWrite instead of digitalWrite

    // Initialize display state
    displayState.nokiaDisplayActive = true;
    displayState.lcdDisplayActive = true;
    displayState.contrast = 50;
    displayState.backlight = 255; // Full brightness initially
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
void setDisplayPower(bool nokiaOn, bool lcdOn)
{
    if (nokiaOn != displayState.nokiaDisplayActive)
    {
        if (nokiaOn)
        {
            nokia.begin();
            nokia.setRotation(2);                        // Maintain rotation
            nokia.setContrast(displayState.contrast);    // Maintain contrast
            analogWrite(PIN_BL, displayState.backlight); // Use stored backlight value
        }
        else
        {
            nokia.clearDisplay();
            nokia.display();
            analogWrite(PIN_BL, LOW);
        }
        displayState.nokiaDisplayActive = nokiaOn;
    }

    if (lcdOn != displayState.lcdDisplayActive)
    {
        if (lcdOn)
        {
            lcd.backlight();
        }
        else
        {
            lcd.noBacklight();
        }
        displayState.lcdDisplayActive = lcdOn;
    }
}
void displayNokiaStatus(const String &message, const unsigned char *icon)
{
    nokia.clearDisplay();

    if (icon != nullptr)
    {
        nokia.drawBitmap(38, 8, icon, 8, 8, BLACK);
        nokia.drawLine(0, 20, 84, 20, BLACK);
        nokia.setCursor(0, 25);
    }
    else
    {
        nokia.setCursor(0, 15);
    }

    nokia.print(message);
    nokia.display();
}
// Modified updateDualDisplayStatus function
void updateDualDisplayStatus(const String &message1, const String &message2, const unsigned char *icon = nullptr)
{
    const int LCD_WIDTH = 16;
    String lcd1 = message1.length() > LCD_WIDTH ? message1.substring(0, LCD_WIDTH) : message1;
    String lcd2 = message2.length() > LCD_WIDTH ? message2.substring(0, LCD_WIDTH) : message2;

    // Update LCD only if content has changed
    if (lcd1 != previousLcdLine1 || lcd2 != previousLcdLine2)
    {
        lcd.clear();
        lcd.print(lcd1);
        if (lcd2.length() > 0)
        {
            lcd.setCursor(0, 1);
            lcd.print(lcd2);
        }

        previousLcdLine1 = lcd1;
        previousLcdLine2 = lcd2;
    }

    // Nokia display handling with word wrapping
    const int NOKIA_LINE_WIDTH = 14; // Characters that fit on Nokia display
    String nokiaMessage = message1 + (message2.length() > 0 ? "\n" + message2 : "");

    if (nokiaMessage != previousNokiaMessage)
    {
        nokia.clearDisplay();

        if (icon != nullptr)
        {
            nokia.drawBitmap(38, 8, icon, 8, 8, BLACK);
            nokia.drawLine(0, 20, 84, 20, BLACK);
            nokia.setCursor(0, 25);
        }
        else
        {
            nokia.setCursor(0, 15);
        }

        // Word wrapping implementation
        int currentY = icon ? 25 : 15;
        int startPos = 0;

        // Handle message1
        while (startPos < message1.length())
        {
            int endPos = min(startPos + NOKIA_LINE_WIDTH, (int)message1.length());
            if (endPos < message1.length())
            {
                int lastSpace = message1.lastIndexOf(' ', endPos);
                if (lastSpace > startPos)
                {
                    endPos = lastSpace;
                }
            }
            nokia.setCursor(0, currentY);
            nokia.print(message1.substring(startPos, endPos));
            startPos = endPos + 1;
            currentY += 8;
        }

        // Handle message2 if present
        if (message2.length() > 0)
        {
            currentY += 2; // Add spacing between messages
            startPos = 0;
            while (startPos < message2.length())
            {
                int endPos = min(startPos + NOKIA_LINE_WIDTH, (int)message2.length());
                if (endPos < message2.length())
                {
                    int lastSpace = message2.lastIndexOf(' ', endPos);
                    if (lastSpace > startPos)
                    {
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
void resetSystem()
{
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
void additionalSetup()
{
    // Reset system state
    resetSystem();

    // Display welcome message
    delayWithMsg(2000, "Welcome to", "PISO-BOTE", 200);

    // Show main menu
    displayMainMenu();
}
void handleError(const String &message1, const String &message2)
{
    ledStatusCode(404);
    delayWithMsg(2000, message1, message2, 404);
    ledStatusCode(200);
}
void updateMenuDisplay()
{
    // Show static welcome message on LCD
    lcd.clear();
    lcd.print(F("Welcome to"));
    lcd.setCursor(0, 1);
    lcd.print(F("PISO-BOTE"));

    String title = currentMenu->title;

    // Nokia-specific menu layout
    nokia.clearDisplay();
    nokia.drawRect(0, 0, 84, 10, BLACK);
    nokia.setCursor(14, 1);
    nokia.print(title);
    nokia.drawLine(0, 12, 84, 12, BLACK);

    if (currentMenu == &mainMenu)
    {
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
    }
    else if (currentMenu == &settingsMenu)
    {
        // Settings menu with icons
        for (size_t i = 0; i < currentMenu->itemCount; i++)
        {
            nokia.setCursor(2, 15 + (i * 10));
            nokia.print(currentMenu->currentItem == i ? ">" : " ");
            nokia.print(currentMenu->items[i].name);

            // Add icons for settings menu items
            if (i == 0)
            { // Contrast
                nokia.drawBitmap(70, 13, SETTINGS_ICON, 8, 8, BLACK);
            }
            else if (i == 1)
            { // Brightness
                nokia.drawBitmap(70, 23, SETTINGS_ICON, 8, 8, BLACK);
            }
        }
    }
    else
    {
        // Post deposit menu
        for (size_t i = 0; i < currentMenu->itemCount; i++)
        {
            nokia.setCursor(2, 15 + (i * 10));
            nokia.print(currentMenu->currentItem == i ? ">" : " ");
            nokia.print(currentMenu->items[i].name);
        }
    }

    nokia.display();
}

void updateProgressDisplay(const String &message1, const String &message2, int progress)
{
    String progressBar = "[";
    for (int i = 0; i < 10; i++)
    {
        progressBar += (i < progress / 10) ? "=" : " ";
    }
    progressBar += "]";

    updateDualDisplayStatus(message1, message2 + " " + progressBar);
}
void pulseColor(int redValue, int greenValue, int blueValue)
{
    for (int brightness = 0; brightness <= 255; brightness += 5)
    {
        analogWrite(PIN_RED, (redValue * brightness) / 255);
        analogWrite(PIN_GREEN, (greenValue * brightness) / 255);
        analogWrite(PIN_BLUE, (blueValue * brightness) / 255);
        delay(10);
    }
    for (int brightness = 255; brightness >= 0; brightness -= 5)
    {
        analogWrite(PIN_RED, (redValue * brightness) / 255);
        analogWrite(PIN_GREEN, (greenValue * brightness) / 255);
        analogWrite(PIN_BLUE, (blueValue * brightness) / 255);
        delay(10);
    }
}

void ledStatusCode(int errorCode)
{
    switch (errorCode)
    {
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

void delayWithMsg(unsigned long duration, String message1, String message2, int statusCode)
{
    unsigned long startTime = millis();

    // Select appropriate icon based on status code
    const unsigned char *icon = nullptr;
    switch (statusCode)
    {
    case 200:
        icon = BOTTLE_ICON;
        break;
    case 404:
        icon = ERROR_ICON;
        break;
    case 102:
        icon = COIN_ICON;
        break;
    }

    updateDualDisplayStatus(message1, message2, icon);

    while (millis() - startTime < duration)
    {
        ledStatusCode(statusCode);
    }
}
int getCapacitiveSensorValue()
{
    long sum = 0;

    // Take multiple samples to reduce noise
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        sum += analogRead(CAPACITIVE_SENSOR_PIN);
        delay(2); // Short delay between readings for stability
    }

    int averageValue = sum / SAMPLE_COUNT;

    // if (DEBUG_SENSORS) {
    //     Serial.print("Raw Capacitive Value: ");
    //     Serial.println(averageValue);
    // }

    return averageValue;
}

// Modified sensor reading functions with debugging
// Check if bottle is detected with debouncing and hysteresis
bool readCapacitiveSensorData()
{
    // Debounce check
    if (millis() - capacitiveSensor.lastReadTime < DEBOUNCE_MS)
    {
        return capacitiveSensor.isDetecting;
    }

    capacitiveSensor.lastReadTime = millis();
    int currentValue = getCapacitiveSensorValue();

    // Update detection state with hysteresis to prevent flickering
    if (currentValue >= DETECTION_THRESHOLD)
    {
        capacitiveSensor.isDetecting = true;
        capacitiveSensor.lastStableValue = currentValue;
    }
    else if (currentValue < NO_BOTTLE_THRESHOLD)
    {
        capacitiveSensor.isDetecting = false;
        capacitiveSensor.lastStableValue = currentValue;
    }

    // if (DEBUG_SENSORS) {
    //     Serial.print("Capacitive Value: ");
    //     Serial.print(currentValue);
    //     Serial.print(" | State: ");
    //     Serial.println(capacitiveSensor.isDetecting ? "BOTTLE DETECTED" : "NO BOTTLE");
    // }

    return capacitiveSensor.isDetecting;
}
int readInductiveSensorData()
{
    delay(50); // Short delay for sensor stabilization
    int reading = digitalRead(inductiveSensorPin);
    // if (DEBUG_SENSORS) {
    //     Serial.print("Inductive Reading: ");
    //     Serial.println(reading);
    // }
    return reading;
}
bool isBottleFullyRemoved()
{
    int currentValue = getCapacitiveSensorValue();
    return currentValue < NO_BOTTLE_THRESHOLD;
}

void testCapacitiveSensor()
{
    Serial.println("\n=== Testing Analog Capacitive Sensor ===");
    Serial.println("Place and remove a plastic bottle multiple times");
    Serial.println("Detection Threshold: " + String(DETECTION_THRESHOLD));
    Serial.println("No Bottle Threshold: " + String(NO_BOTTLE_THRESHOLD));
    Serial.println("Testing for 30 seconds...");

    unsigned long startTime = millis();
    int sampleCount = 0;
    int maxReading = 0;
    int minReading = 1023;

    while (millis() - startTime < 30000)
    {
        int rawValue = getCapacitiveSensorValue();
        bool isDetecting = readCapacitiveSensorData();

        // Update min/max values
        maxReading = max(maxReading, rawValue);
        minReading = min(minReading, rawValue);

        Serial.println("\nReading #" + String(++sampleCount));
        Serial.println("Raw Value: " + String(rawValue));
        Serial.println("Status: " + String(isDetecting ? "BOTTLE DETECTED" : "NO BOTTLE"));

        delay(1000); // Update every second
    }

    Serial.println("\n=== Test Results ===");
    Serial.println("Samples Taken: " + String(sampleCount));
    Serial.println("Minimum Reading: " + String(minReading));
    Serial.println("Maximum Reading: " + String(maxReading));
    Serial.println("Current Thresholds:");
    Serial.println("- Detection: " + String(DETECTION_THRESHOLD));
    Serial.println("- No Bottle: " + String(NO_BOTTLE_THRESHOLD));
}
void setupCapacitiveSensor()
{
    pinMode(CAPACITIVE_SENSOR_PIN, INPUT);
    capacitiveSensor = CapacitiveSensorState(); // Initialize using constructor

    // if (DEBUG_SENSORS) {
    //     testCapacitiveSensor();
    // }
}
// Add this to your setup() function
void sensorSetup()
{
    // pinMode(capacitiveSensorPin, INPUT);
    pinMode(inductiveSensorPin, INPUT);

    // Serial.println("Starting sensor test...");
    // testSensors();
    // Serial.println("Sensor test complete");
}

void rotateServo(Servo &servo, int angle)
{
    // Ensure angle is within valid range
    angle = constrain(angle, 0, 120);
    servo.write(angle);
    delay(15); // Small delay to allow servo to reach position
}

void openCloseBinLid(int lidNum, bool toOpen)
{
    if (lidNum == 1)
    {
        // Servo 1: Rotate to 120 degrees when opening, 0 when closing
        int angle = toOpen ? 120 : 0;
        rotateServo(servo1, angle);
    }
    else if (lidNum == 2)
    {
        // Servo 2: Always rotate to 90 degrees when opening
        int angle = toOpen ? 90 : 0;
        rotateServo(servo2, angle);
    }
}
void waitToRemoveObject()
{
    lcd.clear();
    lcd.print(F("Remove Bottle!"));
    openCloseBinLid(1, true);

    unsigned long startTime = millis();
    while (!isBottleFullyRemoved())
    {
        ledStatusCode(404);

        // Timeout after 10 seconds
        if (millis() - startTime > 10000)
        {
            lcd.clear();
            lcd.print(F("Timeout!"));
            break;
        }
        delay(100);
    }

    isObjectInside = false;
    delay(2000);
    openCloseBinLid(1, false);
    ledStatusCode(200);
}
bool verifyObject()
{
    if (!isObjectInside)
    {
        return false;
    }

    // First notify user and close lid
    delayWithMsg(3000, "Lid is closing...", "Remove hand!!!", 404);
    openCloseBinLid(1, false);
    delay(1000); // Give time for lid to close and readings to stabilize

    unsigned long startTime = millis();
    bool verificationComplete = false;
    bool verificationResult = false;

    // Main verification loop
    while (!verificationComplete && (millis() - startTime < 3000))
    { // 3 second timeout
        ledStatusCode(102);
        lcd.clear();
        lcd.print(F("Verifying...."));

        // Get sensor readings
        bool capacitiveReading = readCapacitiveSensorData();
        int inductiveReading = readInductiveSensorData();

        // Debug output
        if (DEBUG_SENSORS)
        {
            Serial.println("Verification in progress:");
            Serial.println("Capacitive: " + String(capacitiveReading));
            Serial.println("Inductive: " + String(inductiveReading));
        }

        // Check if object is present and verify all conditions
        if (getCapacitiveSensorValue() >= DETECTION_THRESHOLD)
        {
            // Check all three conditions: capacitive, inductive, and weight
            if (readInductiveSensorData() == 1 && isWeightAcceptable())
            {
                lcd.clear();
                lcd.print(F("Verified!"));
                ledStatusCode(200);

                // Open second lid to drop bottle
                openCloseBinLid(2, true);
                delay(3000);
                openCloseBinLid(2, false);

                verificationResult = true;
                verificationComplete = true;
            }
            else
            {
                lcd.clear();
                lcd.print(F("Invalid object"));
                ledStatusCode(404);

                // Handle invalid object
                waitToRemoveObject();
                delayWithMsg(3000, "Lid closing", "remove hand", 404);
                openCloseBinLid(1, false);

                verificationResult = false;
                verificationComplete = true;
            }
        }
        // If we've waited at least 2 seconds and no valid object detected
        else if (millis() - startTime >= 2000)
        {
            lcd.clear();
            lcd.print(F("Invalid object"));
            ledStatusCode(404);

            // Handle invalid object
            waitToRemoveObject();
            delayWithMsg(3000, "Lid closing", "remove hand", 404);
            openCloseBinLid(1, false);

            verificationResult = false;
            verificationComplete = true;
        }

        delay(100); // Small delay to prevent tight loop
    }

    // Handle timeout case
    if (!verificationComplete)
    {
        lcd.clear();
        lcd.print(F("Verification"));
        lcd.setCursor(0, 1);
        lcd.print(F("timeout!"));
        ledStatusCode(404);
        waitToRemoveObject();
        return false;
    }

    return verificationResult;
}
void navigateMenu(int direction)
{
    currentMenu->currentItem = (currentMenu->currentItem + direction + currentMenu->itemCount) % currentMenu->itemCount;
    updateMenuDisplay();
    delay(200);
    while (digitalRead(direction > 0 ? downButton : upButton) == LOW)
        ;
}

void selectMenuItem()
{
    currentMenu->items[currentMenu->currentItem].action();
    updateMenuDisplay();
    delay(200);
    while (digitalRead(selectButton) == LOW)
        ;
}
// Example of how to use in waitForObjectPresence
void waitForObjectPresence()
{
    unsigned long startTime = millis();
    bool objectDetected = false;

    while (!objectDetected)
    {
        ledStatusCode(102);

        if (readCapacitiveSensorData())
        {
            // Additional confirmation check to avoid false positives
            delay(100); // Short delay for stability
            if (readCapacitiveSensorData())
            { // Double-check the reading
                objectDetected = true;
                break;
            }
        }

        if (millis() - startTime >= 3000)
        {
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

void depositAction()
{
    if (maintenanceMode)
    {
        displayNokiaStatus("System Full", ERROR_ICON);
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }

    displayNokiaStatus("Ready for Bottle", BOTTLE_ICON);
    lcd.clear();
    lcd.print(F("Opening bin...."));
    openCloseBinLid(1, true);

    displayNokiaStatus("Insert Bottle", BOTTLE_ICON);
    lcd.clear();
    lcd.print(F("Insert Bottle!"));

    waitForObjectPresence();
    if (verifyObject())
    {
        totalPoints++;
        displayNokiaStatus("Success!", BOTTLE_ICON);
        lcd.clear();
        lcd.print(F("Deposit success!"));
        lcd.setCursor(0, 1);
        lcd.print(F("Points: "));
        lcd.print(totalPoints);
        delay(2000);
        currentMenu = &postDepositMenu;
        updateMenuDisplay();
    }
}
void insertAnotherBottleAction()
{
    depositAction();
}

// RFID Functions
void redeemAction()
{
    if (maintenanceMode)
    {
        displayNokiaStatus("System Full", ERROR_ICON);
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }

    displayNokiaStatus("Present Card", CARD_ICON);
    lcd.clear();
    lcd.print("Present RFID");
    lcd.setCursor(0, 1);
    lcd.print("Card");

    unsigned long startTime = millis();
    while (millis() - startTime < 10000)
    {
        if (detectCard())
        {
            int currentPoints = readPoints();
            if (currentPoints >= 0)
            {
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

void redeemPointsAction()
{
    lcd.clear();
    lcd.print("Redeem Points");
    lcd.setCursor(0, 1);
    lcd.print("Points: ");
    lcd.print(pointsToRedeem);

    unsigned long lastButtonPress = 0;
    int holdTime = 0;
    const int maxRedeemable = min(totalPoints, 20);
    delay(1000);
    while (true)
    {
        lcd.setCursor(8, 1);
        lcd.print("    ");
        lcd.setCursor(8, 1);
        lcd.print(pointsToRedeem);

        if (digitalRead(upButton) == LOW || digitalRead(downButton) == LOW)
        {
            if (millis() - lastButtonPress > 200)
            {
                int direction = (digitalRead(upButton) == LOW) ? 1 : -1;
                pointsToRedeem += direction * (1 + (holdTime / 1000));
                pointsToRedeem = constrain(pointsToRedeem, 0, maxRedeemable);
                lastButtonPress = millis();
                holdTime += 200;
            }
        }
        else
        {
            holdTime = 0;
        }

        if (digitalRead(selectButton) == LOW)
        {
            if (millis() - lastButtonPress > 2000)
            {
                delayWithMsg(2000, "Redemption", "Cancelled", 404);
                pointsToRedeem = 0;
                currentMenu = &mainMenu;
                updateMenuDisplay();
                break;
            }
            else
            {
                totalPoints -= pointsToRedeem;
                if (writePoints(totalPoints))
                {
                    delayWithMsg(2000, "Redeeming: " + String(pointsToRedeem), "Please wait...", 200);
                    dispenseCoin(pointsToRedeem); // Dispense coins equal to pointsToRedeem
                    delayWithMsg(2000, "Redeemed: " + String(pointsToRedeem), "Remaining: " + String(totalPoints), 200);
                }
                else
                {
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
    Serial.println(F("Initializing RFID system..."));
    
    // Initialize SPI bus first
    SPI.begin();
    
    // Configure RFID module pins
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, HIGH);
    delay(50);  // Short delay after power up
    
    // Initialize MFRC522
    mfrc522.PCD_Init();
    delay(100);  // Give time for initialization
    
    // Check if module is responding
    byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    
    if (version == 0x91 || version == 0x92) {
        Serial.println(F("MFRC522 Initialized"));
        Serial.print(F("Firmware Version: 0x"));
        Serial.println(version, HEX);
    } else {
        Serial.println(F("Warning: Unknown MFRC522 version"));
        Serial.print(F("Version: 0x"));
        Serial.println(version, HEX);
    }
    
    // Turn on the antenna
    mfrc522.PCD_AntennaOn();
    delay(50);
    
    // Set antenna gain to maximum
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
    
    // Initialize authentication key (factory default)
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }
    
    // Test communication
    bool success = false;
    for (int i = 0; i < MAX_RFID_INIT_ATTEMPTS; i++) {
        if (mfrc522.PCD_PerformSelfTest()) {
            success = true;
            break;
        }
        delay(RFID_RESET_DELAY);
    }
    
    if (success) {
        Serial.println(F("RFID self-test passed"));
        // Reset the MFRC522 after self-test
        mfrc522.PCD_Reset();
        mfrc522.PCD_Init();
    } else {
        Serial.println(F("Warning: RFID self-test failed"));
    }
    
    // Final initialization
    mfrc522.PCD_Init();
    delay(50);
    
    Serial.println(F("RFID setup complete"));
}

bool detectCard() {
    // Reset the loop if no new card is present
    if (!mfrc522.PICC_IsNewCardPresent()) {
        return false;
    }

    // Select one of the cards
    if (!mfrc522.PICC_ReadCardSerial()) {
        return false;
    }

    // Card is detected
    Serial.print(F("\nCard UID: "));
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();

    // Print card type
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    Serial.print(F("Card type: "));
    Serial.println(mfrc522.PICC_GetTypeName(piccType));

    return true;
}

int readPoints() {
    byte buffer[18];
    byte size = sizeof(buffer);

    // Debug output
    Serial.println(F("Reading points..."));

    // Authenticate using key A
    MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        POINTS_BLOCK,  // The block we want to read
        &key,          // Key A
        &(mfrc522.uid)
    );

    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Authentication failed"));
        return -1;
    }

    // Read the block
    status = mfrc522.MIFARE_Read(POINTS_BLOCK, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Reading failed"));
        return -1;
    }

    // The points are stored in the first two bytes
    int points = (buffer[0] << 8) | buffer[1];
    
    Serial.print(F("Points read: "));
    Serial.println(points);

    return points;
}
bool writePoints(int points) {
    if (points < 0 || points > MAX_POINTS) {
        Serial.println(F("Invalid points value"));
        return false;
    }

    Serial.print(F("Writing points: "));
    Serial.println(points);

    byte buffer[16] = {0};  // Clear buffer
    buffer[0] = (points >> 8) & 0xFF;  // High byte
    buffer[1] = points & 0xFF;         // Low byte

    // Authenticate using key A
    MFRC522::StatusCode status = mfrc522.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        POINTS_BLOCK,
        &key,
        &(mfrc522.uid)
    );

    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Authentication failed"));
        return false;
    }

    // Write the block
    status = mfrc522.MIFARE_Write(POINTS_BLOCK, buffer, 16);
    if (status != MFRC522::STATUS_OK) {
        Serial.println(F("Writing failed"));
        return false;
    }

    Serial.println(F("Write successful"));
    return true;
}

MFRC522::StatusCode authenticateBlock(byte blockNumber) {
    // Make sure we're not trying to access a sector trailer
    if ((blockNumber + 1) % 4 == 0) {
        Serial.println(F("Error: Cannot access sector trailer"));
        return MFRC522::STATUS_ERROR;
    }

    return mfrc522.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        blockNumber,
        &key,
        &(mfrc522.uid)
    );
}
MFRC522::StatusCode authenticateBlock(int blockNumber)
{
    return mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                    blockNumber, &key, &(mfrc522.uid));
}
void storePointsAction() {
    lcd.clear();
    lcd.print("Present RFID Card");
    unsigned long startTime = millis();
    
    while (millis() - startTime < 10000) {
        if (detectCard()) {
            ledStatusCode(102); // Processing
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
void setup()
{
    // Start serial first for debugging
    Serial.begin(9600);
    Serial.println(F("Starting PISO-BOTE initialization..."));

    // Initialize displays
    Serial.println(F("Initializing displays..."));
    if (!setupNokiaDisplay()) {
        Serial.println(F("Nokia display initialization failed. System halted."));
        while (1) { delay(1000); }
    }
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print(F("Initializing..."));
    
    // Initialize buttons and basic pins
    Serial.println(F("Setting up pins..."));
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
    
    // Initialize sensors
    Serial.println(F("Setting up sensors..."));
    setupCapacitiveSensor();
    
    // Set initial LED status
    ledStatusCode(200);

    // Initialize servos
    Serial.println(F("Initializing servos..."));
    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    delay(500);  // Give servos time to initialize
    
    // Test servos
    Serial.println(F("Testing servo movements..."));
    openCloseBinLid(1, false);
    openCloseBinLid(2, false);
    delay(500);

    // Initialize scale
    Serial.println(F("Initializing load cell..."));
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    delay(1000);  // Give the scale time to stabilize
    scale.set_scale(CALIBRATION_FACTOR);
    scale.tare();
    Serial.println(F("Load cell initialized"));

    // Initialize RFID
    Serial.println(F("Setting up RFID..."));
    setUpRFID();
    
    // Initialize GSM module
    Serial.println(F("Setting up GSM module..."));
    sim800lv2.begin(9600);
    delay(3000);  // Give the module time to initialize
    
    // Configure GSM
    Serial.println(F("Configuring GSM..."));
    sim800lv2.println("AT");
    delay(1000);
    sim800lv2.println("AT+CMGF=1");
    delay(1000);
    sim800lv2.println("AT+CNMI=1,2,0,0,0");
    delay(1000);
    
    // Test GSM
    sim800lv2.println("AT");
    delay(1000);
    
    // Initialize coin hopper
    Serial.println(F("Setting up coin hopper..."));
    pinMode(coinHopperSensor_PIN, INPUT_PULLUP);
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW);
    
    // Final display setup
    pinMode(PIN_BL, OUTPUT);
    analogWrite(PIN_BL, 255);
    updateMenuDisplay();

    // Send initialization message
    Serial.println(F("Sending initialization SMS..."));
    sendSMS("PISO-BOTE system initialized");
    
    Serial.println(F("Initialization complete!"));
    lcd.clear();
    lcd.print(F("Ready!"));
    delay(1000);
    
    // Show main menu
    updateMenuDisplay();
}

void loop()
{
    if (maintenanceMode)
    {
        handleMaintenanceMode();
        return;
    }

    if (isBinFull())
    {
        maintenanceMode = true;
        displayNokiaStatus("System Full", ERROR_ICON);
        return;
    }

    if (digitalRead(downButton) == LOW)
    {
        navigateMenu(1);
    }
    if (digitalRead(upButton) == LOW)
    {
        navigateMenu(-1);
    }
    if (digitalRead(selectButton) == LOW)
    {
        selectMenuItem();
    }
    Serial.println(readCapacitiveSensorData());
    delay(100);
}

void sendSMS(String message)
{
    Serial.println("Sending SMS...");

    // Clear any pending serial data
    while (sim800lv2.available())
    {
        sim800lv2.read();
    }

    // Set SMS mode
    sim800lv2.println("AT+CMGF=1");
    delay(2000); // Keep this delay to ensure SMS mode is set

    // Set message destination
    sim800lv2.print("AT+CMGS=\"");
    sim800lv2.print(maintainerNum);
    sim800lv2.println("\"");

    // Increase delay to allow the SIM module more time to respond with ">"
    delay(3000); // Increased from 1000ms to 2000ms

    // Check for ">" response
    if (sim800lv2.find(">"))
    {
        // Send message content
        sim800lv2.println(message);
        delay(500);

        // Send message termination character
        sim800lv2.write(0x1A); // Ctrl+Z character
        delay(1000);

        // Wait for and check response
        unsigned long start = millis();
        bool success = false;

        while (millis() - start < 10000)
        { // 10-second timeout
            if (sim800lv2.available())
            {
                String response = sim800lv2.readString();
                if (response.indexOf("OK") != -1)
                {
                    success = true;
                    break;
                }
            }
            delay(100);
        }

        if (success)
        {
            Serial.println("SMS sent successfully!");
        }
        else
        {
            Serial.println("SMS sending failed - timeout");
        }
    }
    else
    {
        Serial.println("SMS sending failed - no prompt received");
    }
}
bool isBinFull()
{
    unsigned int duration = sonar.ping_median(ITERATIONS);
    float distance = (duration / 2.0) * 0.0343;
    if (distance > 0 && distance <= 10)
    {
        Serial.println("Bin full! Entering maintenance mode...");
        sendSMS("Alert: The PISO-BOTE is full! Please empty it.");
        ledStatusCode(404);
        return true;
    }
    return false;
}

void handleMaintenanceMode()
{
    lcd.clear();
    lcd.print("PISO-BOTE is full");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");

    while (isBinFull())
    {
        ledStatusCode(404);
        delay(5000);
    }
    maintenanceMode = false;
    lcd.clear();
    lcd.print("PISO-BOTE ready");
    ledStatusCode(200);
    delay(2000);
}

// 2. Replace the existing isWeightAcceptable() function with this new version
bool isWeightAcceptable() {
    // Use more readings for better stability
    const int STABLE_READINGS = 10;
    float readings[STABLE_READINGS];
    float totalWeight = 0;
    
    // Take multiple readings with longer delay between them
    for (int i = 0; i < STABLE_READINGS; i++) {
        readings[i] = scale.get_units();
        delay(200);  // Longer delay for better stability
    }
    
    // Calculate average and check stability
    float maxReading = readings[0];
    float minReading = readings[0];
    
    for (int i = 0; i < STABLE_READINGS; i++) {
        totalWeight += readings[i];
        maxReading = max(maxReading, readings[i]);
        minReading = min(minReading, readings[i]);
    }
    
    float averageWeight = totalWeight / STABLE_READINGS;
    float variability = maxReading - minReading;
    
    // Debug output
    Serial.print("Average Weight: ");
    Serial.print(averageWeight);
    Serial.print("g, Variability: ");
    Serial.print(variability);
    Serial.println("g");
    
    // Check if readings are stable enough
    // if (variability > STABILITY_THRESHOLD * 2) {  // Increased threshold
    //     delayWithMsg(1000, "Unstable weight", "Please try again", 404);
    //     return false;
    // }
    
    // More lenient weight validation
    const float WEIGHT_TOLERANCE = 2.0;  // Added tolerance
    
    if (averageWeight < (MIN_ACCEPTABLE_WEIGHT - WEIGHT_TOLERANCE)) {
        delayWithMsg(1000, "Too light:", String(averageWeight, 1) + "g", 404);
        return false;
    }
    
    if (averageWeight > (MAX_ACCEPTABLE_WEIGHT + WEIGHT_TOLERANCE)) {
        delayWithMsg(1000, "Too heavy:", String(averageWeight, 1) + "g", 404);
        return false;
    }
    
    // Weight is acceptable
    delayWithMsg(1000, "Weight OK:", String(averageWeight, 1) + "g", 200);
    return true;
}
int readLDRSensorData()
{
    return analogRead(LDR_PIN);
}

void controlLedInlet(bool isOn)
{
    digitalWrite(LED_INLET_PIN, isOn ? HIGH : LOW);
}

bool isObjectClear()
{
    if (isObjectInside)
    {
        delay(100);
        int lightValue = readLDRSensorData();
        delay(100);
        controlLedInlet(false);
        return lightValue > 200;
    }
    return true;
    // TODO: change to false
}
void dispenseCoin(int count)
{
    // Reset state
    coinCount = 0;
    dispensingActive = true;

    // Clear any pending signals
    delay(50);

    // Add serial debug message
    Serial.println(F("Starting coin dispensing..."));

    // Update display
    lcd.clear();
    lcd.print("Dispensing coins");
    lcd.setCursor(0, 1);
    lcd.print("Count: 0");

    // Activate relay with debounce protection
    digitalWrite(relayPin, HIGH);
    delay(50); // Allow relay to settle

    unsigned long startTime = millis();
    unsigned long lastDebounceTime = 0;

    while (dispensingActive)
    {
        // Check timeout
        if (millis() - startTime > COIN_DISPENSE_TIMEOUT)
        {
            Serial.println(F("Dispensing timeout!"));
            lcd.clear();
            lcd.print("Dispensing error");
            lcd.setCursor(0, 1);
            lcd.print("Contact staff");
            delay(3000);
            break;
        }

        // Read sensor with debounce
        bool currentSensorState = digitalRead(coinHopperSensor_PIN);

        // Implement proper debounce logic
        if (currentSensorState != lastSensorState)
        {
            if (millis() - lastDebounceTime > SENSOR_DEBOUNCE_DELAY)
            {
                if (currentSensorState == LOW)
                { // Coin detected
                    coinCount++;
                    Serial.print(F("Coin counted: "));
                    Serial.println(coinCount);

                    // Update display
                    lcd.setCursor(7, 1);
                    lcd.print(coinCount);

                    // Check if we've dispensed enough coins
                    if (coinCount >= count)
                    {
                        Serial.println(F("Dispensing complete"));
                        dispensingActive = false;
                    }
                }
                lastDebounceTime = millis();
            }
        }

        lastSensorState = currentSensorState;
        delay(10); // Small delay to prevent tight loop
    }

    // Safely deactivate relay
    delay(50);
    digitalWrite(relayPin, LOW);
    delay(50); // Allow relay to settle

    // Display final status
    lcd.clear();
    if (coinCount == count)
    {
        lcd.print("Dispensing done");
        Serial.println(F("Dispensing successful"));
    }
    else
    {
        lcd.print("Incomplete dispense");
        lcd.setCursor(0, 1);
        lcd.print("Coins: " + String(coinCount) + "/" + String(count));
        Serial.println(F("Dispensing incomplete"));
    }
    delay(2000);
}