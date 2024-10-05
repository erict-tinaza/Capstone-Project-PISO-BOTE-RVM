#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <HX711.h>

// Constants
const int POINTS_BLOCK = 4;
const int MAX_DISTANCE = 20;
const int ITERATIONS = 5;
const float MAX_WEIGHT = 200.00;

// Pin definitions
const int upButton = 2;
const int downButton = 3;
const int selectButton = 4;
const int servoPin1 = 5;
const int servoPin2 = 6;
const int capacitiveSensorPin = 7;
const int inductiveSensorPin = 8;
const int PIN_RED = 9;
const int PIN_GREEN = 10;
const int PIN_BLUE = 11;
const int SS_PIN = 53;
const int RST_PIN = 49;
const int TRIGGER_PIN = 22;
const int ECHO_PIN = 23;
const int SIM_RX = 24;
const int SIM_TX = 25;
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 27;
const int LDR_PIN = A0;
const int LED_INLET_PIN = 12;

// Object instantiations
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1, servo2;
MFRC522 mfrc522(SS_PIN, RST_PIN);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial sim800lv2(SIM_RX, SIM_TX);
HX711 scale;

// Global variables
bool isObjectInside = false;
int totalPoints = 0;
int pointsToRedeem = 0;
bool maintenanceMode = false;
String maintainerNum = "+639219133878";
MFRC522::MIFARE_Key key;

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

// Function prototypes
void depositAction();
void redeemAction();
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
int readCapacitiveSensorData();
int readInductiveSensorData();
void rotateServo(Servo &servo, int angle);
void openCloseBinLid(int lidNum, bool toOpen);
void waitToRemoveObject();
void waitForObjectPresence();
bool verifyObject();
MFRC522::StatusCode authenticateBlock(int blockNumber);
void calibrateLoadCell();

// Menu definitions
MenuItem mainMenuItems[] = {
    {"Deposit", depositAction},
    {"Redeem", redeemAction}
};

MenuItem postDepositMenuItems[] = {
    {"Insert Another", insertAnotherBottleAction},
    {"Redeem Points", redeemPointsAction},
    {"Store on RFID", storePointsAction}
};

Menu mainMenu = {mainMenuItems, 2, 0, "Main Menu"};
Menu postDepositMenu = {postDepositMenuItems, 3, 0, "Options"};
Menu *currentMenu = &mainMenu;

// Function implementations
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
        case 201:
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
    lcd.clear();
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print(message2);
    while (millis() - startTime < duration) {
        ledStatusCode(statusCode);
    }
}

int readCapacitiveSensorData() {
    delay(100);
    return digitalRead(capacitiveSensorPin);
}

int readInductiveSensorData() {
    delay(100);
    return digitalRead(inductiveSensorPin);
}

void rotateServo(Servo &servo, int angle) {
    servo.write(angle);
}

void openCloseBinLid(int lidNum, bool toOpen) {
    int angle = toOpen ? 180 : 0;
    Servo &servo = (lidNum == 1) ? servo1 : servo2;
    rotateServo(servo, angle);
}

void waitToRemoveObject() {
    lcd.clear();
    lcd.print("Remove object....");
    openCloseBinLid(1, true);
    while (readCapacitiveSensorData() == 1 || readInductiveSensorData() == 0) {
        ledStatusCode(404);
    }
    isObjectInside = false;
    delay(2000);
    openCloseBinLid(1, false);
    ledStatusCode(200);
}

void waitForObjectPresence() {
    unsigned long startTime = millis();
    while (readCapacitiveSensorData() == 0) {
        ledStatusCode(102);
        if (millis() - startTime >= 3000) {
            ledStatusCode(404);
            delayWithMsg(2000, "No object", "detected!", 404);
            delayWithMsg(2000, "Lid closing", "remove hand", 404);
            openCloseBinLid(1, false);
            ledStatusCode(200);
            isObjectInside = false;
            return;
        }
    }
    Serial.println("Object detected!");
    isObjectInside = true;
}

bool verifyObject() {
    if (!isObjectInside) return false;

    delayWithMsg(3000, "Lid is closing...", "Remove hand!!!", 404);
    openCloseBinLid(1, false);

    unsigned long startTime = millis();
    while (readCapacitiveSensorData() == 1) {
        ledStatusCode(102);
        lcd.clear();
        lcd.print("Verifying....");
        if (millis() - startTime >= 2000) {
            if (readCapacitiveSensorData() == 1 && 
                readInductiveSensorData() == 1 && 
                isWeightAcceptable() &&
                isObjectClear()) {
                lcd.clear();
                lcd.print("Verified!");
                ledStatusCode(200);
                openCloseBinLid(2, true);
                delay(3000);
                openCloseBinLid(2, false);
                delay(3000);
                return true;
            } else {
                lcd.clear();
                lcd.print("Invalid");
                lcd.setCursor(0, 1);
                lcd.print(!isWeightAcceptable() ? "Too heavy!" : "Not clear!");
                waitToRemoveObject();
                delayWithMsg(3000, "Lid closing", "remove hand", 404);
                openCloseBinLid(1, false);
                return false;
            }
        }
    }
    return false;
}

void updateMenuDisplay() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(currentMenu->title);
    lcd.setCursor(0, 1);
    lcd.print("> ");
    lcd.print(currentMenu->items[currentMenu->currentItem].name);
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

void depositAction() {
    if (maintenanceMode) {
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }
    lcd.clear();
    lcd.print("Opening bin....");
    openCloseBinLid(1, true);
    lcd.clear();
    lcd.print("Insert Bottle!");
    waitForObjectPresence();
    if (verifyObject()) {
        totalPoints++;
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

void redeemAction() {
    if (maintenanceMode) {
        delayWithMsg(2000, "PISO-BOTE is full", "Try again later", 404);
        return;
    }
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
    return weight <= MAX_WEIGHT;
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
    return false;
}

void setup() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    pinMode(upButton, INPUT_PULLUP);
    pinMode(downButton, INPUT_PULLUP);
    pinMode(selectButton, INPUT_PULLUP);
    pinMode(capacitiveSensorPin, INPUT);
    pinMode(inductiveSensorPin, INPUT);
    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(LED_INLET_PIN, OUTPUT);

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
    scale.set_scale(1313.03226);
    scale.tare();
}

void loop() {
    if (maintenanceMode) {
        handleMaintenanceMode();
        return;
    }
    
    if (isBinFull()) {
        maintenanceMode = true;
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
    
    delay(100);
}