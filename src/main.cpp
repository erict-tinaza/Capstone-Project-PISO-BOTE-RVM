#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1;
Servo servo2;
const int upButton = 2;
const int downButton = 3;
const int selectButton = 4;
int menu = 1;
int timer_1 = 0;
int timer_2 = 0;

// Servo Pins
const int servoPin1 = 5;
const int servoPin2 = 6;

// Inductive and Capacitive Sensor Pins
const int capacitiveSensorPin = 7;
const int inductiveSensorPin = 8;

// RGB Module pins
const int PIN_RED = 9;
const int PIN_GREEN = 10;
const int PIN_BLUE = 11;

// Global variables
bool isObjectInside = false;
bool isPlasticBottle = false;
/*####################################### FUNCTION DIFINITIONS HERE######################################*/


void pulseColor(int redValue, int greenValue, int blueValue)
{
  //(+) brightness

  for (int brightness = 0; brightness <= 255; brightness += 5)
  {
    analogWrite(PIN_RED, (redValue * brightness) / 255);
    analogWrite(PIN_GREEN, (greenValue * brightness) / 255);
    analogWrite(PIN_BLUE, (blueValue * brightness) / 255);
    delay(10); 
  }

  // (-) brightness
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
  case 200: /*Green Indicator = Ok */
    /* code */
    analogWrite(PIN_RED, 0);
    analogWrite(PIN_GREEN, 255);
    analogWrite(PIN_BLUE, 0);
    break;
  case 201:
    pulseColor(255, 0, 0);
    break;
  case 102: // Orange indicator = Processing
    pulseColor(255, 60, 5);
    break;
  case 404: // Red indicator = error
    pulseColor(255, 0, 0);
    break;
  default:
    break;
  }
}

void delayWithMsg(unsigned long duration, String message1, String message2, int statusCode) {
  unsigned long startTime = millis(); // Record the start time
  lcd.clear();
    lcd.print(message1);
    lcd.setCursor(0, 1);
    lcd.print(message2);
  while (millis() - startTime < duration) {
    ledStatusCode(statusCode);
  }
}

int readCapacitiveSensorData()
{
  int sensorVal = digitalRead(capacitiveSensorPin);
  // delay(1000);
  return sensorVal;
}

int readInductiveSensorData()
{
  int sensorVal = digitalRead(inductiveSensorPin);
  return sensorVal;
}

void rotateServo(Servo &servo, int angle)
{
  servo.write(angle);
}

void openCloseBinLid(int lidNum, bool toOpen)
{
  int angle = toOpen ? 180 : 0;
  if (lidNum == 1)
  {
    rotateServo(servo1, angle);
  }
  else if (lidNum == 2)
  {
    rotateServo(servo2, angle);
  }
}
void waitToRemoveObject()
{
  lcd.clear();
  lcd.print("Remove object....");
  openCloseBinLid(1, true);
  while (readCapacitiveSensorData() == 1 || readInductiveSensorData() == 0)
  {
    ledStatusCode(404);
  }
  isObjectInside = false;
  delay(2000);
  openCloseBinLid(1, false);
  ledStatusCode(200);
}
void waitForObjectPresence()
{
  unsigned long startTime = millis();

  while (readCapacitiveSensorData() == 0)
  {
    ledStatusCode(102);
    if (millis() - startTime >= 3000)
    {
      ledStatusCode(404);
      delayWithMsg(2000, "No object", "detected!", 404);
      delayWithMsg(2000, "Lid closind", "remove hand", 404);
      openCloseBinLid(1, false);
      ledStatusCode(200);
      isObjectInside = false;
      return;
    }
  }
  // lcd.clear();
  // lcd.print("Object present");
  Serial.print("Object detected!");
  isObjectInside = true;
}
bool verifyObject()
{
  if (isObjectInside)
  {
    delayWithMsg(3000, "Lid is closing...", "Remove hand!!!", 404);
    openCloseBinLid(1, false);

    unsigned long startTime = millis();

    while (readCapacitiveSensorData() == 1)
    {
      ledStatusCode(102);
      lcd.clear();
      lcd.print("Verifying....");
      if (millis() - startTime >= 2000)
      {
        if (readCapacitiveSensorData() == 1 && readInductiveSensorData() == 1)
        {
          lcd.clear();
          // lcd.print(readInductiveSensorData());
          lcd.print("Verified!");
          ledStatusCode(200);
          openCloseBinLid(2, true);
          delay(3000);
          openCloseBinLid(2, false);
          delay(3000);
          return true;
        }
        else
        {
          lcd.clear();
          lcd.print("Invalid");
          waitToRemoveObject();
          delayWithMsg(3000, "Lid closind", "remove hand", 404);
          openCloseBinLid(1, false);
          return false;
        }
      }
    }
  }
}
  void updateMenu()
  {
    switch (menu)
    {
    case 0:
      menu = 1;
      break;
    case 1:
      lcd.clear();
      lcd.print(">Deposit");
      lcd.setCursor(0, 1);
      lcd.print(" Redeem");
      break;
    case 2:
      lcd.clear();
      lcd.print(" Deposit");
      lcd.setCursor(0, 1);
      lcd.print(">Redeem");
      break;
    case 3:
      menu = 2;
      break;
    }
  }

  void deposit()
  {
    lcd.clear();
    lcd.print("Opening bin....");
    openCloseBinLid(1, true);
    lcd.clear();
    lcd.print("Insert Bottle!");
    waitForObjectPresence();
    // openCloseBinLid(1, false);
    verifyObject();
  }

  void redeem()
  {
  }

  void executeAction()
  {
    switch (menu)
    {
    case 1:
      deposit();
      break;
    case 2:
      redeem();
      break;
    }
  }

  void setup()
  {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    pinMode(upButton, INPUT_PULLUP);
    pinMode(downButton, INPUT_PULLUP);
    pinMode(selectButton, INPUT_PULLUP);
    pinMode(capacitiveSensorPin, INPUT);
    pinMode(inductiveSensorPin, INPUT);

    // RGB PINS
    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    ledStatusCode(200);

    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    openCloseBinLid(1, false);
    openCloseBinLid(2, false);
    updateMenu();
    Serial.begin(9600);
  }

  void loop()
  {
    // Display sensor reading to serial monitor
    Serial.print("Sensor value: ");
    Serial.println(readCapacitiveSensorData());
    // Serial.println(readInductiveSensorData());
    if (!digitalRead(downButton))
    {
      menu++;
      updateMenu();
      delay(100);
      while (!digitalRead(downButton))
        ;
    }
    if (!digitalRead(upButton))
    {
      menu--;
      updateMenu();
      delay(100);
      while (!digitalRead(upButton))
        ;
    }
    if (!digitalRead(selectButton))
    {
      executeAction();
      updateMenu();
      delay(100);
      while (!digitalRead(selectButton))
        ;
    }
  }
