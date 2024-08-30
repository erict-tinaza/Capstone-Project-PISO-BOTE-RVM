#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd_1(0x27, 16, 2);

//Control button for the menu
const int upButton = 2;
const int downButton = 3;
const int selectButton = 4;
int menu = 1;
int timer_1 = 0;
int timer_2 = 0;

//Servo Pins
const int servoPin1 = 5;
const int servoPin2 = 6;

//Inductive and Capacitive Sensor Pins
const int capacitiveSensor = 7;
const int inductiveSensor = 8;


