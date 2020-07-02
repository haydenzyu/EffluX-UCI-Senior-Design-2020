#ifndef BUTTONS_H
#define BUTTON
#include <SparkFunSX1509.h>
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_TLC59711.h"

extern SX1509 io;

void gpio_setup();

bool button_0();

bool button_1();

bool button_2();

bool button_3();

bool button_4();

bool button_5();

bool button_6();

#endif